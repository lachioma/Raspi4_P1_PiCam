"""Continuous segmented thermal video writer.

Adapted from p3-ir-camera/record_p1_segmented.py's SegmentWriter, proven
across a real multi-day field deployment (161+ hours). Two changes from
that original:

  - width/height are constructor parameters instead of a module-level
    WIDTH, HEIGHT pinned to the P1 - this project also supports the P3.
  - write() takes an optional list of detection overlay boxes, drawn onto
    the saved video (not the raw radiometric data - that must stay an
    unmodified temperature reading) right before the timestamp is burned
    in, so a box that fired during a field deployment is visible directly
    in the archived footage, not only in a live preview nobody was watching.

Everything else - segment rotation, the JSON sidecar, recording_log.csv,
optional raw radiometric data, disk-space awareness - is unchanged in spirit
from the original. See that file's module docstring for the full design
rationale (crash-safety tradeoffs between --format options, why raw data is
streamed frame-by-frame instead of buffered, etc.).
"""

import csv
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from p3_camera import celsius_to_raw

# --thermal-format value -> (file extension, fourcc). The extension must be a real
# container name, not the format string itself: OpenCV picks the container from the
# filename, and "x.avi-mjpg" isn't one, so VideoWriter silently fails to open. (The
# original record_p1_segmented.py has the same latent bug for its avi-mjpg option.)
FORMATS = {
    "avi": ("avi", "XVID"),
    "avi-mjpg": ("avi", "MJPG"),
    "mp4": ("mp4", "mp4v"),
}

# cv2.rotate() codes, keyed by clockwise rotation in degrees.
ROTATE_CODES = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}

_TIMESTAMP_FONT = cv2.FONT_HERSHEY_SIMPLEX
_TIMESTAMP_MARGIN = 2
_BOX_COLOR = (0, 255, 0)


def _rotate(img, degrees):
    code = ROTATE_CODES[degrees]
    return img if code is None else cv2.rotate(img, code)


def _map_temperature_range(thermal_raw, raw_min, raw_max):
    """Render raw 16-bit temperature data to 8-bit using a *fixed* range.

    Unlike the camera's own hardware AGC (ir_brightness), which re-normalizes
    every frame to whatever range is currently in the scene, this always maps
    the same [raw_min, raw_max] window to [0, 255], so a genuine sensor drift
    over hours/days shows up as the video trending toward one end of the
    brightness range instead of being silently re-stretched away.
    """
    scaled = (thermal_raw.astype(np.float32) - raw_min) * (255.0 / (raw_max - raw_min))
    return np.clip(scaled, 0, 255).astype(np.uint8)


def _draw_timestamp(bgr_frame, ts):
    """Burn a date/time stamp (with milliseconds) into the bottom-left corner,
    with the font scale computed per-frame so it always fits."""
    dt = datetime.fromtimestamp(ts)
    text = dt.strftime("%Y-%m-%d %H:%M:%S") + f".{dt.microsecond // 1000:03d}"
    frame_h, frame_w = bgr_frame.shape[:2]
    max_width = frame_w - 2 * _TIMESTAMP_MARGIN
    max_height = frame_h - 2 * _TIMESTAMP_MARGIN

    scale = 0.35
    (text_w, text_h), baseline = cv2.getTextSize(text, _TIMESTAMP_FONT, scale, 1)
    if text_w > max_width or (text_h + baseline) > max_height:
        scale *= min(max_width / text_w, max_height / (text_h + baseline))
        scale = max(scale, 0.1)
        (text_w, text_h), baseline = cv2.getTextSize(text, _TIMESTAMP_FONT, scale, 1)

    org = (_TIMESTAMP_MARGIN, frame_h - _TIMESTAMP_MARGIN - baseline)
    cv2.putText(bgr_frame, text, org, _TIMESTAMP_FONT, scale, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(bgr_frame, text, org, _TIMESTAMP_FONT, scale, (255, 255, 255), 1, cv2.LINE_AA)


def _draw_overlay_boxes(bgr_frame, overlay_boxes):
    for x0, y0, x1, y1, track_id in overlay_boxes:
        cv2.rectangle(bgr_frame, (int(x0), int(y0)), (int(x1), int(y1)), _BOX_COLOR, 1)
        label = f"#{track_id}"
        y_text = y0 - 3 if y0 > 8 else y1 + 9
        cv2.putText(bgr_frame, label, (max(int(x0) - 1, 0), int(y_text)),
                    _TIMESTAMP_FONT, 0.3, _BOX_COLOR, 1, cv2.LINE_AA)


class SegmentWriter:
    """Owns the video writer (and optional raw/timestamp buffers) for one segment file."""

    def __init__(
        self,
        outdir,
        prefix,
        fmt,
        fps,
        width,
        height,
        save_raw,
        save_timestamps,
        rotate_degrees,
        temp_range_c=None,
        raw_every_n=1,
    ):
        self.outdir = outdir
        self.prefix = prefix
        self.fmt = fmt
        self.fps = fps
        self.save_raw = save_raw
        self.save_timestamps = save_timestamps
        self.rotate_degrees = rotate_degrees
        self.raw_every_n = raw_every_n
        self.temp_range_c = temp_range_c
        self.temp_raw_bounds = (
            (celsius_to_raw(temp_range_c[0]), celsius_to_raw(temp_range_c[1]))
            if temp_range_c is not None
            else None
        )
        if rotate_degrees in (90, 270):
            self.out_width, self.out_height = height, width
        else:
            self.out_width, self.out_height = width, height

        self.video_writer = None
        self.video_path = None
        self.raw_path = None
        self.ts_path = None
        self.meta_path = None
        self._raw_fh = None
        self._ts_fh = None
        self.frame_count = 0
        self.raw_frame_count = 0
        self.marker_mismatches = 0
        self.start_time = None

    def open(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{self.prefix}_{stamp}"
        extension, fourcc_code = FORMATS[self.fmt]
        self.video_path = self.outdir / f"{base}.{extension}"
        self.raw_path = self.outdir / f"{base}_raw.dat"
        self.ts_path = self.outdir / f"{base}_timestamps.txt"
        self.meta_path = self.outdir / f"{base}.json"

        fourcc = cv2.VideoWriter_fourcc(*fourcc_code)
        # Written as color (grayscale replicated across BGR channels, plus any
        # detection overlay) since single-channel output is unreliable across
        # OpenCV/ffmpeg backends for MP4; this keeps the file directly
        # playable in VLC etc.
        self.video_writer = cv2.VideoWriter(
            str(self.video_path), fourcc, self.fps, (self.out_width, self.out_height), True
        )
        if not self.video_writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {self.video_path}")

        self._raw_fh = open(self.raw_path, "wb") if self.save_raw else None
        self._ts_fh = open(self.ts_path, "w") if self.save_timestamps else None

        self.frame_count = 0
        self.raw_frame_count = 0
        self.marker_mismatches = 0
        self.start_time = time.time()

        extra = []
        if self.save_raw:
            extra.append(self.raw_path.name)
        if self.save_timestamps:
            extra.append(self.ts_path.name)
        suffix = f" (+ {', '.join(extra)})" if extra else ""
        print(f"[segment] recording -> {self.video_path.name}{suffix}")

    def write(self, ir_brightness, thermal_raw, overlay_boxes=None):
        now = time.time()

        if self.temp_raw_bounds is not None:
            raw_min, raw_max = self.temp_raw_bounds
            display = _map_temperature_range(thermal_raw, raw_min, raw_max)
        else:
            display = ir_brightness

        frame = _rotate(display, self.rotate_degrees)
        bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if overlay_boxes:
            _draw_overlay_boxes(bgr, overlay_boxes)
        _draw_timestamp(bgr, now)
        self.video_writer.write(bgr)

        if self._raw_fh is not None and self.frame_count % self.raw_every_n == 0:
            # Rotated the same way as the video so pixel (r, c) still refers
            # to the same physical spot in both files, but never stamped or
            # boxed: this must stay the camera's true temperature reading.
            self._raw_fh.write(_rotate(thermal_raw, self.rotate_degrees).tobytes())
            self._raw_fh.flush()
            self.raw_frame_count += 1
        if self._ts_fh is not None:
            self._ts_fh.write(f"{now!r}\n")
            self._ts_fh.flush()
        self.frame_count += 1

    def close(self):
        end_time = time.time()
        self.video_writer.release()
        if self._raw_fh is not None:
            self._raw_fh.close()
        if self._ts_fh is not None:
            self._ts_fh.close()

        duration = end_time - self.start_time
        meta = {
            "video_file": self.video_path.name,
            "raw_file": self.raw_path.name if self.save_raw else None,
            "raw_dtype": "uint16" if self.save_raw else None,
            "raw_every_n": self.raw_every_n if self.save_raw else None,
            "raw_shape": [self.raw_frame_count, self.out_height, self.out_width]
            if self.save_raw
            else None,
            "timestamps_file": self.ts_path.name if self.save_timestamps else None,
            "rotate_degrees": self.rotate_degrees,
            "video_source": "fixed_temp_range" if self.temp_range_c else "hardware_agc",
            "temp_min_c": self.temp_range_c[0] if self.temp_range_c else None,
            "temp_max_c": self.temp_range_c[1] if self.temp_range_c else None,
            "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
            "end_time": datetime.fromtimestamp(end_time).isoformat(),
            "duration_seconds": round(duration, 2),
            "frame_count": self.frame_count,
            "target_fps": self.fps,
            "achieved_fps": round(self.frame_count / duration, 2) if duration > 0 else 0.0,
            "marker_mismatches": self.marker_mismatches,
        }
        self.meta_path.write_text(json.dumps(meta, indent=2))
        print(
            f"[segment] closed {self.video_path.name}: {self.frame_count} frames, "
            f"{meta['achieved_fps']:.1f} fps, {self.marker_mismatches} marker mismatches"
        )
        return meta


def append_run_log(log_path, meta):
    is_new = not log_path.exists()
    with open(log_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(meta.keys()))
        if is_new:
            writer.writeheader()
        writer.writerow(meta)
