"""RGB event recorder.

Continuously captures from the Camera Module 2, but only *writes* a video
clip when the thermal detection loop flags an animal in frame - covering
--pre-roll-seconds before the trigger and --post-roll-seconds after it
clears, using an always-current rolling buffer of recent frames to backfill
the pre-roll once a clip is actually opened.

Continuous RGB recording was deliberately not chosen: at any reasonable
resolution/fps it would dwarf the thermal stream's disk footprint over a
multi-day field deployment - see the project README's disk budget section.
Event-triggered recording keeps RGB footage proportional to how much
wildlife activity actually happens, not to how long the recorder runs.

Camera must come from the apt-installed picamera2 (system Python, or a venv
created with --system-site-packages) - see the project README.
"""

import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2

from diskspace import free_space_mb
from event_trigger import EventTrigger
from health_monitor import HealthMonitor
from stats_logger import StatsLogger

STATS_INTERVAL_SECONDS = 60.0
CLIP_OPEN_RETRY_SECONDS = 30.0

# --rgb-format value -> (file extension, fourcc). The extension must be a real container
# name, not the format string itself: OpenCV picks the container from the filename, and
# "rgb_event_x.avi-mjpg" isn't one, so VideoWriter silently fails to open.
FORMATS = {
    "avi": ("avi", "XVID"),
    "avi-mjpg": ("avi", "MJPG"),
    "mp4": ("mp4", "mp4v"),
}

# cv2.rotate() codes, keyed by clockwise rotation in degrees - same convention
# as thermal_source.py's/rgb_source.py's, kept local since the two cameras
# rotate independently.
ROTATE_CODES = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


class _EventClip:
    """One triggered recording: owns its VideoWriter and JSON sidecar."""

    def __init__(self, outdir: Path, prefix: str, fmt: str, fps: float, width: int, height: int):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{prefix}_{stamp}"
        extension, fourcc_code = FORMATS[fmt]
        self.video_path = outdir / f"{base}.{extension}"
        self.meta_path = outdir / f"{base}.json"
        fourcc = cv2.VideoWriter_fourcc(*fourcc_code)
        self.writer = cv2.VideoWriter(str(self.video_path), fourcc, fps, (width, height), True)
        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {self.video_path}")
        self.frame_count = 0
        self.trigger_started_at = None   # wall-clock time the trigger first went active
        self.opened_at = time.time()     # wall-clock time this clip started (pre-roll included)

    def write(self, bgr_frame) -> None:
        self.writer.write(bgr_frame)
        self.frame_count += 1

    def close(self, reason: str) -> dict:
        end_time = time.time()
        self.writer.release()
        meta = {
            "video_file": self.video_path.name,
            "trigger_started_at": self.trigger_started_at,
            "clip_started_at": self.opened_at,
            "clip_ended_at": end_time,
            "duration_seconds": round(end_time - self.opened_at, 2),
            "frame_count": self.frame_count,
            "close_reason": reason,
        }
        try:
            self.meta_path.write_text(json.dumps(meta, indent=2))
        except OSError as e:
            print(f"[rgb-event] could not write sidecar for {self.video_path.name} ({e!r}).")
        print(f"[rgb-event] closed {self.video_path.name}: "
              f"{self.frame_count} frames, {meta['duration_seconds']:.1f}s ({reason})")
        return meta


def run(
    trigger: EventTrigger,
    stop_event,
    outdir: str = "recordings_rgb_events",
    prefix: str = "rgb_event",
    fmt: str = "avi",
    width: int = 640,
    height: int = 480,
    fps: float = 15.0,
    rotate_degrees: int = 0,
    pre_roll_seconds: float = 2.0,
    post_roll_seconds: float = 2.0,
    min_free_mb: float = 1000.0,
    stats_log_path: str | None = None,
):
    """Runs until stop_event is set. Intended to be the target of a daemon thread."""
    from picamera2 import Picamera2  # imported lazily, same reasoning as rgb_source.py

    outdir_path = Path(outdir)
    outdir_path.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "BGR888"},
        controls={"FrameRate": fps},
    )
    picam2.configure(config)
    picam2.start()
    print(f"[rgb-event] Camera Module 2 started at {width}x{height} @ {fps}fps "
          f"(rotate {rotate_degrees}deg, pre-roll {pre_roll_seconds}s, "
          f"post-roll {post_roll_seconds}s).")

    rotate_code = ROTATE_CODES[rotate_degrees]
    # The VideoWriter needs the *post-rotation* frame size - a 90/270 rotation
    # swaps width and height, same as segment_writer.py's SegmentWriter does
    # for the thermal side.
    out_width, out_height = (height, width) if rotate_degrees in (90, 270) else (width, height)

    # Rolling buffer of the last pre_roll_seconds of frames, always kept warm
    # regardless of whether anything is currently being recorded, so a clip
    # can be backfilled with what happened just before the trigger fired.
    buffer = deque()  # [(timestamp, bgr_frame), ...], oldest first

    clip = None
    low_space_warned = False
    clip_retry_after = 0.0
    clip_open_failures = 0
    stats_logger = StatsLogger(stats_log_path) if stats_log_path else None
    health_monitor = HealthMonitor("rgb-event")
    captured_count = 0
    capture_time_total = 0.0
    encoded_count = 0
    encode_time_total = 0.0
    last_stats_time = time.time()

    try:
        while not stop_event.is_set():
            t_cap0 = time.perf_counter()
            bgr = picam2.capture_array()
            if rotate_code is not None:
                bgr = cv2.rotate(bgr, rotate_code)
            capture_time_total += time.perf_counter() - t_cap0
            now = time.time()
            captured_count += 1

            buffer.append((now, bgr))
            while len(buffer) > 1 and now - buffer[0][0] > pre_roll_seconds:
                buffer.popleft()

            active, last_active_time = trigger.snapshot()

            if clip is None and active and now >= clip_retry_after:
                if free_space_mb(outdir_path) < min_free_mb:
                    if not low_space_warned:
                        print(f"[rgb-event] free space below {min_free_mb}MB; "
                              "skipping new event clips until space frees up.")
                        low_space_warned = True
                else:
                    low_space_warned = False
                    try:
                        clip = _EventClip(outdir_path, prefix, fmt, fps, out_width, out_height)
                    except Exception as e:
                        # Used to be uncaught: one failed VideoWriter open (e.g. an
                        # unsupported --rgb-format/size combination) killed this thread
                        # for good, leaving the rest of a multi-day run recording
                        # thermal-only with nothing but a traceback on a console nobody
                        # was watching. Report it, back off, keep capturing.
                        print(f"[rgb-event] could not open a clip ({e!r}); "
                              f"retrying in {CLIP_OPEN_RETRY_SECONDS:.0f}s.")
                        clip_retry_after = now + CLIP_OPEN_RETRY_SECONDS
                        clip_open_failures += 1
                        clip = None
                    else:
                        clip.trigger_started_at = last_active_time
                        for _buf_ts, buf_frame in buffer:
                            clip.write(buf_frame)

            elif clip is not None:
                t_enc0 = time.perf_counter()
                clip.write(bgr)
                encode_time_total += time.perf_counter() - t_enc0
                encoded_count += 1
                if not active and now - last_active_time >= post_roll_seconds:
                    clip.close(reason="post-roll elapsed")
                    clip = None

            if now - last_stats_time >= STATS_INTERVAL_SECONDS:
                elapsed = now - last_stats_time
                captured_fps = captured_count / elapsed
                avg_capture_ms = (capture_time_total / captured_count * 1000) if captured_count else 0.0
                avg_encode_ms = (encode_time_total / encoded_count * 1000) if encoded_count else 0.0
                free_mb = free_space_mb(outdir_path)
                health = health_monitor.sample()
                print(
                    f"[rgb-event] stats: {captured_fps:.1f} captured fps, "
                    f"clip_open={clip is not None}, {avg_capture_ms:.1f} ms/capture avg, "
                    f"{avg_encode_ms:.1f} ms/encode avg, {free_mb:.0f}MB free, "
                    f"{HealthMonitor.brief(health)}"
                )
                if stats_logger is not None:
                    try:
                        stats_logger.log(
                            captured_fps=round(captured_fps, 2),
                            clip_open=clip is not None,
                            avg_capture_ms=round(avg_capture_ms, 2),
                            avg_encode_ms=round(avg_encode_ms, 2),
                            free_disk_mb=round(free_mb, 1),
                            clip_open_failures=clip_open_failures,
                            **health,
                        )
                    except OSError:
                        pass
                captured_count = 0
                capture_time_total = 0.0
                encoded_count = 0
                encode_time_total = 0.0
                last_stats_time = now

                # Checked here rather than only at clip-open time, so a clip that's
                # been open for a long time (an animal that doesn't leave, or several
                # in quick succession - see event_trigger.py) doesn't run the SD card
                # to zero before the per-open check would ever fire again.
                if clip is not None and free_mb < min_free_mb:
                    print(f"[rgb-event] free space below {min_free_mb}MB mid-clip; "
                          "closing it early.")
                    clip.close(reason="low disk space")
                    clip = None
                    low_space_warned = True
    finally:
        if clip is not None:
            clip.close(reason="shutdown")
        if stats_logger is not None:
            try:
                stats_logger.close()
            except OSError:
                pass
        picam2.stop()
        print("[rgb-event] capture stopped, camera released.")
