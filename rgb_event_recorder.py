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

Clip encoding (--rgb-format). The default, avi-jpeg (mjpeg_avi.py), encodes
each frame with cv2.imencode on a thread pool and never blocks capture, so a
clip really contains the pre-roll *and* the event *and* the post-roll, at the
camera's frame rate, with correct playback timing. The cv2.VideoWriter formats
(avi = XVID, avi-mjpg, mp4) are kept for comparison but are synchronous and
slower than 30fps at 1640x1232 on a Pi 4 (XVID 48-69 ms/frame, MJPG 60 ms):
opening a clip with them stalls capture while the ~60-frame pre-roll backlog
is encoded, so short events end up as pre-roll plus a frame or two, and they
declare 30fps while writing ~16, so they play back ~1.8x fast.

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
from mjpeg_avi import JpegClipWriter
from stats_logger import StatsLogger

STATS_INTERVAL_SECONDS = 60.0
CLIP_OPEN_RETRY_SECONDS = 30.0

JPEG_FORMAT = "avi-jpeg"

# cv2.VideoWriter formats: --rgb-format value -> (file extension, fourcc). The extension
# must be a real container name, not the format string itself: OpenCV picks the container
# from the filename, and "rgb_event_x.avi-mjpg" isn't one, so VideoWriter silently fails
# to open.
VIDEOWRITER_FORMATS = {
    "avi": ("avi", "XVID"),
    "avi-mjpg": ("avi", "MJPG"),
    "mp4": ("mp4", "mp4v"),
}
FORMATS = (JPEG_FORMAT, *VIDEOWRITER_FORMATS)

# cv2.rotate() codes, keyed by clockwise rotation in degrees - same convention
# as thermal_source.py's/rgb_source.py's, kept local since the two cameras
# rotate independently.
ROTATE_CODES = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


class _Clip:
    """What every clip type shares: names, timing and the JSON sidecar.

    write(ts, bgr) takes the frame's capture time; the first one fixes
    clip_started_at, so it includes the pre-roll (it used to be the moment the
    clip was opened, which left the pre-roll out of duration_seconds).
    """

    def __init__(self, outdir: Path, prefix: str, extension: str, fmt: str):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{prefix}_{stamp}"
        self.video_path = outdir / f"{base}.{extension}"
        self.meta_path = outdir / f"{base}.json"
        self.fmt = fmt
        self.trigger_started_at = None   # wall-clock time the trigger first went active
        self.first_ts = None             # capture time of the earliest frame (pre-roll start)

    def _meta(self, end_time: float, frame_count: int, reason: str, extra: dict) -> dict:
        start = self.first_ts if self.first_ts is not None else end_time
        meta = {
            "video_file": self.video_path.name,
            "encoder": self.fmt,
            "trigger_started_at": self.trigger_started_at,
            "clip_started_at": start,
            "clip_ended_at": end_time,
            "duration_seconds": round(end_time - start, 2),
            "frame_count": frame_count,
            "close_reason": reason,
            **extra,
        }
        try:
            self.meta_path.write_text(json.dumps(meta, indent=2))
        except OSError as e:
            print(f"[rgb-event] could not write sidecar for {self.video_path.name} ({e!r}).")
        print(f"[rgb-event] closed {self.video_path.name}: {frame_count} frames, "
              f"{meta['duration_seconds']:.1f}s ({reason})"
              + (f", {extra['frames_duplicated']} repeated / {extra['frames_dropped']} dropped"
                 if "frames_dropped" in extra else ""))
        return meta


class _VideoWriterClip(_Clip):
    """A clip written through cv2.VideoWriter - synchronous, see the module docstring."""

    def __init__(self, outdir: Path, prefix: str, fmt: str, fps: float, width: int, height: int):
        extension, fourcc_code = VIDEOWRITER_FORMATS[fmt]
        super().__init__(outdir, prefix, extension, fmt)
        fourcc = cv2.VideoWriter_fourcc(*fourcc_code)
        self.writer = cv2.VideoWriter(str(self.video_path), fourcc, fps, (width, height), True)
        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {self.video_path}")
        self.frame_count = 0
        self._enc_count = 0
        self._enc_seconds = 0.0

    def write(self, ts: float, bgr) -> None:
        if self.first_ts is None:
            self.first_ts = ts
        t0 = time.perf_counter()
        self.writer.write(bgr)
        self._enc_seconds += time.perf_counter() - t0
        self._enc_count += 1
        self.frame_count += 1

    def take_encode_stats(self):
        n, s = self._enc_count, self._enc_seconds
        self._enc_count, self._enc_seconds = 0, 0.0
        return n, s

    def close(self, reason: str) -> dict:
        end_time = time.time()
        self.writer.release()
        return self._meta(end_time, self.frame_count, reason, {})


class _JpegClip(_Clip):
    """A clip written by mjpeg_avi.JpegClipWriter - parallel and non-blocking."""

    def __init__(self, outdir: Path, prefix: str, fps: float, width: int, height: int,
                 quality: int, workers: int):
        super().__init__(outdir, prefix, "avi", JPEG_FORMAT)
        self.writer = JpegClipWriter(self.video_path, width, height, fps,
                                     quality=quality, workers=workers)

    def write(self, ts: float, bgr) -> None:
        if self.first_ts is None:
            self.first_ts = ts
        self.writer.submit(ts, bgr)

    def take_encode_stats(self):
        return self.writer.take_encode_stats()

    def close(self, reason: str) -> dict:
        end_time = time.time()
        stats = self.writer.finish()
        return self._meta(end_time, stats["frames_written"], reason, stats)


def _open_clip(outdir, prefix, fmt, fps, width, height, jpeg_quality, encode_threads):
    if fmt == JPEG_FORMAT:
        return _JpegClip(outdir, prefix, fps, width, height, jpeg_quality, encode_threads)
    return _VideoWriterClip(outdir, prefix, fmt, fps, width, height)


def run(
    trigger: EventTrigger,
    stop_event,
    outdir: str = "recordings_rgb_events",
    prefix: str = "rgb_event",
    fmt: str = JPEG_FORMAT,
    width: int = 640,
    height: int = 480,
    fps: float = 15.0,
    rotate_degrees: int = 0,
    pre_roll_seconds: float = 2.0,
    post_roll_seconds: float = 2.0,
    min_free_mb: float = 1000.0,
    stats_log_path: str | None = None,
    jpeg_quality: int = 75,
    encode_threads: int = 3,
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
          f"post-roll {post_roll_seconds}s, format {fmt}).")

    rotate_code = ROTATE_CODES[rotate_degrees]
    # The video needs the *post-rotation* frame size - a 90/270 rotation
    # swaps width and height, same as segment_writer.py's SegmentWriter does
    # for the thermal side.
    out_width, out_height = (height, width) if rotate_degrees in (90, 270) else (width, height)

    # Rolling buffer of the last pre_roll_seconds of frames, always kept warm
    # regardless of whether anything is currently being recorded, so a clip
    # can be backfilled with what happened just before the trigger fired.
    # Raw frames: ~6MB each at 1640x1232, so ~360MB for 2s at 30fps. Cheap in CPU
    # (no encoding until a clip opens), which is what keeps the idle load low.
    buffer = deque()  # [(timestamp, bgr_frame), ...], oldest first

    clip = None
    low_space_warned = False
    clip_retry_after = 0.0
    clip_open_failures = 0
    clips_closed = 0
    clip_frames_dropped = 0
    stats_logger = StatsLogger(stats_log_path) if stats_log_path else None
    health_monitor = HealthMonitor("rgb-event")
    captured_count = 0
    capture_time_total = 0.0
    encoded_count = 0
    encode_time_total = 0.0
    last_stats_time = time.time()

    def finish_clip(reason: str) -> None:
        nonlocal clip, encoded_count, encode_time_total, clips_closed, clip_frames_dropped
        n, seconds = clip.take_encode_stats()
        encoded_count += n
        encode_time_total += seconds
        meta = clip.close(reason)
        clips_closed += 1
        clip_frames_dropped += meta.get("frames_dropped", 0)
        clip = None

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
                        clip = _open_clip(outdir_path, prefix, fmt, fps, out_width, out_height,
                                          jpeg_quality, encode_threads)
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
                        # The backlog includes this iteration's frame (appended above).
                        for buf_ts, buf_frame in buffer:
                            clip.write(buf_ts, buf_frame)

            elif clip is not None:
                clip.write(now, bgr)
                if not active and now - last_active_time >= post_roll_seconds:
                    finish_clip("post-roll elapsed")

            if now - last_stats_time >= STATS_INTERVAL_SECONDS:
                if clip is not None:
                    n, seconds = clip.take_encode_stats()
                    encoded_count += n
                    encode_time_total += seconds
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
                            clips_closed=clips_closed,
                            clip_frames_dropped=clip_frames_dropped,
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
                    finish_clip("low disk space")
                    low_space_warned = True
    finally:
        if clip is not None:
            finish_clip("shutdown")
        if stats_logger is not None:
            try:
                stats_logger.close()
            except OSError:
                pass
        picam2.stop()
        print("[rgb-event] capture stopped, camera released.")
