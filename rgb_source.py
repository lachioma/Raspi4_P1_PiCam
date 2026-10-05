"""Capture loop for the Raspberry Pi Camera Module 2 (via picamera2/libcamera),
publishing JPEG-encoded frames to a FrameBus for the MJPEG server to serve.

picamera2 must come from the system Python (apt install python3-picamera2) -
it wraps libcamera's compiled bindings and is not a normal pip package. If
running from a venv, create it with --system-site-packages so it can see
the apt-installed picamera2. See README.md.
"""

import time

import cv2

from frame_bus import FrameBus
from overlay import draw_timestamp
from stats_logger import StatsLogger

STATS_INTERVAL_SECONDS = 60.0

# cv2.rotate() codes, keyed by clockwise rotation in degrees - same convention
# as thermal_source.py's, kept local since the two cameras rotate independently
# (they can be mounted at different angles on the same bracket).
ROTATE_CODES = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


def run(
    bus: FrameBus,
    stop_event,
    width: int = 640,
    height: int = 480,
    fps: float = 15.0,
    rotate_degrees: int = 0,
    jpeg_quality: int = 85,
    show_timestamp: bool = True,
    stats_log_path: str | None = None,
):
    """Runs until stop_event is set. Intended to be the target of a daemon thread."""
    from picamera2 import Picamera2  # imported lazily so thermal-only runs don't need it

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "BGR888"},
        controls={"FrameRate": fps},
    )
    picam2.configure(config)
    picam2.start()
    print(f"[rgb] Camera Module 2 started at {width}x{height} @ {fps}fps "
          f"(rotate {rotate_degrees}deg).")

    rotate_code = ROTATE_CODES[rotate_degrees]
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    stats_logger = StatsLogger(stats_log_path) if stats_log_path else None
    captured_count = 0
    capture_time_total = 0.0
    encode_time_total = 0.0
    last_stats_time = time.time()

    try:
        while not stop_event.is_set():
            # capture_array() blocks until the next frame is available at the
            # configured FrameRate, so this loop naturally runs at ~fps with
            # no extra throttling needed (unlike the thermal source, whose
            # camera streams at its own fixed hardware rate).
            t_cap0 = time.perf_counter()
            bgr = picam2.capture_array()
            if rotate_code is not None:
                bgr = cv2.rotate(bgr, rotate_code)
            capture_time_total += time.perf_counter() - t_cap0
            now = time.time()
            captured_count += 1
            if show_timestamp:
                draw_timestamp(bgr, now)

            t_enc0 = time.perf_counter()
            ok, buf = cv2.imencode(".jpg", bgr, encode_params)
            encode_time_total += time.perf_counter() - t_enc0
            if ok:
                bus.publish(buf.tobytes())

            if now - last_stats_time >= STATS_INTERVAL_SECONDS:
                elapsed = now - last_stats_time
                captured_fps = captured_count / elapsed
                avg_capture_ms = (capture_time_total / captured_count * 1000) if captured_count else 0.0
                avg_encode_ms = (encode_time_total / captured_count * 1000) if captured_count else 0.0
                print(f"[rgb] stats: {captured_fps:.1f} captured fps, "
                      f"{avg_capture_ms:.1f} ms/capture avg, {avg_encode_ms:.1f} ms/encode avg")
                if stats_logger is not None:
                    try:
                        stats_logger.log(
                            captured_fps=round(captured_fps, 2),
                            avg_capture_ms=round(avg_capture_ms, 2),
                            avg_encode_ms=round(avg_encode_ms, 2),
                        )
                    except OSError:
                        pass
                captured_count = 0
                capture_time_total = 0.0
                encode_time_total = 0.0
                last_stats_time = now
    finally:
        if stats_logger is not None:
            try:
                stats_logger.close()
            except OSError:
                pass
        picam2.stop()
        print("[rgb] capture stopped, camera released.")
