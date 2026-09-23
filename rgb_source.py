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


def run(
    bus: FrameBus,
    stop_event,
    width: int = 640,
    height: int = 480,
    fps: float = 15.0,
    jpeg_quality: int = 85,
    show_timestamp: bool = True,
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
    print(f"[rgb] Camera Module 2 started at {width}x{height} @ {fps}fps.")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    try:
        while not stop_event.is_set():
            # capture_array() blocks until the next frame is available at the
            # configured FrameRate, so this loop naturally runs at ~fps with
            # no extra throttling needed (unlike the thermal source, whose
            # camera streams at its own fixed hardware rate).
            bgr = picam2.capture_array()
            now = time.time()
            if show_timestamp:
                draw_timestamp(bgr, now)

            ok, buf = cv2.imencode(".jpg", bgr, encode_params)
            if ok:
                bus.publish(buf.tobytes())
    finally:
        picam2.stop()
        print("[rgb] capture stopped, camera released.")
