"""Capture loop for the Thermal Master P1/P3, publishing JPEG-encoded frames
to a FrameBus for the MJPEG server to serve, and - if enabled - running the
real-time animal detector (see live_detection.py) on every frame along the
way.

Reconnect behavior mirrors the existing p3-ir-camera/record_p1_segmented.py:
the camera streams at its own fixed hardware rate regardless of --fps, USB
read errors are tolerated up to a threshold before forcing a reconnect, and
a --max-reconnect-seconds budget stops the thread (rather than retrying
forever) so a hung camera is visible instead of silently freezing the stream.
Detector state (background model, active tracks) persists across a
reconnect - a brief USB hiccup shouldn't need to re-learn the scene or lose
a track it was already following.
"""

import time

import cv2
import usb.core

from frame_bus import FrameBus
from live_detection import EventLogger, LiveDetector, StreamConfig, TrackConfig
from overlay import draw_timestamp
from p3_camera import FrameMarkerMismatchError, Model, P3Camera, get_model_config
from rate_limiter import RateLimiter
from stats_logger import StatsLogger

ROTATE_CODES = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}

ERROR_RECONNECT_THRESHOLD = 20
STATS_INTERVAL_SECONDS = 60.0
# How long a detection overlay is allowed to linger on the stream after the
# detector last ran, before being treated as stale (e.g. detection throttled
# well below the publish rate). Not needed to clear a lost track - the
# detector's own overlay_boxes already excludes those every call.
_OVERLAY_MAX_AGE_SECONDS = 1.0
_BOX_COLOR = (0, 255, 0)


def _connect(model: str, retry_interval: float, max_reconnect_seconds: float, stop_event):
    attempt = 0
    deadline = time.time() + max_reconnect_seconds if max_reconnect_seconds > 0 else None
    while not stop_event.is_set():
        if deadline is not None and time.time() >= deadline:
            print(f"[thermal] giving up after {max_reconnect_seconds}s without a camera.")
            return None
        attempt += 1
        try:
            camera = P3Camera(config=get_model_config(model))
            camera.connect()
            name, version = camera.init()
            camera.start_streaming()
            print(f"[thermal] connected to {name} (firmware {version}).")
            return camera
        except Exception as e:
            print(f"[thermal] connect attempt {attempt} failed ({e!r}); retrying in {retry_interval}s")
            stop_event.wait(retry_interval)
    return None


def _draw_detections(bgr_frame, overlay_boxes):
    for x0, y0, x1, y1, track_id in overlay_boxes:
        cv2.rectangle(bgr_frame, (int(x0), int(y0)), (int(x1), int(y1)), _BOX_COLOR, 1)
        label = f"#{track_id}"
        y_text = y0 - 3 if y0 > 8 else y1 + 9
        cv2.putText(bgr_frame, label, (max(int(x0) - 1, 0), int(y_text)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, _BOX_COLOR, 1, cv2.LINE_AA)


def run(
    bus: FrameBus,
    stop_event,
    model: str = Model.P1,
    rotate_degrees: int = 0,
    fps_limit: float = 0.0,
    jpeg_quality: int = 85,
    show_timestamp: bool = True,
    connect_retry_interval: float = 5.0,
    max_reconnect_seconds: float = 300.0,
    enable_detection: bool = True,
    detect_fps: float = 10.0,
    stream_cfg: StreamConfig | None = None,
    track_cfg: TrackConfig | None = None,
    detections_log_path: str | None = "detections_events.jsonl",
    stats_log_path: str | None = None,
    stationary_timeout_s: float = 60.0,
):
    """Runs until stop_event is set. Intended to be the target of a daemon thread."""
    camera = _connect(model, connect_retry_interval, max_reconnect_seconds, stop_event)
    if camera is None:
        print("[thermal] could not connect; thermal stream will stay empty.")
        return

    detector = None
    logger = None
    detect_gate = RateLimiter(detect_fps)
    overlay_boxes = []
    overlay_timestamp = 0.0
    if enable_detection:
        raw_h, raw_w = camera.config.sensor_h, camera.config.sensor_w
        det_w, det_h = (raw_h, raw_w) if rotate_degrees in (90, 270) else (raw_w, raw_h)
        detector = LiveDetector(det_w, det_h, stream_cfg or StreamConfig(),
                                 track_cfg or TrackConfig(), fps=detect_fps,
                                 stationary_timeout_s=stationary_timeout_s)
        if detections_log_path:
            logger = EventLogger(detections_log_path)
        print(f"[thermal] detection enabled: {det_w}x{det_h} @ up to {detect_fps}fps "
              f"-> {detections_log_path or '(not logged)'}")

    stats_logger = StatsLogger(stats_log_path) if stats_log_path else None
    log_write_failed = False  # rate-limits the "can't write detections" warning to once

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
    publish_gate = RateLimiter(fps_limit)
    consecutive_errors = 0

    captured_count = 0
    detected_count = 0
    detect_time_total = 0.0
    last_stats_time = time.time()

    try:
        while not stop_event.is_set():
            try:
                ir_brightness, _thermal_raw = camera.read_frame_both()
            except FrameMarkerMismatchError:
                ir_brightness = None
            except usb.core.USBError as e:
                print(f"[thermal] USB read error: {e!r}")
                ir_brightness = None

            if ir_brightness is None:
                consecutive_errors += 1
                if consecutive_errors >= ERROR_RECONNECT_THRESHOLD:
                    print(f"[thermal] {consecutive_errors} consecutive frame errors; reconnecting...")
                    try:
                        camera.stop_streaming()
                        camera.disconnect()
                    except Exception:
                        pass
                    camera = _connect(
                        model, connect_retry_interval, max_reconnect_seconds, stop_event
                    )
                    consecutive_errors = 0
                    if camera is None:
                        print("[thermal] could not reconnect; stopping thermal capture.")
                        return
                continue
            consecutive_errors = 0
            captured_count += 1

            now = time.time()
            code = ROTATE_CODES[rotate_degrees]
            frame = ir_brightness if code is None else cv2.rotate(ir_brightness, code)

            if detector is not None and detect_gate.ready(now):
                t0 = time.perf_counter()
                overlay_boxes, events = detector.process(frame)
                detect_time_total += time.perf_counter() - t0
                detected_count += 1
                overlay_timestamp = now
                if logger is not None:
                    for ev in events:
                        try:
                            logger.log(ev, now)
                        except OSError as e:
                            if not log_write_failed:
                                print(f"[thermal] could not write to detections log "
                                      f"({e!r}); further failures won't be reported "
                                      "again until this one clears.")
                                log_write_failed = True
                        else:
                            log_write_failed = False

            if now - overlay_timestamp > _OVERLAY_MAX_AGE_SECONDS:
                overlay_boxes = []

            if now - last_stats_time >= STATS_INTERVAL_SECONDS:
                elapsed = now - last_stats_time
                avg_detect_ms = (detect_time_total / detected_count * 1000) if detected_count else 0.0
                captured_fps = captured_count / elapsed
                detected_fps = detected_count / elapsed
                print(
                    f"[thermal] stats: {captured_fps:.1f} captured fps, "
                    f"{detected_fps:.1f} detected fps, "
                    f"{avg_detect_ms:.1f} ms/detect avg"
                )
                if stats_logger is not None:
                    try:
                        stats_logger.log(
                            captured_fps=round(captured_fps, 2),
                            detected_fps=round(detected_fps, 2),
                            avg_detect_ms=round(avg_detect_ms, 2),
                        )
                    except OSError:
                        pass
                captured_count = 0
                detected_count = 0
                detect_time_total = 0.0
                last_stats_time = now

            if not publish_gate.ready(now):
                continue

            bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            if overlay_boxes:
                _draw_detections(bgr, overlay_boxes)
            if show_timestamp:
                draw_timestamp(bgr, now)

            ok, buf = cv2.imencode(".jpg", bgr, encode_params)
            if ok:
                bus.publish(buf.tobytes())
    finally:
        if detector is not None and logger is not None:
            for ev in detector.flush():
                try:
                    logger.log(ev, time.time())
                except OSError as e:
                    print(f"[thermal] could not write final detections on shutdown ({e!r}).")
        if logger is not None:
            try:
                logger.close()
            except OSError:
                pass
        if stats_logger is not None:
            try:
                stats_logger.close()
            except OSError:
                pass
        try:
            camera.stop_streaming()
            camera.disconnect()
        except Exception:
            pass
        print("[thermal] capture stopped, camera disconnected.")
