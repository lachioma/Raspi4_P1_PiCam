"""Thermal capture + continuous segmented recording + real-time detection,
for unattended field deployment.

Combines two things that used to be separate:

  - continuous segmented thermal recording, adapted from
    p3-ir-camera/record_p1_segmented.py (see segment_writer.py) - every
    frame from the P1 is saved regardless of whether anything was detected,
    same as that proven field-tested design.
  - the real-time detector (live_detection.py, wrapping WildMice/
    thermal_detect's validated detect_stream.py + track.py), run
    continuously on the same frame stream.

Detected animals are boxed directly into the saved thermal video (not only a
live preview nobody may be watching) and logged as JSON-lines events.
Whenever at least one track is currently reportable, the shared
EventTrigger is set active - that's what tells rgb_event_recorder.py to
start or extend a clip on the RGB camera, with a few seconds of lead-in from
its own rolling buffer.

Reconnect behavior mirrors record_p1_segmented.py: USB read errors are
tolerated up to a threshold before forcing a reconnect, and a
--max-reconnect-seconds budget stops the process (rather than retrying
forever) so a hung camera is visible instead of silently freezing the
recording. Detector state persists across a reconnect - a brief USB hiccup
shouldn't need to re-learn the scene or lose a track it was already
following.
"""

import time
from pathlib import Path

import cv2
import usb.core

from diskspace import free_space_mb
from event_trigger import EventTrigger
from live_detection import EventLogger, LiveDetector, StreamConfig, TrackConfig
from p3_camera import FrameMarkerMismatchError, Model, P3Camera, get_model_config, raw_to_celsius
from segment_writer import SegmentWriter, append_run_log
from rate_limiter import RateLimiter
from health_monitor import HealthMonitor
from stats_logger import StatsLogger

ERROR_RECONNECT_THRESHOLD = 20
STATS_INTERVAL_SECONDS = 60.0

# cv2.rotate() codes, keyed by clockwise rotation in degrees - kept local
# rather than imported from segment_writer.py, same convention thermal_
# source.py uses: the detector needs the frame in the same final
# orientation SegmentWriter itself rotates to internally, so overlay box
# coordinates line up with what actually gets drawn onto the saved video.
ROTATE_CODES = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


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


def run(
    trigger: EventTrigger,
    stop_event,
    model: str = Model.P1,
    outdir: str = "recordings_thermal",
    prefix: str = "thermal",
    fmt: str = "avi",
    fps: float = 10.0,
    rotate_degrees: int = 0,
    temp_min_c: float | None = None,
    temp_max_c: float | None = None,
    save_raw: bool = False,
    raw_every_n: int = 1,
    save_timestamps: bool = True,
    min_free_mb: float = 500.0,
    duration: float = 0.0,
    segment_seconds: float = 3600.0,
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
    """Runs until stop_event is set, duration elapses, or disk space runs low."""
    outdir_path = Path(outdir)
    outdir_path.mkdir(parents=True, exist_ok=True)
    run_log_path = outdir_path / "recording_log.csv"
    temp_range_c = (temp_min_c, temp_max_c) if temp_min_c is not None else None

    camera = _connect(model, connect_retry_interval, max_reconnect_seconds, stop_event)
    if camera is None:
        print("[thermal] could not connect; nothing recorded.")
        return

    width, height = camera.config.sensor_w, camera.config.sensor_h

    detector = None
    logger = None
    detect_gate = RateLimiter(detect_fps)
    overlay_boxes = []
    if enable_detection:
        det_w, det_h = (height, width) if rotate_degrees in (90, 270) else (width, height)
        detector = LiveDetector(det_w, det_h, stream_cfg or StreamConfig(),
                                 track_cfg or TrackConfig(), fps=detect_fps,
                                 stationary_timeout_s=stationary_timeout_s)
        if detections_log_path:
            logger = EventLogger(detections_log_path)
        print(f"[thermal] detection enabled: {det_w}x{det_h} @ up to {detect_fps}fps "
              f"-> {detections_log_path or '(not logged)'}")

    stats_logger = StatsLogger(stats_log_path) if stats_log_path else None
    health_monitor = HealthMonitor("thermal")
    log_write_failed = False  # rate-limits the "can't write detections" warning to once

    overall_start = time.time()
    consecutive_errors = 0
    record_gate = RateLimiter(fps)

    captured_count = 0
    detected_count = 0
    detect_time_total = 0.0
    frame_error_count = 0   # cumulative; lets a stall in the stats be traced to the camera
    reconnect_count = 0
    last_stats_time = time.time()

    try:
        while not stop_event.is_set():
            if duration and (time.time() - overall_start) >= duration:
                print("[thermal] requested total duration reached.")
                break
            if free_space_mb(outdir_path) < min_free_mb:
                print(f"[thermal] free space below {min_free_mb}MB; stopping recording "
                      "to avoid filling the disk.")
                break

            segment = SegmentWriter(
                outdir_path, prefix, fmt, fps, width, height,
                save_raw, save_timestamps, rotate_degrees, temp_range_c, raw_every_n,
            )
            segment.open()
            segment_deadline = time.time() + segment_seconds
            give_up = False
            low_space = False

            while (
                not stop_event.is_set()
                and time.time() < segment_deadline
                and not (duration and (time.time() - overall_start) >= duration)
            ):
                try:
                    ir_brightness, thermal_raw = camera.read_frame_both()
                except FrameMarkerMismatchError:
                    segment.marker_mismatches += 1
                    ir_brightness, thermal_raw = None, None
                except usb.core.USBError as e:
                    print(f"[thermal] USB read error: {e!r}")
                    ir_brightness, thermal_raw = None, None

                if ir_brightness is None or thermal_raw is None:
                    consecutive_errors += 1
                    frame_error_count += 1
                    if consecutive_errors >= ERROR_RECONNECT_THRESHOLD:
                        reconnect_count += 1
                        print(f"[thermal] {consecutive_errors} consecutive frame errors; "
                              "reconnecting...")
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
                            give_up = True
                            break
                    continue
                consecutive_errors = 0
                captured_count += 1
                now = time.time()

                if detector is not None and detect_gate.ready(now):
                    code = ROTATE_CODES[rotate_degrees]
                    det_frame = ir_brightness if code is None else cv2.rotate(ir_brightness, code)
                    t0 = time.perf_counter()
                    overlay_boxes, events = detector.process(det_frame)
                    detect_time_total += time.perf_counter() - t0
                    detected_count += 1
                    trigger.set_active(bool(overlay_boxes), now)
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

                if now - last_stats_time >= STATS_INTERVAL_SECONDS:
                    elapsed = now - last_stats_time
                    avg_detect_ms = (detect_time_total / detected_count * 1000) if detected_count else 0.0
                    captured_fps = captured_count / elapsed
                    detected_fps = detected_count / elapsed
                    free_mb = free_space_mb(outdir_path)
                    health = health_monitor.sample()
                    n_tracks = detector.active_track_count if detector is not None else 0
                    # What the camera is looking at, from the latest raw frame: a hot
                    # enclosure/fixture in view shows up here. Relative, not calibrated.
                    scene_c = raw_to_celsius(thermal_raw)
                    scene_mean_c = round(float(scene_c.mean()), 1)
                    scene_max_c = round(float(scene_c.max()), 1)
                    print(
                        f"[thermal] stats: {captured_fps:.1f} captured fps, "
                        f"{detected_fps:.1f} detected fps, "
                        f"{avg_detect_ms:.1f} ms/detect avg, {n_tracks} active tracks, "
                        f"{free_mb:.0f}MB free, scene {scene_mean_c}/{scene_max_c}C mean/max, "
                        f"{HealthMonitor.brief(health)}"
                    )
                    if stats_logger is not None:
                        try:
                            stats_logger.log(
                                captured_fps=round(captured_fps, 2),
                                detected_fps=round(detected_fps, 2),
                                avg_detect_ms=round(avg_detect_ms, 2),
                                active_tracks=n_tracks,
                                free_disk_mb=round(free_mb, 1),
                                scene_mean_c=scene_mean_c,
                                scene_max_c=scene_max_c,
                                frame_errors=frame_error_count,
                                reconnects=reconnect_count,
                                **health,
                            )
                        except OSError:
                            pass  # the free-space check right below will stop recording anyway
                    captured_count = 0
                    detected_count = 0
                    detect_time_total = 0.0
                    last_stats_time = now

                    if free_mb < min_free_mb:
                        print(f"[thermal] free space below {min_free_mb}MB mid-segment; "
                              "stopping recording to avoid filling the disk.")
                        low_space = True
                        break

                if record_gate.ready(now):
                    segment.write(ir_brightness, thermal_raw, overlay_boxes=overlay_boxes)

            try:
                meta = segment.close()
                append_run_log(run_log_path, meta)
            except OSError as e:
                print(f"[thermal] could not finalize segment metadata ({e!r}); continuing.")
            if give_up:
                print("[thermal] could not reconnect within --max-reconnect-seconds; stopping.")
                break
            if low_space:
                break
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
        trigger.set_active(False)
        try:
            camera.stop_streaming()
            camera.disconnect()
        except Exception:
            pass
        print("[thermal] capture stopped, camera disconnected.")
