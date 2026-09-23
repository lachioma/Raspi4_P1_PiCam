"""Acquire from the Thermal Master P1 and the Raspberry Pi Camera Module 2 at
the same time, serve both as independent MJPEG-over-HTTP streams, and run
the real-time animal detector on the thermal frames along the way.

View the result from another machine on the same link at:

    http://<pi-ip>:<port>/            (index page, both streams side by side)
    http://<pi-ip>:<port>/thermal.mjpg
    http://<pi-ip>:<port>/rgb.mjpg

Confirmed detections are boxed in the thermal stream and appended to
--detections-log as they happen (one JSON line per finished track). Console
output (visible via `journalctl` if run as a systemd service) includes a
stats line every 60s with achieved capture/detection fps and mean detection
time per frame - watch that during the multi-hour endurance test to see
whether the Pi is keeping up.

Detection itself is WildMice/thermal_detect's validated real-time detector
(detect_stream.py + track.py, vendored unmodified) - see live_detection.py
and that project's README for how it works.

Example:

    python3 stream_server.py --model p1 --rotate-degrees 0 \\
        --rgb-width 640 --rgb-height 480 --rgb-fps 15 --port 8080
"""

import argparse
import signal
import threading

import mjpeg_server
import rgb_source
import thermal_source
from frame_bus import FrameBus
from live_detection import StreamConfig, TrackConfig
from p3_camera import Model


# track.py's Config has ~18 fields, but detect_stream.py's OnlineTracker only ever reads
# these six (merge_gap, max_dist_frac, iou_weight, max_age for the Kalman association/lifecycle;
# process_var/measure_var for the filter itself). The rest - min_hits, the confidence-tier
# thresholds, min_median_box_frac - belong to track.py's own *offline* summarise()/track_video(),
# which the live pipeline never calls (per WildMice/thermal_detect/README.md: "it is a trigger,
# not a classifier"). Exposing those as flags here would silently do nothing, so only the six
# that actually affect live tracking are surfaced.
LIVE_TRACK_FIELDS = ["merge_gap", "max_dist_frac", "iou_weight", "max_age", "process_var", "measure_var"]


def _add_config_args(group, cfg_instance, fields=None):
    """Expose Config fields as CLI flags, same convention detect_stream.py/track.py themselves
    use - including their bool workaround, since argparse's type= has no native boolean parser."""
    for field, value in vars(cfg_instance).items():
        if fields is not None and field not in fields:
            continue
        flag = f"--{field.replace('_', '-')}"
        if isinstance(value, bool):
            group.add_argument(flag, type=lambda s: s.lower() != "false", default=value)
        else:
            group.add_argument(flag, type=type(value), default=value)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", choices=[Model.P1, Model.P3], default=Model.P1,
        help="Thermal camera model. Default: p1.",
    )
    parser.add_argument(
        "--rotate-degrees", type=int, choices=[0, 90, 180, 270], default=0,
        help="Clockwise rotation applied to the thermal stream. Default: 0.",
    )
    parser.add_argument(
        "--thermal-fps", type=float, default=0.0,
        help="Cap the thermal stream's publish rate (0 = publish every frame "
        "at the camera's native ~25-27fps). Default: 0.",
    )
    parser.add_argument(
        "--rgb-width", type=int, default=640, help="RGB capture width. Default: 640.",
    )
    parser.add_argument(
        "--rgb-height", type=int, default=480, help="RGB capture height. Default: 480.",
    )
    parser.add_argument(
        "--rgb-fps", type=float, default=15.0, help="RGB capture/publish rate. Default: 15.",
    )
    parser.add_argument(
        "--jpeg-quality", type=int, default=85,
        help="JPEG encode quality (1-100) for both streams. Default: 85.",
    )
    parser.add_argument(
        "--no-timestamp", action="store_true",
        help="Disable the burned-in timestamp overlay on both streams.",
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="HTTP bind address. Default: 0.0.0.0 (all interfaces).",
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="HTTP port. Default: 8080.",
    )
    parser.add_argument(
        "--no-rgb", action="store_true",
        help="Run thermal-only (skip the Camera Module 2), e.g. to test on hardware "
        "where it isn't connected yet.",
    )
    parser.add_argument(
        "--no-thermal", action="store_true",
        help="Run RGB-only (skip the P1/P3), e.g. to test the Camera Module 2 in isolation.",
    )
    parser.add_argument(
        "--no-detect", action="store_true",
        help="Disable the real-time animal detector, e.g. to measure the streaming-only "
        "baseline CPU load before comparing it against detection enabled.",
    )
    parser.add_argument(
        "--detect-fps", type=float, default=10.0,
        help="Rate the thermal stream (native ~25-27fps) is thinned down to before being fed "
        "to the detector. Default: 10.",
    )
    parser.add_argument(
        "--detections-log", default="detections_events.jsonl",
        help="JSON-lines file finished detection events are appended to. "
        "Default: detections_events.jsonl.",
    )
    stream_group = parser.add_argument_group(
        "detection tuning (detect_stream.py)",
        "See live_detection.py and WildMice/thermal_detect/README.md's "
        "\"Real-time detection on the Raspberry Pi\" section.",
    )
    _add_config_args(stream_group, StreamConfig())
    track_group = parser.add_argument_group(
        "tracking tuning (track.py)",
        "Only the fields detect_stream.py's OnlineTracker actually reads - see "
        "LIVE_TRACK_FIELDS in this file and WildMice/thermal_detect/track.py.",
    )
    _add_config_args(track_group, TrackConfig(), fields=LIVE_TRACK_FIELDS)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.no_rgb and args.no_thermal:
        raise SystemExit("--no-rgb and --no-thermal can't both be set - nothing to stream.")

    stream_cfg = StreamConfig(**{f: getattr(args, f) for f in vars(StreamConfig())})
    track_cfg = TrackConfig(**{f: getattr(args, f) for f in LIVE_TRACK_FIELDS})

    stop_event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())

    thermal_bus = FrameBus()
    rgb_bus = FrameBus()
    threads = []

    if not args.no_thermal:
        t = threading.Thread(
            target=thermal_source.run,
            kwargs=dict(
                bus=thermal_bus,
                stop_event=stop_event,
                model=args.model,
                rotate_degrees=args.rotate_degrees,
                fps_limit=args.thermal_fps,
                jpeg_quality=args.jpeg_quality,
                show_timestamp=not args.no_timestamp,
                enable_detection=not args.no_detect,
                detect_fps=args.detect_fps,
                stream_cfg=stream_cfg,
                track_cfg=track_cfg,
                detections_log_path=args.detections_log,
            ),
            daemon=True,
            name="thermal-capture",
        )
        t.start()
        threads.append(t)

    if not args.no_rgb:
        t = threading.Thread(
            target=rgb_source.run,
            kwargs=dict(
                bus=rgb_bus,
                stop_event=stop_event,
                width=args.rgb_width,
                height=args.rgb_height,
                fps=args.rgb_fps,
                jpeg_quality=args.jpeg_quality,
                show_timestamp=not args.no_timestamp,
            ),
            daemon=True,
            name="rgb-capture",
        )
        t.start()
        threads.append(t)

    httpd = mjpeg_server.serve(args.host, args.port, thermal_bus, rgb_bus)
    print(f"Serving on http://{args.host}:{args.port}/  (Ctrl+C to stop)")

    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    stop_event.wait()
    print("\nStopping...")
    httpd.shutdown()
    for t in threads:
        t.join(timeout=10)


if __name__ == "__main__":
    main()
