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
time per frame - watch that during a multi-hour test to see whether the Pi
is keeping up.

Detection itself is WildMice/thermal_detect's validated real-time detector
(detect_stream.py + track.py, vendored unmodified) - see live_detection.py
and that project's README for how it works.

Example:

    python3 stream_server.py --model p1 --thermal-rotate-degrees 0 \\
        --rgb-width 640 --rgb-height 480 --rgb-fps 15 --port 8080

Arguments:

  Cameras
  -------
  --model {p1,p3}
      Thermal camera model. Default: p1.
  --thermal-rotate-degrees {0,90,180,270}
      Clockwise rotation applied to the thermal video/detector. Default: 0.
  --rgb-rotate-degrees {0,90,180,270}
      Clockwise rotation applied to the RGB stream - independent of
      --thermal-rotate-degrees, since the two cameras can be mounted at
      different angles on the same bracket. Default: 0.
  --rgb-width N
      RGB capture width, in pixels. Default: 640.
  --rgb-height N
      RGB capture height, in pixels. Default: 480.
  --rgb-fps N
      RGB capture/publish rate. Resolution and fps aren't independent:
      picamera2/libcamera first picks the Camera Module 2 sensor mode that
      best matches --rgb-width/--rgb-height, *then* clamps --rgb-fps to
      whatever that mode supports - resolution wins, fps is silently capped
      if it exceeds the chosen mode's ceiling. Default: 15.

  Streaming
  ---------
  --thermal-fps N
      Cap the thermal stream's publish rate (0 = publish every frame at the
      camera's native ~25-27fps). The average is exact for any target up
      to the native rate (see rate_limiter.py), but a frame can only be
      taken on a camera tick: a target that isn't a whole divisor of 25
      (e.g. 10) gives uneven spacing - frames alternately 2 and 3 ticks
      (80/120ms) apart - and a [note] is printed at startup when that
      applies. 12.5 (the default) is exactly every 2nd frame, evenly
      spaced. Default: 12.5.
  --jpeg-quality N
      JPEG encode quality (1-100) for both streams. Default: 85.
  --no-timestamp
      Disable the burned-in timestamp overlay on both streams.
  --host ADDRESS
      HTTP bind address. Default: 0.0.0.0 (all interfaces).
  --port N
      HTTP port. Default: 8080.
  --no-rgb
      Run thermal-only (skip the Camera Module 2), e.g. to test on hardware
      where it isn't connected yet.
  --no-thermal
      Run RGB-only (skip the P1/P3), e.g. to test the Camera Module 2 in
      isolation.

  Detection
  ---------
  --no-detect
      Disable the real-time animal detector, e.g. to measure the
      streaming-only baseline CPU load before comparing it against
      detection enabled.
  --detect-fps N
      Rate the thermal stream (native ~25-27fps) is thinned down to before
      being fed to the detector - see the --thermal-fps note above about
      spacing (the same applies here). Defaults to whatever --thermal-fps
      ends up being, so the two see the same frames unless set
      independently.
  --stationary-timeout-seconds N
      A tracked object that hasn't moved for this long is ended and folded
      into the background, so it stops re-triggering. Without it, a
      *static* thing that gets detected once (a fixture warming in the
      sun) is detected forever, because the detector deliberately keeps
      detected pixels out of its background so a resting animal doesn't
      vanish - overnight tests saw single tracks last 16-52 hours.
      "Hasn't moved" does not mean perfectly still: the check uses the
      track's box centre, not individual pixels, and the track counts as
      stationary while that centre stays within a radius of about 8% of
      the frame diagonal (roughly 16 px on a 160x120 frame; the constant
      STATIONARY_RADIUS_FRAC in live_detection.py). The clock restarts
      whenever the centre leaves that circle. So an animal feeding or
      grooming in place can be absorbed even though it is moving; it is
      detected again as soon as it moves out of the circle. Each event
      ended this way carries "ended_by": "stationary_timeout" in the
      detections log, to review against the video. 0 disables.
      Default: 60.
  --detections-log PATH
      JSON-lines file finished detection events are appended to. Default:
      detections_events.jsonl.
  --no-stats-log
      Disable the periodic stats JSON-lines logs below. On by default -
      one short line per camera per minute, negligible cost.
  --thermal-stats-log PATH
      JSON-lines log of the thermal/detection stats line that's also
      printed every 60s (captured fps, detected fps, mean detect time).
      Default: thermal_stats.jsonl.
  --rgb-stats-log PATH
      Same idea for the RGB side (captured fps). Default: rgb_stats.jsonl.

  Detection tuning (detect_stream.py's StreamConfig - see live_detection.py
  and WildMice/thermal_detect/README.md's "Real-time detection on the
  Raspberry Pi" section for the full rationale behind each one)
  -----------------------------------------------------------------------
  --bg-alpha N
      Per-frame EMA weight of the newest frame in the background model.
      0.02 at 10fps gives a ~5s time constant; scales inversely with
      --detect-fps. Default: 0.02.
  --noise-alpha N
      Same idea, for the per-pixel noise-scale EMA - deliberately slower
      than --bg-alpha so the noise floor doesn't chase a single event.
      Default: 0.01.
  --freeze-update {true,false}
      Don't fold currently-detected pixels into the background/noise EMAs,
      or an animal that stops moving slowly melts into its own background
      and the track dies. Default: true.
  --warmup N
      Frames to seed the background model before any detection is
      reported. Default: 50.
  --min-delta N
      Minimum absolute intensity rise (0-255) over the background required
      to call a pixel "hot". Default: 22.0.
  --sigma-k N
      The detection threshold is max(--min-delta, --sigma-k x per-pixel
      noise scale) - this is what makes daytime clutter (sun flecks,
      wind-shaken vegetation) usable: it demands a stronger signal wherever
      the scene already flickers. Default: 6.0.
  --sigma-cap N
      Ceiling on the per-pixel noise scale, so a pixel an animal happens to
      occupy cannot desensitise itself. Default: 18.0.
  --min-mean-delta N
      Minimum mean intensity rise inside a candidate blob. Default: 25.0.
  --min-area-frac N
      Smallest blob size worth considering, as a fraction of frame area (so
      it survives other resolutions) - 160x120 -> ~4px at the default.
      Default: 0.0002.
  --max-area-frac N
      Largest blob size worth considering; bigger is a lighting change, not
      an animal. Default: 0.06.
  --morph-kernel N
      Size of the morphological open/close kernel used to clean up the
      threshold mask. Default: 3.
  --max-blobs N
      Frames busier than this many simultaneous blobs are treated as a
      noise burst (e.g. a sudden AGC-wide brightness shift) and discarded
      outright. Default: 12.
  --min-track-frames N
      Hits a track needs before it's considered "reportable" - drawn on the
      stream and eligible to trigger anything downstream. Default: 3.
  --min-event-delta N
      Drop a finished track from --detections-log if its peak intensity
      rise never reached this. Default: 35.0.

  Tracking tuning (track.py's Kalman tracker - only the six fields
  detect_stream.py's OnlineTracker actually reads; see LIVE_TRACK_FIELDS in
  this file for why the rest of track.py's Config isn't exposed here)
  -----------------------------------------------------------------------
  --merge-gap N
      Pixel gap within which separate blobs (e.g. a head blob and a body
      blob from the same animal) are merged into one detection before
      tracking. Default: 3.0.
  --max-dist-frac N
      Gating distance for matching a detection to a predicted track
      position, as a fraction of the frame diagonal. Default: 0.16.
  --iou-weight N
      How much bounding-box overlap counts against centroid distance in the
      association cost. Default: 0.5.
  --max-age N
      Frames a track keeps predicting through a miss (e.g. the animal
      pauses or is briefly occluded) before being finalized - 12 frames is
      ~1.2s at 10fps. Default: 12.
  --process-var N
      Process noise (px) for the constant-velocity Kalman filter. Default:
      4.0.
  --measure-var N
      Measurement noise (px) for the same filter. Default: 6.0.
"""

import argparse
import math
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

# The P1's documented native rate ("~25-27fps") - used only to flag when a requested
# --thermal-fps/--detect-fps won't give evenly spaced frames. A nominal assumption,
# not a measurement of the connected camera (see the stats line for the real one).
NOMINAL_NATIVE_FPS = 25.0


def _note_if_fps_uneven(flag: str, target_fps: float, native_fps: float = NOMINAL_NATIVE_FPS) -> None:
    """The average rate of --thermal-fps/--detect-fps is exact for any target up to the
    camera's rate (rate_limiter.py keeps a fixed schedule), but a frame can only be
    taken on a capture tick: if the target isn't a whole divisor of the native rate
    (e.g. 10 from 25fps) the spacing alternates between two tick multiples (80 and
    120ms) instead of being constant. Worth knowing for the saved video's timing,
    so say so up front.
    """
    if not (0 < target_fps < native_fps):
        return  # 0 (unthrottled) or >= native rate: nothing to thin
    ticks = native_fps / target_fps
    if abs(ticks - round(ticks)) > 1e-6:
        lo, hi = math.floor(ticks), math.ceil(ticks)
        print(
            f"[note] {flag} {target_fps:g} isn't a whole divisor of the P1's assumed "
            f"~{native_fps:g}fps: the average will be {target_fps:g}fps, but frames will be "
            f"spaced unevenly ({lo} or {hi} camera frames apart, ~{lo * 1000 / native_fps:.0f} "
            f"or ~{hi * 1000 / native_fps:.0f}ms). Whole divisors ({native_fps / 2:g}, "
            f"{native_fps / 3:.2f}, {native_fps / 4:g}, {native_fps / 5:g}, ...) are evenly spaced."
        )


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
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--model", choices=[Model.P1, Model.P3], default=Model.P1,
        help="Thermal camera model. Default: p1.",
    )
    parser.add_argument(
        "--thermal-rotate-degrees", type=int, choices=[0, 90, 180, 270], default=0,
        help="Clockwise rotation applied to the thermal stream. Default: 0.",
    )
    parser.add_argument(
        "--thermal-fps", type=float, default=12.5,
        help="Cap the thermal stream's publish rate (0 = publish every frame "
        "at the camera's native ~25-27fps). The average is exact for any target; 12.5 is "
        "exactly every 2nd frame of the P1's nominal 25fps, so it's also evenly spaced. "
        "Default: 12.5.",
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
        "--rgb-rotate-degrees", type=int, choices=[0, 90, 180, 270], default=0,
        help="Clockwise rotation applied to the RGB stream - independent of "
        "--thermal-rotate-degrees, since the two cameras can be mounted at different "
        "angles on the same bracket. Default: 0.",
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
        "--detect-fps", type=float, default=None,
        help="Rate the thermal stream (native ~25-27fps) is thinned down to before being fed "
        "to the detector. Defaults to whatever --thermal-fps ends up being, so the "
        "stream and the detector see the same frames unless set explicitly.",
    )
    parser.add_argument(
        "--stationary-timeout-seconds", type=float, default=60.0,
        help="End a tracked object that hasn't moved for this long and fold it into the "
        "background so it stops re-triggering (0 disables). Default: 60.",
    )
    parser.add_argument(
        "--detections-log", default="detections_events.jsonl",
        help="JSON-lines file finished detection events are appended to. "
        "Default: detections_events.jsonl.",
    )
    parser.add_argument(
        "--no-stats-log", action="store_true",
        help="Disable the periodic captured/detected-fps and detect-time JSON-lines logs "
        "(one per camera - see --thermal-stats-log/--rgb-stats-log). On by default and "
        "cheap (one line per minute); this is for when even that's unwanted.",
    )
    parser.add_argument(
        "--thermal-stats-log", default="thermal_stats.jsonl",
        help="JSON-lines file the thermal/detection stats line (also printed every 60s) is "
        "appended to. Default: thermal_stats.jsonl.",
    )
    parser.add_argument(
        "--rgb-stats-log", default="rgb_stats.jsonl",
        help="JSON-lines file the RGB capture stats line is appended to. "
        "Default: rgb_stats.jsonl.",
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

    if args.detect_fps is None:
        args.detect_fps = args.thermal_fps
    _note_if_fps_uneven("--thermal-fps", args.thermal_fps)
    _note_if_fps_uneven("--detect-fps", args.detect_fps)

    if args.no_stats_log:
        args.thermal_stats_log = None
        args.rgb_stats_log = None

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
                rotate_degrees=args.thermal_rotate_degrees,
                fps_limit=args.thermal_fps,
                jpeg_quality=args.jpeg_quality,
                show_timestamp=not args.no_timestamp,
                enable_detection=not args.no_detect,
                detect_fps=args.detect_fps,
                stream_cfg=stream_cfg,
                track_cfg=track_cfg,
                detections_log_path=args.detections_log,
                stats_log_path=args.thermal_stats_log,
                stationary_timeout_s=args.stationary_timeout_seconds,
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
                rotate_degrees=args.rgb_rotate_degrees,
                jpeg_quality=args.jpeg_quality,
                show_timestamp=not args.no_timestamp,
                stats_log_path=args.rgb_stats_log,
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
