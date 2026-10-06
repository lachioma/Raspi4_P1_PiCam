"""Field-deployable dual-camera recorder: continuous thermal recording,
real-time detection, and event-triggered RGB clips.

Unlike stream_server.py (built for bench testing over a direct Ethernet
link, with an MJPEG preview), this has no network/HTTP component at all -
nothing here needs a laptop watching. It's meant to run as a systemd
service, the same way p3-ir-camera's thermal-recorder.service does; see
field_mode.sh / normal_mode.sh / field-recorder.service.

What it does:
  - saves every thermal frame to segmented video files on the SD card
    (segment_writer.py - same design as p3-ir-camera/record_p1_segmented.py)
  - runs the real-time detector continuously on the thermal stream
    (live_detection.py, wrapping WildMice/thermal_detect's validated
    detect_stream.py + track.py)
  - boxes confirmed detections directly into the saved thermal video and
    logs them as JSON-lines events
  - whenever an animal is in frame, records an RGB clip covering
    --pre-roll-seconds before it appeared and --post-roll-seconds after it
    clears (rgb_event_recorder.py) - continuous RGB recording is
    deliberately not used; see the project README's disk budget for why

Example:

    python3 field_recorder.py --model p1 --thermal-rotate-degrees 180 \\
        --thermal-outdir recordings_thermal --rgb-outdir recordings_rgb_events

Arguments:

  General
  -------
  --model {p1,p3}
      Thermal camera model. Default: p1.
  --thermal-rotate-degrees {0,90,180,270}
      Clockwise rotation applied to the thermal video and the detector.
      Default: 0.
  --duration N
      Total run time in seconds. 0 (default) = run until stopped (Ctrl+C,
      SIGTERM, or disk space running low).

  Thermal recording
  ------------------
  --thermal-outdir PATH
      Directory for thermal segment files, JSON sidecars, and
      recording_log.csv. Default: recordings_thermal.
  --thermal-prefix NAME
      Filename prefix for each thermal segment. Default: thermal.
  --thermal-format {avi,avi-mjpg,mp4}
      Video container/codec, trading off file size against how much of a
      segment truncated mid-write (e.g. by a power loss) survives - see
      segment_writer.py. Default: avi (XVID).
  --thermal-fps N
      Thermal video frame rate; frames from the camera (native ~25-27fps
      for the P1, regardless of this) are thinned down to this rate. The
      average is exact for any target up to the native rate (see
      rate_limiter.py), but a frame can only be taken on a camera tick: a
      target that isn't a whole divisor of 25 (e.g. 10) gives uneven
      spacing - frames alternately 2 and 3 ticks (80/120ms) apart - and a
      [note] is printed at startup when that applies. 12.5 (the default)
      is exactly every 2nd frame, evenly spaced. Default: 12.5.
  --segment-seconds N
      Length of each thermal segment file, so a crash/power-loss only costs
      the in-progress segment. Default: 3600 (1 hour).
  --thermal-min-free-mb N
      Stop thermal recording (and the whole process) once free space on
      --thermal-outdir drops below this. Default: 500.
  --save-raw
      Also save 16-bit raw radiometric data alongside the video. Off by
      default in this dual-camera variant - see the README's disk budget.
  --raw-every-n N
      Persist only every Nth frame's raw data. No effect without
      --save-raw. Default: 1 (every frame).
  --no-timestamps
      Disable the per-frame timestamp log (_timestamps.txt) - independent
      of the burned-in video timestamp, which is always drawn.
  --temp-min-c N
      Bottom of a fixed Celsius range mapped to the video's brightness,
      replacing the camera's hardware AGC. Must be given with
      --temp-max-c. Default: unset (use AGC).
  --temp-max-c N
      Top of that fixed Celsius range. Default: unset (use AGC).
  --connect-retry-interval N
      Seconds to wait between camera connect/reconnect attempts.
      Default: 5.
  --max-reconnect-seconds N
      Give up and exit after this many seconds of failing to (re)connect,
      instead of retrying forever (0 = retry forever). Default: 300.

  RGB event recording
  --------------------
  --no-rgb
      Disable RGB event recording entirely - thermal + detection only.
  --rgb-outdir PATH
      Directory for RGB event clips and their JSON sidecars. Default:
      recordings_rgb_events.
  --rgb-prefix NAME
      Filename prefix for each RGB clip. Default: rgb_event.
  --rgb-format {avi,avi-mjpg,mp4}
      Video container/codec for RGB clips. Default: avi (XVID).
  --rgb-width N
      RGB capture width, in pixels. Default: 640.
  --rgb-height N
      RGB capture height, in pixels. Default: 480.
  --rgb-fps N
      RGB capture rate. Resolution and fps aren't independent:
      picamera2/libcamera first picks the Camera Module 2 sensor mode that
      best matches --rgb-width/--rgb-height, *then* clamps --rgb-fps to
      whatever that mode supports - resolution wins, fps is silently capped
      if it exceeds the chosen mode's ceiling. Run
      `python3 -c "from picamera2 import Picamera2; print(Picamera2().sensor_modes)"`
      on the Pi to see the exact modes/ranges available. Default: 15.
  --rgb-rotate-degrees {0,90,180,270}
      Clockwise rotation applied to the RGB clips - independent of
      --thermal-rotate-degrees, since the two cameras can be mounted at
      different angles on the same bracket. Default: 0.
  --pre-roll-seconds N
      Seconds of RGB footage kept from before the trigger fires.
      Default: 2.
  --post-roll-seconds N
      Seconds an RGB clip keeps recording after the trigger clears.
      Default: 2.
  --rgb-min-free-mb N
      Skip starting new RGB event clips (thermal recording keeps going
      regardless) once free space drops below this. Higher than
      --thermal-min-free-mb by default, so RGB backs off first and
      protects the more essential continuous thermal record. Default: 1000.

  Detection
  ---------
  --no-detect
      Disable the detector entirely. Also disables RGB event recording,
      since nothing would ever trigger it.
  --detect-fps N
      Rate the thermal stream (native ~25-27fps) is thinned down to before
      being fed to the detector - see the --thermal-fps note above about
      spacing (the same applies here). Defaults to whatever --thermal-fps
      ends up being, so the saved video and the detector see the same
      frames unless set independently.
  --stationary-timeout-seconds N
      A tracked object that hasn't moved for this long is ended and folded
      into the background, so it stops re-triggering. Without it, a
      *static* thing that gets detected once (a fixture warming in the
      sun) is detected forever, because the detector deliberately keeps
      detected pixels out of its background so a resting animal doesn't
      vanish - overnight tests saw single tracks last 16-52 hours,
      holding the RGB trigger on (one RGB clip grew to ~46GB).
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
      detections_events.jsonl inside --rgb-outdir.
  --no-stats-log
      Disable the periodic stats JSON-lines logs below. On by default -
      one short line per subsystem per minute, negligible cost.
  --thermal-stats-log PATH
      JSON-lines log of the thermal/detection stats line that's also
      printed every 60s (captured fps, detected fps, mean detect time,
      free disk space on --thermal-outdir) - this is what would have
      shown the SD card filling up during a run nobody was watching live.
      Default: stats.jsonl inside --thermal-outdir.
  --rgb-stats-log PATH
      Same idea for the RGB side (captured fps, whether a clip is
      currently open, free disk space on --rgb-outdir). Default:
      stats.jsonl inside --rgb-outdir.

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
      saved video and eligible to trigger RGB event recording. Default: 3.
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
import sys
import threading
import traceback
from pathlib import Path

import rgb_event_recorder
import thermal_field_recorder
from event_trigger import EventTrigger
from live_detection import StreamConfig, TrackConfig
from p3_camera import Model

# track.py's Config has ~18 fields, but detect_stream.py's OnlineTracker only ever reads
# these six (see live_detection.py / stream_server.py for the same note - duplicated here
# rather than imported, so this field-deployment entry point has no dependency on the
# bench-testing one).
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


def _guarded(fn, name, crashes):
    """Wrap a thread target so an unhandled exception is printed with its traceback
    and recorded in `crashes` (for the exit code), rather than only vanishing the thread."""
    def runner(**kwargs):
        try:
            fn(**kwargs)
        except Exception:
            print(f"[field-recorder] {name} crashed:")
            traceback.print_exc()
            crashes.append(name)
    return runner


def _add_config_args(group, cfg_instance, fields=None):
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
        help="Clockwise rotation applied to the thermal video and the detector. Default: 0.",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="Total run time in seconds. 0 (default) = run until stopped "
        "(Ctrl+C, SIGTERM, or disk space running low).",
    )

    thermal_group = parser.add_argument_group("thermal recording")
    thermal_group.add_argument("--thermal-outdir", default="recordings_thermal")
    thermal_group.add_argument("--thermal-prefix", default="thermal")
    thermal_group.add_argument("--thermal-format", choices=["avi", "avi-mjpg", "mp4"], default="avi")
    thermal_group.add_argument(
        "--thermal-fps", type=float, default=12.5,
        help="Thermal video frame rate; frames from the camera (native ~25-27fps for the P1, "
        "regardless of this) are thinned down to this rate. 12.5 is an exact half of the "
        "P1's nominal 25fps, so it's evenly spaced (any target's *average* is exact). "
        "Default: 12.5.",
    )
    thermal_group.add_argument(
        "--segment-seconds", type=float, default=3600.0,
        help="Length of each thermal segment file, so a crash/power-loss only costs the "
        "in-progress segment. Default: 3600 (1 hour).",
    )
    thermal_group.add_argument(
        "--thermal-min-free-mb", type=float, default=500.0,
        help="Stop thermal recording (and the whole process) once free space on "
        "--thermal-outdir drops below this. Default: 500.",
    )
    thermal_group.add_argument(
        "--save-raw", action="store_true",
        help="Also save 16-bit raw radiometric data alongside the video. Off by default in "
        "this dual-camera variant - see the README's disk budget.",
    )
    thermal_group.add_argument(
        "--raw-every-n", type=int, default=1,
        help="Persist only every Nth frame's raw data. No effect without --save-raw. "
        "Default: 1 (every frame).",
    )
    thermal_group.add_argument(
        "--no-timestamps", action="store_true",
        help="Disable the per-frame timestamp log (_timestamps.txt) - independent of the "
        "burned-in video timestamp, which is always drawn.",
    )
    thermal_group.add_argument(
        "--temp-min-c", type=float, default=None,
        help="Bottom of a fixed Celsius range mapped to the video's brightness, replacing "
        "the camera's hardware AGC. Must be given with --temp-max-c.",
    )
    thermal_group.add_argument("--temp-max-c", type=float, default=None)
    thermal_group.add_argument("--connect-retry-interval", type=float, default=5.0)
    thermal_group.add_argument("--max-reconnect-seconds", type=float, default=300.0)

    rgb_group = parser.add_argument_group("RGB event recording")
    rgb_group.add_argument(
        "--no-rgb", action="store_true",
        help="Disable RGB event recording entirely - thermal + detection only.",
    )
    rgb_group.add_argument("--rgb-outdir", default="recordings_rgb_events")
    rgb_group.add_argument("--rgb-prefix", default="rgb_event")
    rgb_group.add_argument("--rgb-format", choices=["avi", "avi-mjpg", "mp4"], default="avi")
    rgb_group.add_argument("--rgb-width", type=int, default=640)
    rgb_group.add_argument("--rgb-height", type=int, default=480)
    rgb_group.add_argument("--rgb-fps", type=float, default=15.0)
    rgb_group.add_argument(
        "--rgb-rotate-degrees", type=int, choices=[0, 90, 180, 270], default=0,
        help="Clockwise rotation applied to the RGB clips - independent of "
        "--thermal-rotate-degrees, since the two cameras can be mounted at different "
        "angles on the same bracket. Default: 0.",
    )
    rgb_group.add_argument(
        "--pre-roll-seconds", type=float, default=2.0,
        help="Seconds of RGB footage kept from before the trigger fires. Default: 2.",
    )
    rgb_group.add_argument(
        "--post-roll-seconds", type=float, default=2.0,
        help="Seconds an RGB clip keeps recording after the trigger clears. Default: 2.",
    )
    rgb_group.add_argument(
        "--rgb-min-free-mb", type=float, default=1000.0,
        help="Skip starting new RGB event clips (thermal recording keeps going regardless) "
        "once free space drops below this. Higher than --thermal-min-free-mb by default, so "
        "RGB backs off first and protects the more essential continuous thermal record. "
        "Default: 1000.",
    )

    detect_group = parser.add_argument_group("detection")
    detect_group.add_argument(
        "--no-detect", action="store_true",
        help="Disable the detector entirely. Also disables RGB event recording, since "
        "nothing would ever trigger it.",
    )
    detect_group.add_argument(
        "--detect-fps", type=float, default=None,
        help="Rate the thermal stream is thinned down to before being fed to the "
        "detector. Defaults to whatever --thermal-fps ends up being.",
    )
    detect_group.add_argument(
        "--stationary-timeout-seconds", type=float, default=60.0,
        help="End a tracked object that hasn't moved for this long and fold it into the "
        "background so it stops re-triggering (0 disables). Default: 60.",
    )
    detect_group.add_argument(
        "--detections-log", default=None,
        help="JSON-lines file finished detection events are appended to. Default: "
        "detections_events.jsonl inside --rgb-outdir.",
    )
    detect_group.add_argument(
        "--no-stats-log", action="store_true",
        help="Disable the periodic captured/detected-fps, detect-time, and free-disk-space "
        "JSON-lines logs (one per subsystem - see --thermal-stats-log/--rgb-stats-log). "
        "On by default and cheap (one line per minute); this is for when even that's unwanted.",
    )
    detect_group.add_argument(
        "--thermal-stats-log", default=None,
        help="JSON-lines file the thermal/detection stats line (also printed every 60s) is "
        "appended to. Default: stats.jsonl inside --thermal-outdir.",
    )
    detect_group.add_argument(
        "--rgb-stats-log", default=None,
        help="JSON-lines file the RGB capture stats line is appended to. Default: "
        "stats.jsonl inside --rgb-outdir.",
    )

    stream_group = parser.add_argument_group(
        "detection tuning (detect_stream.py)",
        "See live_detection.py and WildMice/thermal_detect/README.md's "
        "\"Real-time detection on the Raspberry Pi\" section.",
    )
    _add_config_args(stream_group, StreamConfig())
    track_group = parser.add_argument_group(
        "tracking tuning (track.py)",
        "Only the fields detect_stream.py's OnlineTracker actually reads.",
    )
    _add_config_args(track_group, TrackConfig(), fields=LIVE_TRACK_FIELDS)
    return parser.parse_args()


def main():
    args = parse_args()

    if args.detect_fps is None:
        args.detect_fps = args.thermal_fps
    _note_if_fps_uneven("--thermal-fps", args.thermal_fps)
    _note_if_fps_uneven("--detect-fps", args.detect_fps)

    if args.detections_log is None:
        args.detections_log = str(Path(args.rgb_outdir) / "detections_events.jsonl")
    if args.no_stats_log:
        args.thermal_stats_log = None
        args.rgb_stats_log = None
    else:
        if args.thermal_stats_log is None:
            args.thermal_stats_log = str(Path(args.thermal_outdir) / "stats.jsonl")
        if args.rgb_stats_log is None:
            args.rgb_stats_log = str(Path(args.rgb_outdir) / "stats.jsonl")

    stream_cfg = StreamConfig(**{f: getattr(args, f) for f in vars(StreamConfig())})
    track_cfg = TrackConfig(**{f: getattr(args, f) for f in LIVE_TRACK_FIELDS})

    stop_event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())

    trigger = EventTrigger()
    threads = []
    crashes = []   # names of worker threads that died from an exception (exit code 1)

    thermal_thread = threading.Thread(
        target=_guarded(thermal_field_recorder.run, "thermal-field-recorder", crashes),
        kwargs=dict(
            trigger=trigger,
            stop_event=stop_event,
            model=args.model,
            outdir=args.thermal_outdir,
            prefix=args.thermal_prefix,
            fmt=args.thermal_format,
            fps=args.thermal_fps,
            rotate_degrees=args.thermal_rotate_degrees,
            temp_min_c=args.temp_min_c,
            temp_max_c=args.temp_max_c,
            save_raw=args.save_raw,
            raw_every_n=args.raw_every_n,
            save_timestamps=not args.no_timestamps,
            min_free_mb=args.thermal_min_free_mb,
            duration=args.duration,
            segment_seconds=args.segment_seconds,
            connect_retry_interval=args.connect_retry_interval,
            max_reconnect_seconds=args.max_reconnect_seconds,
            enable_detection=not args.no_detect,
            detect_fps=args.detect_fps,
            stream_cfg=stream_cfg,
            track_cfg=track_cfg,
            detections_log_path=args.detections_log,
            stats_log_path=args.thermal_stats_log,
            stationary_timeout_s=args.stationary_timeout_seconds,
        ),
        daemon=True,
        name="thermal-field-recorder",
    )
    thermal_thread.start()
    threads.append(thermal_thread)

    if not args.no_rgb and not args.no_detect:
        rgb_thread = threading.Thread(
            target=_guarded(rgb_event_recorder.run, "rgb-event-recorder", crashes),
            kwargs=dict(
                trigger=trigger,
                stop_event=stop_event,
                outdir=args.rgb_outdir,
                prefix=args.rgb_prefix,
                fmt=args.rgb_format,
                width=args.rgb_width,
                height=args.rgb_height,
                fps=args.rgb_fps,
                rotate_degrees=args.rgb_rotate_degrees,
                pre_roll_seconds=args.pre_roll_seconds,
                post_roll_seconds=args.post_roll_seconds,
                min_free_mb=args.rgb_min_free_mb,
                stats_log_path=args.rgb_stats_log,
            ),
            daemon=True,
            name="rgb-event-recorder",
        )
        rgb_thread.start()
        threads.append(rgb_thread)
    elif not args.no_rgb and args.no_detect:
        print("[field-recorder] --no-detect set: RGB event recording disabled too "
              "(nothing would ever trigger it).")

    print("[field-recorder] running. Ctrl+C or SIGTERM to stop.")
    # Watchdog: a worker thread ending while nobody asked to stop (a crash, the camera
    # giving up, low disk space, --duration elapsing) used to leave the process idling
    # forever - an RGB thread killed by one exception at the start of an MJPG test
    # went unnoticed for three days. Shut everything down instead, so it's obvious and
    # a supervisor (systemd Restart=always, as in field-recorder.service) can restart
    # it cleanly, like record_p1_segmented.py's reconnect-then-exit design.
    while not stop_event.wait(timeout=5.0):
        ended = [t.name for t in threads if not t.is_alive()]
        if ended:
            print(f"[field-recorder] {', '.join(ended)} stopped on its own (see above for "
                  "why); shutting everything down.")
            stop_event.set()
    print("\n[field-recorder] stopping...")
    for t in threads:
        t.join(timeout=30)
    sys.exit(1 if crashes else 0)


if __name__ == "__main__":
    main()
