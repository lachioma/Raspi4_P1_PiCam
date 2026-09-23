# Raspi4 P1 + Pi Camera V2

Upgraded thermal + RGB camera rig for animal detection in the field, built on
top of [p3-ir-camera](../../ClaudeAI/p3-ir-camera). Target hardware:
Raspberry Pi 4, Thermal Master P1 (USB), Raspberry Pi Camera Module 2 (CSI).

**Step 1: dual-camera acquisition + streaming both video feeds over
Ethernet.** Done - see "Running" below.

**Step 2 (this stage): real-time on-board animal detection**, using
[WildMice/thermal_detect](../WildMice/thermal_detect)'s own validated
real-time detector (`detect_stream.py` + `track.py`, vendored unmodified -
not `p3-ir-camera/animal_detector.py`'s earlier sketch, and not this
project's own first-pass port, which that project's causal pipeline has
since superseded). Current goal: confirm the Raspberry Pi 4 can run
detection alongside both camera streams continuously for many hours before
spending any more effort improving the algorithm itself.

## Setup on the Raspberry Pi

Camera Module 2 needs `picamera2`, which wraps compiled `libcamera` bindings
and must come from `apt`, not `pip`:

```bash
sudo apt update
sudo apt install -y python3-picamera2 --no-install-recommends
```

Create the venv with `--system-site-packages` so it can see that apt-installed
copy, then install the rest:

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
python3 -c "from picamera2 import Picamera2; print('ok')"  # sanity check
```

### USB permissions for the P1

Same udev rule as the original project - see its README - so the camera is
readable without root:

```bash
sudo tee /etc/udev/rules.d/99-p3-ir.rules << EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="3474", ATTR{idProduct}=="45c2", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Running

```bash
python3 stream_server.py
```

Then, from the laptop (connected via the direct Ethernet link, Pi at
`192.168.50.2`), open in a browser:

```
http://192.168.50.2:8080/
```

That page shows both streams side by side. Each is also its own independent
MJPEG endpoint you can open directly, or load in VLC via *Media > Open
Network Stream*:

```
http://192.168.50.2:8080/thermal.mjpg
http://192.168.50.2:8080/rgb.mjpg
```

Useful flags (`python3 stream_server.py --help` for the full list):

- `--rotate-degrees {0,90,180,270}` - orient the thermal image to match how
  the P1 is mounted.
- `--rgb-width`/`--rgb-height`/`--rgb-fps` - RGB capture resolution/rate
  (default 640x480 @ 15fps; the Camera Module 2 supports much higher, but
  start modest until we know what the direct Ethernet link and the Pi 4's
  CPU can comfortably sustain alongside the thermal stream).
- `--thermal-fps` - cap the thermal publish rate (default: publish every
  frame at the camera's native ~25-27fps; the P1's frames are tiny, so this
  is cheap).
- `--no-rgb` / `--no-thermal` - run with only one camera, e.g. to test each
  independently before running both together.
- `--no-timestamp` - each stream has a burned-in timestamp by default, handy
  right now for confirming both feeds are live and roughly in sync.

Stop with Ctrl+C (or `systemctl stop` later, once this runs as a service) -
both camera connections are released cleanly.

## Real-time detection

Enabled by default. The detector itself is `detect_stream.py` + `track.py`,
copied unmodified from WildMice/thermal_detect - see that project's README
("Real-time detection on the Raspberry Pi") for the causal EMA
background/noise design and its validation against the offline detector (8
of 9 confirmed high-tier tracks recovered on six test clips; 0.24ms/frame
mean cost on their hardware). `live_detection.py` is this project's own thin
adapter: it feeds the vendored code frames one at a time, exposes
currently-tracked boxes for live annotation, and logs finished tracks.

Detections are boxed in the thermal stream (a track needs
`--min-track-frames` hits, default 3, before it's drawn - not every raw
candidate blob) and appended to `--detections-log` as one JSON line per
finished track (default `detections_events.jsonl`), so a multi-hour
unattended run leaves a reviewable trace even if nobody watched the browser
the whole time. Unlike the offline `detect.py`, this has no confidence
tiering - per that project's README, "it is a trigger, not a classifier" -
so expect somewhat more events than the offline `high`/`medium` tiers alone
would report.

```bash
python3 stream_server.py --detections-log detections_events.jsonl
```

Console output includes a stats line every 60s:

```
[thermal] stats: 24.8 captured fps, 10.0 detected fps, 3.2 ms/detect avg
```

- **captured fps** - the P1's actual delivered frame rate (should sit near
  its native ~25-27fps; a sustained drop means something downstream, e.g.
  USB errors, is falling behind).
- **detected fps** - should track `--detect-fps` (default 10) closely; if
  it's meaningfully lower, detection itself is the bottleneck.
- **ms/detect avg** - mean wall-clock time per detector call. WildMice's own
  benchmark measured 0.24ms/frame on their server hardware (~420x headroom
  at 10fps) and estimated 30-80x headroom on a Pi 4 - this stats line is how
  we check that estimate against the real device.

This is exactly what the current multi-hour endurance test is for: run it
unattended (ideally as a systemd service, following the pattern of
`p3-ir-camera/thermal-recorder.service`) and check back on the stats line
and `detections_events.jsonl` after several hours for drift, memory growth,
USB reconnect storms, or the detector silently falling behind.

Every field of `detect_stream.py`'s `StreamConfig` is a CLI flag (e.g.
`--min-delta`, `--bg-alpha`, `--freeze-update`), and so are the six fields of
`track.py`'s `Config` that the live tracker actually reads (`--merge-gap`,
`--max-dist-frac`, `--iou-weight`, `--max-age`, `--process-var`,
`--measure-var` - see `LIVE_TRACK_FIELDS` in `stream_server.py` for why not
all of `track.py`'s fields are exposed: most belong to its offline-only
confidence tiering, which the live pipeline never calls). `--no-detect` runs
the streaming-only baseline, useful for isolating how much CPU headroom
detection itself actually costs.

**Note:** `--bg-alpha`/`--noise-alpha` are per-frame EMA weights tuned
assuming `--detect-fps 10` (their default time constants, ~5s/10s, are
stated in `detect_stream.py`'s `StreamConfig` docstring at that rate) -
changing `--detect-fps` without adjusting them shifts how fast the
background adapts.

## Project layout

- `p3_camera.py` - vendored USB driver for the P1/P3 (copied from
  p3-ir-camera; not modified here - port fixes back manually if needed).
- `thermal_source.py` - thermal capture loop (reconnect-on-error, same
  pattern as `record_p1_segmented.py`); runs detection on every throttled
  frame and publishes annotated JPEGs to a `FrameBus`.
- `rgb_source.py` - Camera Module 2 capture loop via `picamera2`, publishes
  JPEGs to a `FrameBus`.
- `frame_bus.py` - thread-safe "latest frame wins" handoff between a capture
  thread and any number of HTTP client threads.
- `mjpeg_server.py` - plain-stdlib MJPEG-over-HTTP server, two endpoints.
- `overlay.py` - shared burned-in timestamp drawing, used by both sources.
- `stream_server.py` - entry point; wires both capture threads, the HTTP
  server, and the detector's CLI flags together.
- `detect_stream.py`, `track.py` - vendored unmodified from
  `WildMice/thermal_detect` - the validated real-time detector and its
  Kalman tracker. Update by re-copying from there, not by editing here.
- `live_detection.py` - this project's adapter: wraps the vendored code's
  `StreamDetector`/`OnlineTracker` in a `LiveDetector.process(frame)` call
  suited to a persistent camera loop, plus an append-only JSONL event log.

## What's next

- If the endurance test surfaces a CPU/latency problem, look at lowering
  `--detect-fps`, the RGB resolution/fps, or JPEG quality first.
- Mark/annotate the RGB stream too when a detection is confirmed (currently
  thermal-only).
- Revisit streaming efficiency (e.g. hardware H.264 via RTSP) if MJPEG
  bandwidth/CPU becomes a bottleneck once detection is also running.
- If `detect_stream.py`/`track.py` change again upstream, re-copy both files
  wholesale rather than patching around the vendored copies.
