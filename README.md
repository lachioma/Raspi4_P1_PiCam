# Raspi4 P1 + Pi Camera V2

Upgraded thermal + RGB camera rig for animal detection in the field, built on
top of [p3-ir-camera](../../ClaudeAI/p3-ir-camera). Target hardware:
Raspberry Pi 4, Thermal Master P1 (USB), Raspberry Pi Camera Module 2 (CSI).

**Step 1: dual-camera acquisition + streaming both video feeds over
Ethernet.** Done - see "Running (bench testing)" below.

**Step 2: real-time on-board animal detection**, using
[WildMice/thermal_detect](../WildMice/thermal_detect)'s own validated
real-time detector (`detect_stream.py` + `track.py`, vendored unmodified -
not `p3-ir-camera/animal_detector.py`'s earlier sketch, and not this
project's own first-pass port, which that project's causal pipeline has
since superseded). Confirmed running alongside both camera streams on the
Pi 4 - see "Real-time detection" below.

**Step 3 (this stage): field deployment**, via a second entry point,
`field_recorder.py` (new files, independent of `stream_server.py` above -
see "Field deployment" below):

- continuously records the thermal stream to the SD card (segmented, like
  `p3-ir-camera/record_p1_segmented.py`)
- runs the detector continuously and boxes confirmed detections directly
  into the saved thermal video
- when the detector flags an animal in frame, records an RGB clip covering
  a few seconds before it appeared and a few seconds after it clears -
  *not* continuous RGB recording, which the disk budget below rules out
- deploys with `field_mode.sh`/`normal_mode.sh`, the same field-vs-desk
  toggle `p3-ir-camera` uses

`stream_server.py` (bench testing, live MJPEG preview over Ethernet) and
`field_recorder.py` (unattended field deployment, no network) are separate
entry points sharing the same detection code - use whichever fits what
you're doing right now; they don't run at the same time on one Pi since
both want the P1 and the Camera Module 2 exclusively.

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


To see the exact modes/ranges available (resolution, frame rate):

```bash
python3 -c "from picamera2 import Picamera2; print(Picamera2().sensor_modes)"
```

Try this for a more readable output format:

```
LIBCAMERA_LOG_LEVELS=*:ERROR python3 << 'EOF'
from picamera2 import Picamera2

for m in Picamera2().sensor_modes:
    size = f'{m["size"][0]}x{m["size"][1]}'
    print(f'{size:<12} {m["fps"]:>7.2f} fps  {m["bit_depth"]:>2}-bit  {m["format"]}')
EOF
```

For Pi camera module 2, this should give you the following:

```
640x480       200.16 fps  10-bit  SRGGB10_CSI2P
1640x1232      81.07 fps  10-bit  SRGGB10_CSI2P
1920x1080      47.57 fps  10-bit  SRGGB10_CSI2P
3280x2464      21.19 fps  10-bit  SRGGB10_CSI2P
640x480       200.16 fps   8-bit  SRGGB8
1640x1232      81.07 fps   8-bit  SRGGB8
1920x1080      47.57 fps   8-bit  SRGGB8
3280x2464      21.19 fps   8-bit  SRGGB8
```

Each size is really 4 combinations (8-bit or 10-bit RAW, same size/fps/FOV/binning
either way - the bit depth only affects tonal precision of the raw sensor data
feeding the ISP, which is otherwise irrelevant here since this project only ever
consumes picamera2's already-ISP-processed `BGR888` output, never the raw stream
directly). Binning and field of view aren't reported directly by `sensor_modes`,
but are derivable from comparing each mode's `crop_limits` (the sensor-pixel
window read out, before scaling to `size`) against `size` itself and against the
full sensor (3280x2464):

| size | crop window | binning | field of view | max fps |
|---|---|---|---|---|
| 640x480 | 1280x960 | 2x2 | cropped (1280x960 of 3280x2464) | 200.16 |
| 1640x1232 | 3280x2464 | 2x2 | full sensor | 81.07 |
| 1920x1080 | 1920x1080 | none | cropped (1920x1080 of 3280x2464) | 47.57 |
| 3280x2464 | 3280x2464 | none | full sensor | 21.19 |

Picking a mode is a straight resolution/fps/FOV tradeoff: only 1640x1232 and
3280x2464 use the whole sensor - 640x480 and 1920x1080 both narrow the field of
view to get there, not just resolution. `--rgb-width`/`--rgb-height` picks the
mode (whichever one best matches your requested size), and `--rgb-fps` is then
clamped to that mode's own ceiling - see "RGB event recording"'s `--rgb-fps`
entry in `field_recorder.py`'s docstring (and `stream_server.py`'s) for how that
clamping works. There is no way to request an arbitrary binning factor (e.g.
4x4) - these four modes are the complete, fixed set the Raspberry Pi kernel
driver exposes for this sensor; a similar full-FOV, lower-resolution result can
be approximated by taking the 1640x1232 mode and downscaling further in
software, though that trades away the noise benefit of binning done on-sensor
before quantization.


### USB permissions for the P1

Same udev rule as the original project - see its README - so the camera is
readable without root:

```bash
sudo tee /etc/udev/rules.d/99-p3-ir.rules << EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="3474", ATTR{idProduct}=="45c2", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Running (bench testing)

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

- `--thermal-rotate-degrees {0,90,180,270}` - orient the thermal image to
  match how the P1 is mounted. `--rgb-rotate-degrees` does the same for the RGB
  stream, independently (the two cameras can be mounted at different angles
  on the same bracket).
- `--rgb-width`/`--rgb-height`/`--rgb-fps` - RGB capture resolution/rate
  (default 640x480 @ 15fps; the Camera Module 2 supports much higher, but
  start modest until we know what the direct Ethernet link and the Pi 4's
  CPU can comfortably sustain alongside the thermal stream).
- `--thermal-fps` - cap the thermal publish rate. Default: 12.5, an exact
  half of the P1's assumed 25fps native rate (0 publishes every native
  frame instead). `--detect-fps` defaults to match this - see "Real-time
  detection" below for what happens with a value that doesn't divide the
  native rate evenly.
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
- **detected fps** - should track `--detect-fps` closely; if it's
  meaningfully lower, detection itself is the bottleneck. `--detect-fps`
  defaults to whatever `--thermal-fps` is (12.5 by default for both - an
  exact half of the P1's assumed 25fps native rate, so it's delivered
  exactly). A target that isn't an exact sub-multiple of the native rate
  gets silently rounded down to the nearest one actually achievable (e.g.
  10 becomes a steady 8.33fps, not 10.0) - both scripts print a `[warning]`
  at startup when this would happen, rather than leaving it to be
  discovered here.
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

**Note:** `--bg-alpha`/`--noise-alpha` are per-frame EMA weights originally
tuned by WildMice assuming a 10fps detect rate (their stated time constants,
~5s/10s, are at that rate - see `detect_stream.py`'s `StreamConfig`
docstring); at the current 12.5fps default they run slightly faster than
that (proportionally), which is unlikely to matter in practice but is worth
knowing if tuning these further.

## Field deployment: `field_recorder.py`

A separate entry point from `stream_server.py` - no HTTP/MJPEG, no network
dependency at all, meant to run unattended for days as a systemd service.

```bash
python3 field_recorder.py \
    --thermal-rotate-degrees 180 \
    --thermal-outdir recordings_thermal \
    --rgb-outdir recordings_rgb_events \
    --detections-log detections_events.jsonl
```

### How the RGB trigger works

The thermal side (`thermal_field_recorder.py`) runs the same detector as
`stream_server.py` continuously, and on every detection cycle tells a shared
`EventTrigger` (`event_trigger.py`) whether at least one confirmed track is
currently in frame. The RGB side (`rgb_event_recorder.py`) always captures
from the Camera Module 2, keeping a rolling buffer of the last
`--pre-roll-seconds` of frames, but only *writes* a clip while the trigger
is active - opening a new one by first dumping that buffer (the pre-roll),
then continuing to record until `--post-roll-seconds` after the trigger
clears. A second animal appearing before the post-roll timer elapses just
extends the same clip rather than starting a new one, so a bust of activity
becomes one continuous recording instead of several clipped fragments.

The two sides only share that one boolean-plus-timestamp signal - not
discrete "event N started/stopped" messages - deliberately: the two cameras
run at different, independently-varying frame rates and can each stall or
reconnect on their own, so coupling them through a message queue would need
its own retry/ordering logic that a simple shared state doesn't.

Each RGB clip gets a `.json` sidecar (start/end time, frame count, why it
closed). To see *why* a given clip was triggered, cross-reference its
`clip_started_at`/`clip_ended_at` window against `detections_events.jsonl`'s
timestamps - the two aren't line-linked by design, since one clip can span
several finished tracks (and the software doesn't currently write the
track id(s) into the clip's sidecar; worth adding if reviewing becomes
tedious - see "What's next").

### Disk, CPU, and RAM budget

**RAM**: the RGB pre-roll buffer is the only new memory cost, and it's
small: at the defaults (640x480 BGR @ 15fps, 2s pre-roll) that's
`640 x 480 x 3 bytes x 15fps x 2s` ≈ 27MB. Trivial on any Pi 4 (1GB+).

**CPU**: detection's own cost was already measured on real hardware during
the step-2 endurance test - see the "Real-time detection" section's stats
line and WildMice's own benchmark (0.24ms/frame on their server, 30-80x
headroom estimated on a Pi 4). Recording is comparatively cheap: the
thermal segment writer only handles 160x120 frames, and the RGB event
writer only runs `cv2.VideoWriter` while a clip is actually open (i.e.
rarely, unless the camera is pointed at constant activity) - it is not a
continuous cost like detection is.

**Disk - the part that actually needs sizing.** Two independent write
streams, deliberately given different growth characteristics:

- **Thermal (continuous, unconditional)**: fixed cost, roughly proportional
  to `--thermal-fps x hours x scene compressibility` - a static night scene
  compresses far better than a busy daytime one, so treat any single number
  as an order-of-magnitude estimate, not a guarantee. **Verify it directly**:
  run `field_recorder.py` for 10-15 minutes, check the resulting segment
  file's size in `--thermal-outdir`, and scale linearly to a full day/week -
  this is more reliable than any figure quoted here, and costs nothing since
  the detector needs a real test run anyway. `--save-raw` roughly doubles
  video-only figures and adds a fixed, content-independent cost on top
  (resolution x 2 bytes x fps x seconds - see `segment_writer.py`'s
  docstring) - left off by default here specifically because this variant
  already spends part of the disk budget on RGB clips.
- **RGB (event-triggered only)**: proportional to *how much wildlife
  activity actually happens*, not to how long the recorder runs - an
  otherwise-quiet deployment costs almost nothing here regardless of
  `--rgb-width`/`--rgb-height`/`--rgb-fps`, which is the entire reason
  continuous RGB recording was ruled out for a multi-day deployment.

Two safety nets, deliberately asymmetric so a burst of RGB events can't
starve the more essential thermal record: `--thermal-min-free-mb` (default
500) stops the *entire process* if crossed, while `--rgb-min-free-mb`
(default 1000, i.e. it trips first) only skips starting *new* RGB clips -
thermal recording and detection keep running regardless. If a real
deployment's SD card is small relative to expected activity, lower
`--rgb-width`/`--rgb-height`/`--rgb-fps` or raise `--rgb-min-free-mb`
before shortening `--segment-seconds` or touching the thermal-side
settings, which are the deployment's core, always-on record.

### Field mode

Same pattern as `p3-ir-camera`'s `field_mode.sh`/`normal_mode.sh`, pointed
at a new `field-recorder.service` instead of `thermal-recorder.service`
(both scripts and both `.service` files are vendored fresh here so this
project doesn't depend on the other checkout being present on the Pi):

```bash
sudo cp field-recorder.service field-mode-net.service /etc/systemd/system/
sudo systemctl daemon-reload
./field_mode.sh     # disables Wi-Fi/BT/Ethernet, switches to console boot, starts the recorder
# ... later, to get the desktop and networking back for debugging ...
./normal_mode.sh
```

**Before a real deployment**, check `field-recorder.service`'s
`--thermal-rotate-degrees 180`: that value carried over from the
single-camera rig's known mounting, and may not hold for the new
dual-camera bracket.

## Project layout

Shared by both entry points:

- `p3_camera.py` - vendored USB driver for the P1/P3 (copied from
  p3-ir-camera; not modified here - port fixes back manually if needed).
- `detect_stream.py`, `track.py` - vendored unmodified from
  `WildMice/thermal_detect` - the validated real-time detector and its
  Kalman tracker. Update by re-copying from there, not by editing here.
- `live_detection.py` - this project's adapter: wraps the vendored code's
  `StreamDetector`/`OnlineTracker` in a `LiveDetector.process(frame)` call
  suited to a persistent camera loop, plus an append-only JSONL event log.

`stream_server.py` (bench testing, live MJPEG preview):

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

`field_recorder.py` (unattended field deployment):

- `segment_writer.py` - continuous segmented thermal video writer, adapted
  from `p3-ir-camera/record_p1_segmented.py`'s `SegmentWriter` (now takes
  width/height as parameters instead of assuming the P1, and can draw
  detection overlay boxes onto the saved video, not only a live preview).
- `event_trigger.py` - the thread-safe active/last-active-time signal
  described above.
- `rgb_event_recorder.py` - Camera Module 2 capture with a rolling pre-roll
  buffer and an event-triggered `cv2.VideoWriter` (pre/post-roll), driven
  by `EventTrigger`.
- `thermal_field_recorder.py` - thermal capture loop: continuous
  `SegmentWriter` recording, continuous detection, `EventTrigger` updates,
  and JSONL event logging, all on one frame stream.
- `field_recorder.py` - entry point; wires both capture threads and the
  shared `EventTrigger` together (no HTTP server).
- `field-recorder.service`, `field-mode-net.service`, `field_mode.sh`,
  `normal_mode.sh` - field deployment, same pattern as `p3-ir-camera`.
- `diskspace.py` - the shared free-space check both writers use.

## What's next

For `stream_server.py`:

- If the endurance test surfaces a CPU/latency problem, look at lowering
  `--detect-fps`, the RGB resolution/fps, or JPEG quality first.
- Revisit streaming efficiency (e.g. hardware H.264 via RTSP) if MJPEG
  bandwidth/CPU becomes a bottleneck once detection is also running.

For `field_recorder.py`:

- Verify the real thermal segment file size/hour on the actual deployment
  hardware (see the disk budget above) and size `--thermal-min-free-mb` /
  the SD card against it before a real multi-day deployment.
- Write the triggering track id(s) into each RGB clip's `.json` sidecar,
  instead of relying on cross-referencing timestamps against
  `detections_events.jsonl`, if reviewing footage later turns out to need it.
- Confirm `--thermal-rotate-degrees` and `--rgb-rotate-degrees` (RGB,
  independent of the thermal one since the two cameras can be mounted at
  different angles) against the actual dual-camera bracket once it's built,
  rather than assuming the old single-camera rig's value.

For both:

- If `detect_stream.py`/`track.py` change again upstream, re-copy both files
  wholesale rather than patching around the vendored copies.
