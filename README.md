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
  defaults to whatever `--thermal-fps` is (12.5 by default for both - exactly
  every 2nd frame of the P1's ~25fps). The average is exact for any target up
  to the camera's rate; one that isn't a whole divisor of 25 (e.g. 10) just
  has uneven spacing - frames alternately 80 and 120ms apart - and both
  scripts print a `[note]` at startup when that applies. (An earlier version
  of the detect/publish gates restarted their clock from each frame's actual
  arrival instead of keeping a fixed schedule, which made `--detect-fps 12.5`
  run at 10.0 - and `10` at 8.3 - while recording ran at the right rate in the
  same process; `rate_limiter.py` now does all of them. Overnight runs are
  how this showed up: recording at 12.49fps, detection at 10.0.)
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
    --rgb-outdir recordings_rgb_events
```

(`--detections-log` and the stats logs below default to living inside
`--thermal-outdir`/`--rgb-outdir` - no need to name them explicitly unless
you want them somewhere else.)

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

**RAM**: the RGB pre-roll buffer is the main memory cost: raw BGR frames,
`width x height x 3 bytes x fps x pre-roll seconds` - ≈ 27MB at the 640x480 @
15fps defaults, but ≈ 360MB at 1640x1232 @ 30fps. A clip being encoded can add
up to ~90 more queued frames (~540MB) if the encoder falls behind, after which
frames are dropped rather than memory growing. Fine on a 2GB+ Pi 4, worth
checking on a 1GB one.

**CPU**: detection's own cost was already measured on real hardware during
the step-2 endurance test - see the "Real-time detection" section's stats
line and WildMice's own benchmark (0.24ms/frame on their server, 30-80x
headroom estimated on a Pi 4). Recording is comparatively cheap: the
thermal segment writer only handles 160x120 frames, and the RGB event
writer only encodes while a clip is actually open (i.e. rarely, unless the
camera is pointed at constant activity) - it is not a continuous cost like
detection is. What the idle RGB loop does cost is capturing and buffering 30
raw frames a second (the recorder as a whole sits around 0.6 of one core).

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

Both free-space checks run continuously (every ~60s, alongside the stats
line below), not only when opening a new segment/clip - an early version
only checked at those boundaries, so a card that filled up in the middle of
an hour-long thermal segment (or a long RGB clip kept open by back-to-back
triggers) wasn't caught until the next one opened, by which point the
camera's own video writer (OpenCV/FFmpeg) was already silently failing
("Failed to write frame") and - worse - a single disk-full write to
`detections_events.jsonl` or a segment's `.json`/`.csv` sidecar (plain
Python file writes, unlike the video writer, raise on `ENOSPC` rather than
warning and continuing) could crash that capture thread outright with
nothing but a traceback on a console nobody was watching. Both of those
write paths are now caught and logged instead of left to crash, and the
periodic check stops recording proactively well before actually hitting
zero bytes.

### Stats logging

On by default (`--no-stats-log` to disable): every ~60s, each subsystem
writes one JSON-lines record of what it's been doing - `--thermal-stats-log`
(captured fps, detected fps, mean detection time, free disk space on
`--thermal-outdir` - default `stats.jsonl` there) and `--rgb-stats-log`
(captured fps, mean capture time, mean video-encode time, whether a clip is
currently open, free disk space on `--rgb-outdir` - default `stats.jsonl`
there). This is the same information already printed to the console every
minute, just persisted - the point being exactly the scenario above: a
multi-day unattended run where nobody is watching the console live, and the
only way to reconstruct what happened (did fps hold up, when did the disk
start filling, was a clip open at the time, was capture or encoding the
bottleneck) is to check back afterwards. The cost is negligible (one short
line a minute per subsystem), which is why this didn't need a separate
toggleable "test mode" - it's cheap enough to simply leave on.

The capture/encode split in particular is a deliberate diagnostic: a
capture loop's overall wall-clock time naturally includes time spent
*waiting* for the next frame at the configured rate, which isn't itself a
problem - but the encode timing (`cv2.VideoWriter.write()` for RGB event
clips, `cv2.imencode()` for the MJPEG preview) is pure CPU work with no
legitimate wait in it, so `avg_encode_ms` approaching or exceeding the
per-frame budget (`1000 / fps` ms) is a direct sign that software video
encoding - not the camera/ISP hardware - is the bottleneck.

That diagnostic was added investigating why a `--rgb-width 1640
--rgb-height 1232 --rgb-fps 30` run only achieved ~17-19fps in its saved
clips, despite that sensor mode supporting up to 81fps per `sensor_modes`
(see "Setup on the Raspberry Pi"). A 21-hour test settled it: with no clip
open the RGB loop holds 29.98fps (`avg_capture_ms` ~33.3, i.e. just waiting
for the next frame, correctly); with an XVID clip open `avg_encode_ms` is
**48-69ms per frame against a 33ms budget**, and `avg_capture_ms` drops to
~7ms (frames are queued waiting for the encoder). 1000 / (7.3 + 48.3) =
18fps - exactly what was measured. So XVID at 1640x1232 tops out around
18-20fps on this Pi, whatever `--rgb-fps` says, and the thermal thread's
detection time stayed at 3.3-4.1ms alongside it (vs 2.6ms with no RGB
activity), so contention between the two is real but minor next to the
encoder's own cost. The encode time also stepped up from ~49 to ~69ms
(18 -> 13fps) about five hours into the clip, as the file's growth rate
tripled from ~2.3 to ~7GB/hour: the scene getting dark - a noisier image is
both slower to encode and much bigger. Budget RGB disk at *night* rates.

`--rgb-format avi-mjpg` (motion-JPEG through `cv2.VideoWriter`) silently
failed until recently: the saved file got the extension `.avi-mjpg`, which
OpenCV can't map to a container, so `VideoWriter` never opened and (with an
exception nothing caught) the RGB thread died on the first event - a 3-day run
produced no RGB clips at all. Fixed: it now writes a normal `.avi`, a failed
open is logged and retried instead of killing the thread, and
`field_recorder.py` exits (so systemd can restart it) if either worker thread
ends unexpectedly. Once it worked it turned out not to be faster: **~60ms per
frame** (vs 48ms for XVID by day), so ~15-16fps - ffmpeg's MJPEG encoder
behind `VideoWriter` is nothing like the 22.5ms `cv2.imencode` that the
streaming path measured at the same size.

### RGB clip encoding: why the default is `avi-jpeg`

That MJPG run also exposed a worse problem, in how *any* `VideoWriter` format
behaves at this size. The three clips it wrote had 61, 62 and 118 frames -
and the pre-roll alone is ~60 frames (2s at 30fps). Opening a clip wrote that
whole backlog synchronously, ~60 frames x 60ms = **~3.6s with capture
blocked**, so the live frames of the event were never captured, and by the
time the loop resumed the post-roll had already elapsed and the clip closed:
short events were saved as *only the pre-roll plus a frame or two*. (The same
shape showed in the earlier XVID clips: 67 frames in 3.56s.) On top of that
the writer declared 30fps while holding ~16, so playback ran ~1.8x fast.

`--rgb-format avi-jpeg` (the default; `mjpeg_avi.py`) fixes both without
needing anything that wasn't already measured on this Pi:

- each frame is encoded with `cv2.imencode` (22.5ms at 1640x1232) on a pool of
  `--rgb-encode-threads` workers (default 3; `imencode` releases the GIL), and
  one writer thread puts the JPEGs into an AVI in capture order. Capture never
  waits - submitting a frame is a queue append - so the pre-roll backlog is
  encoded while live capture carries on, and the clip really holds pre-roll,
  event and post-roll;
- timing is kept honest: every frame is placed on a fixed 1/fps grid by its
  capture timestamp. A gap is filled by repeating the previous JPEG (free), a
  second frame in an already-filled slot is skipped, so playback is real-time
  whatever the camera delivered. Both counts, plus any frames dropped because
  the backlog was full, are in the clip's `.json` sidecar;
- the AVI is a small hand-written container (one `MJPG` stream plus an `idx1`
  index) rather than OpenCV's writer, which would mean re-encoding. A clip over
  ~1.8GB rolls into `<name>_part2.avi`, `_part3.avi`, ...;
- `clip_started_at` in the sidecar is now the capture time of the earliest
  frame (pre-roll included), so `duration_seconds` is the real length of the
  footage rather than the time since the trigger.

Trade-offs, honestly: idle cost is unchanged (the pre-roll buffer is still raw
frames, ~6MB each, ~360MB for 2s at 1640x1232 - the earlier "27MB" figure was
for the 640x480 defaults), but while a clip is open the encodes cost about 0.7
of a core (30 x 22.5ms), briefly more while the backlog is encoded. File size
is **not yet measured**: JPEG-per-frame has no inter-frame compression, so
expect clips several times larger than XVID's for the same footage, scaling
strongly with `--rgb-jpeg-quality` (default 75) - check `free_disk_mb` in the
RGB stats log over a first event and set quality/`--rgb-min-free-mb`
accordingly. The picamera2 hardware H.264 encoder with its circular buffer
would be far lighter on CPU and disk (the Pi 4's encoder handles widths up to
~2048, so 1640x1232 should work, 3280x2464 not) but needs ffmpeg for a
playable container and a rewrite around picamera2's own encoder pipeline; it
remains the option to reach for if JPEG's CPU or disk cost proves a problem.
`mjpeg_avi_test.py` checks the container and the time-grid logic on the Pi
(`python3 mjpeg_avi_test.py`).

### RGB resolution vs. fps: what the Pi 4 sustains

Measured with `stream_server.py` (JPEG-encoding each frame for the MJPEG
stream, no viewer needed - the encode runs regardless), `--rgb-fps 30`:

| run | RGB captured fps | ms/capture | ms/encode (JPEG) |
|---|---|---|---|
| 1640x1232, RGB only | 29.9-30.0 | 10.4-10.7 | 22.4-22.6 |
| 1640x1232, with thermal + detection | 30.0 | 10.2 | 22.7 |
| 3280x2464, RGB only | 8.3-8.5 | 24.7-26.2 | 92.3-93.2 |

- **1640x1232 @ 30fps streams fine, with ~30% to spare**: at 30fps the loop
  has a 33ms budget, 22.5ms goes on encoding, and the remaining ~10ms shows up
  as `ms/capture` (that is just waiting for the next frame). Running the
  thermal camera and detection alongside changed nothing measurable (+0.2ms
  encode; detection 1.8-1.9ms at 12.5 detected fps - which also confirms the
  fixed-schedule rate limiter on hardware, where the old gate gave 10.0). JPEG
  encoding is ~2x cheaper than the XVID video encoding of event clips was at
  the same size (22.5 vs 48ms).
- **3280x2464 cannot reach 30fps, and mostly not because of the Pi.** The
  sensor's full-resolution mode tops out at **21.19fps** (see the
  `sensor_modes` table in "Setup on the Raspberry Pi") - that is the camera's
  CSI link, so 30fps there is impossible on any host. Only the 1640x1232
  (81fps) and 1920x1080 (47.6fps) modes can do 30fps; any size larger than
  those falls into the 21fps full-resolution mode. The Pi then limits it
  further: encode time scales linearly with pixels (4.0x the pixels, 4.1x the
  time, ~11ns/pixel), and `ms/capture` of ~25 here is real work (copying the
  24MB BGR frame out of the camera buffer), not waiting. 25 + 92ms per frame
  is the 8.5fps seen.
- **Pi 4 has no hardware path for this size.** The Pi 4's hardware H.264
  encoder stops at 1920x1080, and it has no hardware JPEG encoder at all.
  What remains is software, and it could be made faster than 8.5fps - encoding
  on several threads (OpenCV's `imencode` releases the GIL, and the Pi has four
  cores), or asking picamera2 for a `YUV420` main stream and encoding that
  directly (half the bytes, no BGR conversion) - but with the sensor's 21fps
  ceiling and the memory bandwidth, something like 12-18fps is the plausible
  best case, not 30, and it has not been tried. Each 8MP JPEG is also ~1.5-2MB,
  so even 15fps is ~25MB/s for the viewer to decode.
- **Practical choice:** use 1640x1232 for anything live (full field of view,
  2x2 binned so it is also better in low light) and treat 3280x2464 as a
  low-fps / still-image mode.

### Heat and power indicators

For runs of days or weeks, both field stats logs (`health_monitor.py`) also
record, every minute: `cpu_temp_c` (the Pi 4 starts throttling at 80C; the
"GPU" temperature `vcgencmd` shows is this same sensor), any further hwmon
temperature sensors the kernel exposes (`other_temps_c` - an SSD, PoE HAT),
fan speeds (`fans_rpm`), the CPU clock against its ceiling (`cpu_freq_mhz` /
`cpu_freq_max_mhz` - one sample is just where the frequency governor happens
to be that instant, so an idle Pi reads anything from 600 to the maximum;
only a clock *stuck* below max while `load_1m` is high means anything), the
firmware throttle flags decoded both for *now* (`throttled_now`)
and *since boot* (`throttled_since_boot`, which latches so a one-sample
glitch isn't missed) - `under_voltage` (a sagging supply, often mistaken for
heat), `freq_capped`, `throttled`, `soft_temp_limit` - plus the raw
`throttled` hex (`0x0` = clean), the 1-minute `load_1m`, and
`proc_cpu_pct` (this process's CPU, all threads; 100 = one core). The thermal
log adds `scene_mean_c` / `scene_max_c` from the camera's latest raw frame
(relative, not calibrated - a hot enclosure or fixture in view shows up
there), `active_tracks`, camera `frame_errors` and `reconnects`. A
`[warning]` line is also printed once per episode when a throttle/
under-voltage flag goes active or the SoC reaches 80C, so a console watcher
sees it start.

Not logged: the P1's own internal temperature - its frames carry two
metadata rows, but their layout isn't documented in `p3_camera.py`/
`P3_PROTOCOL.md` and a guess could log a plausible-looking wrong number. If
you know the layout (or want to reverse-engineer it), it would be the most
direct reading of the camera's own heating. Ambient/enclosure air temperature
needs an external sensor (e.g. a DS18B20 or I2C BME280 on the Pi's GPIO),
which would be a small addition to `health_monitor.py` if you add one.

### Static objects and `--stationary-timeout-seconds`

`detect_stream.py`'s `freeze_update` (keep detected pixels out of the
background, so an animal that stops doesn't fade into the scene and lose its
track) has no time limit, so anything *static* that gets detected once is
detected forever. Overnight runs saw single tracks of 16 and 52 hours at
0.0px/s, always in the bottom-right corner of the frame (a warm fixture, not
the burned-in timestamp: the detector runs on the raw camera frame, and the
timestamp/boxes are drawn onto a copy afterwards, so it never sees them).
That held the RGB trigger on - one clip grew to 35,500 seconds / ~46GB - and
slowly inflated detection time (2.6 -> ~5ms over 3 days) as stuck tracks
accumulated. `LiveDetector` now ends a track that hasn't moved for
`--stationary-timeout-seconds` (default 60, 0 disables) and copies the
current frame into the background over its box, so it stops re-triggering. The
vendored `detect_stream.py`/`track.py` are untouched.

**What "hasn't moved" means.** It does not require perfect stillness. The
check uses the track's box *centre*, not individual pixels, and the track
counts as stationary while that centre stays within a radius of about 8% of
the frame diagonal - roughly 16 px on a 160x120 frame (the constant
`STATIONARY_RADIUS_FRAC` in `live_detection.py`, not a command-line flag).
The clock restarts whenever the centre leaves that circle. So an animal that
feeds or grooms in place can be absorbed even though it is moving; it is
detected again as soon as it moves out of the circle. The radius is the lever
that matters more than the timeout: it is generous on purpose, because a
fixture's box fluctuates by a few pixels and should still be caught reliably,
so if real animals turn out to be ended early a smaller radius (4-6 px) is the
better fix than a longer timeout. Every event ended this way carries
`"ended_by": "stationary_timeout"` in `detections_events.jsonl`, so they can be
reviewed against the video: a fixture appears as one long event ending exactly
`--stationary-timeout-seconds` after its last movement, while anything that
looks like an animal means the radius is too generous.

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
- `stats_logger.py` - the shared periodic-stats JSONL logger (captured fps,
  detect time, free disk space, ...) all four capture loops use.
- `health_monitor.py` - the heat/power indicators (CPU temperature, clock,
  throttle/under-voltage flags, load) the two field loops add to their stats.
- `rate_limiter.py` - the fixed-schedule frame thinner behind the record,
  detect and publish gates (`--thermal-fps` / `--detect-fps`).

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
  buffer and event-triggered clips (pre/post-roll), driven by `EventTrigger`.
- `mjpeg_avi.py` - the default clip encoder: parallel `cv2.imencode` JPEGs
  written into an AVI on a fixed time grid (see "RGB clip encoding");
  `mjpeg_avi_test.py` checks it.
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
- An overnight test surfaced persistent, essentially-stationary tracks
  lasting *hours* (peak boxes sitting right on a frame edge, near-zero
  speed/displacement) - almost certainly a static hot spot at the frame
  border that crossed the detection threshold once and then never aged out,
  because `freeze_update` (by design) excludes currently-"detected" pixels
  from updating the background, so a false positive gets the same
  protection a genuinely paused animal does. Worth two things: checking the
  actual footage at the reported frame/box coordinates to confirm it's a
  fixture and not real activity, and noting that the *live* tracker (unlike
  the offline `detect.py`/`track.py`, which has `max_edge_frac` and
  `min_median_box_frac` specifically to reject this) has no edge-rejection
  of its own - see WildMice/thermal_detect/README.md: "it is a trigger, not
  a classifier." Masking a known-bad region, or porting a lightweight
  edge/size check into `live_detection.py`, are both options if this
  recurs.

For both:

- If `detect_stream.py`/`track.py` change again upstream, re-copy both files
  wholesale rather than patching around the vendored copies.
