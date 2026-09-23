'''
Real-time thermal detection for the Raspberry Pi recorder.

detect.py is an offline batch tool: it buys accuracy with a *non-causal* background - the median
of a 60 s block, which needs frames from the future. On the Pi there is no future, so the
background here is an exponential moving average updated once per frame, and so is the per-pixel
noise scale that sets the threshold. Everything else - what counts as a blob, how blobs are
merged and tracked - is shared with the offline code so the two stay comparable.

Two things get *simpler* on the Pi than in the offline pipeline:

  * there is no burned-in timestamp caption in a live frame; the caption is drawn when the clip
    is written. Caption masking is only needed when replaying a recorded .avi (--mask-rows).
  * the raw 16-bit sensor frames are available before the 8-bit render, so detection can run on
    them directly and skip the automatic gain control that shifts the whole 8-bit histogram
    frame to frame. Use --raw for that; thresholds are then in sensor counts, not 0-255.

Usage
    # replay a recorded clip to check behaviour against the offline result
    python3 detect_stream.py --video data/recordings_field_test/thermal_20260820_215131.avi

    # replay the 16-bit raw sidecar instead (1 fps in the current recordings)
    python3 detect_stream.py --raw data/recordings_20260909_tree_high/thermal_20260909_163752_raw.dat \\
        --raw-shape 3601,120,160 --min-delta 60 --fps 1

    # measure per-frame cost on the Pi itself
    python3 detect_stream.py --video clip.avi --bench

    # live camera: implement read() in CameraSource below for your sensor
    python3 detect_stream.py --camera

Events are written to stdout as JSON lines, one per finished track, so the caller can pipe them
anywhere. --snapshot-dir also writes the peak frame of each event as a JPEG.
'''

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

import track as offline_track   # reuse the Kalman tracker and the fragment merge


class StreamConfig:
    """Detection parameters for the causal pipeline."""

    def __init__(self, **kw):
        # background / noise, both exponential moving averages
        self.bg_alpha = 0.02         # per-frame weight of the newest frame in the background.
                                     # 0.02 at 10 fps -> ~5 s time constant, the causal analogue
                                     # of the offline 60 s median block.
        self.noise_alpha = 0.01      # slower: the noise floor should not chase a single event
        self.freeze_update = True    # do not fold detected pixels into the background, or an
                                     # animal that pauses slowly erases itself
        self.warmup = 50             # frames to seed the model before any detection is reported
        # thresholding (8-bit units by default; pass --min-delta in sensor counts for --raw)
        self.min_delta = 22.0
        self.sigma_k = 6.0
        self.sigma_cap = 18.0
        self.min_mean_delta = 25.0
        # blob geometry as a fraction of frame area, same convention as detect.py
        self.min_area_frac = 0.00020
        self.max_area_frac = 0.06
        self.morph_kernel = 3
        self.max_blobs = 12
        # event reporting
        self.min_track_frames = 3
        self.min_event_delta = 35.0
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(f'unknown config field: {k}')
            setattr(self, k, v)


# --------------------------------------------------------------------------- sources

class VideoSource:
    """Recorded 8-bit video, for replaying offline material through the online pipeline."""

    def __init__(self, path, mask_rows=0):
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise SystemExit(f'cannot open {path}')
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 10.0
        self.mask_rows = mask_rows

    def __iter__(self):
        while True:
            ok, frame = self.cap.read()
            if not ok:
                break
            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.mask_rows:
                gray = gray.copy()
                gray[-self.mask_rows:, :] = 0
            yield gray.astype(np.float32)
        self.cap.release()


class RawSource:
    """Raw 16-bit sensor frames from the recorder's *_raw.dat sidecar."""

    def __init__(self, path, shape, fps):
        self.path = Path(path)
        self.shape = shape          # (n, h, w)
        self.fps = fps

    def __iter__(self):
        n, h, w = self.shape
        with open(self.path, 'rb') as fh:
            for _ in range(n):
                buf = fh.read(h * w * 2)
                if len(buf) < h * w * 2:
                    break
                yield np.frombuffer(buf, dtype=np.uint16).reshape(h, w).astype(np.float32)


class CameraSource:
    """
    Live sensor frames.

    Fill in read() for the sensor actually fitted. For a FLIR Lepton on SPI the usual route is
    pylepton or a v4l2 node; for an MLX90640 it is the I2C driver. Yield the *raw* frame where
    the driver exposes one - see the module docstring for why.
    """

    def __init__(self, fps=10.0):
        self.fps = fps

    def read(self):
        raise NotImplementedError(
            'CameraSource.read() is a stub: return one HxW numpy frame from your sensor driver')

    def __iter__(self):
        while True:
            frame = self.read()
            if frame is None:
                break
            yield np.asarray(frame, dtype=np.float32)


# --------------------------------------------------------------------------- detector

class StreamDetector:
    """
    Causal background subtraction: one EMA for the background, one for the noise scale.

    The offline code takes a median over a block and a MAD over the same block. Neither is
    available frame-by-frame, so both become running averages. The noise EMA plays the role of
    the per-pixel MAD that makes daylight usable - pixels that flicker every frame build a high
    local threshold, while a pixel an animal crosses keeps its low one.
    """

    def __init__(self, shape, cfg):
        h, w = shape
        self.cfg = cfg
        self.background = None
        self.noise = np.full((h, w), 1.0, np.float32)
        self.frames_seen = 0
        self.min_area = max(3, int(cfg.min_area_frac * h * w))
        self.max_area = int(cfg.max_area_frac * h * w)
        k = int(cfg.morph_kernel)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)) if k > 1 else None

    def __call__(self, frame):
        """Feed one frame, get back its list of (x0, y0, x1, y1, mean_delta, area)."""
        cfg = self.cfg
        if self.background is None:
            self.background = frame.copy()
        delta = frame - self.background
        threshold = np.maximum(cfg.min_delta,
                               cfg.sigma_k * np.minimum(self.noise, cfg.sigma_cap))

        self.frames_seen += 1
        warm = self.frames_seen <= cfg.warmup
        mask = (delta > threshold).astype(np.uint8) * 255
        if self.kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # Update the model *after* thresholding, and skip pixels that are currently part of an
        # object: an animal that stops moving would otherwise melt into the background within a
        # few seconds and vanish from the track.
        hot = mask > 0
        a, na = cfg.bg_alpha, cfg.noise_alpha
        if cfg.freeze_update and not warm:
            upd = ~hot
            self.background[upd] += a * delta[upd]
            self.noise[upd] += na * (np.abs(delta[upd]) - self.noise[upd])
        else:
            self.background += a * delta
            self.noise += na * (np.abs(delta) - self.noise)

        if warm:
            return []

        n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        boxes = []
        for i in range(1, n_lab):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_area or area > self.max_area:
                continue
            mean_delta = float(delta[labels == i].mean())
            if mean_delta < cfg.min_mean_delta:
                continue
            x = int(stats[i, cv2.CC_STAT_LEFT]); y = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH]); h = int(stats[i, cv2.CC_STAT_HEIGHT])
            boxes.append((x, y, x + w, y + h, mean_delta, area))
        return [] if len(boxes) > cfg.max_blobs else boxes


# --------------------------------------------------------------------------- online tracking

class OnlineTracker:
    """
    Thin wrapper over track.py's Kalman tracker, driven one frame at a time.

    The offline tracker walks a finished list of per-frame detections; here the same Track and
    associate() are fed incrementally and a track is emitted as an event the moment it ages out,
    so the caller learns about an animal seconds after it leaves rather than at end of file.
    """

    def __init__(self, width, height, cfg, tcfg):
        self.cfg, self.tcfg = cfg, tcfg
        self.width, self.height = width, height
        self.diag = (width ** 2 + height ** 2) ** 0.5
        self.active = []
        offline_track.Track._next_id = 0

    def step(self, frame_number, boxes):
        """Advance one frame. Returns tracks that finished on this frame."""
        boxes = offline_track.merge_fragments(boxes, self.tcfg.merge_gap)
        for t in self.active:
            t.predict()
        matched, unmatched = offline_track.associate(self.active, boxes, self.tcfg, self.diag)
        for ti, bi in matched:
            self.active[ti].update(boxes[bi], frame_number)
        for bi in unmatched:
            self.active.append(offline_track.Track(boxes[bi], frame_number, self.tcfg))

        done, still = [], []
        for t in self.active:
            (done if t.age_since_hit > self.tcfg.max_age else still).append(t)
        self.active = still
        return [t for t in done if t.hits >= self.cfg.min_track_frames]

    def flush(self):
        out = [t for t in self.active if t.hits >= self.cfg.min_track_frames]
        self.active = []
        return out


def event_of(track, fps, cfg):
    """Summarise a finished track, or None if it is too weak to report."""
    frames = [f for f, _ in track.history]
    boxes = [b for _, b in track.history]
    peak_delta = max(b[4] for b in boxes)
    if peak_delta < cfg.min_event_delta:
        return None
    centres = np.array([[(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0] for b in boxes])
    displacement = float(np.max(np.linalg.norm(centres - centres[0], axis=1)))
    path = float(np.sum(np.linalg.norm(np.diff(centres, axis=0), axis=1))) if len(centres) > 1 else 0.0
    duration = (frames[-1] - frames[0]) / max(fps, 1e-6)
    peak_i = int(np.argmax([b[4] * b[5] for b in boxes]))
    return {
        'track_id': track.id,
        'start_frame': frames[0], 'end_frame': frames[-1], 'n_detections': len(frames),
        'duration_s': round(duration, 2),
        'displacement_px': round(displacement, 1), 'path_px': round(path, 1),
        'speed_px_s': round(path / max(duration, 0.5), 2),
        'peak_frame': frames[peak_i], 'peak_mean_delta': round(peak_delta, 2),
        'peak_area_px': int(max(b[5] for b in boxes)),
        'peak_box': [int(v) for v in boxes[peak_i][:4]],
    }


# --------------------------------------------------------------------------- driver

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0],
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument('--video', help='replay a recorded 8-bit video')
    src.add_argument('--raw', help='replay a 16-bit *_raw.dat sidecar')
    src.add_argument('--camera', action='store_true', help='live sensor (implement CameraSource)')
    p.add_argument('--raw-shape', help='n,h,w for --raw; read from the .json sidecar if omitted')
    p.add_argument('--fps', type=float, default=0.0, help='override the source frame rate')
    p.add_argument('--mask-rows', type=int, default=15,
                   help='blank this many bottom rows when replaying a recorded video, to cover '
                        'the burned-in caption; a live frame has none, so use 0')
    p.add_argument('--realtime', action='store_true', help='pace replay at the real frame rate')
    p.add_argument('--bench', action='store_true', help='report per-frame cost and exit')
    p.add_argument('--snapshot-dir', help='write each event peak frame here as JPEG')
    p.add_argument('--quiet', action='store_true', help='events only, no progress on stderr')
    for field, value in vars(StreamConfig()).items():
        if isinstance(value, bool):
            p.add_argument(f"--{field.replace('_', '-')}", type=lambda s: s.lower() != 'false',
                           default=value)
        else:
            p.add_argument(f"--{field.replace('_', '-')}", type=type(value), default=value)
    args = p.parse_args(argv)

    cfg = StreamConfig(**{f: getattr(args, f) for f in vars(StreamConfig())})
    tcfg = offline_track.Config()

    if args.video:
        source = VideoSource(args.video, mask_rows=args.mask_rows)
        fps = args.fps or source.fps
    elif args.raw:
        shape = args.raw_shape
        if not shape:
            meta = Path(args.raw.replace('_raw.dat', '.json'))
            shape = json.loads(meta.read_text())['raw_shape'] if meta.exists() else None
            if shape is None:
                raise SystemExit('--raw-shape is required when the .json sidecar is missing')
        else:
            shape = [int(v) for v in shape.split(',')]
        fps = args.fps or 1.0
        source = RawSource(args.raw, tuple(shape), fps)
    else:
        source = CameraSource(fps=args.fps or 10.0)
        fps = source.fps

    detector = tracker = None
    snap_dir = Path(args.snapshot_dir) if args.snapshot_dir else None
    if snap_dir:
        snap_dir.mkdir(parents=True, exist_ok=True)
    snap_cache = deque(maxlen=400)      # recent frames, so an event's peak can still be saved

    costs = []
    n_events = 0
    frame_number = 0
    started = time.perf_counter()
    next_due = started

    def emit(tracks):
        nonlocal n_events
        for t in tracks:
            ev = event_of(t, fps, cfg)
            if ev is None:
                continue
            n_events += 1
            print(json.dumps(ev), flush=True)
            if snap_dir:
                for fnum, img in snap_cache:
                    if fnum == ev['peak_frame']:
                        vis = cv2.cvtColor(cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
                                           .astype(np.uint8), cv2.COLOR_GRAY2BGR)
                        x0, y0, x1, y1 = ev['peak_box']
                        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 1)
                        cv2.imwrite(str(snap_dir / f"event_{ev['track_id']:04d}_f{fnum:06d}.jpg"),
                                    cv2.resize(vis, (vis.shape[1] * 3, vis.shape[0] * 3),
                                               interpolation=cv2.INTER_NEAREST))
                        break

    for frame in source:
        if detector is None:
            h, w = frame.shape
            detector = StreamDetector((h, w), cfg)
            tracker = OnlineTracker(w, h, cfg, tcfg)
        t0 = time.perf_counter()
        boxes = detector(frame)
        finished = tracker.step(frame_number, boxes)
        costs.append(time.perf_counter() - t0)
        if snap_dir:
            snap_cache.append((frame_number, frame))
        emit(finished)
        frame_number += 1

        if args.realtime:
            next_due += 1.0 / fps
            sleep = next_due - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
        if not args.quiet and frame_number % 1000 == 0:
            print(f'  {frame_number} frames, {n_events} events', file=sys.stderr, flush=True)

    if tracker is not None:
        emit(tracker.flush())

    if not args.quiet or args.bench:
        c = np.array(costs) * 1000.0
        wall = time.perf_counter() - started
        print(f'{frame_number} frames, {n_events} events, {wall:.1f} s wall', file=sys.stderr)
        if len(c):
            print(f'per-frame: mean {c.mean():.3f} ms, p99 {np.percentile(c, 99):.3f} ms '
                  f'-> headroom {1000.0 / max(c.mean(), 1e-9) / fps:.0f}x at {fps:g} fps',
                  file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
