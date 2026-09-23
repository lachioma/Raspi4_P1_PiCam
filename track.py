'''
Multi-object tracking on top of the boxes produced by detect.py.

detect.py links boxes with a greedy nearest-centroid rule that has no motion model, and it
fragments badly: in thermal_20260821_234136 one animal came out as tracks 13, 14, 15, 17 and 19,
some of them overlapping in time. Two things go wrong, and this stage fixes both.

    fragmentation in space   a warm animal is not one contour. Thresholding splits it into a
                             head-sized blob and a body-sized blob, so one animal produces
                             several boxes per frame and therefore several tracks. Boxes that
                             nearly touch are merged before tracking.

    fragmentation in time    the animal is missed for a few frames - it pauses, turns side-on,
                             passes behind vegetation - and the old linker starts a fresh
                             track. A constant-velocity Kalman filter keeps predicting where it
                             should be, so the track survives the gap and picks it up again.

Usage
    python3 track.py                          # reads output/_per_video/, writes tracks.csv
    python3 track.py --video thermal_20260821_234136
    python3 track.py --vis-trajectories       # draw the path of every confirmed track

Input is the per-video JSON cache written by detect.py, which holds the raw boxes from before
detect.py's own track filtering. Run detect.py first.
'''

import argparse
import csv
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


class Config:
    """Tracking parameters. Every field is exposed on the command line."""

    def __init__(self, **kw):
        # merging contour fragments of one animal within a frame
        self.merge_gap = 3.0         # px; boxes closer than this are one object
        # association
        self.max_dist_frac = 0.16    # gating distance as a fraction of the frame diagonal
        self.iou_weight = 0.5        # how much box overlap counts against centroid distance
        # track lifecycle, in frames at the source frame rate
        self.max_age = 12            # keep predicting this long through a miss (1.2 s at 10 fps)
        self.min_hits = 3            # detections needed before a track is reported
        # measurement/process noise for the constant-velocity filter, in px
        self.process_var = 4.0
        self.measure_var = 6.0
        # track-level acceptance, mirroring detect.py's physical reasoning
        self.min_track_delta = 35.0
        self.high_delta = 80.0
        self.high_disp = 35.0
        self.high_speed = 2.0
        self.high_area_frac = 0.0013
        self.max_edge_frac = 0.8
        self.med_delta = 60.0
        self.med_disp = 35.0          # was 20: clutter that jitters in place but never crosses
                                      # the scene was reaching the medium tier
        self.med_speed = 1.0
        self.med_area_frac = 0.0008
        # A real animal keeps a recognisable footprint for the whole track; stitched clutter is
        # a few pixels in every frame and only looks big at one lucky peak. Measured on three
        # reviewer-confirmed false positives (median box 9-14 px) against nine confirmed animals
        # (25-160 px), so this is on the track's median box, not its peak.
        self.min_median_box_frac = 0.001   # 160x120 -> ~19 px
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(f'unknown config field: {k}')
            setattr(self, k, v)


# --------------------------------------------------------------------------- fragment merging

def merge_fragments(boxes, gap):
    """
    Union boxes that are within `gap` px of each other, so one animal is one detection.

    Repeatedly merges until nothing changes: a head blob and a body blob that each nearly touch
    a middle blob have to end up in the same group even though they do not touch each other.
    The merged box carries the summed area and the max delta of its parts.
    """
    groups = [[b] for b in boxes]
    changed = True
    while changed:
        changed = False
        for i in range(len(groups)):
            if groups[i] is None:
                continue
            for j in range(i + 1, len(groups)):
                if groups[j] is None:
                    continue
                if _group_gap(groups[i], groups[j]) <= gap:
                    groups[i] = groups[i] + groups[j]
                    groups[j] = None
                    changed = True
        groups = [g for g in groups if g is not None]

    merged = []
    for group in groups:
        x0 = min(b[0] for b in group)
        y0 = min(b[1] for b in group)
        x1 = max(b[2] for b in group)
        y1 = max(b[3] for b in group)
        merged.append((x0, y0, x1, y1,
                       max(b[4] for b in group),      # peak rise over background
                       sum(b[5] for b in group)))     # total hot area, not the bounding area
    return merged


def _group_gap(a, b):
    """Smallest edge-to-edge distance between any box of group a and any box of group b."""
    best = float('inf')
    for p in a:
        for q in b:
            dx = max(q[0] - p[2], p[0] - q[2], 0)
            dy = max(q[1] - p[3], p[1] - q[3], 0)
            best = min(best, math.hypot(dx, dy))
    return best


# --------------------------------------------------------------------------- Kalman track

class Track:
    """
    One object, followed with a constant-velocity Kalman filter.

    State is [cx, cy, vx, vy]. Box size is not filtered - a thermal blob's apparent size jumps
    around with the threshold far more than its position does, so size is carried as the last
    observation and used only for reporting and for the IoU term in association.
    """

    _next_id = 0

    def __init__(self, box, frame_number, cfg):
        cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
        self.x = np.array([cx, cy, 0.0, 0.0])
        self.P = np.diag([cfg.measure_var, cfg.measure_var, 100.0, 100.0])
        self.cfg = cfg
        self.id = Track._next_id
        Track._next_id += 1
        self.box = box
        self.hits = 1
        self.age_since_hit = 0
        self.history = [(frame_number, box)]     # observations only, not predictions
        self.start_frame = frame_number
        self.last_frame = frame_number

    def predict(self):
        F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        Q = np.diag([self.cfg.process_var] * 4)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.age_since_hit += 1
        return self.x[:2]

    def update(self, box, frame_number):
        z = np.array([(box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0])
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        R = np.diag([self.cfg.measure_var] * 2)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.box = box
        self.hits += 1
        self.age_since_hit = 0
        self.last_frame = frame_number
        self.history.append((frame_number, box))


def _iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def associate(tracks, boxes, cfg, diag):
    """
    Match predicted track positions to detections.

    Greedy lowest-cost matching rather than Hungarian: frames here hold a handful of objects at
    most, where greedy and optimal assignment agree, and it keeps the script dependency-free.
    Cost blends predicted-centroid distance with box overlap, gated by --max-dist-frac.
    """
    gate = cfg.max_dist_frac * diag
    pairs = []
    for ti, track in enumerate(tracks):
        px, py = track.x[0], track.x[1]
        for bi, box in enumerate(boxes):
            cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
            dist = math.hypot(cx - px, cy - py)
            if dist > gate:
                continue
            cost = dist / gate - cfg.iou_weight * _iou(track.box, box)
            pairs.append((cost, ti, bi))
    pairs.sort()

    matched, used_t, used_b = [], set(), set()
    for _cost, ti, bi in pairs:
        if ti in used_t or bi in used_b:
            continue
        matched.append((ti, bi))
        used_t.add(ti)
        used_b.add(bi)
    unmatched_boxes = [i for i in range(len(boxes)) if i not in used_b]
    return matched, unmatched_boxes


def track_video(raw, fps, width, height, cfg):
    """Run the tracker over one video's raw per-frame detections. Returns finished tracks."""
    diag = math.hypot(width, height)
    active, finished = [], []

    for frame_number, boxes in raw:
        boxes = merge_fragments([tuple(b) for b in boxes], cfg.merge_gap)
        for track in active:
            track.predict()
        matched, unmatched = associate(active, boxes, cfg, diag)
        for ti, bi in matched:
            active[ti].update(boxes[bi], frame_number)
        for bi in unmatched:
            active.append(Track(boxes[bi], frame_number, cfg))
        still_active = []
        for track in active:
            if track.age_since_hit > cfg.max_age:
                finished.append(track)
            else:
                still_active.append(track)
        active = still_active

    finished.extend(active)
    return [t for t in finished if t.hits >= cfg.min_hits]


# --------------------------------------------------------------------------- scoring

def summarise(track, fps, width, height, timestamps, video_name, cfg):
    """Turn a finished track into a report row, with the same physical tiering as detect.py."""
    frames = [f for f, _ in track.history]
    boxes = [b for _, b in track.history]
    centres = np.array([[(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0] for b in boxes])
    displacement = float(np.max(np.linalg.norm(centres - centres[0], axis=1)))
    path = float(np.sum(np.linalg.norm(np.diff(centres, axis=0), axis=1))) if len(centres) > 1 else 0.0
    duration = (frames[-1] - frames[0]) / max(fps, 1e-6)
    speed = path / max(duration, 0.5)
    peak_delta = max(b[4] for b in boxes)
    peak_area = max(b[5] for b in boxes)
    frame_area = max(width * height, 1)
    on_edge = sum(1 for b in boxes
                  if b[0] <= 0 or b[1] <= 0 or b[2] >= width or b[3] >= height)
    edge_frac = on_edge / len(boxes)
    # how much of the track's span actually carries a detection; a low value means the filter
    # coasted through long gaps and the link is less trustworthy
    span = frames[-1] - frames[0] + 1
    continuity = len(frames) / span

    # median bounding-box footprint over the whole track, not the one lucky peak frame
    median_box = float(np.median([(b[2] - b[0]) * (b[3] - b[1]) for b in boxes]))

    def passes(delta, disp, spd, area_frac):
        return (peak_delta >= delta and displacement >= disp and speed >= spd
                and peak_area >= area_frac * frame_area and edge_frac <= cfg.max_edge_frac
                and median_box >= cfg.min_median_box_frac * frame_area)

    if passes(cfg.high_delta, cfg.high_disp, cfg.high_speed, cfg.high_area_frac):
        confidence = 'high'
    elif passes(cfg.med_delta, cfg.med_disp, cfg.med_speed, cfg.med_area_frac):
        confidence = 'medium'
    else:
        confidence = 'low'

    peak_i = int(np.argmax([b[4] * b[5] for b in boxes]))
    peak_box = boxes[peak_i]
    start_time = None
    if timestamps and frames[0] < len(timestamps):
        start_time = datetime.fromtimestamp(timestamps[frames[0]], tz=timezone.utc).isoformat()

    return {
        'video_name': video_name, 'track_id': track.id, 'confidence': confidence,
        'start_frame': frames[0], 'end_frame': frames[-1], 'n_detections': len(frames),
        'duration_s': round(duration, 2), 'start_time': start_time,
        'displacement_px': round(displacement, 1), 'path_px': round(path, 1),
        'speed_px_s': round(speed, 2), 'edge_frac': round(edge_frac, 2),
        'continuity': round(continuity, 2), 'median_box_px': round(median_box, 1),
        'peak_frame': frames[peak_i], 'peak_mean_delta': round(peak_delta, 2),
        'peak_area_px': int(peak_area),
        'peak_x_min': int(peak_box[0]), 'peak_y_min': int(peak_box[1]),
        'peak_x_max': int(peak_box[2]), 'peak_y_max': int(peak_box[3]),
        'score': round(peak_delta * (peak_area ** 0.5) * (1.0 + displacement / 20.0), 1),
    }


# --------------------------------------------------------------------------- per video

def process_one(args):
    cache_file, cfg, vis_dir, data_dir, vis_min_conf = args
    result = json.loads(Path(cache_file).read_text())
    if 'raw' not in result:
        return {'video': result.get('video'), 'error': 'no raw boxes; rerun detect.py'}
    name = result['video']
    fps = result.get('fps') or 10.0
    width, height = result['width'], result['height']

    Track._next_id = 0
    tracks = track_video(result['raw'], fps, width, height, cfg)

    ts_path = Path(data_dir) / f'{name}_timestamps.txt' if data_dir else None
    timestamps = None
    if ts_path and ts_path.exists():
        try:
            timestamps = [float(l) for l in ts_path.read_text().split() if l.strip()]
        except ValueError:
            timestamps = None

    rows, paths = [], {}
    for track in tracks:
        row = summarise(track, fps, width, height, timestamps, name, cfg)
        if row['peak_mean_delta'] < cfg.min_track_delta:
            continue
        rows.append(row)
        paths[track.id] = [[f, *map(int, b[:4])] for f, b in track.history]

    if vis_dir and rows:
        _draw_trajectories(Path(data_dir) / f'{name}.avi', Path(vis_dir) / name,
                           rows, paths, vis_min_conf)
    return {'video': name, 'tracks': rows, 'paths': paths}


def _draw_trajectories(video_path, out_dir, rows, paths, vis_min_conf):
    """One image per track: the peak frame with the object's whole path drawn over it."""
    rank = {'high': 0, 'medium': 1, 'low': 2}
    wanted = [r for r in rows if rank[r['confidence']] <= rank[vis_min_conf]]
    if not wanted or not video_path.exists():
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    by_frame = {}
    for r in wanted:
        by_frame.setdefault(r['peak_frame'], []).append(r)

    cap = cv2.VideoCapture(str(video_path))
    frame_number, last = 0, max(by_frame)
    while frame_number <= last:
        ok, frame = cap.read()
        if not ok:
            break
        for r in by_frame.get(frame_number, []):
            vis = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            pts = paths[r['track_id']]
            for (_f0, ax0, ay0, ax1, ay1), (_f1, bx0, by0, bx1, by1) in zip(pts, pts[1:]):
                cv2.line(vis, ((ax0 + ax1) // 2, (ay0 + ay1) // 2),
                         ((bx0 + bx1) // 2, (by0 + by1) // 2), (255, 128, 0), 1, cv2.LINE_AA)
            cv2.circle(vis, ((pts[0][1] + pts[0][3]) // 2, (pts[0][2] + pts[0][4]) // 2),
                       2, (255, 0, 255), -1)
            cv2.rectangle(vis, (r['peak_x_min'], r['peak_y_min']),
                          (r['peak_x_max'], r['peak_y_max']), (0, 255, 0), 1)
            vis = cv2.resize(vis, (vis.shape[1] * 3, vis.shape[0] * 3),
                             interpolation=cv2.INTER_NEAREST)
            cv2.putText(vis, f"t{r['track_id']} {r['confidence']} {r['duration_s']}s "
                             f"{r['displacement_px']}px", (5, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imwrite(str(out_dir / f"track_{r['track_id']:04d}.jpg"), vis,
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
        frame_number += 1
    cap.release()


# --------------------------------------------------------------------------- driver

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0],
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--cache-dir', default='./output/_per_video',
                        help="detect.py's per-video results")
    parser.add_argument('--data-dir', default='./data/recordings_field_test',
                        help='videos and timestamp sidecars, for visualisation')
    parser.add_argument('--out', default='tracks.csv')
    parser.add_argument('--paths-json', default='tracks_paths.json',
                        help='full per-track trajectories')
    parser.add_argument('--video', nargs='*', default=None, help='limit to these video stems')
    parser.add_argument('--workers', type=int, default=min(16, os.cpu_count() or 1))
    parser.add_argument('--vis-trajectories', action='store_true',
                        help='draw each track path onto its peak frame')
    parser.add_argument('--vis-dir', default='./output_tracks')
    parser.add_argument('--vis-min-confidence', choices=['high', 'medium', 'low'],
                        default='medium')
    for field, value in vars(Config()).items():
        parser.add_argument(f"--{field.replace('_', '-')}", type=type(value), default=value)
    args = parser.parse_args(argv)

    cfg = Config(**{f: getattr(args, f) for f in vars(Config())})
    cache_files = sorted(Path(args.cache_dir).glob('*.json'))
    if args.video:
        stems = set(args.video)
        cache_files = [c for c in cache_files if c.stem in stems]
    if not cache_files:
        print(f'No detect.py results in {args.cache_dir}; run detect.py first')
        return 1

    vis_dir = args.vis_dir if args.vis_trajectories else None
    jobs = [(str(c), cfg, vis_dir, args.data_dir, args.vis_min_confidence)
            for c in cache_files]
    print(f'{len(jobs)} video(s), {args.workers} worker(s)')

    all_rows, all_paths, failed = [], {}, []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for done, result in enumerate(pool.map(process_one, jobs), 1):
            if 'error' in result:
                failed.append(result)
            else:
                all_rows.extend(result['tracks'])
                if result['paths']:
                    all_paths[result['video']] = result['paths']
            if done % 200 == 0 or done == len(jobs):
                print(f'  {done}/{len(jobs)}   {len(all_rows)} tracks', flush=True)

    all_rows.sort(key=lambda r: -r['score'])
    if all_rows:
        with open(args.out, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(all_rows[0].keys()))
            writer.writeheader()
            writer.writerows(all_rows)
    with open(args.paths_json, 'w') as fh:
        json.dump(all_paths, fh)

    print(f'\n{len(all_rows)} tracks')
    for tier in ('high', 'medium', 'low'):
        sel = [r for r in all_rows if r['confidence'] == tier]
        print(f'  {tier:6s} {len(sel):6d} tracks in {len({r["video_name"] for r in sel}):4d} videos')
    print(f'\n  {args.out}\n  {args.paths_json}')
    if vis_dir:
        print(f'  {vis_dir}/<video>/track_*.jpg')
    if failed:
        print(f'{len(failed)} video(s) skipped: {failed[0]["error"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
