"""Live-camera plumbing around WildMice/thermal_detect's validated real-time
detector, vendored unmodified as detect_stream.py and track.py.

Those two files are copied as-is from ../../Code/WildMice/thermal_detect -
see that project's README ("Real-time detection on the Raspberry Pi") for
how the causal pipeline was designed (EMA background + EMA per-pixel noise
scale in place of detect.py's non-causal 60s-block median/MAD) and
validated (8/9 confirmed high-tier tracks recovered against the offline
detector on six clips; 0.24ms/frame mean cost on their test hardware).
Update those two files by re-copying from there, not by editing them here.

This module only adds what a live camera stream needs that a one-shot CLI
replay tool doesn't: a persistent per-frame process() call, live overlay
boxes for currently-tracked (not yet necessarily finished) objects, and an
append-only event log.
"""

import json
from pathlib import Path

import numpy as np

import track as _track
from detect_stream import OnlineTracker, StreamConfig, StreamDetector, event_of

TrackConfig = _track.Config


class EventLogger:
    """Appends one JSON line per finished, reportable track - the same event
    shape detect_stream.py itself prints to stdout, plus a wall-clock
    timestamp (the vendored code only knows frame numbers, not time of day)."""

    def __init__(self, path):
        self.path = Path(path)
        self._fh = open(self.path, "a")

    def log(self, event: dict, timestamp: float) -> None:
        self._fh.write(json.dumps({**event, "timestamp": round(timestamp, 3)}) + "\n")
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()


class LiveDetector:
    """One instance per camera stream. Not thread-safe - call process() from
    a single thread, same as the capture loop that owns the camera.

    Persists its background model and active tracks for the object's whole
    lifetime, so a brief camera reconnect doesn't need to re-learn the scene
    or lose a track it was already following - construct one instance per
    capture-thread lifetime, not per connection attempt.
    """

    # A track whose centroid stays within this fraction of the frame diagonal for
    # stationary_timeout_s is "stationary" (160x120 -> ~16 px). Generous enough that a
    # grooming/feeding animal counts as stationary, small enough that a walking one doesn't.
    STATIONARY_RADIUS_FRAC = 0.08

    def __init__(self, width: int, height: int, cfg: StreamConfig, track_cfg: TrackConfig,
                 fps: float, stationary_timeout_s: float = 20.0):
        """stationary_timeout_s: see _retire_stationary(); 0 disables it."""
        self.cfg = cfg
        self.fps = fps
        self.stationary_timeout_s = stationary_timeout_s
        self._width, self._height = width, height
        self._stationary_radius = self.STATIONARY_RADIUS_FRAC * (width ** 2 + height ** 2) ** 0.5
        self._detector = StreamDetector((height, width), cfg)
        self._tracker = OnlineTracker(width, height, cfg, track_cfg)
        self._anchors = {}   # track id -> (cx, cy, frame_index) where it last moved
        self.frame_index = -1

    @property
    def active_track_count(self) -> int:
        return len(self._tracker.active)

    def process(self, frame: np.ndarray):
        """frame: 2D grayscale, any numeric dtype, at the resolution passed
        to __init__. Returns (overlay_boxes, events):

          overlay_boxes - (x0, y0, x1, y1, track_id) for each currently
          active track that has already earned cfg.min_track_frames hits,
          meant for live annotation of the frame just processed.

          events - a list of event dicts (see detect_stream.event_of) for
          any tracks that finished (aged out, or were retired as stationary
          - those carry "ended_by": "stationary_timeout") on this call -
          usually [].
        """
        self.frame_index += 1
        frame32 = frame.astype(np.float32)
        boxes = self._detector(frame32)
        finished = self._tracker.step(self.frame_index, boxes)
        events = [ev for t in finished if (ev := event_of(t, self.fps, self.cfg)) is not None]
        events += self._retire_stationary(frame32)

        overlay_boxes = [
            (t.box[0], t.box[1], t.box[2], t.box[3], t.id)
            for t in self._tracker.active
            if t.hits >= self.cfg.min_track_frames
        ]
        return overlay_boxes, events

    def _retire_stationary(self, frame32: np.ndarray) -> list:
        """End any track that hasn't moved for stationary_timeout_s, and fold the
        region it occupies into the background so it stops re-triggering.

        Why this exists: detect_stream.py's freeze_update keeps currently-detected
        pixels out of the background model, so an animal that stops moving doesn't
        melt into the scene and vanish from its track. That protection has no time
        limit, so a *static* object that gets detected once - a fixture warming up in
        the sun, something that falls into view - is protected forever: overnight
        tests produced single "tracks" lasting 16 and 52 hours, with a speed of 0.0
        px/s, pinning the RGB trigger on (one RGB clip filled ~46GB). A real animal
        that really does sit still for stationary_timeout_s is absorbed the same way,
        which is fine - it is detected again as soon as it moves.

        Done here, around the vendored detector, rather than by editing it: only its
        public attributes (the tracker's `active` list, the detector's `background`
        array) are touched.
        """
        if self.stationary_timeout_s <= 0:
            return []
        events = []
        live_ids = set()
        for track in list(self._tracker.active):
            if track.hits < self.cfg.min_track_frames:
                continue
            live_ids.add(track.id)
            cx = (track.box[0] + track.box[2]) / 2.0
            cy = (track.box[1] + track.box[3]) / 2.0
            anchor = self._anchors.get(track.id)
            if anchor is None or ((cx - anchor[0]) ** 2 + (cy - anchor[1]) ** 2) ** 0.5 > self._stationary_radius:
                self._anchors[track.id] = (cx, cy, self.frame_index)
                continue
            if (self.frame_index - anchor[2]) / self.fps < self.stationary_timeout_s:
                continue

            pad = 2
            x0 = max(int(track.box[0]) - pad, 0)
            y0 = max(int(track.box[1]) - pad, 0)
            x1 = min(int(track.box[2]) + pad, self._width)
            y1 = min(int(track.box[3]) + pad, self._height)
            self._detector.background[y0:y1, x0:x1] = frame32[y0:y1, x0:x1]
            self._tracker.active.remove(track)
            live_ids.discard(track.id)
            ev = event_of(track, self.fps, self.cfg)
            if ev is not None:
                ev["ended_by"] = "stationary_timeout"
                events.append(ev)
        for stale in [tid for tid in self._anchors if tid not in live_ids]:
            del self._anchors[stale]
        return events

    def flush(self):
        """Report any tracks still active as final events. Call once when
        the capture loop is stopping for good (not on a reconnect)."""
        return [ev for t in self._tracker.flush() if (ev := event_of(t, self.fps, self.cfg)) is not None]
