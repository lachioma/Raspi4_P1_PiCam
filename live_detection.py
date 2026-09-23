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

    def __init__(self, width: int, height: int, cfg: StreamConfig, track_cfg: TrackConfig, fps: float):
        self.cfg = cfg
        self.fps = fps
        self._detector = StreamDetector((height, width), cfg)
        self._tracker = OnlineTracker(width, height, cfg, track_cfg)
        self.frame_index = -1

    def process(self, frame: np.ndarray):
        """frame: 2D grayscale, any numeric dtype, at the resolution passed
        to __init__. Returns (overlay_boxes, events):

          overlay_boxes - (x0, y0, x1, y1, track_id) for each currently
          active track that has already earned cfg.min_track_frames hits,
          meant for live annotation of the frame just processed.

          events - a list of event dicts (see detect_stream.event_of) for
          any tracks that finished (aged out) on this call - usually [].
        """
        self.frame_index += 1
        boxes = self._detector(frame.astype(np.float32))
        finished = self._tracker.step(self.frame_index, boxes)

        overlay_boxes = [
            (t.box[0], t.box[1], t.box[2], t.box[3], t.id)
            for t in self._tracker.active
            if t.hits >= self.cfg.min_track_frames
        ]
        events = [ev for t in finished if (ev := event_of(t, self.fps, self.cfg)) is not None]
        return overlay_boxes, events

    def flush(self):
        """Report any tracks still active as final events. Call once when
        the capture loop is stopping for good (not on a reconnect)."""
        return [ev for t in self._tracker.flush() if (ev := event_of(t, self.fps, self.cfg)) is not None]
