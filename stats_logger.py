"""Shared periodic-stats JSONL logger, used by all four capture loops
(thermal_source.py, rgb_source.py, thermal_field_recorder.py,
rgb_event_recorder.py) so the console-only stats line each already printed
also survives in a file - the whole point being debugging a run nobody was
watching live (exactly what prompted this: an overnight field_recorder.py
run hit a full SD card, and the only record of what happened was a console
nobody was looking at).

One aggregate line per --stats-interval-seconds (default 60s), not one per
frame - at that rate the cost is a few hundred bytes of disk I/O a minute,
negligible next to the video/detection work already happening every frame.
That's why there's no separate "test mode" toggle for this: the ongoing
cost doesn't justify one. --no-stats-log (in both entry points) turns it
off entirely if ever wanted regardless.
"""

import json
import time


class StatsLogger:
    def __init__(self, path):
        self.path = path
        self._fh = open(path, "a")

    def log(self, **fields) -> None:
        """Each call writes one JSON line; pass whatever fields are relevant
        to the caller (captured_fps, free_disk_mb, clip_open, ...). A
        timestamp is added automatically unless the caller supplies one."""
        fields.setdefault("timestamp", round(time.time(), 3))
        self._fh.write(json.dumps(fields) + "\n")
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()
