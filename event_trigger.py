"""Thread-safe hand-off between the thermal detection loop and the RGB event
recorder: "is there a reportable animal in frame right now, and when did we
last see one".

Deliberately just a level signal, not a queue of discrete events - the two
capture loops run at different, independently-varying frame rates and can
each stall/reconnect on their own, so coupling them through explicit
start/stop messages would need its own retry/ordering logic. A shared
"active since when" timestamp lets the RGB side implement its own pre/post-
roll timing purely from its own clock, and naturally merges two triggers in
quick succession into one continuous recording instead of two clipped ones.
"""

import threading
import time


class EventTrigger:
    def __init__(self):
        self._lock = threading.Lock()
        self._active = False
        self._last_active_time = 0.0

    def set_active(self, active: bool, now: float | None = None) -> None:
        """Called by the thermal detection loop every detection cycle with
        whether at least one reportable track exists right now."""
        now = now if now is not None else time.time()
        with self._lock:
            self._active = active
            if active:
                self._last_active_time = now

    def snapshot(self) -> tuple[bool, float]:
        """Called by the RGB event recorder: (is currently active, wall-clock
        time it was last seen active - 0.0 if never)."""
        with self._lock:
            return self._active, self._last_active_time
