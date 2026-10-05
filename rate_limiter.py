"""Thin a stream of frames (arriving at the camera's own ~25fps) down to a
target average rate, for the record / detect / publish gates.

The schedule is advanced by a fixed interval from the *previous scheduled
time*, not from whenever the frame that fired it actually arrived. That
matters: a frame can only fire on a capture tick, so it nearly always fires a
little after its scheduled time, and restarting the clock from the actual fire
time throws that lateness away on every cycle. The detection gate used to do
exactly that (`next_due = max(now, next_due) + interval`), which is why
--thermal-fps 12.5 recorded at 12.49fps (this schedule-based logic, in the
segment writer) while detection ran at 10.0fps in the same process: against a
25fps source each fire landed a coin-flip either side of the boundary and the
lateness compounded into a 2-or-3-ticks-per-fire mix averaging 2.5 ticks.

With the schedule held fixed the long-run average equals the target for any
target at or below the source rate. For a target that isn't a whole divisor of
the source rate (e.g. 10fps from 25fps) the *spacing* alternates between two
tick multiples (80 and 120ms) instead - same average, uneven gaps.
"""


class RateLimiter:
    def __init__(self, fps: float):
        self.interval = 1.0 / fps if fps > 0 else 0.0
        self._next_due = 0.0

    def ready(self, now: float) -> bool:
        """True if a frame arriving at `now` should be kept. fps <= 0 keeps all."""
        if self.interval <= 0:
            return True
        if now < self._next_due:
            return False
        self._next_due += self.interval
        if self._next_due < now:
            # Fell behind by more than a whole interval (first call, a reconnect
            # stall): resync rather than firing back-to-back to "catch up".
            self._next_due = now + self.interval
        return True
