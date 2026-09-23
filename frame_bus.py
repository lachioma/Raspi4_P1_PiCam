"""Thread-safe single-slot "latest frame" handoff between a capture thread and
any number of HTTP client-serving threads.

Only the newest encoded JPEG is ever kept - if nobody has consumed the
previous frame yet it is simply overwritten. This is what "video streaming"
should mean here: an HTTP client always gets shown the most current frame,
never a growing backlog of stale ones, regardless of how fast the capture
thread runs versus how fast a particular client's connection can drain.
"""

import threading


class FrameBus:
    def __init__(self):
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._frame_id = 0

    def publish(self, jpeg_bytes: bytes) -> None:
        with self._condition:
            self._jpeg = jpeg_bytes
            self._frame_id += 1
            self._condition.notify_all()

    def get_latest(
        self, last_seen_id: int = -1, timeout: float = 5.0
    ) -> tuple[bytes | None, int]:
        """Block until a frame newer than last_seen_id is published, or timeout.

        Returns (jpeg_bytes_or_None, frame_id). jpeg_bytes is None only if no
        frame has ever been published within the timeout (e.g. the capture
        source is still starting up or has died) - callers should treat that
        as "nothing to send yet", not as a fatal error.
        """
        with self._condition:
            if self._frame_id == last_seen_id or self._jpeg is None:
                self._condition.wait(timeout=timeout)
            return self._jpeg, self._frame_id
