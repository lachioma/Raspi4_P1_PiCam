"""Event-clip video as one JPEG per frame in an AVI, encoded in parallel.

Why this exists. The RGB event clips used to go through cv2.VideoWriter, whose
encoders (XVID 48-69 ms/frame, MJPG 60 ms/frame at 1640x1232 on a Pi 4) can't
keep up with 30fps, and which block the capture loop while they work. Opening a
clip also wrote the whole 2 s pre-roll backlog (~60 frames) synchronously - about
3.6 s of blocked capture - so short events were recorded as pre-roll plus a frame
or two, with the event itself and the post-roll lost. cv2.imencode, by contrast,
takes 22.5 ms/frame at that size (measured streaming), and it releases the GIL.

So: encode each frame with cv2.imencode on a small thread pool (several frames
in flight at once, ~130 frames/s with three workers against a 30fps source),
and let one writer thread put the finished JPEGs into an AVI in capture order.
Nothing in the capture loop waits for any of it - submitting a frame is a queue
append - so the pre-roll backlog is encoded while live capture carries on.

Timing is kept honest rather than assumed. Each frame is placed on a fixed
1/fps grid by its capture timestamp: a gap (capture hiccup, a dropped frame) is
filled by repeating the previous JPEG, which costs no encoding, and a second
frame landing in an already-filled slot is skipped. The clip therefore plays
back at real-time speed whatever the camera actually delivered - the old writer
declared 30fps while writing ~16 and played back ~1.8x fast.

The AVI is written by MjpegAviWriter below rather than OpenCV: it's a small,
fixed layout (one video stream, 'MJPG', an idx1 index), and OpenCV's writer would
mean re-encoding. Classic AVI tops out around 2GB, so a long clip rolls over into
`<name>_part2.avi` etc. at MAX_PART_BYTES.
"""

import struct
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2

MAX_PART_BYTES = 1_800_000_000

# Fixed layout of the header MjpegAviWriter emits, so the fields patched at close()
# can be addressed by offset. _write_header() asserts the header is exactly this long.
_HEADER_SIZE = 224
_RIFF_SIZE_POS = 4
_AVIH_MAX_BYTES_PER_SEC_POS = 36
_AVIH_TOTAL_FRAMES_POS = 48
_AVIH_SUGGESTED_BUFFER_POS = 60
_STRH_LENGTH_POS = 140
_STRH_SUGGESTED_BUFFER_POS = 144
_MOVI_SIZE_POS = 216
_MOVI_FOURCC_POS = 220   # idx1 chunk offsets are measured from here


def _chunk(fourcc: bytes, data: bytes) -> bytes:
    return fourcc + struct.pack("<I", len(data)) + data


class MjpegAviWriter:
    """Minimal AVI (RIFF) container for a sequence of already JPEG-encoded frames."""

    def __init__(self, path, width: int, height: int, fps: float):
        self.path = Path(path)
        self.width, self.height = int(width), int(height)
        self._rate = max(int(round(fps * 1000)), 1)  # dwRate / dwScale(1000) = fps; allows 12.5
        self._fh = open(self.path, "wb")
        self._index = []          # (chunk offset from the 'movi' fourcc, size), one per frame
        self._max_chunk = 0
        self._write_header()
        self._pos = _HEADER_SIZE

    @property
    def frame_count(self) -> int:
        return len(self._index)

    @property
    def bytes_written(self) -> int:
        return self._pos

    def _write_header(self) -> None:
        usec_per_frame = int(round(1_000_000_000 / self._rate))
        avih = struct.pack(
            "<14I",
            usec_per_frame, 0, 0, 0x10,       # per-frame usec, max bytes/s (patched), padding, HASINDEX
            0, 0, 1, 0,                       # total frames (patched), initial frames, streams, buffer (patched)
            self.width, self.height, 0, 0, 0, 0,
        )
        strh = struct.pack(
            "<4s4sIHH8I4h",
            b"vids", b"MJPG", 0, 0, 0,        # type, handler, flags, priority, language
            0, 1000, self._rate, 0,           # initial frames, scale, rate, start
            0, 0, 0xFFFFFFFF, 0,              # length (patched), buffer (patched), quality, sample size
            0, 0, self.width, self.height,    # rcFrame
        )
        strf = struct.pack(
            "<IiiHH4sIiiII",
            40, self.width, self.height, 1, 24, b"MJPG",
            self.width * self.height * 3, 0, 0, 0, 0,
        )
        strl = b"strl" + _chunk(b"strh", strh) + _chunk(b"strf", strf)
        hdrl = b"hdrl" + _chunk(b"avih", avih) + b"LIST" + struct.pack("<I", len(strl)) + strl
        header = (
            b"RIFF" + struct.pack("<I", 0) + b"AVI "
            + b"LIST" + struct.pack("<I", len(hdrl)) + hdrl
            + b"LIST" + struct.pack("<I", 0) + b"movi"
        )
        assert len(header) == _HEADER_SIZE, len(header)
        self._fh.write(header)

    def write_jpeg(self, jpeg: bytes) -> None:
        size = len(jpeg)
        pad = size & 1                       # chunks are word-aligned
        self._index.append((self._pos - _MOVI_FOURCC_POS, size))
        self._fh.write(b"00dc" + struct.pack("<I", size) + jpeg + (b"\0" if pad else b""))
        self._pos += 8 + size + pad
        self._max_chunk = max(self._max_chunk, size)

    def close(self) -> None:
        if self._fh is None:
            return
        fh, self._fh = self._fh, None
        try:
            n = len(self._index)
            idx_start = self._pos
            idx = b"".join(b"00dc" + struct.pack("<III", 0x10, off, size) for off, size in self._index)
            fh.write(b"idx1" + struct.pack("<I", len(idx)) + idx)
            end = idx_start + 8 + len(idx)

            def patch(pos: int, value: int) -> None:
                fh.seek(pos)
                fh.write(struct.pack("<I", value))

            patch(_RIFF_SIZE_POS, end - 8)
            patch(_AVIH_MAX_BYTES_PER_SEC_POS, int(self._max_chunk * self._rate / 1000))
            patch(_AVIH_TOTAL_FRAMES_POS, n)
            patch(_AVIH_SUGGESTED_BUFFER_POS, self._max_chunk)
            patch(_STRH_LENGTH_POS, n)
            patch(_STRH_SUGGESTED_BUFFER_POS, self._max_chunk)
            patch(_MOVI_SIZE_POS, idx_start - _MOVI_FOURCC_POS)
        finally:
            fh.close()


class JpegClipWriter:
    """Encode frames to JPEG on a thread pool and write them, in order and on a fixed
    time grid, into an AVI (rolling over into _partN files if it gets big).

    submit() never blocks; finish() waits for what's in flight and closes the file.
    """

    MAX_FILL_SECONDS = 5.0   # a longer gap is a stall, not something to pad with a frozen frame

    def __init__(self, path, width: int, height: int, fps: float, quality: int = 75,
                 workers: int = 3, max_pending: int = 90, max_part_bytes: int = MAX_PART_BYTES):
        self.path = Path(path)
        self.parts = [self.path]
        self._width, self._height, self._fps = int(width), int(height), float(fps)
        self._params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        self._max_pending = max_pending
        self._max_part_bytes = max_part_bytes
        self._avi = MjpegAviWriter(self.path, width, height, fps)
        self._pool = ThreadPoolExecutor(max_workers=workers, thread_name_prefix="jpeg-enc")
        self._pending = deque()                 # (timestamp, Future), in submission order
        self._cond = threading.Condition()
        self._closing = False
        self._stats_lock = threading.Lock()
        self._enc_count = 0
        self._enc_seconds = 0.0
        self._t0 = None
        self._slots = 0                          # frames written == time-grid slots filled
        self._last_jpeg = None
        self._write_error = None
        self.first_ts = None
        self.last_ts = None
        self.frames_dropped = 0                  # never encoded: the backlog was full
        self.frames_skipped = 0                  # a second frame in an already-filled slot
        self.frames_duplicated = 0               # repeated to fill a gap
        self.encode_errors = 0
        self._writer = threading.Thread(target=self._writer_loop, daemon=True, name="jpeg-writer")
        self._writer.start()

    @property
    def frame_count(self) -> int:
        return self._slots

    def submit(self, ts: float, bgr) -> bool:
        """Queue one frame (captured at `ts`). False if it was dropped for lack of room."""
        with self._cond:
            if self._closing or len(self._pending) >= self._max_pending:
                self.frames_dropped += 1
                return False
            if self.first_ts is None:
                self.first_ts = ts
            self.last_ts = ts
            self._pending.append((ts, self._pool.submit(self._encode, bgr)))
            self._cond.notify()
        return True

    def take_encode_stats(self):
        """(frames encoded, seconds spent encoding them) since the last call."""
        with self._stats_lock:
            n, s = self._enc_count, self._enc_seconds
            self._enc_count, self._enc_seconds = 0, 0.0
        return n, s

    def finish(self) -> dict:
        with self._cond:
            self._closing = True
            self._cond.notify_all()
        self._writer.join()
        self._pool.shutdown(wait=True)
        try:
            self._avi.close()
        except OSError as e:
            self._write_error = self._write_error or e
        return {
            "frames_written": self._slots,
            "frames_duplicated": self.frames_duplicated,
            "frames_dropped": self.frames_dropped,
            "frames_skipped": self.frames_skipped,
            "encode_errors": self.encode_errors,
            "write_error": repr(self._write_error) if self._write_error else None,
            "parts": [p.name for p in self.parts],
        }

    def _encode(self, bgr) -> bytes:
        t0 = time.perf_counter()
        ok, buf = cv2.imencode(".jpg", bgr, self._params)
        elapsed = time.perf_counter() - t0
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        with self._stats_lock:
            self._enc_count += 1
            self._enc_seconds += elapsed
        return buf.tobytes()

    def _writer_loop(self) -> None:
        while True:
            with self._cond:
                while not self._pending and not self._closing:
                    self._cond.wait()
                if not self._pending:
                    return
                ts, future = self._pending.popleft()
            try:
                jpeg = future.result()
            except Exception:
                self.encode_errors += 1
                continue
            if self._write_error is not None:
                continue                         # disk trouble: keep draining, stop writing
            try:
                self._place(ts, jpeg)
            except OSError as e:
                self._write_error = e
                print(f"[rgb-event] clip write failed ({e!r}); the rest of this clip is lost.")

    def _place(self, ts: float, jpeg: bytes) -> None:
        """Put `jpeg` at the grid slot for `ts`, repeating the last frame over any gap."""
        if self._t0 is None:
            self._t0 = ts
        slot = int(round((ts - self._t0) * self._fps))
        if self._slots and slot < self._slots:
            self.frames_skipped += 1
            return
        missing = slot - self._slots
        if self._last_jpeg is None:
            missing = 0
        max_fill = int(self.MAX_FILL_SECONDS * self._fps)
        if missing > max_fill:
            self._t0 += (missing - max_fill) / self._fps   # re-anchor: don't pad a stall
            missing = max_fill
        for _ in range(missing):
            self._put(self._last_jpeg)
            self.frames_duplicated += 1
        self._put(jpeg)
        self._last_jpeg = jpeg

    def _put(self, jpeg: bytes) -> None:
        if self._avi.frame_count and self._avi.bytes_written + len(jpeg) + 8 > self._max_part_bytes:
            self._avi.close()
            part = self.path.with_name(f"{self.path.stem}_part{len(self.parts) + 1}{self.path.suffix}")
            self.parts.append(part)
            self._avi = MjpegAviWriter(part, self._width, self._height, self._fps)
        self._avi.write_jpeg(jpeg)
        self._slots += 1
