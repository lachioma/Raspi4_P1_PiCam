"""Checks for mjpeg_avi.py: the hand-written AVI container has to be readable by
OpenCV (i.e. ffmpeg), and the clip writer's time grid has to repeat/skip frames
the way it claims. Run on the Pi (needs cv2 and numpy):

    python3 mjpeg_avi_test.py          # plain, no pytest needed
    python3 -m pytest mjpeg_avi_test.py
"""

import tempfile
from pathlib import Path

import cv2
import numpy as np

from mjpeg_avi import JpegClipWriter, MjpegAviWriter

W, H = 64, 48


def _solid(value):
    return np.full((H, W, 3), value, np.uint8)


def _read_all(path):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(frame)
    cap.release()
    return frames, fps


def test_avi_writer_roundtrip():
    values = (0, 80, 160, 240, 120)
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "x.avi"
        writer = MjpegAviWriter(path, W, H, 10)
        for v in values:
            ok, buf = cv2.imencode(".jpg", _solid(v))
            assert ok
            writer.write_jpeg(buf.tobytes())
        writer.close()

        data = path.read_bytes()
        assert data[:4] == b"RIFF" and data[8:12] == b"AVI " and b"idx1" in data
        assert int.from_bytes(data[4:8], "little") == len(data) - 8   # RIFF size patched at close

        frames, fps = _read_all(path)
        assert len(frames) == len(values)
        assert frames[0].shape == (H, W, 3)
        assert abs(fps - 10) < 0.01
        for frame, v in zip(frames, values):                  # right frames, right order
            assert abs(float(frame.mean()) - v) < 8


def test_clip_writer_fills_gaps_and_skips_extras():
    fps = 10
    t0 = 1000.0
    # (offset, brightness). 0.505 lands in slot 5, so slots 3-4 must be filled by
    # repeating the last frame; 0.62 lands in the slot 0.6 already took, so it's skipped.
    stamps = [(0.0, 0), (0.1, 50), (0.2, 100), (0.505, 150), (0.6, 200), (0.62, 250)]
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "clip.avi"
        clip = JpegClipWriter(path, W, H, fps, quality=95, workers=2)
        for offset, v in stamps:
            assert clip.submit(t0 + offset, _solid(v))
        stats = clip.finish()

        assert stats["frames_written"] == 7
        assert stats["frames_duplicated"] == 2
        assert stats["frames_skipped"] == 1
        assert stats["frames_dropped"] == 0 and stats["encode_errors"] == 0
        assert stats["write_error"] is None

        frames, _ = _read_all(path)
        assert len(frames) == 7
        expected = [0, 50, 100, 100, 100, 150, 200]
        for frame, v in zip(frames, expected):
            assert abs(float(frame.mean()) - v) < 8
        n, seconds = clip.take_encode_stats()
        assert n == len(stamps) and seconds > 0


def test_clip_writer_rolls_into_parts():
    rng = np.random.default_rng(0)
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "clip.avi"
        clip = JpegClipWriter(path, W, H, 10, quality=90, workers=2, max_part_bytes=12_000)
        for i in range(6):
            noise = rng.integers(0, 256, (H, W, 3), dtype=np.uint8)   # ~several KB each
            clip.submit(1000.0 + i * 0.1, noise)
        stats = clip.finish()

        assert len(stats["parts"]) > 1
        assert stats["parts"][0] == "clip.avi" and stats["parts"][1] == "clip_part2.avi"
        total = sum(len(_read_all(Path(d) / name)[0]) for name in stats["parts"])
        assert total == 6 == stats["frames_written"]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
