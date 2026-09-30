"""Shared free-disk-space check.

Used independently by both the thermal segment writer and the RGB event
recorder, so a filling SD card is caught proactively by each writer before
it produces a truncated/corrupt file, rather than discovered later.
"""

import shutil


def free_space_mb(path) -> float:
    return shutil.disk_usage(path).free / (1024 * 1024)
