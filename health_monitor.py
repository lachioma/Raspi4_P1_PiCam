"""Heat and power indicators for the periodic stats lines, so a multi-day or
multi-week run can be checked afterwards for the Pi slowing down because it got
hot or because the supply sagged - instead of guessing from fps.

Everything is read from sysfs / procfs / the firmware (a handful of tiny file
reads a minute), and any source that isn't there on a given system just comes
back None / empty, so this is safe to call anywhere.

What is reported (one dict per sample, ready to splat into StatsLogger.log):

  cpu_temp_c            SoC temperature. The Pi 4 starts throttling at 80C
                        (hard limit 85C). On a Pi 4 the "GPU" temperature
                        `vcgencmd measure_temp` shows is this same sensor.
  other_temps_c         any further hwmon temperature sensors the kernel
                        exposes (e.g. a USB/NVMe SSD, a PoE HAT), by name.
  fans_rpm              any hwmon fan tachometers (official Pi fan/case).
  cpu_freq_mhz          current clock of core 0, and cpu_freq_max_mhz its
                        ceiling. One sample is only where the frequency
                        governor happens to be that instant - an idle Pi
                        bounces between ~600 and the maximum (readings of
                        900, 1000 and 1800 MHz seconds apart at 54C are
                        normal) - so this says nothing about throttling on
                        its own. It matters only as a clock *stuck* below
                        max while load_1m is high; the flags below are the
                        reliable throttling indicator.
  throttled_now /
  throttled_since_boot  decoded `get_throttled` flags: under_voltage (supply
                        sagging - often mistaken for heat), freq_capped,
                        throttled, soft_temp_limit. "Since boot" latches, so a
                        single glitch between two samples is still visible.
  throttled             the raw hex bitmask behind those two ("0x0" = clean).
  load_1m               1-minute load average (a few cores' worth is fine on a
                        4-core Pi; sustained >4 means the CPU is saturated).
  proc_cpu_pct          this process's CPU use over the sample interval, all
                        threads summed (100 = one core fully busy).

A [warning] line is also printed, once per episode, when the firmware reports an
active throttle/under-voltage flag or the SoC reaches 80C - so someone watching
a console sees it as it starts rather than only in the log afterwards.

Not reported: the P1 camera's own internal temperature. Its frame carries two
metadata rows, but their layout isn't documented in p3_camera.py/P3_PROTOCOL.md
and guessing at it would risk logging a plausible-looking wrong number.
"""

import subprocess
import time
from pathlib import Path

CPU_TEMP_WARN_C = 80.0

_THERMAL_ZONE = Path("/sys/class/thermal/thermal_zone0/temp")
_THROTTLED_SYSFS = Path("/sys/devices/platform/soc/soc:firmware/get_throttled")
_CPUFREQ = Path("/sys/devices/system/cpu/cpu0/cpufreq")
_HWMON = Path("/sys/class/hwmon")
_LOADAVG = Path("/proc/loadavg")

# get_throttled bits: 0-3 are "right now", 16-19 the same events "since boot".
_THROTTLE_FLAGS = {0: "under_voltage", 1: "freq_capped", 2: "throttled", 3: "soft_temp_limit"}


def _read_int(path: Path):
    try:
        return int(path.read_text().strip(), 0)
    except (OSError, ValueError):
        return None


def _read_throttled():
    """The firmware's throttle bitmask as an int, or None."""
    value = _read_int(_THROTTLED_SYSFS)
    if value is not None:
        return value
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2
        ).stdout.strip()  # "throttled=0x0"
        return int(out.split("=", 1)[1], 0) if "=" in out else None
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def _read_hwmon():
    """Temperatures (C) and fan speeds (rpm) from every hwmon device except the CPU's
    own thermal sensor, which is reported separately as cpu_temp_c."""
    temps, fans = {}, {}
    if not _HWMON.is_dir():
        return temps, fans
    for hw in sorted(_HWMON.glob("hwmon*")):
        try:
            name = (hw / "name").read_text().strip()
        except OSError:
            continue
        if name == "cpu_thermal":
            continue
        for f in sorted(hw.glob("temp*_input")):
            value = _read_int(f)
            if value is not None:
                temps[f"{name}:{f.name[:-6]}"] = round(value / 1000.0, 1)
        for f in sorted(hw.glob("fan*_input")):
            value = _read_int(f)
            if value is not None:
                fans[f"{name}:{f.name[:-6]}"] = value
    return temps, fans


def _decode(mask, offset):
    return [name for bit, name in _THROTTLE_FLAGS.items() if (mask >> (bit + offset)) & 1]


class HealthMonitor:
    """One per capture loop; call sample() at each stats interval."""

    def __init__(self, tag: str):
        self.tag = tag
        self._last_cpu = time.process_time()
        self._last_wall = time.monotonic()
        self._in_warning = False

    def sample(self) -> dict:
        temp_raw = _read_int(_THERMAL_ZONE)
        cpu_temp_c = round(temp_raw / 1000.0, 1) if temp_raw is not None else None
        other_temps, fans = _read_hwmon()

        freq = _read_int(_CPUFREQ / "scaling_cur_freq")
        freq_max = _read_int(_CPUFREQ / "cpuinfo_max_freq")

        mask = _read_throttled()
        throttled_now = _decode(mask, 0) if mask is not None else None
        throttled_ever = _decode(mask, 16) if mask is not None else None

        load_1m = None
        try:
            load_1m = float(_LOADAVG.read_text().split()[0])
        except (OSError, ValueError, IndexError):
            pass

        cpu, wall = time.process_time(), time.monotonic()
        proc_cpu_pct = round(100.0 * (cpu - self._last_cpu) / (wall - self._last_wall), 1) \
            if wall > self._last_wall else None
        self._last_cpu, self._last_wall = cpu, wall

        hot = cpu_temp_c is not None and cpu_temp_c >= CPU_TEMP_WARN_C
        if (throttled_now or hot) and not self._in_warning:
            print(f"[{self.tag}] [warning] heat/power: cpu {cpu_temp_c}C, "
                  f"firmware flags now: {throttled_now or 'none'} "
                  f"(since boot: {throttled_ever or 'none'}).")
        self._in_warning = bool(throttled_now) or hot

        return {
            "cpu_temp_c": cpu_temp_c,
            "other_temps_c": other_temps,
            "fans_rpm": fans,
            "cpu_freq_mhz": round(freq / 1000) if freq is not None else None,
            "cpu_freq_max_mhz": round(freq_max / 1000) if freq_max is not None else None,
            "throttled": hex(mask) if mask is not None else None,
            "throttled_now": throttled_now,
            "throttled_since_boot": throttled_ever,
            "load_1m": load_1m,
            "proc_cpu_pct": proc_cpu_pct,
        }

    @staticmethod
    def brief(health: dict) -> str:
        """Short form for the console stats line."""
        return (f"{health['cpu_temp_c']}C, {health['cpu_freq_mhz']}MHz, "
                f"throttled={health['throttled']}, load {health['load_1m']}, "
                f"proc {health['proc_cpu_pct']}% CPU")
