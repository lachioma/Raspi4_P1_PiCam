"""Diagnostic: when does the P1 fire its shutter (FFC), and what can we see of it?

Run on the Pi with the field recorder / stream server STOPPED (only one process
can own the camera):

    python3 ffc_probe.py --minutes 6

While streaming it does two things at once and cross-checks them:

1. Polls the camera's own debug log (the extended status register, which
   P3_PROTOCOL.md says carries lines like "[91403] I/shutter: === Shutter close ===")
   and records every new line with the Pi's wall clock and the camera's own tick.
   `read_debug_log()` exists in p3_camera.py but nothing in this project has used
   it yet, so the first thing the run tells us is whether the log is readable
   mid-stream at all (the first raw buffer is saved in the output for inspection).
2. Watches the mean brightness of every frame for the one-frame drop the FFC
   causes (measured earlier in the hour-long recording: about -6 to -11 grey
   levels, recovering over ~1 s).

At the end it prints the shutter lines' spacing (from the camera's own ticks, so
USB timing doesn't matter) and how well the log lines line up with the
brightness steps. Everything is also written to --out (JSON lines).

Read-only by default. `--scan-registers` additionally reads register ids with the
same 0x0101 "read register" command the driver already uses for the model name
and serial, before streaming starts, to look for anything that reports or holds
an FFC setting. It never writes. Nothing here sends the shutter command or any
other write; changing or disabling the FFC needs a command we don't have yet
(see the README's FFC section), and sending guesses to the camera is a separate,
deliberate step.
"""

import argparse
import json
import re
import statistics
import struct
import time
from pathlib import Path

import usb.core

from p3_camera import (
    COMMANDS,
    FrameMarkerMismatchError,
    Model,
    P3Camera,
    crc16_ccitt,
    get_model_config,
)

# "[91403] I/shutter: === Shutter close ===": a camera tick in brackets, a level
# letter and slash, then text, up to the next "[tick] X/" or the end.
LOG_RE = re.compile(r"\[(\d+)\]\s+[A-Z]/.*?(?=\[\d+\]\s+[A-Z]/|$)")
STEP_DROP = 4.0         # per-frame drop in mean brightness counted as an FFC candidate
MATCH_WINDOW_S = 3.0    # a log line and a brightness step this close are "the same" FFC


def read_command(cmd_type: int, reg: int, resp_len: int, param: int = 0x0081) -> bytes:
    """18-byte read-register command, laid out as in P3_PROTOCOL.md (response length
    at offset 12). p3_camera.build_command() puts it at offset 14, which doesn't
    reproduce the shipped commands, so it isn't used here."""
    payload = struct.pack("<HHH6xHH", cmd_type, param, reg, resp_len, 0)
    return payload + struct.pack("<H", crc16_ccitt(payload))


def poll_log(camera: P3Camera, length: int):
    """One read of the extended status register -> (raw bytes, parsed log lines)."""
    data = bytes(camera.dev.ctrl_transfer(0xC1, 0x22, 0, 0, length, 1000))
    text = "".join(chr(b) if 32 <= b <= 126 else " " for b in data[1:])   # byte 0 is the status
    lines = []
    for m in LOG_RE.finditer(text):
        line = " ".join(m.group(0).split())
        lines.append((int(m.group(1)), line))
    return data, lines


def scan_registers(camera: P3Camera, lo: int, hi: int, length: int) -> None:
    mine = read_command(0x0101, 0x01, 30)
    if mine != COMMANDS["read_name"]:
        print("[scan] note: my read-register command differs from COMMANDS['read_name'] "
              f"({mine.hex()} vs {COMMANDS['read_name'].hex()}). The camera doesn't check "
              "CRCs, so this is only a cosmetic difference if the first reads below work.")
    print(f"[scan] reading registers 0x{lo:02x}-0x{hi:02x} ({length} bytes each, read-only)")
    for reg in range(lo, hi + 1):
        try:
            camera._send_command(read_command(0x0101, reg, length))
            camera._read_status()
            data = bytes(camera._read_response(length))
            camera._read_status()
        except usb.core.USBError as e:
            print(f"  reg 0x{reg:02x}: USB error {e}")
            continue
        if any(data):
            trimmed = data.rstrip(b"\x00")
            text = "".join(chr(b) if 32 <= b <= 126 else "." for b in trimmed)
            print(f"  reg 0x{reg:02x}: {trimmed.hex()}  |{text}|")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default=Model.P1)
    p.add_argument("--minutes", type=float, default=6.0,
                   help="How long to stream and watch. The FFC fires about every 90s, so 6 min "
                        "gives ~4 cycles (default 6).")
    p.add_argument("--poll-every-frames", type=int, default=5,
                   help="Poll the debug log once per this many frames (default 5, ~5x per second).")
    p.add_argument("--log-bytes", type=int, default=128,
                   help="Length of each extended-status read (default 128, as in read_debug_log).")
    p.add_argument("--out", default="ffc_probe.jsonl")
    p.add_argument("--scan-registers", action="store_true",
                   help="Read-only scan of register ids before streaming (see the module docstring).")
    p.add_argument("--scan-min", type=lambda s: int(s, 0), default=0x00)
    p.add_argument("--scan-max", type=lambda s: int(s, 0), default=0x3F)
    p.add_argument("--scan-length", type=int, default=64)
    args = p.parse_args()

    camera = P3Camera(config=get_model_config(args.model))
    camera.connect()
    name, version = camera.init()
    print(f"[probe] connected to {name} (firmware {version})")

    if args.scan_registers:
        scan_registers(camera, args.scan_min, args.scan_max, args.scan_length)

    camera.start_streaming()
    out = open(args.out, "w", buffering=1)

    def emit(event: dict) -> None:
        out.write(json.dumps(event) + "\n")

    t_start = time.time()
    deadline = t_start + args.minutes * 60
    seen = set()
    log_events = []     # (wall time, camera tick, line)
    steps = []          # (wall time, frame, drop)
    poll_errors = 0
    frame_errors = 0
    frame = 0
    prev_mean = None
    raw_saved = False

    print(f"[probe] streaming for {args.minutes:g} min; log output below appears as it is seen")
    try:
        while time.time() < deadline:
            try:
                ir_brightness, _ = camera.read_frame_both()
            except (usb.core.USBError, FrameMarkerMismatchError):
                frame_errors += 1
                continue
            if ir_brightness is None:
                frame_errors += 1
                continue
            frame += 1
            now = time.time()

            mean = float(ir_brightness.mean())
            if prev_mean is not None and mean - prev_mean <= -STEP_DROP:
                steps.append((now, frame, mean - prev_mean))
                emit({"type": "brightness_step", "t": now, "frame": frame, "drop": round(mean - prev_mean, 2)})
                print(f"[step] frame {frame}: mean brightness {mean - prev_mean:+.1f}")
            prev_mean = mean

            if poll_errors < 5 and frame % args.poll_every_frames == 0:
                try:
                    data, lines = poll_log(camera, args.log_bytes)
                except usb.core.USBError as e:
                    poll_errors += 1
                    print(f"[probe] log poll failed ({e}); {5 - poll_errors} tries left before I stop polling")
                    continue
                if not raw_saved:
                    emit({"type": "first_raw_log_buffer", "t": now, "hex": data.hex()})
                    raw_saved = True
                for tick, line in lines:
                    if line in seen:
                        continue
                    seen.add(line)
                    log_events.append((now, tick, line))
                    emit({"type": "log", "t": now, "frame": frame, "tick": tick, "line": line})
                    print(f"[log] {line}")
    except KeyboardInterrupt:
        print("\n[probe] interrupted, summarising what was seen")
    finally:
        out.close()
        try:
            camera.disconnect()
        except Exception:
            pass

    # ---- summary ----
    elapsed = time.time() - t_start
    print(f"\n=== summary ({elapsed:.0f}s, {frame} frames, {frame_errors} frame errors, "
          f"{poll_errors} log-poll errors) ===")
    print(f"debug-log lines seen: {len(log_events)}   brightness steps seen: {len(steps)}")
    if not log_events:
        print("No debug-log lines were returned. Either the log isn't readable while streaming or "
              "its buffer is empty between messages; check 'first_raw_log_buffer' in the output file.")

    shutter = [e for e in log_events if "shutter" in e[2].lower()]
    print(f"shutter-related log lines: {len(shutter)}")
    for _, _, line in shutter[:12]:
        print(f"  {line}")
    ticks = sorted({tick for _, tick, line in shutter if "close" in line.lower()})
    if len(ticks) >= 2:
        gaps = [(b - a) / 1000.0 for a, b in zip(ticks, ticks[1:])]   # assumes ticks are ms
        print(f"camera-clock spacing of 'Shutter close' lines (if ticks are ms): "
              f"median {statistics.median(gaps):.1f}s, min {min(gaps):.1f}s, max {max(gaps):.1f}s")
    if len(steps) >= 2:
        gaps = [b[0] - a[0] for a, b in zip(steps, steps[1:]) if b[0] - a[0] > 30]
        if gaps:
            print(f"wall-clock spacing of brightness steps (>30s apart only): median "
                  f"{statistics.median(gaps):.1f}s")
    if shutter and steps:
        matched = sum(any(abs(s[0] - e[0]) <= MATCH_WINDOW_S for e in shutter) for s in steps)
        print(f"brightness steps within {MATCH_WINDOW_S:g}s of a shutter log line: {matched}/{len(steps)}")
    print(f"details: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
