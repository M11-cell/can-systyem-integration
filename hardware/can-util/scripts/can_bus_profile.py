#!/usr/bin/env python3
"""
Parse candump output and estimate per-traffic-class CAN bus utilization.

Usage:
  # Live (requires candump from can-utils):
  candump -ta can0 | ./can_bus_profile.py --bitrate 500000 --duration 30

  # From a saved log:
  ./can_bus_profile.py --log capture.log --bitrate 500000

  # Spawn candump automatically:
  ./can_bus_profile.py --interface can0 --bitrate 500000 --duration 30

candump line examples:
  (1700000000.123456) can0 0205B801#AABBCCDDEEFF0011
  (1700000000.123456) can0  0205B801   [8]  AA BB CC DD EE FF 00 11
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Iterable, Iterator, TextIO

# Nominal extended CAN 2.0B data frame bit count (no bit stuffing).
def nominal_frame_bits(dlc: int) -> int:
    dlc = max(0, min(8, dlc))
    return 1 + 29 + 6 + (8 * dlc) + 15 + 11


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF


def decode_arb_id(raw: int) -> int:
    """Strip SocketCAN EFF/RTR flags; return 29- or 11-bit arbitration field."""
    if raw & CAN_EFF_FLAG:
        return raw & CAN_EFF_MASK
    # candump often prints 29-bit IDs zero-padded without setting bit 31
    if raw > CAN_SFF_MASK:
        return raw & CAN_EFF_MASK
    return raw & CAN_SFF_MASK


# --- ID → traffic class (29-bit arbitration values; matches prefixes.hpp) ---

SPARK_STATUS0_TYPE = 0x0205B800
SPARK_STATUS1_TYPE = 0x0205B840
SPARK_STATUS2_TYPE = 0x0205B880
SPARK_LEGACY_PERIOD0_TYPE = 0x02051800
SPARK_LEGACY_PERIOD1_TYPE = 0x02051840
SPARK_LEGACY_PERIOD2_TYPE = 0x02051880
SPARK_LEGACY_PERIOD3_TYPE = 0x020518C0
SPARK_LEGACY_PERIOD4_TYPE = 0x02051900
SPARK_SET_STATUSES_TYPE = 0x02050400
SPARK_REQUEST_STATUS_TYPE = 0x020502C0

# Wheel RPM command: (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (device_id + 0x80)
# see system_controller.cpp — api class 1 index 2, device field includes 0x80.
WHEEL_VEL_TYPE = 0x02050480
# Wheel current (torque) command: COMMAND_PREFIX_CURRENT_CONTROL | device_id.
WHEEL_CURRENT_TYPE = 0x020514C0
SPARK_FRAME_TYPE_MASK = 0x1FFFFFC0
WHEEL_MAINTAIN_ID = 0x02052C80  # COMMAND_PREFIX_MAINTAIN_VELOCITY & CAN_EFF_MASK

SERVO_PREFIX = 0x0C08C000
SERVO_MASK = 0xFFFFC000

ENCODER_ABS_SPEED_PREFIXES = (
    0x0108C700,
    0x0108C800,
    0x0108C900,
    0x0108CB00,
)
ENCODER_MASK = 0xFFFFFF00


def classify(arb_id: int) -> str:
    """Map 29-bit arbitration ID to a human-readable traffic bucket."""
    devtype = (arb_id >> 24) & 0x1F
    mfr = (arb_id >> 16) & 0xFF

    frame_type = arb_id & SPARK_FRAME_TYPE_MASK
    if frame_type == WHEEL_VEL_TYPE:
        return "wheel_cmd_velocity"
    if frame_type == WHEEL_CURRENT_TYPE:
        return "wheel_cmd_current"
    if arb_id == WHEEL_MAINTAIN_ID:
        return "wheel_cmd_maintain_mask"

    if devtype == 0x02 and mfr == 0x05:
        if frame_type == SPARK_STATUS0_TYPE:
            return "spark_status_0"
        if frame_type == SPARK_STATUS1_TYPE:
            return "spark_status_1"
        if frame_type == SPARK_STATUS2_TYPE:
            return "spark_status_2"
        if frame_type == SPARK_LEGACY_PERIOD0_TYPE:
            return "spark_legacy_period0"
        if frame_type == SPARK_LEGACY_PERIOD1_TYPE:
            return "spark_legacy_period1"
        if frame_type == SPARK_LEGACY_PERIOD2_TYPE:
            return "spark_legacy_period2"
        if frame_type == SPARK_LEGACY_PERIOD3_TYPE:
            return "spark_legacy_period3"
        if frame_type == SPARK_LEGACY_PERIOD4_TYPE:
            return "spark_legacy_period4"
        if frame_type == SPARK_SET_STATUSES_TYPE:
            return "spark_cmd_set_statuses"
        if frame_type == SPARK_REQUEST_STATUS_TYPE:
            return "spark_cmd_request_status"
        return "spark_other"

    if devtype == 0x10 and mfr == 0x81:
        return "arm_motor_cmd"

    if (arb_id & SERVO_MASK) == SERVO_PREFIX:
        return "servo_cmd"

    for prefix in ENCODER_ABS_SPEED_PREFIXES:
        if (arb_id & ENCODER_MASK) == prefix:
            return "arm_encoder_telemetry"

    if devtype == 0x00 and mfr == 0x08:
        return "bab_telemetry"

    # SIL / compat SCRB motor + safety IDs (Compat-README.md)
    if arb_id in {
        0x0001808C,
        0x0001810C,
        0x0001848C,
        0x0001850C,
        0x0001858C,
        0x0001860C,
        0x0001868C,
    }:
        return "sil_compat_cmd"

    return "other"


def rollup(category: str) -> str:
    if category.startswith("spark_"):
        return "spark_total"
    if category.startswith("wheel_cmd"):
        return "wheel_cmd_total"
    if category == "arm_motor_cmd":
        return "arm_cmd_total"
    return category


@dataclass
class Stats:
    frames: int = 0
    bits: int = 0
    by_id: dict[int, int] = field(default_factory=lambda: defaultdict(int))

    def add(self, arb_id: int, dlc: int) -> None:
        self.frames += 1
        self.bits += nominal_frame_bits(dlc)
        self.by_id[arb_id] += 1


# candump: (ts) can0 12345678#AABBCCDD  or  (ts) can0 12345678 [8] AA BB ...
RE_HASH = re.compile(
    r"^\([^)]+\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)",
)
RE_BRACKET = re.compile(
    r"^\([^)]+\)\s+\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]",
)


def parse_line(line: str) -> tuple[int, int] | None:
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    m = RE_HASH.match(line)
    if m:
        arb = decode_arb_id(int(m.group(1), 16))
        hex_data = m.group(2)
        dlc = len(hex_data) // 2
        return arb, dlc
    m = RE_BRACKET.match(line)
    if m:
        arb = decode_arb_id(int(m.group(1), 16))
        dlc = int(m.group(2), 10)
        return arb, dlc
    return None


def iter_lines(source: Iterable[str], duration_s: float | None) -> Iterator[str]:
    t0 = time.monotonic()
    for line in source:
        if duration_s is not None and (time.monotonic() - t0) >= duration_s:
            break
        yield line


def run_analysis(
    lines: Iterable[str],
    bitrate: int,
    duration_s: float | None,
    top_ids: int,
) -> int:
    by_class: dict[str, Stats] = defaultdict(Stats)
    by_rollup: dict[str, Stats] = defaultdict(Stats)
    t_first: float | None = None
    t_last: float | None = None
    ts_re = re.compile(r"^\(([0-9.]+)\)")

    for line in iter_lines(lines, duration_s):
        m_ts = ts_re.match(line)
        if m_ts:
            ts = float(m_ts.group(1))
            if t_first is None:
                t_first = ts
            t_last = ts

        parsed = parse_line(line)
        if parsed is None:
            continue
        arb_id, dlc = parsed
        cat = classify(arb_id)
        by_class[cat].add(arb_id, dlc)
        by_rollup[rollup(cat)].add(arb_id, dlc)

    if t_first is not None and t_last is not None and t_last > t_first:
        window_s = t_last - t_first
    elif duration_s is not None:
        window_s = duration_s
    else:
        window_s = 1.0

    total_bits = sum(s.bits for s in by_class.values())
    total_fps = sum(s.frames for s in by_class.values()) / window_s
    total_bps = total_bits / window_s
    total_pct = 100.0 * total_bps / bitrate if bitrate else 0.0

    print(f"Window: {window_s:.2f} s  |  Bitrate: {bitrate/1000:.0f} kbit/s (nominal, no stuffing)")
    print(f"Total:  {total_fps:8.1f} fps  |  {total_bps/1000:8.1f} kbit/s  |  {total_pct:5.1f}% bus")
    print()
    print(f"{'Category':<28} {'fps':>8} {'kbit/s':>10} {'bus %':>7} {'frames':>8}")
    print("-" * 65)

    rows = sorted(by_class.items(), key=lambda kv: kv[1].bits, reverse=True)
    for cat, st in rows:
        fps = st.frames / window_s
        bps = st.bits / window_s
        pct = 100.0 * bps / bitrate if bitrate else 0.0
        print(f"{cat:<28} {fps:8.1f} {bps/1000:10.1f} {pct:7.1f} {st.frames:8d}")

    print()
    print("Roll-up groups:")
    print(f"{'Group':<28} {'fps':>8} {'kbit/s':>10} {'bus %':>7}")
    print("-" * 55)
    for grp, st in sorted(by_rollup.items(), key=lambda kv: kv[1].bits, reverse=True):
        fps = st.frames / window_s
        bps = st.bits / window_s
        pct = 100.0 * bps / bitrate if bitrate else 0.0
        print(f"{grp:<28} {fps:8.1f} {bps/1000:10.1f} {pct:7.1f}")

    if top_ids > 0:
        all_ids: dict[int, int] = defaultdict(int)
        for st in by_class.values():
            for aid, cnt in st.by_id.items():
                all_ids[aid] += cnt
        print()
        print(f"Top {top_ids} arbitration IDs by frame count:")
        print(f"{'CAN ID':>12} {'class':<24} {'fps':>8} {'frames':>8}")
        print("-" * 56)
        for aid, cnt in sorted(all_ids.items(), key=lambda kv: kv[1], reverse=True)[:top_ids]:
            fps = cnt / window_s
            print(f"{'0x'+format(aid, '08X'):>12} {classify(aid):<24} {fps:8.1f} {cnt:8d}")

    print()
    print("Note: percentages omit CAN bit stuffing (~10–20% on real traffic).")
    print("      Compare A/B captures with the same window length and stack config.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Profile CAN bus load from candump output.")
    parser.add_argument("--bitrate", type=int, default=500_000, help="Bus bitrate (default: 500000)")
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds (live/stdin)")
    parser.add_argument("--log", type=str, default=None, help="Read candump log file instead of stdin")
    parser.add_argument("--interface", type=str, default=None, help="Run candump -ta on this interface")
    parser.add_argument("--top-ids", type=int, default=15, help="Show top N IDs (0 to disable)")
    args = parser.parse_args()

    if args.interface:
        cmd = ["candump", "-ta", args.interface]
        if args.duration is None:
            args.duration = 30.0
        print(f"Running: {' '.join(cmd)} for {args.duration:.0f} s ...", file=sys.stderr)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
        assert proc.stdout is not None
        try:
            return run_analysis(proc.stdout, args.bitrate, args.duration, args.top_ids)
        finally:
            proc.terminate()
            proc.wait(timeout=2)

    if args.log:
        with open(args.log, encoding="utf-8", errors="replace") as f:
            return run_analysis(f, args.bitrate, args.duration, args.top_ids)

    if sys.stdin.isatty():
        parser.print_help()
        print("\nPipe candump in, e.g.:  candump -ta can0 | ./can_bus_profile.py --duration 30", file=sys.stderr)
        return 1

    return run_analysis(sys.stdin, args.bitrate, args.duration, args.top_ids)


if __name__ == "__main__":
    sys.exit(main())
