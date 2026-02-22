#!/usr/bin/env python3
"""
Simple health checker for SO101 follower bus.

Checks periodically:
- voltage (raw register, default addr 62, 0.1V units on STS/SCS setups)
- torque enable register (default addr 40)
- present position range sanity (default addr 56)

By default this script reads the serial port directly, so run it when no other
process owns the bus. For controller-based integration, this checker should be
moved behind a single bus-owner service.
"""

import argparse
import json
import time
from pathlib import Path
from typing import Dict, List, Optional


def load_backend():
    import importlib.util

    p = Path(__file__).parent / "calibrate_endstops.py"
    spec = importlib.util.spec_from_file_location("cal", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_u16_safe(backend, sid: int, addr: int) -> Optional[int]:
    try:
        return backend.read_u16(sid, addr)
    except Exception:
        return None


def parse_range_map(text: str) -> Dict[int, List[int]]:
    # format: "1:700-3300,2:900-3200"
    out: Dict[int, List[int]] = {}
    if not text.strip():
        return out
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        sid_s, rng = part.split(":", 1)
        lo_s, hi_s = rng.split("-", 1)
        out[int(sid_s)] = [int(lo_s), int(hi_s)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--period-ms", type=int, default=500)

    ap.add_argument("--addr-pos", type=int, default=56)
    ap.add_argument("--addr-torque", type=int, default=40)
    ap.add_argument("--addr-voltage", type=int, default=62)

    ap.add_argument("--min-voltage", type=int, default=45, help="raw units (0.1V)")
    ap.add_argument(
        "--expect-torque",
        choices=["on", "off", "ignore"],
        default="ignore",
        help="expected torque state for all ids",
    )
    ap.add_argument(
        "--position-ranges",
        default="",
        help='per-id limits e.g. "1:700-3300,2:900-3200"',
    )
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    ranges = parse_range_map(args.position_ranges)

    mod = load_backend()
    b = mod.SerialBackend(args.port, args.baud)

    try:
        while True:
            ts = time.time()
            per_id = {}
            alerts = []

            for sid in args.ids:
                pos = read_u16_safe(b, sid, args.addr_pos)
                tq = read_u16_safe(b, sid, args.addr_torque)
                v_raw = read_u16_safe(b, sid, args.addr_voltage)
                v = None if v_raw is None else (v_raw & 0xFF)

                row = {"pos": pos, "torque": tq, "voltage": v, "voltage_raw": v_raw}
                per_id[sid] = row

                if v is None:
                    alerts.append(f"ID {sid}: no voltage read")
                elif v < args.min_voltage:
                    alerts.append(f"ID {sid}: low voltage {v} (<{args.min_voltage})")

                if args.expect_torque != "ignore" and tq is not None:
                    want = 1 if args.expect_torque == "on" else 0
                    if tq != want:
                        alerts.append(f"ID {sid}: torque={tq}, expected={want}")

                if sid in ranges and pos is not None:
                    lo, hi = ranges[sid]
                    if not (lo <= pos <= hi):
                        alerts.append(f"ID {sid}: pos={pos} outside [{lo},{hi}]")

            payload = {"ts": ts, "ids": per_id, "alerts": alerts}

            if args.json:
                print(json.dumps(payload, ensure_ascii=False))
            else:
                print(f"\n[{time.strftime('%H:%M:%S')}] health check")
                for sid in args.ids:
                    r = per_id[sid]
                    print(
                        f"ID {sid}: pos={r['pos']} tq={r['torque']} v={r['voltage']} (raw={r['voltage_raw']})"
                    )
                if alerts:
                    print("ALERTS:")
                    for a in alerts:
                        print(f"- {a}")
                else:
                    print("OK")

            if args.once:
                break

            time.sleep(max(0.05, args.period_ms / 1000.0))
    finally:
        b.close()


if __name__ == "__main__":
    main()
