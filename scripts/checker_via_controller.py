#!/usr/bin/env python3
"""
Health checker using sts-scs-motor-controller queue/control-loop (single bus owner).
"""

import argparse
import json
import time


def parse_range_map(text: str):
    out = {}
    if not text.strip():
        return out
    for part in text.split(","):
        sid_s, rng = part.strip().split(":", 1)
        lo_s, hi_s = rng.split("-", 1)
        out[int(sid_s)] = (int(lo_s), int(hi_s))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--period-ms", type=int, default=500)
    ap.add_argument("--min-voltage", type=int, default=45)
    ap.add_argument("--expect-torque", choices=["on", "off", "ignore"], default="ignore")
    ap.add_argument("--position-ranges", default="")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    ranges = parse_range_map(args.position_ranges)

    from sts_scs_motor_controller_py import FeetechPyController

    ctrl = FeetechPyController.new_sts3215(args.port, args.ids, args.baud, 20)
    idx = {sid: i for i, sid in enumerate(args.ids)}

    try:
        # warm-up for first snapshot
        time.sleep(0.05)
        while True:
            positions, voltages, torques, ts = ctrl.get_last_health()
            alerts = []
            per = {}

            for sid in args.ids:
                i = idx[sid]
                pos = int(round(positions[i])) if i < len(positions) else None
                vol = int(voltages[i]) if i < len(voltages) else None
                tq = torques[i] if i < len(torques) else None
                per[sid] = {"pos": pos, "voltage": vol, "torque": tq}

                if vol is None or vol == 0:
                    alerts.append(f"ID {sid}: no/invalid voltage ({vol})")
                elif vol < args.min_voltage:
                    alerts.append(f"ID {sid}: low voltage {vol} (<{args.min_voltage})")

                if args.expect_torque != "ignore" and tq is not None:
                    want = args.expect_torque == "on"
                    if tq != want:
                        alerts.append(f"ID {sid}: torque={tq}, expected={want}")

                if sid in ranges and pos is not None:
                    lo, hi = ranges[sid]
                    if not (lo <= pos <= hi):
                        alerts.append(f"ID {sid}: pos={pos} outside [{lo},{hi}]")

            payload = {"ts": ts, "ids": per, "alerts": alerts}

            if args.json:
                print(json.dumps(payload, ensure_ascii=False))
            else:
                print(f"\n[{time.strftime('%H:%M:%S')}] health via controller")
                for sid in args.ids:
                    r = per[sid]
                    print(f"ID {sid}: pos={r['pos']} tq={r['torque']} v={r['voltage']}")
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
        ctrl.close()


if __name__ == "__main__":
    main()
