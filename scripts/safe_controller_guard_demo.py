#!/usr/bin/env python3
"""Demo CLI: send guarded joint commands through controller.

Example:
python3 scripts/safe_controller_guard_demo.py \
  --port /dev/tty_white_follower_so101 --ids 1 2 3 4 5 6 \
  --position-ranges "1:700-3300,2:900-3200,3:900-3100,4:900-3150,5:200-3900,6:1200-3200" \
  --set "2:2100,3:2200"
"""

import argparse
import json
import time


def parse_ranges(text: str):
    out = {}
    if not text.strip():
        return out
    for part in text.split(","):
        sid_s, rng = part.strip().split(":", 1)
        lo_s, hi_s = rng.split("-", 1)
        out[int(sid_s)] = (int(lo_s), int(hi_s))
    return out


def parse_set(text: str):
    out = {}
    for part in text.split(","):
        sid_s, pos_s = part.strip().split(":", 1)
        out[int(sid_s)] = int(pos_s)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, required=True)
    ap.add_argument("--position-ranges", required=True)
    ap.add_argument("--set", required=True, dest="set_values")
    ap.add_argument("--speed-cap", type=float, default=120.0)
    ap.add_argument("--min-voltage", type=int, default=45)
    ap.add_argument("--comm-timeout", type=float, default=0.8)
    ap.add_argument("--fault-policy", choices=["hold", "torque_off"], default="hold")
    args = ap.parse_args()

    from sts_scs_motor_controller_py import FeetechPyController
    from guard_pipeline import GuardConfig, SafetyGuardPipeline

    ctrl = FeetechPyController.new_sts3215(args.port, args.ids, args.baud, 20)
    try:
        guard = SafetyGuardPipeline(
            ctrl,
            GuardConfig(
                ids=args.ids,
                position_ranges=parse_ranges(args.position_ranges),
                speed_cap_ticks_per_s=args.speed_cap,
                min_voltage_raw=args.min_voltage,
                comm_drop_timeout_s=args.comm_timeout,
                on_fault=args.fault_policy,
            ),
        )
        result = guard.apply_goal(parse_set(args.set_values), now_s=time.time())
        print(json.dumps({"allowed": result.allowed, "applied": result.applied, "alerts": result.alerts}, indent=2))
    finally:
        ctrl.close()


if __name__ == "__main__":
    main()
