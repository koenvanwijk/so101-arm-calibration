#!/usr/bin/env python3
"""Reproducible guard test matrix runner (sim-only)."""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from guard_bambot_adapter import BambotSimControllerAdapter
from guard_pipeline import EndEffectorSafeInterface, GuardConfig, SafetyGuardPipeline


def run():
    out = []

    # Base config
    ids = [1, 2, 3, 4, 5, 6]
    ranges = {1: (700, 3300), 2: (900, 3200), 3: (900, 3100), 4: (900, 3150), 5: (200, 3900), 6: (1200, 3200)}

    # 1 nominal
    c1 = BambotSimControllerAdapter(ids)
    g1 = SafetyGuardPipeline(c1, GuardConfig(ids=ids, position_ranges=ranges))
    r1 = g1.apply_goal({2: 2000, 3: 2100}, now_s=time.time())
    out.append({"case": "nominal", "pass": r1.allowed and r1.applied == "command_applied", "result": r1.applied})

    # 2 range violation -> hold
    c2 = BambotSimControllerAdapter(ids)
    g2 = SafetyGuardPipeline(c2, GuardConfig(ids=ids, position_ranges=ranges))
    r2 = g2.apply_goal({2: 100}, now_s=time.time())
    out.append({"case": "range_violation", "pass": (not r2.allowed) and r2.applied == "hold"})

    # 3 low voltage -> hold
    c3 = BambotSimControllerAdapter(ids)
    c3.inject_voltage(3, 30)
    g3 = SafetyGuardPipeline(c3, GuardConfig(ids=ids, position_ranges=ranges, min_voltage_raw=45))
    r3 = g3.apply_goal({3: 2000}, now_s=time.time())
    out.append({"case": "low_voltage", "pass": (not r3.allowed) and any(a["code"] == "low_voltage" for a in r3.alerts)})

    # 4 comm drop -> hold
    c4 = BambotSimControllerAdapter(ids)
    c4.force_stale_timestamp(2.0)
    g4 = SafetyGuardPipeline(c4, GuardConfig(ids=ids, position_ranges=ranges, comm_drop_timeout_s=0.5))
    r4 = g4.apply_goal({4: 2000}, now_s=time.time())
    out.append({"case": "comm_drop", "pass": (not r4.allowed) and any(a["code"] == "comm_drop" for a in r4.alerts)})

    # 5 speed cap
    c5 = BambotSimControllerAdapter(ids)
    g5 = SafetyGuardPipeline(c5, GuardConfig(ids=ids, position_ranges=ranges, speed_cap_ticks_per_s=20.0))
    t0 = time.time()
    _ = g5.apply_goal({5: 2000}, now_s=t0)
    r5 = g5.apply_goal({5: 3000}, now_s=t0 + 0.05)
    out.append({"case": "speed_cap", "pass": (not r5.allowed) and any(a["code"] == "speed_violation" for a in r5.alerts)})

    # 6 end effector routed through guard
    c6 = BambotSimControllerAdapter(ids)
    g6 = SafetyGuardPipeline(c6, GuardConfig(ids=ids, position_ranges=ranges))
    eef = EndEffectorSafeInterface(g6, eef_id=6, min_tick=1200, max_tick=3200)
    r6 = eef.close(now_s=time.time())
    out.append({"case": "eef_safe_api", "pass": r6.allowed and r6.applied == "command_applied"})

    ok = all(x["pass"] for x in out)
    print(json.dumps({"all_pass": ok, "matrix": out}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(run())
