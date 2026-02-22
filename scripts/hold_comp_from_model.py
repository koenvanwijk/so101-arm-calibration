#!/usr/bin/env python3
"""
Prototype gravity-hold compensation from SO101 URDF model.

This is an initial practical approximation for safety hold mode:
- compute simple gravity moment terms for shoulder/elbow/wrist_pitch chain
- output small per-joint bias ticks to keep pose from sagging

Not a full rigid-body dynamics solver; intended as conservative feedforward bias.
"""

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

TICKS = 4096


def tick_to_rad(t: int) -> float:
    return (2.0 * math.pi) * (t % TICKS) / TICKS


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class CompGains:
    shoulder: float = 42.0
    elbow: float = 34.0
    wrist_pitch: float = 18.0


@dataclass
class CompLimits:
    shoulder: int = 120
    elbow: int = 100
    wrist_pitch: int = 70


def compute_bias_ticks(q2: float, q3: float, q4: float, gains: CompGains, lim: CompLimits) -> Dict[int, int]:
    """
    q2/q3/q4 = shoulder pitch / elbow / wrist_pitch angles in radians.

    Simple gravitational terms (planar approximation):
      tau2 ~ sin(q2) + 0.7*sin(q2+q3) + 0.35*sin(q2+q3+q4)
      tau3 ~ sin(q2+q3) + 0.45*sin(q2+q3+q4)
      tau4 ~ sin(q2+q3+q4)

    Convert tau-ish to small bias ticks via gain and clamp.
    """
    s2 = math.sin(q2)
    s23 = math.sin(q2 + q3)
    s234 = math.sin(q2 + q3 + q4)

    b2 = int(round(clamp(gains.shoulder * (s2 + 0.7 * s23 + 0.35 * s234), -lim.shoulder, lim.shoulder)))
    b3 = int(round(clamp(gains.elbow * (s23 + 0.45 * s234), -lim.elbow, lim.elbow)))
    b4 = int(round(clamp(gains.wrist_pitch * s234, -lim.wrist_pitch, lim.wrist_pitch)))

    # SO101 ids: 2=shoulder_lift, 3=elbow, 4=wrist_pitch
    return {2: b2, 3: b3, 4: b4}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id2", type=int, required=True, help="current raw tick ID2 shoulder_lift")
    ap.add_argument("--id3", type=int, required=True, help="current raw tick ID3 elbow")
    ap.add_argument("--id4", type=int, required=True, help="current raw tick ID4 wrist_pitch")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    q2 = tick_to_rad(args.id2)
    q3 = tick_to_rad(args.id3)
    q4 = tick_to_rad(args.id4)

    bias = compute_bias_ticks(q2, q3, q4, CompGains(), CompLimits())

    if args.json:
        import json

        print(json.dumps({"bias_ticks": bias, "angles_rad": {"q2": q2, "q3": q3, "q4": q4}}, ensure_ascii=False))
    else:
        print("Hold compensation bias ticks (prototype):")
        for sid in (2, 3, 4):
            print(f"ID{sid}: {bias[sid]:+d}")


if __name__ == "__main__":
    main()
