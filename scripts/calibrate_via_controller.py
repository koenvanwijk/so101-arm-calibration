#!/usr/bin/env python3
"""
SO101 endstop calibration via sts-scs-motor-controller queue/control-loop.

This avoids direct serial ownership in calibration scripts so other clients can
observe state concurrently through the same controller process/API.

Current version uses position-only stall detection (no load register reads).
"""

import argparse
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

TICKS = 4096


@dataclass
class ServoResult:
    servo_id: int
    min_stop: Optional[int]
    max_stop: Optional[int]
    midpoint: Optional[int]


def wrap_tick(v: int, mod: int = TICKS) -> int:
    return v % mod


def circular_delta(a: int, b: int, mod: int = TICKS) -> int:
    d = (b - a) % mod
    if d > mod // 2:
        d -= mod
    return d


def circular_span(samples: List[int], mod: int = TICKS) -> int:
    if not samples or len(samples) == 1:
        return 0
    pts = sorted([s % mod for s in samples])
    gaps = [pts[i + 1] - pts[i] for i in range(len(pts) - 1)]
    gaps.append((pts[0] + mod) - pts[-1])
    largest_gap = max(gaps)
    return mod - largest_gap


def circular_midpoint(a: int, b: int, mod: int = TICKS) -> int:
    d = circular_delta(a, b, mod)
    return wrap_tick(a + d // 2, mod)


class ControllerClient:
    def __init__(self, port: str, ids: List[int], baud: int, timeout_ms: int = 20):
        try:
            from sts_scs_motor_controller_py import FeetechPyController
        except Exception as exc:
            raise SystemExit(
                "Could not import sts_scs_motor_controller_py. Build/install the python binding first."
            ) from exc

        self.ids = ids
        self.id_to_index: Dict[int, int] = {sid: i for i, sid in enumerate(ids)}
        self.ctrl = FeetechPyController.new_sts3215(port, ids, baud, timeout_ms)

    def close(self):
        self.ctrl.close()

    def read_positions(self) -> Dict[int, int]:
        vals = self.ctrl.get_last_positions()
        # values are f64 in rust API; in feetech transport these are ticks
        return {sid: int(round(vals[self.id_to_index[sid]])) for sid in self.ids}

    def read_position(self, sid: int) -> Optional[int]:
        try:
            return self.read_positions()[sid]
        except Exception:
            return None

    def set_goal(self, sid: int, tick: int) -> bool:
        try:
            self.ctrl.set_goal_positions([sid], [float(wrap_tick(tick))])
            return True
        except Exception:
            return False

    def set_torque(self, sid: int, enabled: bool) -> bool:
        try:
            self.ctrl.set_torque([sid], enabled)
            return True
        except Exception:
            return False


def detect_stop(
    client: ControllerClient,
    servo_id: int,
    direction: int,
    step_ticks: int,
    pause_s: float,
    stall_window: int,
    move_eps: int,
    no_wrap: bool,
    hard_min: int,
    hard_max: int,
) -> Optional[int]:
    pos_hist: List[int] = []

    current = client.read_position(servo_id)
    if current is None:
        return None

    for _ in range(3000):
        proposed = current + direction * step_ticks
        boundary_hit = False
        if no_wrap:
            if proposed < hard_min:
                proposed = hard_min
                boundary_hit = True
            elif proposed > hard_max:
                proposed = hard_max
                boundary_hit = True
            target = proposed
        else:
            target = wrap_tick(proposed)

        if not client.set_goal(servo_id, target):
            return None

        time.sleep(pause_s)

        pos = client.read_position(servo_id)
        if pos is None:
            return None

        pos_hist.append(pos)
        if len(pos_hist) > stall_window:
            pos_hist.pop(0)

        if len(pos_hist) >= stall_window:
            moved = circular_span(pos_hist)
            if moved <= move_eps:
                return pos

        current = pos
        if no_wrap and boundary_hit:
            return current

    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, required=True)
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--pause-ms", type=int, default=80)
    ap.add_argument("--stall-window", type=int, default=8)
    ap.add_argument("--move-eps", type=int, default=2)
    ap.add_argument("--no-wrap", action="store_true", default=True)
    ap.add_argument("--allow-wrap", action="store_true")
    ap.add_argument("--hard-min", type=int, default=0)
    ap.add_argument("--hard-max", type=int, default=4095)
    args = ap.parse_args()

    dry_run = not args.apply
    no_wrap = args.no_wrap and not args.allow_wrap

    print(f"Mode: {'DRY-RUN' if dry_run else 'APPLY'}")
    print(f"Setpoint wrapping: {'DISABLED' if no_wrap else 'ENABLED'}")
    print("Backend: CONTROLLER (queue/control-loop)")

    client = ControllerClient(args.port, args.ids, args.baud)
    results: List[ServoResult] = []

    try:
        for sid in args.ids:
            print(f"\n== Calibrating ID {sid} ==")
            client.set_torque(sid, True)

            if dry_run:
                print("dry-run: would detect min/max and move midpoint")
                results.append(ServoResult(sid, None, None, None))
                continue

            min_stop = detect_stop(
                client,
                sid,
                direction=-1,
                step_ticks=args.step,
                pause_s=args.pause_ms / 1000.0,
                stall_window=args.stall_window,
                move_eps=args.move_eps,
                no_wrap=no_wrap,
                hard_min=args.hard_min,
                hard_max=args.hard_max,
            )
            max_stop = detect_stop(
                client,
                sid,
                direction=+1,
                step_ticks=args.step,
                pause_s=args.pause_ms / 1000.0,
                stall_window=args.stall_window,
                move_eps=args.move_eps,
                no_wrap=no_wrap,
                hard_min=args.hard_min,
                hard_max=args.hard_max,
            )

            midpoint = None
            if min_stop is not None and max_stop is not None:
                midpoint = circular_midpoint(min_stop, max_stop)
                client.set_goal(sid, midpoint)

            print(f"ID {sid} min_stop={min_stop} max_stop={max_stop} midpoint={midpoint}")
            results.append(ServoResult(sid, min_stop, max_stop, midpoint))

    finally:
        client.close()

    print("\n=== Suggested zero ticks ===")
    for r in results:
        print(
            f"ID {r.servo_id}: zero_tick={r.midpoint} (min={r.min_stop}, max={r.max_stop})"
        )


if __name__ == "__main__":
    main()
