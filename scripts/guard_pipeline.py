#!/usr/bin/env python3
"""Safety guard pipeline for controller-backed SO101 workflows.

Conservative defaults:
- hold mode on fault (freeze to last known position + keep torque on)
- optional torque-off fallback if hold command fails or policy requests it
- speed cap, per-ID range checks, voltage checks, comm-drop checks
- structured alerts (dict payloads suitable for JSON logs)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Protocol, Tuple


class ControllerLike(Protocol):
    def get_last_health(self): ...
    def set_goal_positions(self, ids: List[int], positions: List[float]): ...
    def set_torque(self, ids: List[int], enabled: bool): ...


@dataclass
class GuardConfig:
    ids: List[int]
    speed_cap_ticks_per_s: float = 120.0
    position_ranges: Dict[int, Tuple[int, int]] = field(default_factory=dict)
    min_voltage_raw: int = 45
    comm_drop_timeout_s: float = 0.8
    on_fault: str = "hold"  # hold | torque_off
    torque_off_on_hold_failure: bool = True


@dataclass
class GuardResult:
    allowed: bool
    applied: str
    alerts: List[dict]


class SafetyGuardPipeline:
    def __init__(self, controller: ControllerLike, config: GuardConfig):
        self.ctrl = controller
        self.cfg = config
        self._id_to_idx = {sid: i for i, sid in enumerate(config.ids)}
        self._last_commanded: Dict[int, int] = {}
        self._last_cmd_ts: Optional[float] = None

    def _alert(self, code: str, severity: str, message: str, **ctx) -> dict:
        return {"code": code, "severity": severity, "message": message, "context": ctx}

    def _engage_hold(self, positions: Dict[int, int], alerts: List[dict]) -> str:
        ids = list(positions.keys())
        goals = [float(positions[sid]) for sid in ids]
        try:
            self.ctrl.set_goal_positions(ids, goals)
            self.ctrl.set_torque(ids, True)
            alerts.append(self._alert("hold_engaged", "critical", "Hold mode engaged", ids=ids))
            return "hold"
        except Exception as exc:
            alerts.append(self._alert("hold_failed", "critical", "Hold command failed", error=str(exc)))
            if self.cfg.torque_off_on_hold_failure:
                try:
                    self.ctrl.set_torque(self.cfg.ids, False)
                    alerts.append(self._alert("torque_off_fallback", "critical", "Torque off fallback engaged"))
                    return "torque_off"
                except Exception as exc2:
                    alerts.append(self._alert("torque_off_failed", "critical", "Torque off fallback failed", error=str(exc2)))
            return "fault_uncontained"

    def _engage_torque_off(self, alerts: List[dict]) -> str:
        try:
            self.ctrl.set_torque(self.cfg.ids, False)
            alerts.append(self._alert("torque_off", "critical", "Torque disabled due to fault"))
            return "torque_off"
        except Exception as exc:
            alerts.append(self._alert("torque_off_failed", "critical", "Torque off failed", error=str(exc)))
            return "fault_uncontained"

    def apply_goal(self, goal_positions: Dict[int, int], now_s: Optional[float] = None) -> GuardResult:
        alerts: List[dict] = []
        try:
            positions, voltages, _torques, ts = self.ctrl.get_last_health()
        except Exception as exc:
            alerts.append(self._alert("health_read_failed", "critical", "Unable to read health snapshot", error=str(exc)))
            applied = self._engage_torque_off(alerts)
            return GuardResult(False, applied, alerts)

        observed: Dict[int, int] = {}
        for sid in self.cfg.ids:
            i = self._id_to_idx[sid]
            observed[sid] = int(round(positions[i])) if i < len(positions) else 0
            v = int(voltages[i]) if i < len(voltages) else 0
            if v == 0 or v < self.cfg.min_voltage_raw:
                alerts.append(self._alert("low_voltage", "critical", "Voltage below minimum", sid=sid, voltage=v, min=self.cfg.min_voltage_raw))

        if now_s is not None and (now_s - ts) > self.cfg.comm_drop_timeout_s:
            alerts.append(self._alert("comm_drop", "critical", "Snapshot too old", age_s=now_s - ts, max_age_s=self.cfg.comm_drop_timeout_s))

        # command validity checks
        checked_ids: List[int] = []
        checked_goals: List[float] = []
        for sid, goal in goal_positions.items():
            if sid not in self._id_to_idx:
                alerts.append(self._alert("unknown_id", "error", "Unknown motor id in command", sid=sid))
                continue
            lo_hi = self.cfg.position_ranges.get(sid)
            if lo_hi is not None:
                lo, hi = lo_hi
                if not (lo <= goal <= hi):
                    alerts.append(self._alert("range_violation", "critical", "Goal outside allowed range", sid=sid, goal=goal, lo=lo, hi=hi))

            if self._last_cmd_ts is not None and sid in self._last_commanded and now_s is not None:
                dt = max(1e-6, now_s - self._last_cmd_ts)
                speed = abs(goal - self._last_commanded[sid]) / dt
                if speed > self.cfg.speed_cap_ticks_per_s:
                    alerts.append(self._alert("speed_violation", "critical", "Requested speed exceeds cap", sid=sid, speed=speed, cap=self.cfg.speed_cap_ticks_per_s))

            checked_ids.append(sid)
            checked_goals.append(float(goal))

        if alerts:
            if self.cfg.on_fault == "torque_off":
                applied = self._engage_torque_off(alerts)
            else:
                applied = self._engage_hold(observed, alerts)
            return GuardResult(False, applied, alerts)

        self.ctrl.set_goal_positions(checked_ids, checked_goals)
        if now_s is not None:
            self._last_cmd_ts = now_s
        for sid, g in goal_positions.items():
            self._last_commanded[sid] = int(g)
        return GuardResult(True, "command_applied", alerts)


class EndEffectorSafeInterface:
    """Safe end-effector API routed through guard pipeline."""

    def __init__(self, guard: SafetyGuardPipeline, eef_id: int, min_tick: int = 900, max_tick: int = 3200):
        self.guard = guard
        self.eef_id = eef_id
        self.min_tick = min_tick
        self.max_tick = max_tick

    def set_gripper_tick(self, tick: int, now_s: Optional[float] = None) -> GuardResult:
        clamped = max(self.min_tick, min(self.max_tick, int(tick)))
        return self.guard.apply_goal({self.eef_id: clamped}, now_s=now_s)

    def open(self, now_s: Optional[float] = None) -> GuardResult:
        return self.set_gripper_tick(self.max_tick, now_s=now_s)

    def close(self, now_s: Optional[float] = None) -> GuardResult:
        return self.set_gripper_tick(self.min_tick, now_s=now_s)
