import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from guard_pipeline import EndEffectorSafeInterface, GuardConfig, SafetyGuardPipeline


class FakeController:
    def __init__(self, ids, positions=None, voltages=None, ts=None):
        self.ids = ids
        self.positions = positions or [2048 for _ in ids]
        self.voltages = voltages or [60 for _ in ids]
        self.torques = [True for _ in ids]
        self.ts = ts if ts is not None else time.time()
        self.goal_calls = []
        self.torque_calls = []
        self.fail_goal = False
        self.fail_health = False

    def get_last_health(self):
        if self.fail_health:
            raise RuntimeError("health fail")
        return self.positions, self.voltages, self.torques, self.ts

    def set_goal_positions(self, ids, positions):
        if self.fail_goal:
            raise RuntimeError("goal fail")
        self.goal_calls.append((ids, positions))

    def set_torque(self, ids, enabled):
        self.torque_calls.append((ids, enabled))


class GuardPipelineTests(unittest.TestCase):
    def test_nominal_command_allowed(self):
        ctrl = FakeController([1, 2])
        guard = SafetyGuardPipeline(ctrl, GuardConfig(ids=[1, 2], position_ranges={1: (1000, 3000)}))
        res = guard.apply_goal({1: 2000}, now_s=time.time())
        self.assertTrue(res.allowed)
        self.assertEqual(res.applied, "command_applied")

    def test_range_violation_triggers_hold_default(self):
        ctrl = FakeController([1])
        guard = SafetyGuardPipeline(ctrl, GuardConfig(ids=[1], position_ranges={1: (1500, 2500)}))
        res = guard.apply_goal({1: 1200}, now_s=time.time())
        self.assertFalse(res.allowed)
        self.assertEqual(res.applied, "hold")

    def test_speed_cap_violation(self):
        ctrl = FakeController([1])
        guard = SafetyGuardPipeline(ctrl, GuardConfig(ids=[1], speed_cap_ticks_per_s=10.0))
        t0 = time.time()
        self.assertTrue(guard.apply_goal({1: 2000}, now_s=t0).allowed)
        res = guard.apply_goal({1: 3000}, now_s=t0 + 0.05)
        self.assertFalse(res.allowed)

    def test_low_voltage_fault(self):
        ctrl = FakeController([1], voltages=[40])
        guard = SafetyGuardPipeline(ctrl, GuardConfig(ids=[1], min_voltage_raw=45))
        res = guard.apply_goal({1: 2100}, now_s=time.time())
        self.assertFalse(res.allowed)

    def test_comm_drop_fault(self):
        ctrl = FakeController([1], ts=time.time() - 2.0)
        guard = SafetyGuardPipeline(ctrl, GuardConfig(ids=[1], comm_drop_timeout_s=0.5))
        res = guard.apply_goal({1: 2100}, now_s=time.time())
        self.assertFalse(res.allowed)

    def test_torque_off_fallback_when_hold_fails(self):
        ctrl = FakeController([1])
        ctrl.fail_goal = True
        guard = SafetyGuardPipeline(
            ctrl,
            GuardConfig(ids=[1], on_fault="hold", torque_off_on_hold_failure=True, position_ranges={1: (1500, 2500)}),
        )
        res = guard.apply_goal({1: 1000}, now_s=time.time())
        self.assertFalse(res.allowed)
        self.assertEqual(res.applied, "torque_off")

    def test_end_effector_interface_routes_via_guard(self):
        ctrl = FakeController([6])
        guard = SafetyGuardPipeline(ctrl, GuardConfig(ids=[6], position_ranges={6: (1200, 3200)}))
        eef = EndEffectorSafeInterface(guard, eef_id=6, min_tick=1200, max_tick=3200)
        res = eef.open(now_s=time.time())
        self.assertTrue(res.allowed)


if __name__ == "__main__":
    unittest.main()
