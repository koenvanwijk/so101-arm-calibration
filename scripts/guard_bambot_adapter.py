#!/usr/bin/env python3
"""Lightweight Bambot/simulator adapter for guard validation.

Reuses SimBackend from calibrate_endstops (safe/offline). Limitations:
- kinematics and dynamics are simplified (position + synthetic load)
- no realistic bus jitter/packet corruption model
- voltage is synthetic constant unless overridden
"""

import time
from typing import List

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from calibrate_endstops import SimBackend


class BambotSimControllerAdapter:
    def __init__(self, ids: List[int], voltage_raw: int = 60):
        self.ids = ids
        self.sim = SimBackend(ids)
        self._id_to_idx = {sid: i for i, sid in enumerate(ids)}
        self._torque = [True for _ in ids]
        self._voltage = [int(voltage_raw) for _ in ids]
        self._ts = time.time()

    def get_last_health(self):
        positions = [float(self.sim.state[sid]["pos"]) for sid in self.ids]
        return positions, list(self._voltage), list(self._torque), self._ts

    def set_goal_positions(self, ids, positions):
        for sid, p in zip(ids, positions):
            self.sim.write_u16(int(sid), 42, int(round(p)), dry_run=False)

    def set_torque(self, ids, enabled):
        for sid in ids:
            i = self._id_to_idx[sid]
            self._torque[i] = bool(enabled)

    def inject_voltage(self, sid: int, voltage_raw: int):
        self._voltage[self._id_to_idx[sid]] = int(voltage_raw)

    def force_stale_timestamp(self, age_s: float):
        self._ts = time.time() - age_s
