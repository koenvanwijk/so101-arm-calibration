# so101-arm-calibration

Robot-arm specific calibration utilities for SO101-style arms using Feetech STS/SCS servos.

This repo contains **application-level** tooling (endstop probing, midpoint/offset logic, visualization),
which is intentionally separate from the generic motor-controller core.

It also includes a conservative safety/guard pipeline for controller-backed motion and end-effector commands.

## Included

- `scripts/calibrate_endstops.py` — direct-serial, slow load-monitored endstop calibration
- `scripts/calibrate_via_controller.py` — queue/controller-based calibration (shared bus-owner model)
- `docs/endstop_calibration.md` — procedure and safety notes
- `docs/guard_validation.md` — guard validation checklist

## Usage

### Real hardware (direct serial)

```bash
python3 scripts/calibrate_endstops.py \
  --port /dev/tty_pink_follower_so101 \
  --baud 1000000 \
  --ids 1 2 3 4 5 6 \
  --apply
```

### Real hardware (via controller queue)

```bash
python3 scripts/calibrate_via_controller.py \
  --port /dev/tty_pink_follower_so101 \
  --baud 1000000 \
  --ids 1 2 3 4 5 6 \
  --apply
```

### Simulator

```bash
python3 scripts/calibrate_endstops.py \
  --simulate \
  --ids 1 2 3 \
  --visualize
```

## Checker / watchdog

Added:
- `scripts/checker_watch.py` (direct serial)
- `scripts/checker_via_controller.py` (recommended; single bus-owner model)
- `scripts/guard_pipeline.py` (controller-backed guard enforcement)
- `scripts/guard_bambot_adapter.py` (lightweight simulation adapter)
- `scripts/run_guard_matrix.py` (reproducible guard test matrix)
- `scripts/safe_controller_guard_demo.py` (guarded command demo CLI)

Guard stack enforces:
- hold-mode default fault handling (freeze + keep torque)
- optional torque-off fallback
- speed caps
- position sanity ranges
- low-voltage handling
- comm-drop/stale-snapshot handling
- structured alerts

Example (controller-backed):

```bash
python3 scripts/checker_via_controller.py \
  --port /dev/tty_white_follower_so101 \
  --ids 1 2 3 4 5 6 \
  --min-voltage 45 \
  --expect-torque off \
  --position-ranges "1:700-3300,2:900-3200,3:900-3100,4:900-3150,5:200-3900,6:1200-3200"
```

## Gravity-hold compensation (prototype)

Added:
- `model/so101_bambot.urdf` (sourced from Bambot)
- `scripts/hold_comp_from_model.py`

This script computes conservative bias ticks for IDs 2/3/4 (shoulder/elbow/wrist_pitch)
from current raw ticks using a URDF-informed planar approximation.

Example:

```bash
python3 scripts/hold_comp_from_model.py --id2 1010 --id3 3088 --id4 2873
```

## End-effector safe API

`EndEffectorSafeInterface` (in `scripts/guard_pipeline.py`) routes gripper/open/close commands through the same guard checks before issuing commands.

## Scope

This repo is arm/workflow specific.

Notes:
- `calibrate_via_controller.py` currently uses position-only stop detection.
- For load-based stop detection + offset register writes, use the direct-serial script for now.
- `checker_watch.py` currently reads serial directly; ideal end-state is routing it through a single bus-owner service.

Generic reusable motor-control functionality remains in:
- `../sts-scs-motor-controller`
