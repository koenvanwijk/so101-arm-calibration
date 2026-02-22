# so101-arm-calibration

Robot-arm specific calibration utilities for SO101-style arms using Feetech STS/SCS servos.

This repo contains **application-level** tooling (endstop probing, midpoint/offset logic, visualization),
which is intentionally separate from the generic motor-controller core.

## Included

- `scripts/calibrate_endstops.py` — direct-serial, slow load-monitored endstop calibration
- `scripts/calibrate_via_controller.py` — queue/controller-based calibration (shared bus-owner model)
- `docs/endstop_calibration.md` — procedure and safety notes

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

Added: `scripts/checker_watch.py`

It can monitor:
- voltage (default register `62`)
- torque enabled flag (default register `40`)
- position sanity ranges (default register `56`)

Example:

```bash
python3 scripts/checker_watch.py \
  --port /dev/tty_white_follower_so101 \
  --ids 1 2 3 4 5 6 \
  --min-voltage 45 \
  --expect-torque off \
  --position-ranges "1:700-3300,2:900-3200,3:900-3100,4:900-3150,5:200-3900,6:1200-3200"
```

## Scope

This repo is arm/workflow specific.

Notes:
- `calibrate_via_controller.py` currently uses position-only stop detection.
- For load-based stop detection + offset register writes, use the direct-serial script for now.
- `checker_watch.py` currently reads serial directly; ideal end-state is routing it through a single bus-owner service.

Generic reusable motor-control functionality remains in:
- `../sts-scs-motor-controller`
