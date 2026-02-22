# so101-arm-calibration

Robot-arm specific calibration utilities for SO101-style arms using Feetech STS/SCS servos.

This repo contains **application-level** tooling (endstop probing, midpoint/offset logic, visualization),
which is intentionally separate from the generic motor-controller core.

## Included

- `scripts/calibrate_endstops.py` — slow, load-monitored endstop calibration
- `docs/endstop_calibration.md` — procedure and safety notes

## Usage

### Real hardware

```bash
python3 scripts/calibrate_endstops.py \
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

## Scope

This repo is arm/workflow specific.

Generic reusable motor-control functionality remains in:
- `../sts-scs-motor-controller`
