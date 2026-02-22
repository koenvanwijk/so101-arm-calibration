# Guard stack validation (SO101)

## Safety defaults

- Fault policy default: **hold mode**
  - freeze to last observed pose
  - keep torque enabled
- If hold command fails: optional **torque-off fallback** (enabled by default)
- Guard checks before command apply:
  - per-ID range checks
  - speed cap checks (ticks/s)
  - minimum voltage checks
  - communication drop/stale snapshot checks
- All guard decisions emit **structured alerts** (`code`, `severity`, `message`, `context`).

## Test matrix

Runner:

```bash
python3 scripts/run_guard_matrix.py
```

Pass/fail criteria:

1. `nominal`: in-range command under speed cap must apply (`command_applied`).
2. `range_violation`: out-of-range command must be blocked and hold engaged.
3. `low_voltage`: low voltage must block command and emit `low_voltage` alert.
4. `comm_drop`: stale health timestamp must block command and emit `comm_drop` alert.
5. `speed_cap`: excessive command delta/time must block and emit `speed_violation`.
6. `eef_safe_api`: end-effector close/open commands must route through same guard pipeline and succeed only when safe.

Overall verdict is `all_pass: true` only when all cases pass.

## Bambot simulation integration

`BambotSimControllerAdapter` reuses `SimBackend` for safe/offline validation.

Limits:
- no true dynamic model, synthetic load/voltage behavior
- no realistic serial noise timing model
- good for logic/guard-path validation, not final hardware certification

Hardware-in-the-loop validation is still required before production use.
