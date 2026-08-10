# Run recordings

One directory per task run, written automatically by `stage_runner.py`
(disable with `--no-record`). **These are data — not gitignored.**

    <runid>/stream.csv    100 Hz UDP samples (cycle=2; `--udp-cycle 1` = 200 Hz)
    <runid>/run.json      metadata, the COMMANDED program, per-stage marks

`run_id` = `<UTC-ish timestamp>_<task>_<side>`, e.g.
`20260810T124126_hinge_area_left_left`.

## Reading a run

`t_mono` is one monotonic clock, zeroed when recording starts, shared by
`stream.csv` and the `stages` marks in `run.json`. To isolate a stage,
slice the stream between its `t_start` and `t_end`.

`tcp_x..tcp_rz` is the **controller's own** TCP pose — compare it against
`commanded.poses` in `run.json` for the 1:1. It needs no FK of ours, which
matters because our FK is where several 2026-08 errors lived.

## What is NOT here

**The hand.** `bench_udp_fields` (butterfli_hw, both arms, `hand=1`
requested) measured `handState` ALL-ZERO — the hand does not report on the
arm's UDP line; Modbus `ANGLE_ACT` at 2-5 Hz is the only feedback path,
and `rm_set_hand_angle` must never be called while modbus mode is active.
That bench ran on fw < 1.7.3 and its own note says handState needs >=
1.7.3; both arms are now V1.7.4, so this is worth re-checking.

## Provenance

`run.json` records `sdk: "hardware"` or `"emulated"` and `sim: true/false`.
Check it before drawing conclusions — an emulated run has no IK and models
the movel chain as a timed no-op.

## Size

~500 KB per 15 s at 100 Hz (~2 MB/minute). Kept deliberately.
