# Emulator roadmap — what the collected data can teach it, and what is still missing

*2026-08-15. Basis: the 72-run blend/task matrix (2026-08-14), the three
chain-semantics SIM screens (2026-08-15), and the analyses in
MOTION_FINDINGS §9. The emulator's standing role (memory + MODE_CHARACTERIZATION):
right for dispatch, chain mechanics, arrival events, queue depth — never for
joint kinematics. Everything below respects that boundary: these are
**behavioural** models, not joint-rate predictions.*

## A. Improvements implementable NOW from data already collected

Ordered by value to the workflow (each names the data that calibrates it):

1. **Blend geometry in `movel_chain`.** Today the emulator stops at every
   waypoint regardless of r. The measured law is complete enough to model:
   first corner of a chain never blends (positional; 18/18 runs); the last
   corner blends; cut(entry+exit) ≈ c·(r/100)·min(L_in,L_out) with
   c ≈ 0.70 at 90° (exact, both radii, chain_rmix run), c ≈ 1.2–1.5 at
   120–179°, c ≈ 0 below 60°, c = 0 at exact-180 retraces; corner minimum
   speed ≈ 26/52 mm/s at r=25/50 through ~175° U-turns, 60–90 mm/s at 90°,
   scaling ~linearly with v at r ≤ 25 and collapsing at r=50 + 0.45.
   Calibration set: `runs/2026081*` + `analyse_coverage.py`.
2. **Per-move (v, r) in the chain queue.** The emulator queues poses only
   and executes the whole chain with the closing move's v — the controller
   honors v and r PER MOVE (chain_rmix/chain_vmix, 2026-08-15). Store
   (pose, v, r) per queued entry. Validation targets: plateaus 94/243/145
   mm/s for v% = 40/100/60 at the 0.25 baseline.
3. **Angular throttling (H67).** v_eff = min(v, ω_cap/ω_required) per
   segment — already modelled in `orientation_cost.py`; port the same rule
   into `_plan_cartesian` timing so emulated durations stop under-counting
   rotating segments. Validation: the throttled segments in any toplid run.
4. **arm_status choreography.** Emit the measured state machine: MOVE_L →
   IDLE mid-run with the tool still moving at blends (the "[BLEND ACTIVE]"
   signature analyse_run detects), status 9 at the tail, IDLE dwell at r=0
   corners. Analysis pipelines can then be developed fully offline.
5. **Known-hazard warnings (not simulations).** Two behaviours are
   reproducible but unexplained — flag, don't model: (a) short
   exact-retrace spur + r ≥ 25 → the deterministic 2.0–2.7 s freeze
   (12/12 toplid runs at point13); (b) hinge-class geometry + r = 25 →
   intermittent silent early chain termination at a waypoint (3/6 runs,
   SIM included). The emulator should print a warning when a submitted
   chain matches either signature.
6. **SIM-mode channel behaviour.** When emulating SIM, zero the joint
   speed/current channels the way the controller's SIM does (measured:
   ~0.4°/s while moving 56° of travel), so gates like "drop SIM runs
   before quoting joint loads" can be tested offline.
7. **UDP position-channel aliasing (REAL mode).** Inject the measured
   noise model: position error ∝ v × timing jitter (median 0.9 mm at
   0.10 m/s → 1.5 mm at 0.35, p99 5–7 mm), summed-arc inflation 7–10 %.
   Calibrated in `verify_blend_measure.py` (PERP_JITTER, 108 % arc).
   Lets the analysis stack be regression-tested against realistic streams.
8. **Duration calibration.** The 72-run table gives measured duration per
   (task, r, v) — fit `_plan_cartesian`'s dip model so emulated durations
   land within ~10 % of REAL. Currently unvalidated.

Already fixed this session: the process-global algo toolframe trap
(`_set_algo_toolframe` asserted at every IK/FK entry) — see the memory note
in project-emulator-cannot-predict-movel.

## B. Data gaps, and how to collect each

| # | Gap | Why it matters | Collection recipe (cheapest first) |
|---|-----|----------------|------------------------------------|
| 1 | Cut coefficient c(θ) between 90° and 165° | the blend model interpolates blindly there | one synthetic SIM path: fixed 150 mm segments, corners at 100/115/130/145/160°, r=25/50 — two runs |
| 2 | Chain semantics at a 0.45 baseline | everything above 0.35 is extrapolation | `chain_semantics_004/005` — READY, two SIM runs |
| 3 | Root cause of the r=25 freeze / early termination | silent coverage loss in production | SIM matrix: spur length {10,20,30,50 mm} × r {20,25,30,35,50} × adjacent-segment length; SIM reproduces both faithfully |
| 4 | r > 50 behaviour | never tested; the cut law may saturate | add r=75 to one blend_corner SIM sweep |
| 5 | v% → speed mapping precision | measured 94/243/145 vs commanded 100/250/150 (−3–6 %) | v% sweep 10..100 on one long straight, SIM, one run |
| 6 | Controller chain-queue depth limit | 40-move chains work; the ceiling is unknown | chain N short moves, N = 50/100/200, until refusal — SIM |
| 7 | Entry-phase timing (movej/movej_p durations) | recordings start at the stroke; entries are unmeasured | start the recorder before `goto_start_sequence` once per task (one-line toggle) |
| 8 | Blend arc lateral profile (the actual curve shape) | needed if the emulator should reproduce trajectories, not just cuts | NO new runs — fit the U-turn arcs in the existing SIM streams |
| 9 | Dual-arm chained dispatch interaction | all 2026-08-14/15 runs are single-arm | two-arm chained SIM run, both arms dispatching serpentines concurrently |
| 10 | REAL corner speeds at 0.45 with per-move v | 0.45 REAL data exists only at uniform v | fold into the first REAL run of `toplid_left_002` at a 0.45 baseline (after gap 2 closes) |

Priorities: gaps 2 and 3 first (they gate the redesigned paths and explain
the only silent failures); then 1 and 5 (they complete the blend model);
the rest opportunistically.
