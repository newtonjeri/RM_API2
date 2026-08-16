# Emulator roadmap — what the collected data can teach it, and what is still missing

*2026-08-15. Basis: the 72-run blend/task matrix (2026-08-14), the three
chain-semantics SIM screens (2026-08-15), and the analyses in
MOTION_FINDINGS §9. The emulator's standing role (memory + MODE_CHARACTERIZATION):
right for dispatch, chain mechanics, arrival events, queue depth — never for
joint kinematics. Everything below respects that boundary: these are
**behavioural** models, not joint-rate predictions.*

## STATUS 2026-08-16: items 1-8 IMPLEMENTED, then AUDITED and corrected

**First report of this work overstated the accuracy** (it quoted +-3 %
from the five runs the constants were tuned on). A held-out audit the same
day gave the real picture, against MOTION-WINDOW durations (run.json
duration includes recorder padding and is not the right truth):

| set | n | mean \|err\| | median | max |
|---|---|---|---|---|
| tuned-on (calibration) | 5 | 3.5 % | 4.3 % | 6.3 % |
| **held-out** | **15** | **8.2 %** | **6.6 %** | **22.2 %** |

The audit changed the model three times before settling: freeze dwells are
now interpolated between two measured points (0.98 s/site at r=25, 0.30 at
r=50 - a single constant was 25-46 % wrong at r=50), and H67 is applied as
a per-segment time-scaling. Angular-ACCELERATION variants were implemented,
measured, and rejected: they fix the cap-ladder trend but cost 12 points at
the operating cap, because the real limiter at raised caps is joint speed,
which this emulator does not model.

Residual known-bad regions: angular-cap projections (-12 % at 0.8, -22 % at
1.0 on rotation-heavy tasks) and short/fast chains (-12 to -17 %).

Stream geometry: blend cuts land 2-4 mm short of the controller's; vertex
miss within 0.6 mm; which corners blend, the exemption, and r=0 stops are
exact. Frame boundary validated T = 0.00 mm against four REAL streams.

The boundary fix also exposed a latent bug: `test_single_arm_planned`
(C6) commanded a movej_p 0.20 m along world +X from rest_pose, which is
795 mm from the base against a ~788 mm tip reach. It passed only because
the old emulator applied movej_p offsets in the ALGO frame while also
reporting poses in that frame. Offset corrected to 0.10 m and documented
in the test. Regression state: dry-run 250/250, emulated suite 20/20.

## A. Improvements implemented (2026-08-16) — original plan

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

## A2. Cross-tree note — the alix port, and why it is NOT auto-syncable

*Recorded 2026-08-17 after coordinating with the alix session.*

A second copy of this emulator lives at
`~/alix_ws/src/alix/core/python/src/alix_emulator/rm_emulator.py`
(62,275 B, 2026-08-11) against this one (112,902 B, 2026-08-15). The port
predates BOTH the measured-law work and the controller-frame pose boundary,
so it still returns ALGO-frame poses from `current_pose` while taking
CONTROLLER-frame `movel` targets — a 90° round-trip error for any consumer
that reads a pose and commands from it.

**Measured exposure over there: ZERO frame-exposed call sites** (12 files
import it; the only live `rm_movel` call asserts the joints do NOT move, so
it never round-trips). The defect is latent — a trap for the next consumer,
not a live bug.

**Why a re-sync is a decision, not a chore.** That one live call is a
deliberate tripwire for their F20: *"if anyone ever makes `movel` kinematic,
it fails loudly and the change gets the hardware review it needs."* Our
`movel` IS now kinematic (`_plan_chain` solves IK per sample and drives the
joints). Replayed here, their assertion fails exactly as designed:

    ret=0, joints MOVED, max joint delta 154.98°   → `assert after == before` FAILS

So adopting this file over there is not "take the newer version", it is
"should emulator `movel` become kinematic" — a behaviour change their D4 rule
assigns to Newton, awake, and it would necessarily retire F20's timer
behaviour (their test rewritten from "movel must not move" to "movel must
move, within this fidelity bound"). **Nothing here should be pushed into that
tree without that decision.**

Alix recorded an interim option, and it is an OPTION AWAITING NEWTON, not an
agreement — `alix/plan/FINDINGS.md:722-725`, verbatim:

> **Newton's call.** The cheap middle option, if the hold continues: pin the
> port's consumers to the algo-frame convention explicitly, so the trap is
> documented at the point of use rather than discovered by whoever writes
> the next one.

It sits beside two others alix also left open — re-sync, or do nothing. An
earlier revision of this section said "agreed", which converted a
flagged-for-decision option into a settled action and attributed the
agreement to a side that never gave it. Corrected 2026-08-17.

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
| 9 | Dual-arm chained dispatch interaction | all 2026-08-14/15 runs are single-arm, and the 4-minute budget assumes RL_paired arms run in PARALLEL | **the vendor's own pattern is the reference**: `RMDemo_DoubleRoboticArm` runs each arm in its own Python thread with its own `RoboticArm` instance (first arm carries the thread mode, the second constructs bare — which is exactly what `dual_arm_common.connect_both` already does). Two-arm chained SIM run, both arms dispatching serpentines concurrently |
| 11 | `rm_movec` `loop` semantics | the vendor demo uses `loop=2`; if it repeats the arc, one command scrubs a ring N times with no junctions (CLEANING_MOTION_SPEC §3) | `chain_semantics_007`: same arc box, loop 0/1/2, count traced revolutions + check `connect=1` still chains afterwards — one SIM run |
| 10 | REAL corner speeds at 0.45 with per-move v | 0.45 REAL data exists only at uniform v | fold into the first REAL run of `toplid_left_002` at a 0.45 baseline (after gap 2 closes) |

Priorities: gaps 2 and 3 first (they gate the redesigned paths and explain
the only silent failures); then 1 and 5 (they complete the blend model);
the rest opportunistically.
