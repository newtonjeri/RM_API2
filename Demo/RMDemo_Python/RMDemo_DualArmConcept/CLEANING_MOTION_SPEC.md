# Cleaning-motion design spec — 0.8 m/s program, wiping action, 4-minute cleaning inside a 6-minute activity

*2026-08-15. Research basis: commode_c fixture configs (32 task YAMLs, 20
regions, meshes + region annotations), glove_frames.yaml, the measured motion
laws in MOTION_FINDINGS §9, and a J4 screen of every task family. Targets set
by Newton: line speed toward 0.8 m/s, human-like wiping action, proximal
glove surfaces where primary contact degrades below half, whole activity
cleaning under 4 minutes, inside a whole activity under 6 (Newton,
2026-08-17). Task-config structure may change if speed is not lost.*

---

## 0. DESIGN CONTRACT (agreed with Newton, 2026-08-16)

**Design speeds: 0.8 m/s linear / 1.0 rad/s angular — applied under law,
never as globals.**

1. LINEAR 0.8 m/s is the per-move design target. A move may carry it only
   where its family's cap-aware J4 screen passes at <=90 %. Everything
   else runs at its family ceiling (table in section 2) via per-move v.
   The vendor 1.8 m/s is a hard maximum, never a target. The four
   over-limit families (front, side, seat_ring, deep) stay <=0.19 m/s
   until their redesigns land.
2. ANGULAR 1.0 rad/s is the working cap. Every task is re-screened for J4
   AT the raised cap before running there (raising the cap removes the
   protective throttle — hardware-verified). First hardware run per
   family gets a J5-J7 H63 review.
3. LAWS, always in force: line_acc = max(default 1.6, 3 x line_speed) —
   the default is the FLOOR; angular_acc stays 4.0; limits are verified
   by readback, never trusted; limits RATCHET — reset_limits.py after
   every session.

**Wiping action:** serpentine strokes with padded turnarounds; movec arcs
at direction changes and for annular regions (hardware-verified: 0.4 mm
tracking, no stops, 2x junction speed vs blends); scrubbing = exact-180
retraces (~2.5 Hz, coverage-safe); entries chained so the unblendable
first corner and its stop land at the touchdown, outside the area. No
stops except touchdown and physically-required reversals (r=0).

**Blend radius:** per move. r=10 on dense geometry (the freeze rule —
hardware-confirmed deterministic planner freezes at r>=25 near short
segments); large r only on sparse padded geometry; arcs preferred over
blends wherever a turn can be an arc.

**Coverage:** stroke spacing <= effective glove band (footprint width
minus the 1.5 cm tolerance: 20 mm for frames 1/2); every generated path
numerically verified >=99 % of its area hull before first run. Contact
rule for frame selection (proximal surfaces allowed): keep the pad's
LONG axis within 30 deg of the surface (26.7 deg frames 3/4), i.e.
f = t/(D sin theta) >= 0.5, with the convex-curvature check below
R ~ 40 mm.

**Budget — TWO numbers, do not conflate them** (Newton, 2026-08-17):

* **CLEANING < 4 minutes.** ~65 m per arm; 240 s less in-cleaning
  overheads (task entries, lid/seat articulation, UGV standoff) leaves
  ~180 s of motion -> requires **>=0.36 m/s sustained average**. Best
  measured today is 0.21 m/s, a **x1.7 gap**. It cannot be closed by one
  knob: the speed program + cap 1.0 + slow-family redesigns + task
  chaining + arc regeneration must move TOGETHER. This is the number
  every section below is written against.
* **WHOLE ACTIVITY < 6 minutes.** Cleaning plus the peripheral
  operations — bin drop/raise, and the glove wear/removal instances.
  The extra ~120 s is the allowance for those. **Not measured:** no
  glove or bin task has a recorded duration
  (`alix_tasks/Reports/task_metrics.csv` holds only `lid_close_left`
  and `toplid_right`), so 120 s is a design envelope, not a
  measurement. First measured glove/bin cycle should be checked
  against it.

The four over-limit families stay <=0.19 m/s under item 1 for SAFETY,
independent of either clock.

**Process:** every change goes screen (offline, cap- and V_LIST-aware)
-> SIM -> REAL; H63 dwell rule everywhere; final-waypoint arrival
verified per run; E-stop in hand on REAL.

> **READING NOTE on item 1 — how to read the J4 screen. No spec term
> changed.** *Rewritten 2026-08-17; the previous note argued the screen was
> "necessary, not sufficient" and proposed adding a criterion. Withdrawn —
> see §0b for the production-form measurements that refuted it.*
>
> The J4 screen is the gate, and on the production motion form it is
> **correct**: per-move mixed radius and speed with the cap applied puts J4
> at 60 % / 87 % / 44 % on the three REAL runs, worst joint every time, zero
> H63 dwell (§0b).
>
> Two things it does not do, both handled by the Process line above rather
> than by a new criterion:
> * It bounds **J4 only**, so the §2 ceilings are upper bounds on J4, not
>   task ceilings. `top` is held at ≤ 0.25 for that reason.
> * It says nothing about the other six joints. **SIM does, for free** — it
>   matches REAL's binding joint on 31 of 31 comparable production
>   configurations. Run SIM before REAL, as item 3 of Process already
>   requires; that is where all seven joints get seen.
>
> `toplid_left` @0.80 put four joints over limit at once — but 0.80 is far
> above any family ceiling in §2, so item 1 already forbids it. The screen
> was never the thing that would have caught it.


### 0b. GATE NOTE — the J4 screen, tested against the production motion form

*Rewritten 2026-08-17 on Newton's instruction, replacing a 148-line
amendment proposal and its correction history. That material argued the J4
screen of §0 item 1 was "necessary but not sufficient" and proposed adding a
mandatory seven-joint REAL audit. **The proposal is withdrawn.** It rested on
two paths, and neither is a production motion:*

* ***`hinge_area`*** *— archived. It survives only in
  `alix_tasks/config/archive/reference/`, is not a live commode_c config, and
  was used for a handful of blend characterisation runs. Newton, 2026-08-17:
  "I never used hinge area recently, that is a pre-existing motion, I used for
  some few blend tests."*
* ***`top_left`*** *— every REAL run of it is `blend_r10/r25/r50`, a
  FIXED-radius characterisation sweep. §0 mandates blend radius **per move**.
  There is no production-form `top_left` run in the corpus.*

**What the production motion form actually shows.** The runs that implement
§0's blend policy — per-move mixed radius, per-move mixed speed, angular cap
applied, on `toplid_left_002` — were recorded 2026-08-15. REAL, reported
channel, all seven joints:

| run | v | worst joint | % of limit | H63 dwell ≥98 % |
|---|---|---|---|---|
| `20260815T153752` chain_rmix_vmix_capp | 0.25 | **J4** | 60 % | **0 ms** |
| `20260815T154329` chain_rmix_vmix_capp | 0.45 | **J4** | 87 % | **0 ms** |
| `20260815T201624` chain_arc_r25 | 0.25 | **J4** | 44 % | **0 ms** |

**J4 binds on all three. All seven joints under limit. Zero dwell.** At
0.45 J4 reads 87 %, inside the 90 % screen threshold, and the full audit
confirms J4 is genuinely the worst joint. **On this motion form
the J4 screen names the right joint and is sufficient.** §0 item 1 stands
unchanged.

**SIM corroborates, and it is free.** With the same estimator forced on both
sides (`survey_binding.py --pairs`, logic stamp `186b743034bb`), SIM predicts
REAL's binding joint on **31 of 31 comparable production configurations** —
including both production-form pairs above. The only disagreements anywhere
in the corpus are `hinge_area`, the archived path. This is the evidence for
§0's Process line (screen → SIM → REAL): SIM screens all seven joints for
nothing, and it tracks REAL wherever a stable binding joint exists.

**What remains true, and is a note rather than a gate.** The §2 ceiling table
is a J4 screen, so it bounds J4 only. On `top` at a fixed r=10 sweep, J1
measured 85–86 % against J4 65–80 % — which is why that row is held at
≤ 0.25 (see §2). Treat J1-heavy rows as upper bounds on J4 rather than as
task ceilings, and prefer a SIM run over an assumption before raising any of
them. That is guidance for reading §2, not an additional gate on §0.

**Two measured facts worth keeping from the withdrawn material:**

* **The only H63 exposures in the REAL corpus are speed, not radius.**
  `20260811T184109_toplid_left` (v=0.8, J4 105.9 %, 330 ms) and
  `20260811T222451_toplid_right` (v=0.6, J4 99.8 %, 110 ms) — both at r=10,
  both above any production cap, and both the runs H63 was calibrated on. No
  r=10 REAL run at or below 0.5 m/s carries any dwell.
* **The binding joint is a property of the task and does not move with
  speed** at fixed radius (H73). `toplid` → J4 at every speed and radius
  tested; `top` → J1 on the fixed-radius sweeps.

*Method rules that produced this section, kept because they were each paid
for: read a run's own `run.json` config before quoting it; never cite a
single run where siblings exist; resolve a run's family from `path_file`, not
its label, and state n; a dead CHANNEL is not a dead RUN (SIM's speed channel
is dead, its position channel is not); and quote the tool's `logic stamp`
with any figure, because a number can be correct when computed and go stale
when the code beneath it changes.*

## 1. The budget arithmetic — what 4 minutes of CLEANING actually requires

*Scope note, 2026-08-17: this section is about the **cleaning** budget, 4
minutes. The 6-minute figure in §0 is the WHOLE ACTIVITY and additionally
covers bin drop/raise and the glove wear/removal instances. Nothing in §1–§7
is written against the 6-minute number, and the ×1.7 gap below is not
relieved by it.*

Parsed from every commode_c cleaning config: **64.3 m (left) / 65.4 m
(right) of commanded cleaning path**, split across 16 tasks per arm, mostly
RL_paired (arms parallel). 240 s minus realistic overheads (task entries,
lid/seat articulation, UGV standoff moves at 0.25↔0.38 m) leaves roughly
**180 s of cleaning motion → a sustained average TCP speed ≥ 0.36 m/s.**
Best measured today (toplid rev 3 at the 0.45 baseline): **0.21 m/s
average**. The gap is ×1.7 — and it cannot be closed by one knob. Three
levers must move together:

1. **speed where physics allows it** (§2),
2. **path-length and overhead rationalization** (§5),
3. **the angular cap** — the quiet dominant limit (§2b).

## 2. Where 0.8 m/s is real — the per-family speed-ceiling table

J4 screen (exact, redundancy-invariant) of every left task at 0.25 m/s;
`v_ceil` = commanded line speed putting the worst segment at 90 % of J4.
(Screen vs measured calibration on toplid rev 3: screen 71 %, measured
59.5 % — the screen over-predicts ~15 %, so ceilings are conservative.)

| family | tool | worst J4 @0.25 | v_ceil m/s | throttled segs | verdict |
|---|---|---|---|---|---|
| lidsides_back | glove_3 | 18 % | **1.28** | 4/12 | 0.8 REAL |
| bowl_inside_back | glove_2 | 21 % | **1.10** | 11/11 | cap-bound (§2b) |
| seat_body_hinge | glove_2 | 21 % | **1.07** | 0/10 | 0.8 REAL |
| lid_seat_hinge | glove_2 | 26 % | **0.87** | 0/6 | 0.8 REAL |
| bowl_inside_ring | glove_1 | 33 % | 0.69 | 31/31 | cap-bound |
| seat_ring_bottom | glove_2 | 43 % | 0.53 | 41/55 | cap-bound |
| bottomlid | glove_2 | 50 % | 0.45 | 35/45 | mixed |
| lid_side | glove_1 | 50 % | 0.45 | 9/13 | mixed |
| bowl_inside_rim | glove_3 | 59 % | 0.38 | 22/23 | cap-bound |
| top | glove_2 | 65 % | **<=0.25** | 38/42 | **J1-bound (measured)** |
| toplid | glove_2 | 70 % | 0.32 | 7/27 | J4-bound |
| **deep** | glove_1 | **118 %** | **0.19** | 41/44 | OVER at 0.25 |
| **side** | glove_1 | **142 %** | **0.16** | 18/45 | OVER at 0.25 |
| **seat_ring** | glove_2 | **162 %** | **0.14** | 35/52 | OVER at 0.25 |
| **front** | glove_1 | **293 %** | **0.08** | 17/40 | OVER at 0.25 |

**`top` row corrected 2026-08-16.** Its J4-derived ceiling of 0.35 does not
hold: nine REAL runs at a commanded **0.25** put J1 above J4 on every one
(J1 86 % reported / 90 % house window, against J4 61–95 %), so `top` is
J1-bound at or below 0.25 and must not be raised (H73). Every other J1-heavy row is suspect for the same reason — the
column screens J4 only — and none of them has been run. Ceilings here are
upper bounds on J4 alone, not task ceilings.

Three consequences:

* **"0.8 m/s" is a per-move property, not a global setting.** Four families
  take it as-is; the per-move-v architecture (verified §9.3d) delivers it
  surgically. A global 0.8 would abort exactly like the old 0.45 toplid did.
* **Four families are illegal at today's 0.25** — `front`, `seat_ring`,
  `side`, `deep` (23.5 m, ~36 % of the path). These, not toplid, decide the
  4-minute cleaning question. Their J4 blowups are pose-family problems (frame_1
  X-press at low/awkward reach), fixable in order of preference: stroke
  DIRECTION change (J4/m is direction-dependent), ik_frame change with the
  §4 contact rule, UGV standoff change (the F49 lesson), pose-family
  redesign. Each needs its own screen-guided redesign like toplid got.
* **The angular cap 0.6 rad/s binds more path than J4 does** (§2b).

### 2b. The angular cap is the bigger half of the speed program

10 of 16 families have most segments throttled by TCP_ANGULAR_VELOCITY
0.6 rad/s — on those, raising line speed changes nothing. The cap is a
configurable limit exactly like line_speed (same vendor-advisory caveats,
H62, ratchet rule). Raising it toward 1.0–1.2 rad/s would directly raise
`v_eff` on curved/rotating strokes — the bulk of seat_ring, top, bowl and
rim work — and its joint-space cost lands mostly on the wrist (J5–J7),
which every measurement shows loafing (≤36 % in all REAL runs). **TESTED SIM + REAL (§10.2/§10.4). Cap 1.0 is SAFE on top_left — H63
dwell 0 ms on every joint at every cap, all peaks ≤86 %, wrist cost
modest (J6 61→79 %, J5 ≤25 %, J7 ≤55 %) — so 1.0 is adopted as the
working cap. But the gain is smaller than the headline: genuine cruise
gain concentrates on LONG rotating segments (top_left only ~5 %); the
r≥25 duration drops were freeze-shrinkage. It is DOUBLE-EDGED —
throttling protects J4-critical strokes, so any cap raise requires
re-screening J4 at the new cap (hardware-validated: 65→80 % on
top_left).** Screen exactly
like the speed ladder: SIM first, one task, angular cap stepped 0.6 → 0.8
→ 1.0, H63 dwell rule on every joint.

### 2c. Acceleration and the scrub limit

acc = max(1.6, 3v) → at 0.8 the ramp is v²/2a = 133 mm each way: only
segments ≥ ~350 mm ever touch 0.8. Short strokes are acceleration-bound:
peak speed on a stroke of length L is √(aL) (triangle profile) — 0.44 m/s
on an 80 mm scrub at a=2.4. Human-scrub cadence lives here: an 80 mm
back-and-forth cycle costs ~0.4 s → **~2.5 Hz scrubbing is feasible** —
if the retraces stay crisp (measured: exact-180 reversals cross zero in
<200 ms; the apex reversal in rev 3 was <80 ms).

## 3. Wiping action — human motion mapped to controller primitives

What a hand does on a fixture, and the primitive that reproduces it:

| human element | robot primitive | status |
|---|---|---|
| long sweeping strokes on open surfaces | blended movel serpentine, padded turns | VERIFIED (§9.3g) |
| curved strokes following contours; rim/edge following | **`rm_movec`** chained arcs | **VERIFIED ON HARDWARE 2026-08-15** (§10.4): radial error median 0.37–0.43 mm (SIM 0.00), no stops, junctions ~2× faster than blended corners |
| scrub cycles on soiled spots | short exact-180 retraces at max accel (§2c), 2–3 Hz | components verified |
| flowing direction changes, no dead stops | blend chains + turnarounds in padding | VERIFIED |
| varying contact pressure | NOT commandable (position control + glove compliance); hover offset is the proxy | out of scope |

**`loop` — an untested multiplier for scrub cycles (from the vendor's own
dual-arm demo, `RMDemo_DoubleRoboticArm`, 2026-08-16).** That demo calls
`rm_movec(..., loop=2)`, and its English docstring documents the argument as
"Number of loops" (the SDK's own text reads `loop (int): 规划圈数` — Newton to
confirm the reading). If it means what the demo implies, ONE command wipes a
ring N times with **zero junctions and zero re-dispatch** — the natural
primitive for scrubbing an annular region, and a direct answer to the
"~2.5 Hz scrub" line in §2c. Our `chain_semantics_006` deliberately left
`loop` unscreened. **This is an IMPROVEMENT OPPORTUNITY, not a blocker** —
the vendor ships a working example of it, so nothing downstream waits on it.
When convenient, `chain_semantics_007` settles it in one SIM run: same arc
geometry, `loop` 0/1/2, measuring revolutions traced, whether `connect=1`
still chains after a looped arc, and the per-revolution time. Design against
chained single arcs today; adopt `loop` as a simplification if it verifies.

The single biggest wiping-action win is **movec for the annular regions**.
`seat_ring`, `bowl_inside_rim`, `bowl_inside_ring` are circles authored as
10–20-segment polygons: every vertex is a corner that fights the angular
cap and the blend engine. As 2–4 chained arcs they have NO corners — the
tool sweeps continuously like a hand wiping a ring, and the angular cap
becomes the only (honest) speed limit. This also directly attacks the
`seat_ring` family's 0.14 m/s ceiling: its J4 spikes live at polygon
vertices where translation direction snaps.

**Screen COMPLETE (`chain_semantics_006`, SIM + REAL — §10.1/§10.4):**
movec accepts `connect=1` mid-chain, traces the true 3-point circle
(0.00 mm median radial error in SIM, 0.37–0.43 mm on hardware), and
crosses its junctions without stopping at ~2× the speed of an equivalent
blended 90° corner. Tangent-arc entry also makes the first-corner
exemption moot. The annular redesigns may proceed on this primitive.

## 4. Proximal glove surfaces — the contact-area rule

Glove pads (glove_frames.yaml): frames 1/2 = 35×80 mm, t = 20 mm
compliance; frames 3/4 = 20×80 mm, t = 18 mm. Contact model (flat pad,
locally planar surface, misalignment θ between press axis and −normal):
the pad conforms up to depth t, so the contact band along the tilted axis
of dimension D is `min(D, t/sin θ)`:

    contact fraction  f = min(1, t / (D · sin θ))

* Tilt about the pad's SHORT axis (band runs along L=80): **f < ½ at
  θ > asin(2t/L) = 30.0°** (frames 1/2) or **26.7°** (frames 3/4).
* Tilt about the LONG axis (band across W=35): f = t/(W sinθ) ≥ 0.57 even
  at 90° — the width never drops below half. **The ½ rule therefore binds
  on lengthwise tilt only: keep the pad's LONG axis within 30° of the
  surface, or switch frames.**
* Curvature corrections (be careful — Newton's warning): on a CONVEX
  surface of radius R the flat-pad contact length is ~2√(2tR) — for
  R ≥ 80 mm this exceeds L and changes nothing; below R ≈ 40 mm it, not
  tilt, caps contact (seat-ring outer edges, rim lips). CONCAVE surfaces
  only help. Combined rule: f = min(f_tilt, 2√(2tR)/L on convex work).
* The 1.5 cm dimensional tolerance already derates the footprint for
  coverage spacing; the ½-area rule above is about PRESSURE/wipe quality,
  a separate check. Both must pass.

**Where it applies now:** the four over-limit families all press with
frame_1 (X) at stretched poses. Candidate: clean `front`/`side` upper
bands with frame_2 (Z-press, wrist rolled 90°) whose pose family the J4
screen scores far lower — legality decided by computing θ per cleaning
point (surface normal from the region OBJ) against the achievable tool
orientation, applying f ≥ 0.5. This computation belongs in the generator,
not in hand-tuning: emit `ik_frame` per STAGE, chosen by best f.

## 5. Path & overhead rationalization

* **Regenerate paths at glove-complete density with the §9 recipe**
  (two-pass serpentine, padded turns, per-move v/r, chained approach). The
  toplid case: original 6.2 m was under-covered; the correct-coverage
  redesign is 7.4 m but at a far higher achievable average. Some configs
  carry redundant double-passes (toplid's old rim down+up) — regeneration
  recovers those meters.
* **Task chaining.** The rest→prestart→approach entry per task was a TEST
  protocol. In production, consecutive tasks in the same articulation
  state should chain: task-end → next prestart directly. At 16 tasks/arm
  and ~5–8 s per full entry, this is **60–90 s of wall clock** — possibly
  the single largest line item after the slow families.
* **Schedule around articulation**: group closed-lid regions, then
  lid_open, then lid_seat_open (regions.yaml already encodes states);
  overlap UGV standoff moves with the other arm's cleaning where the
  pairing allows.

## 6. Task-config schema extension (backward compatible)

Bare `[a, b]` sequence entries stay legal (defaults apply). Extended form:

```yaml
cleaning_sequence:
  - [point1, point2]                          # legacy: task defaults
  - {seg: [point2, point3], v: 100, r: 35}    # per-move speed/blend
  - {arc: [point3, point5], via: point4, v: 80, r: 25}   # movec
  - {scrub: [point5, point6], cycles: 3, v: 100}         # retrace n times
stage_sequence:                                # per-stage frame switch
  - {stage: upper_band, ik_frame: L_glove_frame_2}
  - {stage: lower_band, ik_frame: L_glove_frame_1}
```

`v` is % of the task baseline (movel semantics), `r` per move; a scrub
expands to n exact-180 retraces (coverage-safe, §9.2). The generator owns
frame selection per stage via the §4 rule. stage_runner/test harness read
the same structure the screens already understand (R_LIST/V_LIST map 1:1).

## 7. The projected budget (honest, with today's numbers)

Efficiency factor measured (avg TCP / commanded cap): 0.70 at 0.25,
0.72 at the v45 mixed baseline. Applying family ceilings × 0.7, per arm:

| group | path | assumed avg | time |
|---|---|---|---|
| fast four (0.8 program) | ~4.6 m | 0.45 | ~10 s |
| medium (at ceiling, angular cap 0.6) | ~36 m | 0.22 | ~165 s |
| slow four (AFTER redesign, target ≥0.25 ceiling) | ~23.5 m | 0.17 | ~140 s |
| **total motion** | **64 m** | | **~315 s** |

**Over budget at today's angular cap — by design honesty.** The two
closures: (a) the angular-cap raise (§2b) plausibly moves the medium group
0.22 → 0.30+ (−45 s) and slow group similarly (−30 s); (b) task chaining
recovers 60–90 s of overhead that the activity budget must also contain.
With both, the cleaning phase lands at ≈ 210–240 s. **The 4-minute CLEANING target is
reachable, and it is reachable ONLY with the angular-cap program and the
slow-family redesigns — not with line speed alone.**

*This is the cleaning budget and the 6-minute whole-activity figure does not
relieve it (§0). At ≈ 210–240 s of cleaning, the remaining ~120–150 s of the
6-minute envelope is what bin drop/raise and the glove instances must fit
into — untested, since no glove or bin task has a measured duration yet.*

### 7b. Measured caution — the r ≥ 25 freeze hazard on dense geometry

The cap-ladder runs exposed that dense task paths (top_left, 42 waypoints)
spend up to 37 % of runtime in deterministic >1 s freezes at r = 25/50 —
r = 10 shows zero (§10.3). Until the freeze matrix characterizes the
mechanism, dense-geometry tasks run r = 10; large radii only on sparse
padded geometry; arcs preferred over blends at direction changes.

## 8. Execution order (each step SIM-gated like everything in §9)

1. `chain_semantics_006` — movec chain/blend semantics (SIM).
2. Angular-cap ladder on one cap-bound task (top_left): 0.6/0.8/1.0, SIM
   then REAL, H63 dwell on all joints.
3. Redesign the four over-limit families (direction/frame/standoff via §4
   rule + §5 recipe); screen; SIM; REAL at each family's ceiling.
4. Regenerate annular tasks as movec arcs; A/B against polygon versions.
5. Task-chaining entry in stage_runner (same-articulation groups).
6. Full-activity dual-arm dry-run in SIM; walltime audit vs §7 table.

Data gaps this closes en route: movec semantics, angular-cap scaling law,
J4-vs-direction maps for the slow families, dual-arm chained dispatch
(EMULATOR_ROADMAP B9).
