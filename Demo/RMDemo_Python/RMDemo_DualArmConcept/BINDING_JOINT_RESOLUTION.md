# Which joint binds — the J1/J4 question, settled against hardware

*2026-08-16. Settles the contradiction flagged in the handoff: `TASK_KINEMATICS`
H49 says "J1 is the joint that binds, not J4", while `CLEANING_MOTION_SPEC` §0
gates the whole speed programme on a J4 screen. Everything below is re-derived
independently from `runs/` and `plans/`, not quoted.*

**The answer: both are right about their own task, and neither is right as a
general rule. The binding joint is PER TASK — measured J4 on `toplid`, J1 on
`top_left`, J1/J5 on `hinge_area`. What does not survive is the idea that any
one joint gates a task.**

> **Read [§ The complete r=10 picture](#the-complete-r10-picture--every-real-run-2026-08-17)
> first — it is the authoritative table.** Every REAL run in the corpus at the
> mandated blend radius, with the selection rule and n stated. The partial
> tables in §2 came first and are narrower; two findings that were built on
> partial tables (H79, H80) are retracted below. Anything quoting `hinge_area`
> J4 at 95 % or "the binding joint moves with speed" is reading a pooled or
> label-filtered sample.

---

## 1. The two screens are not measuring the same object

Verified from source, not inferred:

| | J4 segment screen (`orientation_cost --segments`) | plan audit (`predict_task`, `TASK_KINEMATICS`) |
|---|---|---|
| input | the **Cartesian program** the runner dispatches | the **saved MoveIt plan's** joint velocities |
| covers | J4 only | all 7 joints |
| speed law | exact — `rate = (dJ4/ds) · v_eff`, `v_eff = min(v, cap/ω)` | **none** — a scalar `k` from a 5-point table |
| why only J4 | J4 is redundancy-invariant (H66): the elbow angle is a function of shoulder→wrist distance, which the commanded pose fixes | the plan pins the redundancy, so all 7 become readable |
| what resolves the redundancy | nobody — the answer is the same for every arm angle | **MoveIt** |

`k` is not a function of commanded speed: `k(0.45, 1.6) = 0.925` and
`k(0.45, 3.6) = 1.085` — same line speed, different `k`. So the plan audit
cannot answer "at what commanded speed does J1 go over?", and the J4 screen
cannot answer "what does J1 do at all". They were never in a position to
contradict each other.

### 1a. The plan's redundancy resolution is not the one that executes

`execute_path` dispatches **chained `rm_movel`** at `cleaning_speed_pct` of the
controller's `line_speed` cap ([stage_runner.py:333-400](src/stage_runner.py#L333-L400));
the saved plan supplies only *named-pose joint values for `movej`*
([stage_runner.py:24-26](src/stage_runner.py#L24-L26)). The plan's joint
trajectory is **never played back for the cleaning path**. The controller
re-solves the redundancy itself, so every non-J4 number in the plan is a proxy
for a quantity the controller computes its own way.

---

## 2. What the hardware says

All 13 REAL cleaning recordings, plus the 9 REAL `top_left` cap-ladder runs.
Peaks are `d(position)/dt` over the `execute_path` window, central difference
across ±1 sample — the only estimator valid in both SIM and REAL, since SIM's
`speed{n}` channel is dead while its position channel is faithful
(`MODE_CHARACTERIZATION` §1).

**Method check before any of it is used.** This extraction reproduces
`MODE_CHARACTERIZATION` §2's published SIM peaks for `toplid_left` @0.80 to
within ~7 % (`J1 195/210, J3 228/241, J4 268/272`), and reproduces §10.4's
published "J1 runs 83–86 %" on `top_left` **exactly**, on the reported channel.

### 2a. `toplid` — J4, as the J4 screen says

| run | ls / la | J1 | J2 | J3 | J4 | J5 | J6 | J7 | binds |
|---|---|---|---|---|---|---|---|---|---|
| 20260811T220322 | 0.45 / 1.6 | 88 | 58 | 90 | **93** | 24 | 54 | 38 | J4 |
| 20260811T220453 | 0.45 / 2.4 | 95 | 61 | 102 | **103** | 26 | 56 | 39 | J4 |
| 20260811T220617 | 0.45 / 3.6 | 111 | 66 | **115** | 111 | 28 | 65 | 36 | J3 |
| 20260811T184109 | 0.80 / 2.4 | 112 | 81 | 116 | **124** | 35 | 106 | 80 | J4 |
| 20260811T222354 (R) | 0.50 / 3.6 | 91 | 72 | 80 | **116** | 20 | 67 | 30 | J4 |

Plan says J4 for both `toplid_left` and `toplid_right`, and it is right on 11 of
the 12 toplid runs. **Note the 0.80 row: four joints over limit at once.** J4 was
merely the highest.

### 2b. `top_left` — J1, at only 0.25 m/s. H49 is confirmed here

Nine REAL runs, `line_speed 0.25`, cap 0.6 → 0.8 → 1.0.

| cap | r | reported `speed{n}` J1 | d(pos)/dt J1 | d(pos)/dt J4 | binds |
|---|---|---|---|---|---|
| 0.6 | 10 | 85 % | 100 % | 77 % | J1 |
| 0.6 | 25 | 83 % | 99 % | 84 % | J1 |
| 0.8 | 10 | 86 % | 102 % | 95 % | J1 |
| 1.0 | 10 | 86 % | 103 % | 94 % | J1 |
| 1.0 | 25 | 86 % | 104 % | 84 % | J1 |
| 1.0 | 50 | 65 % | 78 % | 79 % | J4 |

**The rank is the finding, not the magnitude.** J1 outranks J4 at r=10 and
r=25 on every cap, on hardware, at a commanded 0.25 m/s — and that holds under
every estimator tried. The magnitude does **not** hold: J1's peak reads

| estimator | J1 peak, `top_left` r=10 cap 0.6 |
|---|---|
| reported `speed{n}` | **86 %** |
| d(pos)/dt, 7-sample window (the house `SPEED_WINDOW`) | **90 %** |
| d(pos)/dt, ±1 sample (20 ms) | **100 %** |

so "J1 is at its limit" is a statement about the narrowest window only, and
the d(pos)/dt column above is the ±1-sample read. Quote **90 %** — the house
window — for any decision, and treat the spread as the reason a magnitude
needs its estimator named (H78, and SPEED_INVESTIGATION's standing warning
that four defensible estimators over one window disagree by 2.6×–16.6×).

The plan called this correctly (`top_left` plan: J1 at 83 %). This is the case
H49 is right about, and it is not a planning artifact.

### 2c. `hinge_area_left` — J5, which neither screen names

| | J1 | J2 | J3 | J4 | **J5** | J6 | J7 |
|---|---|---|---|---|---|---|---|
| REAL 20260810T173010 | 42 | 17 | 42 | 26 | **50** | 28 | 29 |
| SIM 20260810T172759 | 41 | 15 | 38 | 25 | **45** | 27 | 26 |
| **plan** (util) | **94** | 44 | 78 | 72 | 90 | 46 | 49 |

The plan ranks J1 first by 4 points; hardware ranks J5 first by 8. Stable across
every smoothing width tried. The J4 screen never had an opinion.

### 2d. The scoreboard

| predictor | covers | named the REAL binding joint |
|---|---|---|
| J4 segment screen | J4 only, exactly | correct wherever J4 is in fact hottest; silent otherwise |
| saved plan (`predict_task`) | 7 joints, via **MoveIt's** redundancy | **11/13**, not 13/13 |
| **SIM** | 7 joints, via the **real controller** | **5/5** on matched pairs, under-reading REAL by +1..+19 pp |

With the estimator forced the same on both sides (`analyse_run.py --rates
derived`), the `top_left` r=10 cap-1.0 SIM/REAL pair agrees to **0–7 points on
every one of the seven joints** and names J1 worst in both:

```
        J1   J2   J3   J4   J5   J6   J7    worst
SIM     90%  41%  55%  81%  26%  80%  58%   J1
REAL    90%  42%  55%  82%  26%  87%  58%   J1
```

That is the evidence for gating on SIM, and it is why the estimator must be
forced: left on `auto` the SIM side falls back to d(position)/dt while REAL
keeps the reported channel, and the comparison then prices the estimator
rather than the mode.

Both plan misses are near-ties under 5 points (`hinge_area` J1 94 vs J5 90;
`toplid` @3.6 J3 115 vs J4 111 measured). The plan ranks reliably when its
margin is wide and flips when it is narrow — ordinary argmax behaviour, worth
stating only because "13/13" reads as exact.

---

## 3. What this means for the §0 contract

1. **Keep the J4 screen.** It is exact in kind and calibrated conservative:
   predicted 85 % at raised caps against 78–80 % measured (§10.4); 71 % against
   59.5 % on toplid rev 3. It is the only thing computable offline for a task
   that has never run, which is 20 of 24.
2. **Stop treating ≤90 % J4 as a clearance.** §0 item 1 lets a move carry
   0.8 m/s "only where its family's cap-aware J4 screen passes at ≤90 %". On
   `top_left` that gate passes at 0.25 m/s while J1 sits at 100 %. The gate is
   **necessary, not sufficient**, and needs a second condition: a SIM run with
   **all seven** joints under limit. §0's own Process line already says
   screen → SIM → REAL; it is item 1's wording that overstates the screen.
3. **`top` cannot go to its §2 table ceiling of 0.35 m/s.** It is J1-bound at
   or below 0.25. The §2 row calls it "cap-bound" on J4 evidence and misses the
   joint that actually limits it. Every other J1-heavy row in that table is
   suspect for the same reason and none of them has been run.
4. **`max_velocity_scaling` is not a lever.** `TASK_KINEMATICS` §4 proposes
   lowering it to buy margin. It appears only in the task-config YAMLs, is read
   by nothing in the runtime, and cannot affect a path executed as chained
   `movel`. The levers that exist are `cleaning_speed_pct` / `line_speed`,
   `line_acc`, the angular cap, blend radius, and the path geometry itself.
5. **`line_acc` governs achieved speed more than `line_speed` does** at these
   segment lengths. At a fixed `line_speed 0.45`, `toplid_left` achieved
   282 → 347 → 394 mm/s as `line_acc` went 1.6 → 2.4 → 3.6, never reaching the
   commanded 450. This is §2c's "only segments ≥ ~350 mm ever touch 0.8"
   observed directly, and it means per-move `v` is the weaker half of the pair.

### What is still not screened

Nothing offline predicts J1, J2, J3, J5, J6 or J7 for a task that has never run.
Not the plan (11/13, and its one clear J1 call succeeded while its J5 ranking
failed), not the emulator (6/24, chance — `MODE_CHARACTERIZATION` §3), and not a
per-metre re-derivation of the plan's own geometry: that was tried here and
over-reads J1 by 1.85× on `toplid_left` while under-reading J6/J7, and then
under-reads J1 by 1.4× on `top_left`. Unreliable in both directions, so it is
not offered as a tool. **SIM is the only complete predictor, and it is cheap.**

---

## Recorded findings

* **H73** — **THE BINDING JOINT IS PER TASK, and three different joints have
  been measured binding on three tasks** *(extended by H79 — it is per task
  AND per SPEED; "per task" alone is incomplete)*: J4 on `toplid_left/right`, **J1 on
  `top_left`** (highest joint at a commanded 0.25 m/s, at every cap, r=10 and
  r=25, under every estimator — 86 % reported / 90 % house window / 100 %
  at ±1 sample), J5 on `hinge_area_left`. Neither "J4 binds" nor "J1 binds"
  is a general fact, and a single-joint gate cannot clear a task.
* **H74** — **A J4 pass is not a clearance, demonstrated.** `top_left` passes any
  cap-aware J4 screen at 0.25 m/s (J4 61–95 %) while J1 is the constraining
  joint above it. And on `toplid_left` @0.80 **four joints were over limit
  simultaneously** (J1 112 %, J3 116 %, J4 124 %, J6 106 %, ±1-sample read) —
  J4 was the highest, not the only one.
* **H75** — **the plan named the REAL binding joint 11/13, not 13/13.** Misses:
  `hinge_area_left` (plan J1, measured J5) and `toplid_left` @0.45/3.6 (plan J4,
  measured J3). Both are sub-5-point near-ties in the measurement. Corrects
  `MODE_CHARACTERIZATION` §4 and the supporting clause in H66.
* **H76** — **`max_velocity_scaling` cannot buy execution margin.** The cleaning
  path never plays back the plan; `execute_path` is chained `rm_movel` at
  `cleaning_speed_pct` ([stage_runner.py:333-400](src/stage_runner.py#L333-L400)).
  The parameter lives only in the task-config YAMLs and no runtime code reads
  it. Retires the remedy proposed in `TASK_KINEMATICS` §4.
* **H77** — **`line_acc`, not `line_speed`, sets achieved speed on these
  segment lengths.** `toplid_left` at a fixed `line_speed 0.45` achieved
  282 / 347 / 394 mm/s at `line_acc` 1.6 / 2.4 / 3.6, against a commanded cap of
  450 mm/s it never reached. J4 utilisation tracked it 82 / 87 / 94 %.
* **H78** — **the two joint-speed estimators differ by ~20 % and the choice
  changes verdicts, not rankings.** On `top_left` the reported `speed{n}`
  channel gives J1 83–86 % where `d(position)/dt` gives 99–104 %; on the
  `toplid` failure run, 106 % against 124 %. Rank order is preserved. Fix one
  estimator per decision and say which — H63's dwell rule was calibrated on the
  reported channel.

---

## RETRACTED 2026-08-17 — H79 and H80, and the method that produced them

**H79 and H80 are withdrawn. They were recorded from a peer session's reported
table without re-measuring it, and re-measurement does not support them.**
Newton disputed the hinge_area result; he was right. Everything below is
measured directly from `runs/` in this repo.

### What the cited evidence actually is

H80 rested on run `20260814T200622`, described as "hinge_area_right, r=25,
0.25 m/s" showing J1 at 99.6 % with 30 ms of H63 dwell. The run is
`blend_r25_v250_right` — a **blend-characterisation run** on the
hinge_area_right path, at **r=25**.

`r=25` is a blend radius this project has already ruled out for dense geometry.
MOTION_FINDINGS §10.3: *"the r >= 25 freeze hazard is EVERYWHERE on dense task
geometry ... r=10 shows zero"*, and CLEANING_MOTION_SPEC §0 mandates **r=10 on
dense geometry**. Judging a family's speed ceiling from a run at a forbidden
blend radius is not evidence about the family.

### hinge_area_right — all six REAL runs, reported channel

| run | r | J1 | J4 | J5 | binds | dwell >=98 % |
|---|---|---|---|---|---|---|
| 20260814T200053 | 10 | 76 | 49 | **83** | J5 | 0 ms |
| 20260814T200132 | 25 | 71 | 39 | **85** | J5 | 0 ms |
| 20260814T200205 | 50 | **83** | 52 | 81 | J1 | 0 ms |
| 20260814T200542 | 10 | 68 | 51 | **83** | J5 | 0 ms |
| **20260814T200622** | **25** | **100** | 52 | 85 | J1 | **30 ms** |
| 20260814T200657 | 50 | 67 | 51 | **84** | J5 | 0 ms |

**The cited run is one outlier in six.** Its same-configuration twin
(`200132`, identical path, r and speed) reads J1 at **71 %**. J1 across the six
spans 67–100 % — a 33-point spread on one path at one operating point, which is
run-to-run variation, not a property of the family. J4 is stable at **39–52 %**
throughout, which is what the J4 screen says. **Five of six runs carry zero
dwell.**

### hinge_area_left — both speeds, and the confound is blend radius

| v | r | J1 | J4 | J5 | binds | dwell |
|---|---|---|---|---|---|---|
| 0.25 | **10** | 77 / 67 | 49 / 52 | 73 / 62 | J1 | 0 ms |
| 0.25 | 25 | 77 / 85 | 51 / 52 | 76 / 65 | J1 | 0 ms |
| 0.25 | 50 | 73 / 69 | 52 / 52 | 73 / 67 | J5 / J1 | 0 ms |
| 0.45 | **10** | 79 | **63** | 75 | J1 | 0 ms |
| 0.45 | 25 | 79 | **95** | 83 | J4 | 0 ms |
| 0.45 | 50 | 79 | **95** | 75 | J4 | 0 ms |

**H79's "the binding joint moves with speed" is an artifact of blend radius.**
J4 reaches 95 % only at r=25 and r=50. At the mandated **r=10**, J4 goes
49–52 % at 0.25 and **63 %** at 0.45 — it never approaches its limit, and J1
remains the highest joint at both speeds. At the contract's operating point the
binding joint does **not** change with speed on this family.

**Zero H63 dwell in all nine hinge_area_left runs**, at both speeds and all
three radii.

### What this leaves standing, and what it corrects in this document

* **H80 — withdrawn.** `hinge_area`'s 0.35 ceiling is not shown unsafe. The
  single 30 ms exposure is one run at a forbidden blend radius whose twin shows
  71 %.
* **H79 — withdrawn.** The binding joint did not move with speed; the blend
  radius moved, and it was set outside the contract.
* **§2c above is also too strong.** It called `hinge_area_left` J5-bound from
  one pendant-override run. The blend corpus shows **J1 and J5 are both hot on
  this path — 62–85 % at 0.25, neither near limit — and which one is highest
  varies run to run.** Read §2c as "J1/J5 are the hot pair here", not as a
  binding-joint verdict.
* **H73 stands, and is unaffected.** `toplid` binds J4 and `top_left` binds J1,
  both reproducibly and both measured here from the runs. The claim that no
  single-joint gate clears a task does not depend on hinge_area.

### The method failure, recorded because it is the reusable lesson

These two findings entered this document from a peer session's summary table.
I verified its *provenance* — that the run existed, that a hash resolved — and
did not re-measure the *numbers*. Provenance is not corroboration. **A finding
is not corroborated until the claim itself is re-derived from the primary data
in this repo**, which is the sole source of truth for the emulator, the motion
requirements and the findings. Two consequences worth carrying:

1. **Always read the run's own configuration before citing it.** `r`, `v` and
   the angular cap decide whether a number describes the family or describes a
   setting the contract forbids. A run named `blend_*` is a characterisation
   sweep, not a task at its operating point.
2. **One run is not a measurement.** Both retracted findings would have failed
   the moment the sibling runs were read.

---

## The complete r=10 picture — every REAL run, 2026-08-17

*This replaces every partial table in this document. Selection rule stated so
it can be checked: all 200 run directories read; 94 are REAL with a live
`speed{n}` channel; 27 of those are at r=10. Family resolved from `run.json`'s
top-level `path_file`, not the directory name. Peak per joint as % of
`limits_in_force.joint_speed`, reported channel. H63 dwell is time at ≥98 % of
a limit, max over joints. Nothing pooled across radii.*

**Reproduce it — do not re-type it, and do not write another one-off:**

```
python3 src/survey_binding.py                 # exactly the table below
python3 src/survey_binding.py --radius all    # every radius, grouped
python3 src/survey_binding.py --family toplid --per-run
```

`survey_binding.py` was written *because* all three retracted findings came
from one-off scripts that each selected a different wrong subset. It imports
`analyse_run` rather than re-deriving, so the repo has one estimator; it
excludes SIM outright; it resolves families from `path_file`; and it prints
its selection rule and n above every table. The 40 REAL runs on synthetic
geometry (`blend_corner_001.py`, `test_motion_001.py`) are labelled
`synth:*` rather than left blank, so they cannot be silently merged into a
cleaning family. Every REAL run in the corpus is attributed — none fall
through.

| family | v | J1 | J4 | J5 | binds | dwell | n |
|---|---|---|---|---|---|---|---|
| `top_left` | 0.25 | **84.6–85.9** | 65.1–80.3 | 18–25 | **J1 ×3** | 0 | 3 |
| `hinge_area` | 0.25 | 66.6–77.4 | 49.5–52.1 | 61.6–82.8 | **J1 ×2, J5 ×2** | 0 | 4 |
| `hinge_area` | 0.45 | **79.0** | 63.2 | 75.1 | **J1** | 0 | 1 |
| `toplid` | 0.25 | 25.8–57.6 | 27.8–58.4 | 8–20 | J4 ×5, J1 ×2 | 0 | 7 |
| `toplid` | 0.45 | 66.6–93.8 | 75.0–95.6 | 13–25 | **J4 ×7**, J1 ×1 | 0 | 8 |
| `toplid` | 0.50 | 80.3 | 96.9 | 16.2 | J4 | 0 | 1 |
| `toplid` | 0.60 | 36.7 | **99.8** | 8.7 | J4 | **110 ms** | 1 |
| `toplid` | 0.80 | 103.3 | **105.9** | 28.3 | J4 | **330 ms** | 1 |

**H73 is confirmed on the full corpus and the binding joint is a property of
the task:** `toplid` → J4, `top_left` → J1, `hinge_area` → J1/J5. **No family
changes its binding joint with speed at fixed r=10.** That is the claim H79
made and could not show.

**Two corrections to a peer re-derivation, recorded because they are safety
statements** (RM_API2 session, same day; it had fixed its own radius-pooling
defect but resolved families from run *labels*, so it saw only the 2026-08-14
`blend_*` runs and missed the 2026-08-10/11 task-named runs on the same paths
— verified byte-identical in `commanded.poses`, 28 waypoints, 27 segments,
same tool frame):

1. **"Every REAL run at r=10 carries zero H63 dwell" is false, and the error
   matters.** The only two dwell exposures in the whole REAL corpus —
   `20260811T222451_toplid_right` (110 ms) and `20260811T184109_toplid_left`
   (330 ms) — are **both at r=10**. They are the runs H63 itself was calibrated
   on. **What separates them is line speed, not blend radius:** 0.6 and
   0.8 m/s, above any production cap. Say it as *"no r=10 REAL run at or below
   0.5 m/s carries dwell"*; attaching the safety property to the radius
   mis-attributes H63's own evidence and would let an over-speed run inherit a
   clean bill from its radius.

   > **r=10 on those two runs is RECORDED, not inferred** — checked because a
   > peer was about to caveat it as inferred from `stage_runner` dispatch.
   > Both carry `commanded.blend_pct = 10`. The confusion is that the
   > **top-level** `blend_pct` is `None` — on *every* run in the corpus, not
   > just these — so reading that field alone makes any run look unrecorded.
   > Across all 94 REAL runs, `commanded.blend_pct` is present on **94 of 94**
   > (0:12, 10:27, 25:28, 35:2, 50:25). Nothing is inferred anywhere in this
   > table. `survey_binding.py` reads the `commanded` field, says so in its
   > selection-rule header, and now names any run whose radius is unrecorded
   > instead of dropping it from a `--radius` filter — a silent drop is the
   > same failure as a wrong sample, spelled differently.
2. **`toplid` at 0.45 is J4-bound, 7 of 8 runs.** The lone J1 run (89.2 vs
   86.7, a 2.5-point margin) is the one a label-based filter caught. The
   proposed "J4-bound at 0.25, J1-bound at 0.45 crossover" was correctly
   flagged tentative and does not survive: it is 1-in-8 against 7 runs binding
   J4 on the identical path.

### The SIM corpus, added 2026-08-17 — and a fourth subset error, mine

Newton: *"why did you not use the SIM runs, they have good data, that forms the
ground breaking work for the cleaning motion/path geometry."* He is right, and
this was the same failure a fourth time.

`survey_binding.py` originally **excluded SIM outright**, reasoning from its
dead `speed{n}` channel. That discarded **106 of the corpus's 200 runs — more
than REAL contains** — when the repo's own tooling recovers those rates from
the faithful position channel (`analyse_run --rates derived`,
MODE_CHARACTERIZATION 1). What went with them:

* the whole **`chain_rmix_vmix_capp`** family — mixed radius, mixed speed,
  angular cap applied, i.e. **the production motion form** — which is largely
  SIM. A screen blind to the production form is not a screen.
* three families with **no REAL run at all** (`blend_r25_capp`, `chain`, and
  `hinge_area_right` at some configurations).
* the evidence that SIM predicts REAL, which is the whole argument for using
  SIM as the seven-joint screen the J4 gate cannot be.

**SIM vs REAL, matched (family, v, r), forced `derived` on both sides so the
comparison prices the mode and not the estimator (H78). r=10:**

| family | v | n SIM/REAL | binds SIM | binds REAL |
|---|---|---|---|---|
| `top_left` | 0.25 | 3/3 | **J1 (3/3)** | **J1 (3/3)** |
| `toplid` | 0.25 | 9/7 | **J4 (8/9)** | **J4 (6/7)** |
| `toplid` | 0.45 | 4/8 | **J4 (3/4)** | **J4 (7/8)** |
| `toplid` | 0.50 / 0.60 / 0.80 | 1/1 each | J4 | J4 |
| `hinge_area` | 0.25 | 7/4 | J5 (3/7) | J1 (2/4) — *disagree* |

**Binding joint agrees on 6 of 7 matched configurations at r=10, and on 31 of
33 across all radii** — `survey_binding.py --pairs`, logic stamp
`186b743034bb`.

> **This figure moved twice in one hour, and the two moves have OPPOSITE
> character. Read the labels before citing either.** The first was a defect —
> a stale number that should never have been published. The second was a
> definition improving — a comparison that was never valid being removed, so
> the ratio moved because the exclusion was real. A reader six months out
> cannot reconstruct which was which from the numbers alone, and "corrected
> three times" reads as an eroding figure when in fact it ends more trustworthy
> than it started.
>
> **First: 33/34 → 31/34, a real error, mine.** The `--pairs` implementation
> took the max of each joint ACROSS runs and then arg-maxed — pooling, the
> same defect retracted from H79. I fixed it to per-run modal counting, re-ran
> `--radius 10`, and **did not re-run `--radius all`**. So a corrected figure
> (6/8) and a stale one (33/34) stood in the same sentence, both credited to
> the fixed tool. It had been quoted to four sessions first.
> **Re-run every figure a fix touches, not the one that prompted the fix.**
>
> **Second: 31/34 → 31/33 and 6/8 → 6/7, a methodology correction, not an
> error** (raised by rm-api2-15). One "matched" configuration was
> `hinge_area, v=unrecorded, r=10`. Its four runs — the corpus's earliest,
> 2026-08-10 16:09–17:31 — **predate the `line_speed_cap_m_s` field, which is
> absent from their `run.json` rather than null.** Comparing SIM against REAL
> there matches on a variable neither side records. They were probably at the
> same speed; probably is not a match. It is now excluded from the count and
> **named in the output** rather than dropped, because a silent exclusion is
> the same failure as a wrong sample. It happened to be a miss, so removing it
> raises both ratios — which is exactly why the reasoning has to be stated
> before the number is quoted.
>
> **Every figure this tool prints now carries a `logic stamp`** — a hash of
> `survey_binding.py`. A figure quoted with a different stamp came from
> different code and is stale, whatever it looked like when computed. That is
> the only check that catches the first failure above, since the number was
> correct when computed and nothing about "33" looks wrong.
> *(Idea: rm-api2-15.)* Both disagreements are on `hinge_area`, the one family
already established as J1/J5 both-hot and run-to-run variable — so SIM
reproduces even the instability. On the production form
(`chain_rmix_vmix_capp_v250_left`, SIM `20260815T153521` vs REAL
`20260815T153752`) every joint agrees within **one point**:

```
        J1   J2   J3   J4   J5   J6   J7   worst
SIM    41%  36%  43%  62%  12%  32%  20%    J4
REAL   40%  36%  42%  63%  13%  33%  21%    J4
```

**This is the evidence for §0's Process line and for the gate amendment.** SIM
screens all seven joints, it tracks REAL's binding joint, and it is free. What
SIM must NOT do is issue an H63 dwell verdict — derived rates run high with a
sign, so SIM dwell is printed as `(adv)` and never quoted as an exposure.

*A defect found while writing this and worth recording: the first `--pairs`
implementation took the max of each joint ACROSS runs and then arg-maxed. On
`toplid` 0.45/r=10 that reported J1 — 95 vs 94, from two different runs —
while seven of the eight runs individually bind J4. Pooling again, in new
clothes. The binding joint is a per-run fact; `--pairs` now counts runs and
prints `J4(7/8)`, and labels the joint columns as an envelope so nobody reads
a verdict off them.*

**Fourth method rule:** *a channel being unusable is not the same as a run
being unusable.* Check whether another channel answers the question before
discarding the recording — and state how many runs a filter removed, because
"SIM excluded" hid the loss of more than half the corpus behind three words.

---

**Third method rule, from the label error:** *resolve a run's family from
`path_file`, never from its label, and state n with every table.* Run-naming convention
changed between sessions; a label filter silently drops half the corpus. All
three failures in this document have the same shape — **a subset mistaken for
the population** (pooling took too much, labelling too little, single-run
citation took one). Publishing the selection rule and the n makes each visible
on sight, which is why both are stated at the top of the table above.

---

## Superseded — the peer table this section originally carried

*Kept for the audit trail. Retracted above; do not cite.*

## Corroborated and extended, 2026-08-17 — an independent REAL-run audit

*The RM_API2 session re-measured every REAL task run in the corpus (reported
channel, peak per joint plus H63 dwell) without using this document's method.
It confirms the conclusion, and then goes past it in a way that corrects H73.*

| family | v | J1 | J4 | binding | H63 dwell |
|---|---|---|---|---|---|
| hinge_area | 0.25 | **99.6** | 52.5 | **J1** | **30 ms on 1 run of 12** |
| top | 0.25 | **86.3** | 80.3 | **J1** | 0 |
| toplid | 0.25 | 56.2 | 62.6 | J4 | 0 |
| toplid | 0.45 | 89.2 | **98.2** | J4 | 0 |
| hinge_area | 0.45 | 79.5 | **95.2** | J4 | 0 |

**`top` independently confirmed.** J1 84.6–86.3 % against J4 57.8–80.3 % at
0.25, and **flat across angular caps 0.6 / 0.8 / 1.0** — so it is a line-speed
and pose property, not a cap artifact. That closes the one alternative
explanation this document could not rule out from `top_left` alone. Their
figure puts the J1-limited ceiling near **0.266 m/s** under the 90 % rule.

### H79 — the binding joint moves with SPEED, not only with task

`hinge_area` is **J1-bound at 0.25** (99.6 %, J4 at 52.5 %) and **J4-bound at
0.45** (95.2 %, J1 at 79.5 %). The same family, two speeds, two different
binding joints — and they cross.

**This corrects H73.** H73 concluded "the binding joint is per task" from three
tasks measured at one speed each; that was the right conclusion from the
evidence available and still an incomplete statement of the rule. The rule is:
**a binding joint is only defined for a (task, speed) pair.** Consequently a
single-joint ceiling is not a ceiling — screening a family at 0.25 says nothing
about which joint binds at 0.45, and the answer can change identity, not merely
magnitude.

It also explains something this document left as a loose end: the J4 screen
*looked* well-calibrated on `toplid` (screen 71 % vs measured 59.5 %) because
`toplid` is J4-bound at both speeds tested. It was validated on the one family
where its blind spot cannot show.

### H80 — `hinge_area`'s listed ceiling of 0.35 m/s is unsafe

Run `20260814T200622` (hinge_area_right, r=25, **0.25 m/s**) put **J1 at 99.6 %
with 30 ms of H63 dwell at >=98 %** — the only dwell exposure anywhere in the
0.25 corpus — while **J4 sat at 52.5 %**. The J4 screen licensed **0.35** for
that family on the strength of that 52.5 %.

30 ms is below the 110 ms that stalled `20260811T222451`, so the run completed.
That is the point: it completed while carrying the only measured H63 exposure in
the corpus, on a joint the gate does not look at, at a speed **below** the one
the gate licensed.

### What this does NOT change

The J4 screen stays. It remains the only thing computable offline for a task
that has never run, and it remains sound in the direction that matters — what
it flags is real. What the audit removes is any remaining argument that it
could be *extended* into sufficiency: J4 is screened **because** it is
redundancy-invariant, and J1/J5 are not offline-computable without a saved plan
precisely because they move with the controller's null-space resolution. The
screen is structurally necessary-not-sufficient, not accidentally so.

**Gate amendment proposed, not applied.** `CLEANING_MOTION_SPEC` §0b now
carries the table above and a proposed amendment to §0 item 1 — J4 screen
<=90 % **and** the family's first REAL run at that speed audited across all
seven joints with H63 dwell = 0 ms. §0 is an agreed contract with Newton, so
neither session applied it. It is flagged for his sign-off.
