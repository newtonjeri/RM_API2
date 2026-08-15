# Motion findings — seven questions, answered from the data

*2026-08-14. Every number here was recomputed after the RM75 model was
verified (`ROBOT_MODEL.md`), and re-checked a second time before this was
written. Two claims from the first pass did not survive that re-check and are
retracted in §0 — read that first.*

**Method, so the numbers can be trusted or attacked:**

* Joint utilisation is `|q̇| / limit` from the controller's own speed channel,
  never differenced from position (the position field is not synchronous with
  the 100 Hz push — see `project-udp-position-channel-aliases`).
* TCP speed is differenced over a **70 ms window**; single-sample
  differencing reads up to 162 % of a cap the arm never exceeded.
* Everything task-related is restricted to the **`execute_path` stage** using
  the stage boundaries in `run.json`. The approach stages are `movej` and
  reach joint limits legitimately; mixing them in inflates every statistic.
* **SIM runs are excluded from all joint-load figures.** Simulation populates
  positions but leaves the joint-speed channel at zero, so a SIM run reads
  0 % utilisation regardless of what it commanded. 14 of the 27 toplid runs
  are SIM.

---

## 0. Two retractions from the first pass

**R1 — "toplid fails at 35 % of 1.8 m/s" was based on a run that never ran
the stroke.** `20260811T222451_toplid_right_right` has stages only as far as
`open_tenth`; there is no `execute_path`. The 100 % utilisation and 110 ms
dwell I quoted came from the **approach**, which is `movej` and is *supposed*
to use the full joint rate. The stage filter fell through silently to the
whole run. Corrected ladder in §1.

**R2 — the "joint-rate gain" metric is invalid and is withdrawn.** It divided
peak joint utilisation by TCP *linear* speed to predict a speed ceiling. But
`q̇ = J⁻¹[v; ω]` — joint rate is driven by the tool's angular velocity as
well, and our paths are rotation-heavy. Tested against a run whose outcome is
known, it predicts a ceiling of **0.13–0.35 m/s for a stroke that completed
at 0.45 m/s with J4 at 96 %**. The figures "worst point allows 0.38 m/s" and
"median 1.19 m/s" are wrong. No tool was built on it.

What replaces both: the measured ladder, which needs no model.

---

## 1. Why does toplid fail, at a fraction of 1.8 m/s?

**Because the binding constraint is joint rate, not TCP speed — and the
failure predictor is DWELL near the limit, not the peak.**

Every REAL toplid run that reached the cleaning stroke:

| side | cap m/s | worst joint | dwell ≥98 % | TCP p95 | outcome |
|---|---|---|---|---|---|
| left | 0.25 | 28 / 47 / 58 % | 0 ms | 106–266 mm/s | completed ×3 |
| left | 0.45 | 79 / 86 / 87 / **96 %** | 0 ms | 461–474 mm/s | completed ×4 |
| **left** | **0.80** | **106 %** | **330 ms** | 717 mm/s | **DID NOT FINISH** |
| right | 0.45 | 75 / 84 / 92 % | 0 ms | 460–475 mm/s | completed ×3 |
| right | 0.50 | **97 %** | 0 ms | 522 mm/s | completed |
| right | 0.60 | — | — | — | **never reached the stroke** |

Three things this settles:

1. **The premise "both arms fail" is too strong.** Eleven of twelve REAL
   strokes completed. There is exactly **one** stroke failure — left at 0.80 —
   and one *approach* failure — right at 0.60, which stopped before
   `execute_path` and is therefore not a cleaning-motion problem at all.
2. **Peak utilisation does not predict failure; dwell does.** 96 % and 97 %
   both completed with **0 ms** above 98 %. The failure sat at 106 % with
   **330 ms** — the same signature H63 identified (0 ms completed / 110 ms
   stalled / 330 ms violent).
3. **1.8 m/s is unreachable on this geometry.** At the failure the tool was
   doing 717 mm/s, 40 % of the vendor maximum, with a joint already over its
   limit. 1.8 m/s is a specification for configurations where the Jacobian
   maps TCP velocity into small joint rates; a cleaning stroke that must hold
   a tool against a surface while re-orienting is not one of them.

**Operating ceiling on this path: 0.45–0.50 m/s with roughly 3–20 % of
headroom, and no margin at all above that.**

---

## 2. Do blend radius and `connect` work? Can `connect` alone hold speed?

**Blend works. `connect` alone does not — and the answer to the direct
question is no.**

REAL blend-path runs, corrected corner locator:

| case | median speed retained | n |
|---|---|---|
| `connect=1`, `r=0`, every corner | **2.4 %** | 20 |
| `connect=1`, `r=25/50`, **corner 1** | **3.6 %** | 7 |
| `connect=1`, `r=25/50`, corners 2–5 | **58.1 %** | 28 |

`connect=1` only means *"plan this with the next segment, do not execute
yet"* — it is a latch, not a smoothing mode. The **radius** is what rounds
the corner. With `r=0` the tool stops dead at every waypoint even though the
whole chain is connected.

Confirmed independently by the controller's own state machine, which owes
nothing to our speed estimator:

* `r=0` → the entire chain runs as **one unbroken `MOVE_L`**, and the tool
  stops *inside* it.
* `r=25/50` → the controller **leaves `MOVE_L` at each blend** with the tool
  still travelling 137–287 mm/s. That is the blend arc between two queued
  trajectories.

**The first corner is never blended, at any radius.** It reads 3.6 %, which
is statistically the same as `r=0`'s 2.4 %, and `--reverse` proved the effect
follows corner *position*, not corner angle. On `test_motion_001`, 22 of 34
corners show blend transitions.

`stage_runner` dispatches **r=10**, below anything measured here.

---

## 3. Are the cleaning tasks inside the advised operating volume?

**Yes, comfortably — and singularity is not a factor.** REAL runs,
`execute_path` only:

| task | max reach from shoulder | % of 627 mm | min \|J4\| |
|---|---|---|---|
| toplid_left | 543–547 mm | **87 %** | 31–35° |
| toplid_right | 539–547 mm | **86–87 %** | 34–38° |
| hinge_area_left | 580 mm | 93 % | 47° |

Type-2 singularity is `q4 = 0`; the closest any stroke comes is **31°**. The
worst σ_min across every recording is **0.0285**, 2.9× the SDK's own
threshold. Nothing needed changing in the URDF for this.

**But note what 87 % of reach means.** That is the outer part of the envelope,
where the vendor's own cross-section marks reduced manoeuvrability. It is
inside the workspace and outside the singular set, yet it is where a given
tool motion costs the most joint motion. That is consistent with §1: legal,
reachable, and expensive.

---

## 4. Is J4 concentration a `movel` artifact? No — MoveIt is worse

The premise was that MoveIt distributes motion better. **It does not.** Same
task, `toplid_left`, `execute_path` only, like for like:

| joint | movel travel | MoveIt travel | movel share | MoveIt share | movel %lim | MoveIt %lim |
|---|---|---|---|---|---|---|
| J1 | 263.9° | 328.8° | 8.2 % | 9.7 % | **94 %** | 66 % |
| J2 | 596.8° | 735.4° | 18.7 % | 21.7 % | 56 % | 58 % |
| J3 | 294.8° | 374.1° | 9.2 % | 11.0 % | **93 %** | 66 % |
| **J4** | 1074.2° | 1354.9° | **33.6 %** | **40.0 %** | **96 %** | 83 % |
| J5 | 108.3° | 112.6° | 3.4 % | 3.3 % | 25 % | 17 % |
| J6 | 624.4° | 287.9° | 19.5 % | 8.5 % | 55 % | 37 % |
| J7 | 237.4° | 196.2° | 7.4 % | 5.8 % | 32 % | 23 % |
| **total** | **3200°** | **3390°** | | | | |

Concentration measured as a Gini coefficient of joint travel: **movel 0.358,
MoveIt 0.412**. MoveIt is *more* concentrated on J4, not less. The only real
distribution difference is J6, which `movel` uses more than twice as much.

**What actually differs is the rates, and the stopping:**

* `movel` runs **J1, J3 and J4 at 93–96 %** of their limits. MoveIt's worst is
  83 %.
* MoveIt's plan **never stops**: 0 of 1820 interior waypoints have all joint
  velocities below 0.01 rad/s.
* `movel` spends **1.78 s — 8.8 % of the stroke — below 10 % of cruise.**

And that is the whole timing difference: **MoveIt 18.2 s vs movel 20.2 s**,
a 2.0 s gap against 1.78 s lost to near-stops. MoveIt finishes the same joint
work sooner *while staying 13 points further from every limit*, purely by not
stopping.

So what looks unnatural is not the distribution. It is **three joints
saturating while the tool repeatedly halts at waypoints** — constant-rate
motion punctuated by dead stops. Both causes are in §5.

### 4b. The joints ARE out of sync — measured, and the tool offset causes it

*Added 2026-08-14, from Newton's observation that J6 and J4 look unsynchronised
during X translation.* Tool velocity decomposed per joint through a world-frame
tool Jacobian, validated to a **12 % residual** against the measured TCP
velocity. (The obvious operator is wrong twice over: `orientation_cost.jacobian`
sits at the **flange**, in the **base** frame, and its rows 3–5 are **Euler
rates, not angular velocity** — so an ω×r tool correction makes it worse, not
better. Difference the tool position directly.)

Share of the work done on the world X axis, and how often each pair pushes the
same way:

| | toplid @0.45 | test_motion_001 @0.25 | @0.35 |
|---|---|---|---|
| X-dominant share of motion | 91 % | 87 % | 82 % |
| **J4** | **54.6 %** | **51.7 %** | **51.9 %** |
| **J6** | **23.0 %** | **27.3 %** | **26.5 %** |
| J2 | 19.5 % | 16.2 % | 16.6 % |
| everything else | 2.9 % | 4.8 % | 5.0 % |
| **3-D cancellation** | **3.33×** | **4.40×** | **4.74×** |
| J4 & J6 same direction | 41 % | **29 %** | **26 %** |
| J2 & J4 same direction | 4 % | 15 % | 12 % |

Three joints do ~97 % of the X translation, the same three on two unrelated
paths. The joints generate **3.3–4.7× more tool motion than the net result**,
and J4/J6 oppose each other **59–74 %** of the time. Newton's read was right,
and it is *worse* on `test_motion_001` than on toplid.

**The cause is the tool offset.** Same recorded joint trajectory, asking what
different tool offsets would see:

| tool | J4 X share | J6 X share | cancellation |
|---|---|---|---|
| flange, no tool | 69.8 % | **14.5 %** | 3.31× |
| L_glove_1 (155 mm) | 53.3 % | 26.7 % | 4.16× |
| L_glove_4 (220 mm) | 51.7 % | **27.4 %** | 4.39× |

Holding tool orientation while translating forces J6 to counter-rotate against
J4; with the tool ~220 mm off the flange that counter-rotation drags the tool
*backwards* along X. Putting any tool on the end nearly doubles J6's authority
and lifts cancellation by a third. **Shortening the glove is not a lever** —
155 mm and 220 mm are within 1 point of each other.

**But cancellation is not what makes the motion look bad.** MoveIt scores
*worse* on both measures — **3.80× cancellation** and **39 °/s** null-space
drift against movel's 3.33× and 33 °/s — while being the motion judged better.
A hypothesis that the waste was uncontrolled null-space drift is also refuted:
correlation with cancellation is only **+0.17**, and cancellation is still
3.26× in the quartile where the arm angle drifts slowest. The 3.3× floor is
**geometrically inherent to dragging a tool along a straight Cartesian line** —
shoulder and elbow must oppose to keep the wrist on a line. It is why the
motion reads as mechanical: people clean in arcs, not straight lines.

**Untested prediction.** `test_motion_001` @0.25 has the worst coordination of
the three (4.40×, J4/J6 opposed 71 %) but the best smoothness numbers
(near-stops 3.1 % vs toplid's 8.8 %, J-max 59 % vs 96 %). If the look is driven
by stops and saturation it should nonetheless appear *smoother* than toplid
@0.45. If it looks equally jerky, the explanation above is wrong and
coordination matters more than credited. Deciding this needs runs labelled
against video — see §8.5.

---

## 5. What is needed for fast, natural, continuous motion

Ordered by measured payoff, not by guess:

1. **Raise the blend radius.** `stage_runner` uses `r=10`; retention improved
   monotonically from `r=25` to `r=50` and the near-stops cost 1.78 s per
   stroke. This is the single largest lever and it costs nothing.
2. **Add a lead-in segment** so the never-blended first corner is a throwaway
   rather than a real waypoint.
3. **Raise `line_acc`, not `line_speed`.** H57: at fixed `line_speed 0.45`
   the median rose 62.6 → 87.6 % of cap as `line_acc` went 1.6 → 3.6, while
   p95 stayed pinned at the cap. The motion is acceleration-bound. Note H62 —
   RealMan advise against changing the TCP defaults, so this needs their
   agreement.
4. **Keep segments ≥ 4× the ramp distance** `v²/2a`, or there is no cruise to
   hold in the first place.
5. **Hold `line_speed` at 0.45–0.50 until 1–4 are done.** §1 shows the
   headroom above that is zero, and dwell above 98 % is what breaks it.

**What NOT to do:** do not chase 1.8 m/s, and do not lower the global speed to
protect a few waypoints. The retracted gain metric was an attempt to identify
those waypoints offline and it did not work; identifying them still needs a
method that accounts for angular velocity, which we do not yet have.

---

## 6. Phase 2 — early collision detection with planning left to the controller

**Feasible, with one structural caveat that decides the design.**

Offline collision checking before dispatch is available and cheap:
`rm_algo_safety_robot_self_collision_detection` is the controller's own
algorithm, and our model carries the fixture.

The caveat: **the controller re-resolves the redundancy as it executes.**
Measured within a single `movel` stroke, with the approach stages excluded,
the **arm angle swings 144° peak-to-peak**. So the configuration you
collision-check offline is *not* the configuration that runs. Clearance
verified for one arm angle says nothing about the one the controller picks.

Two designs survive that:

* **Check across the whole arm-angle range** at each waypoint — conservative,
  offline, cheap, and it makes no assumption about the controller.
* **Pin the redundancy** with `rm_algo_inverse_kinematics_rm75_for_arm_angle`
  and dispatch joint targets — which is the MoveIt-plans / controller-executes
  architecture already in PHASE_PLAN §1, and the reason it was chosen.

Relegating *path* planning to the controller while keeping collision
responsibility is only sound under the first option.

---

## 7. Does Ruckig Pro use the wrong URDF parameters?

**No. Ruckig is unaffected, and the limits it is given are correct.**

Ruckig is a time-parameterisation library: position, velocity, acceleration
and jerk. It carries **no dynamics model** — no mass, no inertia, no torque —
so the 2.6× link-mass error in `butterfli.urdf` cannot reach it.

The limits it consumes come from
`alix_moveit_config/config/joint_limits.yaml`, which **overrides** the URDF
and is right:

| | joint_limits.yaml | controller | URDF |
|---|---|---|---|
| J1/J2 velocity | 3.14 rad/s = 180 °/s | 180 °/s ✅ | 3.14 ✅ |
| J3–J7 velocity | 3.92 rad/s = 225 °/s | 225 °/s ✅ | 3.14 ❌ (25 % low) |
| acceleration | 10.472 rad/s² = 600 °/s² | 600 °/s² ✅ | — |
| jerk | 1500 rad/s³ | — | — (measured ceiling, 2026-07-31) |

**Where the URDF errors do bite:**

* **Kinematics — 2.8 mm.** `butterfli.urdf` places the flange at
  `0.114 + 0.05 = 0.164 m` from joint 6, against the measured **0.1612 m**.
  MoveIt therefore plans to a tool 2.8 mm further out than the controller
  executes to. For a contact cleaning task that is a press-depth error, and
  it is systematic, not noise.
* **Dynamics — 2.6×.** Harmless today because nothing plans from those
  masses. It becomes real the moment anything uses gravity compensation,
  effort limits, or a dynamics-based collision check.

Neither affects Ruckig. Both should be fixed before the ROS bridge relies on
that file.

---

---

## 8. Review of the 2026-08-13 blend runs — re-read 2026-08-14

*Same recordings, re-analysed with the corrected corner locator, the verified
robot model, and the metrics that did not exist when they were first read.
Purpose: decide what the next hardware session should measure.*

**The original log for these runs is not usable.** Corners were located by
cumulative arc, which drifts 9–34 mm against a ±19.5 mm window, so corners 2–5
were measured mid-segment. Everything below is recomputed.

### 8.1 Blend radius and speed are separable — and that is the useful part

Corners 2–5 only (the first corner never blends — §2), REAL runs:

| commanded | r=0 | r=25 | r=50 |
|---|---|---|---|
| 0.10 m/s | 4 mm/s · 4% | 40 mm/s · 42% | 70 mm/s · 72% |
| 0.20 m/s | 5 mm/s · 3% | 89 mm/s · 46% | 155 mm/s · 77% |
| 0.25 m/s | 2 mm/s · 1% | 122 mm/s · 55% | 178 mm/s · 71% |
| 0.35 m/s | 5 mm/s · 2% | 165 mm/s · 67% | *(run aborted)* |

Read down the columns and the structure is clean:

* **`r` sets the FRACTION of cruise you keep** — ~50% at r=25, ~73% at r=50,
  ~3% at r=0, and that fraction barely moves with speed.
* **Speed sets the ABSOLUTE corner speed** — at r=25 it goes 40 → 89 → 122 →
  165 mm/s, essentially linear in the command.
* **r=0 is a dead stop at every speed**: 2–5 mm/s regardless. Confirming §2 —
  `connect=1` alone cannot hold speed through a waypoint.

The apparent "retention improves with speed" in the ratio column is mostly the
H57 artifact: at 0.35 the achieved median is only ~60 % of cap, so the cruise
reference it is divided by is depressed. The absolute column is the honest one.

### 8.2 They are NOT independent — both spend the same joint-rate budget

| commanded | r=0 | r=25 | r=50 |
|---|---|---|---|
| 0.10 | 24% | 25% | 25% |
| 0.20 | 39% | 42% | 41% |
| 0.25 | 47% | 48% | 48% |
| 0.35 | 48% | 53% | **54%** |

*(worst joint, % of its rate limit)*

Raising **speed** is expensive: 0.10 → 0.35 costs ~29 points of joint budget.
Raising **blend radius** is cheap: r=0 → r=50 costs ~6 points at 0.35 and
nothing measurable below that.

Which settles the trade directly:

> **r=50 at 0.25 m/s gives 178 mm/s through a corner for 48 % of the joint
> limit. r=25 at 0.35 m/s gives 165 mm/s for 53 %.** More corner speed, less
> joint demand. **Spend the budget on blend radius before speed.**

That is the opposite of how the tasks are configured today — `stage_runner`
dispatches **r=10**, the smallest radius, and buys speed instead.

### 8.3 SIM cannot screen blend behaviour

Corners 2–5, all speeds pooled:

| | SIM | REAL |
|---|---|---|
| r=0 | 14% | **3%** |
| r=25 | 63% | 49% |
| r=50 | 88% | 73% |

SIM shows a corner holding 14 % of cruise where the metal stops dead. It
over-states every radius, and worst at the one that matters as the control.
**Run SIM for reachability and dispatch validity; do not read a blend number
off it.**

### 8.4 What the next session should measure

Everything above is one path — `blend_corner_001`, 65 mm segments, **constant
orientation by design**. The J4/J6 opposition in §4b is driven by *holding
orientation while translating*, so this path exhibits the coupling in its
purest form and a rotating cleaning stroke will not behave identically.
**The blend numbers above must be re-established on the real tasks.**

| what | why |
|---|---|
| **r = 10, 25, 50 on `toplid` and `hinge_area`** | r=10 is what ships and has never been measured; 8.1 was measured on a path neither task resembles |
| **at 2 speeds each (0.25 and 0.45)** | 8.1 says r and speed separate — this tests that on a rotating path |
| **REAL only for blend numbers** | 8.3 |
| **a lead-in segment** | the first corner never blends; a throwaway corner makes every measured corner a real one |

Drop from the sweep: `r=0` beyond a single control run per task (its answer is
known and identical everywhere), and the 0.10 rung (below any operating speed).

**Pre-flight gates already available, to run before dispatching any of it:**
`orientation_cost.py --segments` for per-segment elbow demand, and
`preflight_j4` inside the ladder, which refuses a rung over 100 % of J4's
limit. §8.2 says raising r costs ~6 points of joint budget, so screen at the
radius you intend to run, not at r=0.

### 8.5 Open, and not answerable from this data

* **Which video is which run.** The subjective "some look better than others"
  cannot be attached to a measurement until runs are labelled. The next session
  should record which run each clip belongs to — that is the only way to settle
  whether the bad look tracks near-stops and saturation (§5) or joint
  coordination (§4b).
* **Whether 8.1's separability holds on a rotating path.** On `blend_corner_001`
  the angular cap can never bind, because orientation is constant. On a cleaning
  stroke it throttles 12 of 36 segments, and a throttled segment's corner may
  behave differently.

---

## 9. What blending actually removes — the 2026-08-14 task runs (72 runs)

*Written 2026-08-14. Method: every run's trace was matched to its own commanded
waypoint sequence with a monotone (sequence-order-preserving) alignment, then
each commanded arc position was tested for "was the tool near this point at the
sequence-correct time". This replaces the point-set test used earlier in the
day, which was blind on these paths: `toplid`/`hinge_area` revisit waypoints
(hinge returns to `point1` six times), so distance-to-nearest-segment is small
everywhere whether or not a stroke was actually executed. Validation: SIM r=0
runs measure 100 % covered with zero corner cuts, and the known
first-corner-never-blends behaviour reproduces exactly. Geometry numbers below
are SIM (position channel exact); REAL runs, matched raw with an
aliasing-scaled tolerance, agree within 2–3 points throughout. The later run of
each task pair is the `--reverse` order — treated as its own commanded
sequence.*

### 9.1 Coverage by radius (v = 0.25, both arms, SIM ≈ REAL)

| r | toplid covered | hinge covered | toplid stroke time |
|----|----|----|----|
| 10 | ~89 % | ~92 % | 32.0 s |
| 25 | ~73 % | ~77–81 % | 30.5 s |
| 50 | ~52–57 % | ~60–64 % | 23.0 s |

"Covered" = within 2 mm of the commanded line at the right point in the
sequence. Left and right arms are identical to the millimetre in SIM.

### 9.2 Where the loss is, and the law it follows

Summing the per-corner cuts reproduces each run's total uncovered length to
within 2–22 % — **the loss is entirely at corners**. Its distribution:

* **82–95 % of all lost coverage sits at reversal corners (turn > 165°).**
* Corners of 60–165° carry the rest; corners under 60° contribute ~0.
* At every cutting corner the cut obeys, on SIM and REAL alike:

      cut(entry+exit) ≈ 1.3–1.5 × (r/100) × min(L_in, L_out)

  (SIM medians 1.46–1.52 across r = 10/25/50; corr(L, cut) = 0.81–0.86.
  Per side ≈ 0.7 × (r/100) × L.) Angle does not enter beyond a threshold:
  from ~90° up to 179° the normalised cut is flat.
* **Exact 180.0° retraces are exempt.** The hinge fan spokes (out and back on
  the same line, L 30–43 mm) show zero cut at every r, in every run, both
  arms, both directions — while 172–176° near-reversals of similar length cut
  at the full rate. It is collinearity, not segment length, that protects:
  a serpentine U-turn has lateral room to arc across and clips both stroke
  ends early; a true retrace has none and the controller stops and returns.
* **First corner of a chain never blends** — confirmed on tasks, both
  directions (18/18 runs, zero cut even at a 175° reversal). Reversing the
  path moves which corner is exempt. The **last** corner blends normally.
* **The blend geometry is speed-independent**: per-corner cuts on the
  synthetic path are identical from 0.10 to 0.35 m/s in SIM. Speed changes
  feasibility and time, not the shape of the cut.

The mechanism at a near-180° reversal is early turnaround, not corner
rounding: at r = 25/50 most reversal vertices are never approached within
30 mm of arc position. The stroke **end region** is what disappears — on
toplid's ~300–400 mm strokes, r = 50 removes ~250 mm per reversal.

### 9.3 Failures in this data set

* **toplid REAL 0.45 with r = 25/50 aborted** with J4 pegged at 98.2 %, no
  error on any channel, arm idle after — leaving **1.32–1.38 m of path
  unexecuted** (the earlier "193–203 mm short" was the straight-line distance
  from the stop point to the path end, which criss-crossing made meaningless).
  r = 10 at 0.45 completed. This corrects §8.2's "radius is ~6 points of joint
  budget": that was true on the non-rotating synthetic path at ≤0.35; on a
  rotating task at 0.45, r ≥ 25 is the difference between completing and
  aborting.
* **Intermittent silent early chain termination at r = 25 on hinge**: 3 of 6
  hinge r = 25 runs stopped exactly at a waypoint 1–3 segments before the end
  (~100–310 mm unexecuted) — no error code, normal status tail, and one of
  the three is a SIM run, so it is planner/chain behaviour, not dynamics. Both
  arms, both directions affected; never seen at r = 10 or 50. Until
  understood, production runs should verify the final waypoint was reached.

### 9.3b Continuity measurements (added same day, REAL v = 0.25 unless noted)

* **Exact-180° retrace tips are true stops at every r**: 1–4 mm/s minimum,
  170–290 ms below 10 % of cruise (hinge spokes, r = 10/25/50). Coverage-safe
  but not stop-free.
* **Blended serpentine U-turns are continuous but slow**: minimum speed
  through the turnaround, median across toplid's ~175° reversals — r = 10:
  10–21 mm/s; r = 25: 26 mm/s; r = 50: 52 mm/s. Larger r carries *more* speed
  through the turn. Shallow corners (<60°) at r = 25 never drop below
  61 mm/s and spend 0 ms under 10 % cruise.
* **Deterministic ~2 s freeze at toplid's `point13` spur when r ≥ 25**: every
  r = 25/50 toplid run — SIM and REAL, left and right, forward and reverse
  (12/12) — stalls 2.0–2.7 s within ~13 mm of `point13` (a 29 mm exact-retrace
  spur entered from a 54 mm segment). Never at r = 10, and never at the hinge
  fan spokes (30–43 mm) at any r. It erases most of r = 25's time savings
  (30.5 s vs r = 10's 32.0 s despite 27 % less path). SIM reproduces it
  faithfully, so it is screenable before hardware. Same family of chain
  misbehaviour as the r = 25 hinge early termination (§9.3): short-segment
  structures + mid-range r.

### 9.3c Speed picture and claim verification (added same day)

* **Orientation-rate does NOT predict J4** — corr(deg/m, J4 peak) ≈ 0 across
  segments in four REAL runs; hinge's worst J4 segments carry only 3° of
  rotation (J4 is the translation workhorse, §4b). What is true: toplid's J4
  ceiling is ONE segment — `point13→point12` (41° over 405 mm). Both 0.45
  aborts died there at an identical clamped 122.7 % J4; r = 10 survived it at
  a momentary 108 % because it entered slow. **Blending killed the 0.45 runs
  by carrying U-turn speed into that segment.** The lever is local: slow,
  de-rotate, or un-blend that one entry — not "less rotation everywhere".
* **Corner speed scales with v at r ≤ 25** (hinge corner minima ≈ double from
  0.25→0.45); **r = 50 breaks down at 0.45** (minima drop, shallow corners dip
  to 5 mm/s — near-stops). r = 50 is a 0.25-only radius.
* **Short segments are acceleration-limited, not cap-limited**: with
  acc = 3v the ramp length is v/6 m each way (75 mm at 0.45), so hinge's
  ~100 mm segments never reach 0.45 — 71–73 % of hinge's 0.45 run time sits
  below half cap. Raising v beyond ~3 × segment-length [m/s] buys nothing.
* **Direction is a free variable**: matched fwd/rev cells differ by ≤3 J4
  points and ≤2 s (stall pattern). Use it to place the exempt first corner.
* Fastest completing configs measured: hinge 0.45/r50 19.9 s (coverage 53 %,
  near-stops); hinge 0.45/r25 26.2 s (continuous); toplid 0.45/r10 23.4 s
  (89 %). Time from r at 0.25: r10→r50 saves ~28 % on both tasks.
* **Unverified, SIM-screenable before any hardware run**: per-command mixed r
  in one chain; per-command v change mid-chain; chaining the approach move to
  consume the first-corner exemption.

### 9.3d The chain-semantics screens (prepared 2026-08-14, ready to run)

Three one-rung SIM runs on the LEFT arm answer the three unverified items.
One path geometry — four IDENTICAL 90° corners (200 mm strokes, 45 mm steps)
in blend_corner_001's proven box, constant orientation, worst segment 52 % of
the J4 limit at 0.25 — so any per-corner difference is the per-move
parameter. Each path file's header carries the predicted signature for
honored / latch-first / latch-last; run.json records the dispatched
`(v, r, connect)` per move verbatim (`commanded.program`).

    cd src
    python3 test_blend_corner.py --side left --mode SIM \
            --path ../paths/chain_semantics_001.py     # per-move r
    python3 test_blend_corner.py --side left --mode SIM \
            --path ../paths/chain_semantics_002.py     # per-move v
    python3 test_blend_corner.py --side left --mode SIM \
            --path ../paths/chain_semantics_003.py     # chained approach

Then pull the three run dirs and:

    python3 analyse_coverage.py ../runs/<run_dir>      # per-corner cuts
    python3 analyse_run.py      ../runs/<run_dir>      # dips / stalls

Read 001 by corner cuts + which corners stop; 002 by per-stroke speed
plateaus (100/250/150 mm/s if honored); 003 by whether corner P1 blends
(~16 mm cut, no stop) while the prestart→P0 corner does not. The ladder is
ONE rung at 0.25 by design — this box's J4 crosses its limit near
0.385 m/s (§blend_corner_001), so do not pass --speed here.

Dry-run status (this machine, emulator): dispatch mechanics, entry variants,
program construction and run.json recording verified end-to-end; the
emulator cannot execute the movel geometry itself (its known movel-IK
limitation), so the semantics answers must come from the controller SIM
runs. En route the emulator gained a fix: the vendor algo library's
toolframe is process-global and any module constructing its own `Algo`
resets it — `rm_emulator` now re-asserts the active tool frame at every
IK/FK entry point instead of assuming it persists.

**RESULTS (SIM, left arm, 2026-08-15 12:36-12:38 — all three HONORED):**

* **Per-move r IS honored** (`chain_rmix` 123649). Cuts track the program
  exactly: A 0 (exempt, stop 3.4 mm/s), B **r=0 honored — full stop
  2.3 mm/s, zero cut, chain continues**, C (r=25) 7.9 mm cut at
  89.6 mm/s, D (r=50) 15.8 mm cut at 133 mm/s. Neither latch hypothesis
  fits. Mixed radii in one chain are real.
* **Per-move v IS honored, and a v change does not break the blend**
  (`chain_vmix` 123725). Stroke plateaus 94 / 243 / 145 mm/s against
  predicted 100 / 250 / 150; corners between v-changes stay continuous
  (67–90 mm/s, zero stops). Latch-first (all ~100) and latch-last
  (all ~150) both excluded.
* **The chained approach consumes the first-corner exemption**
  (`blend_r25_capp` 123749). The approach corner P0: zero cut, stop at
  2.9 mm/s — the exemption landed there. Corner P1, exempt in 001/002,
  **now blends** (7.9 mm cut, 89 mm/s through). Independent confirmation
  from the controller state machine (analyse_run H): 4 mid-run MOVE_L
  exits with the tool still moving; "corners that blended: 1* 2 3 4 5".
* Calibration note: at these 90°/45 mm corners the cut is
  **0.70 × (r/100) × minL** (both radii, exactly), vs 1.2–1.5 measured on
  the tasks' ≥60° corners with longer segments — the coefficient is
  geometry-dependent; the §9.4 rule (1.4×) is the conservative bound.

All three levers the path redesign needs are therefore real: r per corner,
v per segment (slow ONLY into the J4-critical segment), and an entry that
spends the unblendable corner on the touchdown instead of the path.

### 9.3e The redesigned task path (2026-08-15) and the 0.45 m/s question

**`paths/toplid_left_002.py`** — same cleaned footprint as the original
(every stroke end, rim point and edge point IS an original waypoint), built
on the verified semantics: serpentine over the 7 fan strokes, chained
approach (exemption spent at a touchdown 25 mm outside the top-right
corner), left turns padded 20 mm past the rim at r=35 (all cuts land in the
padding), right-edge corners r=12 unpadded (~2–4 mm cuts; the right hops
double as a right-edge pass), rim + top-edge passes kept at r=10, and ONE
deliberate r=0 stop at the point12 fan reversal. 3.43 m vs 6.20 m; the
killer traverse, the second rim pass and the point13 spur are gone. Screens
worst 71 % of J4 at 0.25 (original's measured range: 56–73 %). Comparison
figure: `paths/toplid_left_002_vs_original.png`.

**Revision 2 (same day) — glove-complete stroke density.** Against
`glove_frames.yaml` (L_glove_frame_2: 35 mm brush width) with Newton's
1.5 cm tolerance, the guaranteed band per stroke is 20 mm — and the
ORIGINAL task's 24–42 mm row spacing never guaranteed coverage. Rev 2
interpolates the fan to 14 rows (positions linear, orientations slerped
between proven neighbours; max gap 19.5 mm) plus a short wedge row at the
point12 apex that U-turns inside already-covered ground. Verified
numerically: **100.00 % of the original-area hull within 10 mm of a
cleaning centerline** (17 130 samples). 6.91 m, 41 waypoints, worst J4
71 % at 0.25. En route: a second reach lesson — the wedge legs screened
133 % when they carried point13's orientation toward the apex; slerping
the wedge orientation like the rows fixed it to <71 %. Emulator roadmap
from the collected data: `EMULATOR_ROADMAP.md`.

**Design lesson — the right edge has NO padding room.** The fan's right
ends sit at the arm's practical boundary (898 mm base distance, the
original's own max). A 20 mm extension there pushed the J4 screen from
~70 % to 110–244 % — near the straight-elbow region, dq4/ds explodes with
millimetres. Padding is a LEFT-side tool on this task.

**Does all this generalise to 0.45 m/s? Partly measured, partly screened,
one hard NO:**

* Generalises on evidence: blend-cut geometry is speed-independent
  0.10–0.35 (SIM, exact) and hinge coverage at 0.45 REAL matches 0.25
  within noise; corner speed at r ≤ 25 SCALES with v (minima ~double from
  0.25→0.45); the v-mix screen already ran mixed speeds in one chain.
* Not yet verified at a 0.45 baseline: the chain semantics themselves.
  Screens ready: `chain_semantics_004` (r-mix) and `005` (v-mix,
  plateaus ~250/450/350 if honored) — same box, J4 screens ~94 % at 0.45,
  marginal by design, SIM refuses safely.
* Known NOT to generalise: **r = 50 at 0.45** (hinge REAL corner minima
  collapse, shallow corners dip to 5 mm/s — r=50 is a 0.25-only radius;
  toplid_left_002's r=35 sits below that regime but 004 will bound it),
  and **this task's strokes at 0.45**: they measured 56–73 % of J4 at
  0.25, which scales past 100 % at 0.45 regardless of blending. At a 0.45
  baseline the stroke entries in V_LIST must drop to ~65 (0.29 m/s,
  ≤ ~85 % J4); 0.45 then buys its time on hops, rim and edge — and H67
  already throttles the rotating strokes below 0.45 anyway.

### 9.3f Results of the 0.45 screens and the toplid_left_002 SIM run (2026-08-15)

* **0.45 chain semantics: HONORED, identical geometry.** `chain_semantics_004`
  ran (no refusal at ~94 % predicted J4) with cuts bit-for-bit equal to the
  0.25 twin (0 / 0+stop / 7.9 / 15.8 mm); B's mid-chain r=0 stopped at
  2.0 mm/s; r=25/50 corners carried 143–162 mm/s (more than at 0.25 — the
  scaling holds; the r=50 collapse seen on hinge REAL did not reproduce on
  this clean SIM geometry). `chain_semantics_005` plateaus 251/424/340 vs
  predicted 250/450/350 — per-move v honored at operating speed, same ~5 %
  mapping bias as everywhere.
* **toplid_left_002 SIM: coverage goal met, continuity goal partly met.**
  Full path reached (0 short), NO freeze, NO early termination, 42.4 s.
  **Area coverage measured from the traced path: 100.00 %** of the
  original-area hull within the 10 mm effective half-band (17 130 samples).
  In-area cuts: zero at every corner (only the wedge hop corner cut
  1.9/2.0 mm — in padding). Designed stops behaved (touchdown 0.43 s,
  apex ~0.55 s).
* **NEW FINDING — the blend floor.** The controller BLENDED only 4 of 40
  corners and executed **29 brief mid-run stops** (sub-0.4 s, tool at
  rest — arm_status IDLE spans) at the turnarounds: segments of 9–25 mm
  are apparently TOO SHORT to blend at any commanded r, and the controller
  falls back to a full stop. Consistent with prior data: hinge corners
  with ≥30 mm segments blended; the 45 mm steps of chain_semantics blend;
  nothing below ~25 mm ever has. All 29 stops sit at padded/edge corners
  OUTSIDE the cleaned area, so coverage and in-area continuity are intact —
  but the no-stops goal fails at the turnarounds.
* **Fix APPLIED (rev 3, same day): two-pass serpentine.** Odd rows
  top→bottom, then even rows bottom→top; hops become 18–39 mm. Floor-risk
  moves: 29 → 5 mid-path (18–27.5 mm; the fan's bottom convergence is
  geometric) + the rim-entry hop. Structural bonuses: the pass transition
  runs UP the right edge as one 171 mm cleaning move (the right boundary
  strip gets its own pass); row 14 reaches the apex through a blendable
  ~90° corner; the wedge row runs apex→left in one stroke (no U-turn);
  touchdown moved to the top-LEFT padding (the reach-safe side).
  Re-verified: 100.00 % area coverage, worst J4 71 % at 0.25, reach max
  898.3 mm, 7.33 m, 40 waypoints. The SIM rerun of this file also
  MEASURES the blend floor (which of 18/21.3/22/24.6/27.5 mm hops blend).
  Figure: `paths/toplid_left_002_rev3.png`.

### 9.4 Practical rule

Choose r per stroke from the allowed end-loss δ:  **r ≈ 133 · δ / L**.
A 293 mm toplid stroke with 20 mm acceptable end-clip ⇒ r ≈ 9. Since `r` is a
per-command parameter, mixed radii are legal: r = 0 (or small) on moves ending
at coverage-critical stroke ends, large r elsewhere. Two path-design outs:
overshoot each stroke end by 0.7 × (r/100) × L so the clip lands outside the
area, or use exact-retrace strokes (out-and-back, then side-step), which
blending provably does not clip — the side-step corners then cut only
1.5 × (r/100) × step, a few mm.

---

## Reproducing any of this

```bash
cd src
python3 verify_robot_model.py                       # the model these rest on
python3 analyse_run.py ../runs/<run_dir> --plot     # one run, every metric
python3 analyse_run.py ../runs/*toplid*left --quiet --json out.json
```

Restrict to the cleaning stroke with the `stages` block in `run.json`, and
drop SIM runs before quoting any joint-load figure. Those two steps are what
turned §1 from wrong into right.

---
