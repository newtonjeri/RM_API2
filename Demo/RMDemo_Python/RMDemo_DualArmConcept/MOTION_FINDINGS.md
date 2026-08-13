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
