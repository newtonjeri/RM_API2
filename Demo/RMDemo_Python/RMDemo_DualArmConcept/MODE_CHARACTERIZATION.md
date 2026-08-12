# Emulator vs SIM vs REAL — what each mode measures, and what predicts what

*2026-08-12. All numbers computed from `runs/` (35 recordings), the saved
plans, and offline `rm_algo_*`. No hardware was available for this work.*

**The question.** 20 of the 24 commode_c cleaning tasks have never been
executed. Can we predict how they will behave before running them, and which
mode is worth running?

**The answer, up front.**

| predictor | binding joint vs REAL | needs |
|---|---|---|
| **the saved plan** | **13/13** | nothing — fully offline |
| **SIM** | **9/9** | the controller |
| **the emulator** | **6/24** (chance) | nothing |

The plan is the best predictor and it is free. **The emulator must not be
used to screen these tasks** — §3 shows why, and it is not a bug that can be
patched away.

---

## 1. Channel fidelity — the three modes are not interchangeable

`stream.csv` looks identical in every mode: same 62 columns, same 100 Hz,
plausible tool motion. Measured over the `execute_path` window
(`mode_compare.py --channels`):

| channel | emulator | SIM | REAL |
|---|---|---|---|
| joint **position** | modelled (§3) | **faithful** — 56.1° travel vs REAL 56.0–56.2° | yes |
| joint **speed** field | n/a | **DEAD** — 0.1–0.5 °/s while moving | 62–238 °/s |
| joint **current** | n/a | idle only, 0.4–3.1 A | 4.5–16.7 A |
| stage **timing** | trapezoid model | **within 2 %** | yes |
| tool path length | modelled | 0.0–5.5 m | 4.7–5.8 m |

**The trap.** SIM's `speed{n}` column reads ~0.4 °/s *while the arm is
moving 56° of joint travel*. Anything that reads `stream.csv` without
gating on `run.json["sim"]` will silently average simulated and real
physics — 21 of the 35 recordings are SIM.

**The way through.** SIM's *position* channel is faithful, so differentiate
it. That is what makes SIM useful at all.

---

## 2. SIM predicts REAL, and it flagged the failure before it happened

Nine matched pairs (same task, same limits; the `cap=0.25` group pairs by
duration because the pendant override is unreadable and therefore
unrecorded):

```
duration ratio SIM/REAL   0.9864 .. 1.0260   (median 0.9993)
binding joint agreement   9/9
utilisation error         -12 .. -1 pp       (median -4)
```

SIM **under-reads** REAL by a median of 4 percentage points. Treat a SIM
utilisation of *U* as REAL *U +1..+12 pp*.

### The validation case

`20260811T184017` (SIM) and `20260811T184109` (REAL) ran `toplid_left` at
`line_speed 0.80`. SIM completed in 18.77 s. REAL **stopped silently** at
13.46 s and reported nothing.

```
              SIM d(pos)/dt p99      REAL reported peak
   J4              225.1 = 100 %          238.3 = 106 %
   SIM peak per joint: [210, 142, 241, 272, 71, 226, 161]
   limits            : [180, 180, 225, 225, 225, 225, 225]
```

**SIM read J4 at exactly 100 % of its limit, and four joints over limit on
peak — before the REAL run was made.** SIM does not enforce joint limits, so
it completed; but the saturation was there to be measured. A SIM run is a
free pre-flight check.

---

## 3. The emulator cannot predict joint rates, and the reason is structural

`rm_emulator.movel_chain` models the Cartesian path with RealMan's own
offline solver. After fixing two real defects (§5) it solves the whole
stroke with zero IK failures. It is still useless for this purpose:

```
Spearman rank correlation of utilisation, emulator vs plan : +0.090
binding-joint agreement                                    : 6/24 (25 %)
plan flags >= 90 %                                         : 11 tasks
emulator flags >= 95 %                                     : 23 tasks
```

### Side by side, same tasks, same limits

`emu_stroke.py --validate --ls 0.45 --la 3.6`:

| task | EMULATOR | SIM | REAL |
|---|---|---|---|
| toplid_left | J5 **365 %** | J4 75 % | **J4 78 %** |
| toplid_right | J1 **188 %** | J4 71 % | **J4 74 %** |
| hinge_area_left | J7 **238 %** | J1 31 % | **J1 29 %** |
| hinge_area_right | J7 **290 %** | J5 46 % | — |

**SIM lands within 3 points of REAL and names the same joint every time. The
emulator gets both the joint and the magnitude wrong in all four.**

### Root cause, measured

Per-joint travel on `toplid_left`, emulator against the saved plan:

```
joint     J1    J2    J3     J4      J5    J6    J7
emu      875   721   219   1354    1127   354   745
plan     329   735   374   1355     113   288   196
ratio   2.7x  1.0x  0.6x   1.00x   10.0x 1.2x  3.8x
```

**J4 matches the plan to 1 %. J5 is out by a factor of ten.**

That split is the whole story. The RM75 has one redundant DOF; J4 sits
outside it and is therefore fully determined by the Cartesian path, so every
solver agrees on it. J1, J5 and J7 live in the null space, where the choice
is the solver's — and `rm_algo_inverse_kinematics` makes a different choice
from MoveIt, and from the controller.

REAL confirms which side is right: `toplid_left` at 0.45/3.6 measured
J5 at 55 °/s — low, like the **plan** (38), not like the emulator (821).

### The fix that did not work

Constraining the search to the arm angle that minimises joint motion
(`rm_emulator._ik_min_motion`, added and tested) narrows J5 from 10× to
5.6× and leaves J1 at 2.2× and J7 at 3.6×. Better, still wrong. Kept in the
source, documented, **not** made the default.

**Conclusion.** Null-space joint rates are not predictable without the
controller's own redundancy scheme, which is unpublished. The emulator
remains valuable for what it was built for — dispatch, chain semantics,
arrival events, queue depth — and must not be extended into a kinematic
screen.

---

## 4. The plan predicts REAL, and here is the calibration

Over all 13 REAL recordings the plan named the binding joint **13/13**. The
ratio of measured to planned utilisation tracks the commanded limits:

| `line_speed` / `line_acc` | REAL d(pos)/dt ÷ PLAN |
|---|---|
| 0.45 / 1.6 | 0.92, 0.93 |
| 0.45 / 2.4 | 0.96, 0.99, 1.05 |
| 0.45 / 3.6 | 1.06, 1.11 |
| 0.50 / 3.6 | 1.18 |
| 0.80 / 2.4 | **1.42** — this run stopped silently |

Spread within a setting ≤ 0.09. `predict_task.py` interpolates this and
applies it to every plan.

### Predicted REAL utilisation, all 24 cleaning tasks

| `line_acc` at `line_speed 0.45` | over limit | ≥ 95 % | verdict |
|---|---|---|---|
| **1.6** | **0** | 0 | **safe** — 8 tasks at 92 % |
| 2.4 | 0 | 8 | marginal |
| 3.6 | **11** | 0 | **unsafe** |

The eleven predicted over the limit at 3.6 are `lid_seat_hinge_area_*`,
`seat_ring_bottom_*`, `side_*`, `bottomlid_*`, `hinge_area_*` and
`bowl_inside_front_right`. That is the condition under which
`20260811T222451` stopped mid-path and reported nothing (H45).

**Recommended operating point for anything untested: `line_speed 0.45,
line_acc 1.6`.** It costs ~17 % of stroke time and puts nothing over the
limit.

---

## 5. Self-corrections made during this analysis

Recorded because each was found by measurement after being asserted:

1. **"The emulator has no IK"** — true when first read, already fixed in the
   working tree by the time I tested it. My initial read was of a superseded
   version.
2. **Tool frame set through the wrong field.** `rm_frame_t.x/y/z` is the
   payload **centroid**; the offset belongs in `frame.pose.position`, in
   metres. Putting it in `x/y/z` left the tool at the flange and produced
   *"3444 of 3531 samples had no IK solution (97.5 %)"*. Same family as the
   1000× centroid of F31 — adjacent fields, silent when confused.
3. **The emulator solved IK in the wrong frame.** `rm_movel` takes poses in
   the CONTROLLER frame; `rm_algo_*` solves in the URDF frame; they differ
   by the install pose `Ry(+90°)`. `_plan_cartesian` never applied it, so
   every target sat 868 mm from where the solver looked — 98.2 % IK failure.
   Fixed, with quaternion SLERP replacing Euler interpolation.
4. **Utilisation sampled at stride 20 under-read the peak by up to 11
   points** (`hinge_area_left` 88 % sampled against 95 % true;
   `seat_ring_bottom_left` 89 % against 100 %). A peak is exactly the
   statistic a stride destroys. `orientation_cost.py` now takes utilisation
   at full plan resolution; the count of tasks ≥ 90 % rose from 9 to **11**.
5. **Elbow excursion had the same defect** — unwrapping is only valid while
   consecutive samples differ by < 180°, and at stride 20 several tasks step
   176°. Now computed at full resolution, with the four still-ambiguous
   tasks named rather than quoted.
6. **A 20 mm/s motion threshold mis-measured the stall.** It read motion
   continuing to t+27.2 s in `20260811T222451`; the joint channel showed
   0.2 °/s at that moment. The "motion" was the tool-frame restore on
   cleanup — a metre-scale TCP jump with the joints still. The stall really
   is at t+11.9 s.

The pattern in 2, 3, 4, 5 and 6 is the same: **a derived channel lied and a
more primitive one corrected it.** Joint position corrected TCP pose; full
resolution corrected a stride; the joint-speed channel corrected a TCP
speed threshold.

---

## 6. Recorded findings

* **H52** — SIM reproduces REAL joint POSITION and TIMING faithfully
  (duration ratio median 0.9993; binding joint 9/9) but its `speed{n}` and
  `current{n}` channels are dead. Differentiate SIM positions; treat the
  result as REAL utilisation **−4 pp** (range −1 to −12).
* **H53** — **SIM flagged the H45 failure before it happened**: at
  `line_speed 0.80` SIM read J4 at 100 % of limit and four joints over on
  peak, and completed. A SIM run is a free pre-flight screen.
* **H54** — **the emulator cannot predict movel joint rates** (rank
  correlation +0.09, binding joint 6/24). Cause: `rm_algo_inverse_kinematics`
  resolves the redundant DOF differently from MoveIt and the controller —
  J5 travel 10× the plan's, while J4 (outside the null space) matches to
  1 %. Minimum-motion arm-angle search narrows it to 5.6×, not enough.
* **H55** — **the saved plan predicts the REAL binding joint 13/13**, and
  REAL/PLAN utilisation is 0.92–1.42 rising monotonically with the commanded
  Cartesian limits. This is the screen to use.
* **H56** — at `line_speed 0.45 / line_acc 3.6`, **11 of 24 cleaning tasks
  are predicted over a joint speed limit**; at `line_acc 1.6`, none are.
