# Butterfli — Phase Plan: ROS 2 planning + controller execution

*Living document. Newton owns the schedule and the orchestrator; this file is
the shared contract for what "feasible" means and what closes each gate.*
*Created 2026-08-07 · **substantially revised 2026-08-07 evening** after the
C15/C16 hardware sessions overturned the arm–pole synchronization design.*

---

## 0. Status at a glance

| | |
|---|---|
| **Architecture** | ROS 2 (MoveIt/MTC) plans and validates; the RealMan controller executes. **Unchanged and still sound.** |
| **What changed** | Arm–pole *synchronization* cannot be done with planned moves. Proven model-free on both arms (C16). Sync moves to **CANFD passthrough**; everything else stays on planned moves. |
| **Hardware state** | Both arms **V1.7.4**, algo 1.5.9, lifts 1:1 true mm over **0–315 mm**. Firmware mismatch resolved. |
| **Vendor verdict** | **RealMan CONFIRMED the defect (2026-08-08, WeChat)**: "conflict between the third-generation controller's control of the hoist's movement and the arm's movement … documented problem, engineers working on it, no completion date. Temporary solution: blocking mode for the hoist." Their workaround IS serialization — matching C16. **Planned-backend sync is now LOCKED in code** (`RM_UNLOCK_PLANNED_SYNC=1` exists solely for post-fix re-tests); sync runs on **canfd only**. |
| **Offline work** | **Complete.** WP1 + WP2/C12 (clearance map: all stages OK), C14 offline (trees identical, one **15.3 mm** constant), R8, C10/C11 tests. Dry run **105/105**, emulated suite 13/13. |
| **2026-08-08 hardware session** | **C17 PASSED on BOTH arms (7/7 each)** — canfd sync works: pole and arm move together, both complete, no faults, dispatch skew < 17 ms. R1b closed, the architecture is confirmed end to end. **C10 PASSED (7/7)**: 20-segment chains accepted, blending real (r=50 saves 0.64 s), and a mid-chain bad target fails *that segment only* while the chain completes — the dispatcher must check every return code. |
| **Biggest open risk** | **C14 tool frames were written with a 17.2 mm error** and the active tool frame was not restored (both bugs found + fixed 2026-08-08 — see F15). Re-write the frames and confirm the active frame before any Cartesian work. C11 is unaffected (joint-space only) and can run first. |

---

## 1. The architecture

**ROS 2 decides WHERE to move; the RealMan controller decides HOW to get
there.** MoveIt/MTC are a **task planner and a validator — not a path source
to be reproduced**. They own the scene, URDF/TF, IK, reachability and the
collision verdict; they do *not* hand over a dense trajectory. Execution
sends the controller a **short list of sparse targets**.

The safety question is therefore **not** "does the executed path match
MoveIt's path?" — nothing is trying to match it. It is "**is the motion the
controller actually produces collision-free in this scene?**":

```
  MTC task ──▶ MoveIt: reachability, IK,  ──▶ sparse targets ──▶ controller
  (stages)     goal-state collision, order    movej / movel       execution
                        │                     canfd for SYNC          │
                        │                            │                │
                        │                   OFFLINE verify:    ONLINE backstop:
                        │                   predict the         self-collision
                        │                   controller's own    detection
                        │                   motion (rm_algo),   (free, always on)
                        │                   FCL vs PRIMITIVES   [fence NOT used]
                        │                            │
                        └──── planning scene ────────┘
                              URDF · TF · SRDF
                     (C11 rehearsal capture validates the predictor once)
```

MoveIt's *timing* is discarded; the controller re-times everything. That is
acceptable for cleaning: geometry and coverage matter, velocity profiles do
not.

### 1.1 Execution domains — the one structural constraint

| Motion | Domain | Why |
|---|---|---|
| Named poses, transits, stroke entry, cleaning strokes | **planned** (`rm_movej` / `rm_movel`) | Controller profiles, arrival events, TCP limits enforced |
| Pole between arm moves | **planned** (`rm_set_lift_height`) while the arm is idle | Safe; this is what ZIGZAG01 does |
| **Arm + pole moving together** | **CANFD passthrough** (`rm_movej_canfd` stream + `rm_set_lift_height`) | The only combination that works — F9 |

---

## 2. What ZIGZAG01 actually demonstrates

*(Corrected 2026-08-07 evening. `ZIGZAG01` is an **in-house** Web-GUI program
written in our lab — not a vendor example.)*

It was originally read here as proof that synchronized arm+pole motion works.
**It is not.** Re-reading it against the C16 findings, it is proof of
something more useful: **the safe usage pattern**.

Its lift blocks are one `height=330, speed=100, BLOCKING` followed by **seven**
`height=0, speed=25, NON-BLOCKING`, each issued between MOVEL groups — i.e.
**always while the arm is idle**. Consequences, all consistent with C16:

- the pole is always the device that gets truncated, never the arm — which is
  why the program has never produced a Position Command Step Warning;
- the pole descends in ~25 mm bursts, needing seven re-kicks to cover 330 mm;
- "pole and arm move together but not strictly synchronized" — the observed
  behaviour — **is the truncation**, seen from outside.

**So it is an existence proof of the sequencing pattern, not of
synchronization.** Treat it as the reference for how to command the pole
around planned arm motion.

---

## 3. What the bench tests have established

| # | Finding | Consequence |
|---|---|---|
| ~~F1~~ | ~~A lift command during an in-flight planned arm move aborts it~~ | **Superseded by F9** — the effect is bidirectional, not lift→arm only |
| F2 | Sim mode **executes the arm and is fully observable** (UDP push, arrival events, TCP polls) | The rehearsal-validation loop is viable (C5) |
| F3 | Sim mode does **not** execute lift or hand | Rehearsals cover arm geometry only |
| F4 | Hand: device-2 arrival events never fire; blocking `rm_set_hand_angle` returns −4 during arm motion | Hand completion is duration-based (`acked_angle`); modbus RTU is the true-feedback path |
| F5 | Planned `rm_movej` + concurrent **hand** works (C6, 7/7 ×3) | Hand/glove actuation during arm motion is safe — hand traffic does not touch the motion planner |
| F6 | Lift is a profiled positioner; re-targeting mid-motion restarts the profile | One target per pole goal |
| ~~F7~~ | ~~Per-arm lift gearing differs~~ | **RESOLVED 2026-08-07**: right upgraded to V1.7.4; **both arms 1:1 true mm, 0–315 mm**. Gearing is now auto-detected from `ctrl_info.version` at connect, so a rollback needs no edit |
| F8 | canfd `expand` cannot drive the lift (`bench_lift_expand`, both poles, 3 unit hypotheses) | No single-command 8-axis sync; the pole is always a separate command |
| **F9** | **Concurrent lift + PLANNED arm move is unsupported, in BOTH directions.** The command issued *second* truncates the first device's motion. Arm truncated ⇒ latched `Position Command Step Warning` (16384) on the **moving** joints (J2/J4/J6; stationary joints stay Normal). Pole truncated ⇒ silent stop ~25 mm in, no error, no event. Model-free (C16), both arms, all speeds, all dispatch offsets | **VENDOR-CONFIRMED 2026-08-08** — a documented Gen-3 defect, no fix ETA; RealMan's workaround = hoist in blocking mode (= serialization). **Sync = CANFD only; the planned backend is LOCKED** behind `RM_UNLOCK_PLANNED_SYNC=1` for post-fix re-testing. Planned moves everywhere else, pole only while the arm is idle |
| F10 | TCP limits **as configured on these arms**: 0.250 m/s linear, 1.600 m/s² linear acc, 0.600 rad/s angular, 4.000 rad/s² angular acc. Joints J1–J6 180 °/s, J7 225 °/s, 600 °/s². *All higher than the API-doc defaults* | Answers butterfli_hw TODO 6b: the Cartesian tool-speed cap lives in the **controller**, so the proposed trajectory post-processor is unnecessary here. Read limits from the arm, never the manual |
| F11 | Self-collision detection is **whole-arm, at planning time**; the arms shipped with it OFF | Tests enable it at connect (`RM_SELF_COLLISION=0` opts out). Distinct from the GUI's "Collision Protection Level" 0–8 contact threshold |
| **F12** | **Pole speed model corrected.** Drive constants (1250 rpm, RR 0.005) give **100 % = 104.2 mm/s, k = 1.042 mm/s per %**. butterfli_hw's 1.85 implied 2220 rpm — above the drive ceiling; its ×1.5 hw→physical conversion inflated every velocity. The profile is **acceleration-limited**, so short strokes never reach cruise | `lift_travel_time_s()` models a trapezoid/triangle. Validated arm-idle: 140 mm @ 50 % predicts 3.49 s vs **3.47 s measured**; C15 Phase A within ±7 % across 10–100 % on both arms |
| **F13** | `rm_clear_system_err()` **returns 0 without clearing joint error 16384**. Per-joint clear (`rm_set_joint_clear_err`) or the GUI's per-joint button is required | Our `--clear-errors` is insufficient for this fault class — a known gap, and question 4 to RealMan |
| **F15** | **The offline `rm_algo` was constructed as the WRONG arm variant.** `rm_force_type_e.RM_MODEL_RM_B_E` is the base RM75 with no wrist force sensor and is **17.2 mm short**. Measured against both controllers (C14 capture, joints≈0): controller Arm_Tip z = 0.867699 (right) / 0.867698 (left); `RM_MODEL_RM_ISF_E` = 0.867700 — agreement to 1 µm | The `Arm_Tip → ConnectorLink` constant is **15.3 mm, not 32.5 mm**; the tool frames written on 2026-08-08 are 17.2 mm too long and must be rewritten. Variant now set in ONE place (`segment_verifier.FORCE_MODEL_NAME`, env `RM_FORCE_MODEL`). Two related traps: `rm_get_current_arm_state()["pose"]` is reported through the **mounting angle + work frame + tool frame** (the "868 mm mismatch" was the 90° mounting rotation, not a fault), and `rm_frame_t.to_dictionary()` keys the name as **`name`**, not `frame_name` — reading the wrong key silently skipped the tool-frame restore |
| **F16** | **Three ways a rehearsal can pass while proving nothing** (all found 2026-08-08, all fixed). (a) The path stage was matched by NAME (`execute_path`), but stage names differ per task — `cleaning_tasks/config` also uses `execute_cleaning_path`, `square1_motion`, `test_motion` — so a 2012-waypoint stroke collapsed to 2 targets. (b) The plan was resolved from the ROS workspace, and the lab machine's workspace held a different plan under the same filename. (c) The realtime push streams while the arm is IDLE, so index-based subsampling spent 10 of 11 picks inside dwell clusters and read a stage minimum of 46.9 mm against a predicted 19.9 mm — while the paths agreed to 0.0001° | Path stages are now found by **size**; the plan comes only from this repo's `plans/`; the analysis resamples both paths by **arc length**. A rehearsal where every stage collapses to 2 targets now WARNS. Verified end to end against the emulator, and all three are locked in the dry run (F1j/F1m) |
| F14 | Right arm `ready→zero` takes **6.72 s** vs left **3.72 s** at the same 20 %, identical joint deltas, same firmware | Per-arm motion config differs; any arm-duration estimate is per-arm. Unexplained |

---

## 4. PHASE 1 GATE CHECKLIST — the approval table for Phase 2

Legend: ✅ passed · 🟡 partial · ⛔ blocked · ⬜ not started ·
🔵 *finding delivered* (the test's job was to answer a question, and it did).

### 4.1 Execution-layer gates

| ID | Test | What it proves | Blocking? | STATUS | Evidence |
|---|---|---|---|---|---|
| C1 | `test_dual_connect` | Both arms reachable, state readable, handles distinct | yes | ✅ | 9/9, 2026-08-06 |
| C5 | `test_sim_motion_visibility` | Simulated motion observable → rehearsal loop viable | yes | ✅ | 5/5 + YES on all three channels |
| C6 | `test_single_arm_planned` | Planned moves + pole homing + concurrent hand; Cartesian accuracy | yes | ✅ | 7/7 ×3; `dx=+0.196 m` vs +0.20 |
| C7 | `test_hand_only` | Hand controllable with measured feedback (both paths) | yes | ✅ | 8/8 both arms |
| C8 | `test_pole_only` | Pole acceptance + fault diagnosis + recovery | yes | ✅ | **7 PASS / 1 SKIP, both arms, 2026-08-07** (was never-run; now closed) |
| C15 | `test_pole_speed` | Is the pole slow at low speed, or slowed by arm motion? | yes | ✅ | **Phase A 16/16 both arms** — no speed floor; Phase B measured 2.5–3.5× coupling |
| C16 | `test_arm_pole_baseline` | Concurrency with **no models applied** to either device | yes | 🔵 | **Delivered F9.** Phase 0 (singles) passes; every concurrent cell fails. This test *reporting failure* is the successful outcome |
| C9 | `test_single_arm_locked` | Full concept sequence incl. arm+pole sync, one arm | yes | ✅ **closed by C17 2026-08-08** (they share the canfd path; both arms 7/7 with `--no-hands`). Remaining nice-to-have: one run WITH hands for the full sequence. The `planned` backend stays LOCKED (F9, vendor-confirmed) |
| C2 | `test_dual_locked` | Locked dual-arm, barrier per step, dispatch skew | yes | 🟡 Arm-only 6/6. Sync steps were blocked by F9 — **now unblocked by the canfd backend (C17)**; needs one re-run to close |
| C3 | `test_dual_chained` | Chained advance-on-finish ordering invariant | yes | 🟡 Arm-only 4P/1F — **1.6 s gate-latency anomaly still unexplained** (R5). Sync steps now unblocked by canfd; needs one re-run to close |
| C4 | `test_dual_free` | Free-running mode, independent completion | no | 🟡 | Arm-only 4/4; sync untested |

**Honest read:** every gate that does not involve *simultaneous* arm+pole
motion has passed on hardware. The sync gates are blocked not by a bug in our
code but by a controller limitation we have now characterised precisely. The
remaining work on them is to verify the **canfd** backend on hardware.

### 4.2 Bridge gates — ROS 2 ↔ controller

None of C1–C16 sends a target that came from MoveIt. These close that gap.

| ID | Test | Question | Blocking? | STATUS |
|---|---|---|---|---|
| C12 | **Segment collision verifier** (offline) | Predict the controller's motion per segment, FCL-sweep vs the commode meshes at scene pose + `rm_algo` self-collision | yes | ✅ **2026-08-08** — `segment_verifier.py` (WP1) + `run_hinge_verify.py` (WP2) ran on the real saved `hinge_area_right` plan. **Clearance map: all 4 arm stages OK** — transits keep 19.9 mm (Mode A = MoveIt reference exactly); the stroke + retreat are *contact stages* (MoveIt's own path touches by design), judged by touch-fraction vs reference (48 % vs 40 %, 12 % vs 25 %); zero self-collisions. **Mode B not needed for this task.** Residual: joint-linear movel approximation → calibrated by C11 |
| C14 | **Frame alignment** | URDF `*_ConnectorLink` vs RealMan `Arm_Tip` (Newton's design); recreate the glove/ik frames in the controller tree | yes | 🟡 **offline ✅, hardware half must be REDONE.** The trees are kinematically identical (0.000°, zero spread, both arms) — but the constant is **15.3 mm, not 32.5 mm**: the offline solver had been built as `RM_MODEL_RM_B_E` (no force sensor, 17.2 mm short). See F15. The frames written 2026-08-08 are 17.2 mm too long AND the active tool frame was not restored (`to_dictionary()` keys the name as `name`, not `frame_name`). Both bugs fixed; re-run `--create-frames` and confirm the active frame |
| C10 | Chained-target execution (hardware) | `connect=1` queue depth, one arrival event vs N, blend-% reference, mid-chain failure behaviour, `rm_moves` spline | yes | ✅ **PASSED 2026-08-08, 7/7** (right arm). Depths 2/5/10/20 all accepted, none rejected. Blend is real: r=50 runs 2.53 s vs r=0 3.16 s on the same 3-corner path. **Mid-chain invalid target returns ret=1 for THAT SEGMENT ONLY and the chain still completes** — the dispatcher must check every return code, not just the closing one. `rm_moves` spline accepted. Mode B is unblocked |
| C11 | Rehearsal-validation loop (hardware) | SIM-execute → UDP capture → FCL; does the capture match the C12 predictor? | yes | 🟡 **test written 2026-08-08** — `test_rehearsal_validate.py`, split into a **capture** half (SIM, no physical motion) and an **`--replay` analysis** half that runs anywhere, so the lab session is only the capture. Analysis proven offline on a synthetic capture. Produces a **per-stage residual** (joint deg, tool mm, clearance optimism) → the FCL margin C12 must carry. Awaiting a hardware capture |
| C17 | **CANFD sync** (hardware) | Does the shipped `RM_SYNC_BACKEND=canfd` path reproduce `bench_sync` — arm streamed while the pole runs, both complete, no faults? | yes | ✅ **PASSED BOTH ARMS 2026-08-08, 7/7 each.** Sync steps: lift 5.96 s / arm 4.01 s, dispatch skew 6–17 ms (budget 50), pole outlasts the arm by +1.90 s, arrival event for every device, zero latched faults. **R1b closed — the architecture is confirmed end to end** |
| C13 | Fence characterization | What the fence bounds (TCP vs body), reject-vs-stop | **no** | ⬜ Dropped from blocking — safety rests on primitives + offline FCL |

### 4.3 Approval rule

> **Phase 2 is approved when C1, C5, C6, C7, C8, C15, C16, C2, C3 and
> C10, C11, C12, C14, C17 are ✅/🔵.**
> C4 (free mode) and C13 (fence) may stay open — the cleaning task needs
> neither. C9 closes as ✅ once C17 passes (they share the canfd path).

---

## 5. The bridge design (what Phase 2 builds)

### 5.1 Target selection — which command for which segment

| Segment | Command | Why |
|---|---|---|
| Named poses (`rest`, `ready`, transit) | `rm_movej` with **MoveIt's joint values** | Unambiguous; MoveIt has validated the goal state |
| **Stroke entry** | `rm_movej` with MoveIt's joint solution | **Pins the arm configuration** — see the guard below |
| **Cleaning strokes** | `rm_movel` | Cartesian straightness is the functional requirement; dense `movel` would stutter, sparse is what the 20-point path is |
| **Any segment with concurrent pole motion** | `rm_movej_canfd` stream | F9 |

**The guard on `rm_movel`: pin the configuration before the stroke.** It
solves IK seeded from the *current* configuration, so a stroke is
deterministic only relative to where it started. Entering every stroke via a
`movej` to MoveIt's joint solution removes that freedom, and that is what
makes "verify once offline, trust thereafter" sound. Branch flips *between
runs* are the real risk; pinning kills them.

### 5.2 `trajectory_connect` — confirmed in the SDK docs

> `connect=0`: plan and execute now, not connected to what follows.
> `connect=1`: plan this trajectory together with the next, do not execute
> yet; **in blocking mode it returns immediately even on success**.

```python
for p in path[:-1]:
    rm_movel(p, v, r, connect=1, block=0)    # queued
rm_movel(path[-1], v, r, connect=0, block=0) # plans + EXECUTES the chain
# then wait for ONE arrival event
```

1. **`connect=1` never blocks and never completes** — a dispatcher that waits
   on one waits forever.
2. **Our `ArrivalMonitor` already handles this**: it latches only on
   `trajectory_connect == 0` ([`dual_arm_common.py`](src/dual_arm_common.py)).
3. **Blend radius `r` is a PERCENTAGE (0–100), not millimetres.** ZIGZAG01's
   "10" is 10 %. What the percentage is *of* is undocumented → C10.

Unknown and gating Mode B: queue depth, mid-chain failure behaviour, and
whether `rm_moves` (spline, needs ≥3 connected points) follows a curved
surface better than blended `movel`.

### 5.3 Two path modes — pick per segment

| | **Mode A — sparse targets** | **Mode B — reduced MoveIt path** |
|---|---|---|
| Source | named poses + the 20 cleaning points | MoveIt path (geometry only), simplified |
| Command | one at a time | chained with `connect=1`, closed by `connect=0` |
| Use when | the direct move is obviously safe | **constrained geometry — reaching inside the commode** |
| Verified by | C12 | C12 on every sub-segment |

**Selection rule:** start in Mode A; escalate to Mode B when C12 reports the
direct move is not clean. The verifier decides, not a guess.

### 5.4 Dispatch invariants

> **(a) Never issue a motion command to one device while the other is
> mid-trajectory in the PLANNED domain.** It truncates the other's motion —
> and if the victim is the arm, it latches joint faults. Enforce with a
> runtime assertion in the dispatcher, not a convention. *This is also
> RealMan's own prescribed workaround (2026-08-08): run the hoist in
> blocking mode, i.e. strictly serialized with arm motion.*
>
> **(b) If arm and pole must move together, the arm must be in the CANFD
> stream.** That is the only combination the controller supports.
>
> **(c) The hand/glove is exempt** — hand traffic does not touch the motion
> planner (F5, C6 7/7 ×3).

### 5.5 Rehearsal validation (the collision-safety closure)

1. SIM mode, execute the segment; 2. capture the actual joints over UDP;
3. FCL-check the capture against the planning scene; 4. only then run for
real. Per task, not per cycle. Also the natural archive format — it feeds the
studio visualizer (`STUDIO_LINK_RESEARCH.md` Phase 0).

### 5.6 Interfaces

| Component | Owner | Responsibility |
|---|---|---|
| `cleaning_path_gen` | existing | fixture → cleaning points |
| MTC task builder | Phase 2 | cleaning points → stages → validated sparse targets |
| **Segment verifier** | Phase 2 | predict controller motion → FCL + self-collision → verdict |
| **Rehearsal validator** | Phase 2 | SIM execute + UDP capture + FCL; calibrates the verifier |
| **RM dispatcher** | Newton | targets, peripherals, invariants, arrival events |

**Predictor caveat:** the offline `rm_algo` is v1.6.0 while both controllers
now run **1.5.9**. C11 measures the residual and carries it as an FCL margin.

---

## 6. PHASE 2 — hinge-area cleaning

### 6.1 The task already exists

`cleaning_tasks/config/commode_cleaning/commode_c/hinge_area_right_cleaning_points.yaml`:
**20 cleaning points** (8 lid-sides-back + 12 hinge area), start-pose-origin
translation deltas + Euler RPY deltas in `butterfli_ref_frame`,
`ik_frame: R_glove_frame_4`, articulation "lid closed, body static", surface
"concave_interior + near_planar".

**Scope Phase 2 to `hinge_area_right`, right arm, sequential execution.**

### 6.2 Integration point

The file sets `cleaning_path_mode: ruckig_pro_only`. Phase 2 adds a **new
mode** (`controller_planned`) alongside it, so both are runnable on the same
task — the strongest possible A/B evidence for the architecture.

### 6.3 Work packages

| WP | Deliverable | Depends on | Hardware? | Est. |
|---|---|---|---|---|
| WP1 | Segment verifier (`rm_algo` predictor + FCL + self-collision) | — | no | 0.5 d |
| WP2 | C12 report on the real 20-point hinge path; clearance map | WP1 | no | 0.5 d |
| WP3 | MTC task builder for `hinge_area_right` | — | no | 1 d |
| WP4 | Rehearsal validator (SIM + UDP capture + FCL) | C5 | yes (C11) | 1 d |
| WP5 | Dispatcher integration + invariant assertions + wrist-force logging | C10, C14, C17 | yes | 1 d |
| WP6 | End-to-end hinge run + acceptance | WP1–WP5 | yes | 1 d |

### 6.4 Definition of done

> `hinge_area_right` runs end-to-end from an MTC plan — verified,
> rehearsal-validated, executed with planned moves (canfd where the pole
> moves with the arm) — **5 consecutive times** with zero faults, every
> waypoint within tolerance, and measured deviation inside the clearance
> budget.

"Implemented" is not done; 5 clean consecutive runs is done.

---

## 7. Schedule

| Phase | Content | Hardware | Status |
|---|---|---|---|
| **1A** | Architecture, this plan, offline verification | no | ✅ **done** — dry run 97/97, emulated suite 13/13 (incl. backend + lock drills) |
| **1B** | Hardware gate session | yes | 🟡 **partly done 2026-08-07**: C8 ✅, C15 ✅, C16 🔵, C9/C2/C3 characterised. **Remaining: C17, C14, C10, C11** |
| **1C** | WP1+WP2 (C12), draft C14/C17 tests | no | ⬜ next |
| **2** | WP3–WP6, hinge task | partly | 5 working days after 1B closes |

### 7.1 Remaining 1B session (~30 min) — after the 2026-08-08 session

C17, C10 and the C14 capture are **done**. What is left:

```bash
cd RM_API2/Demo/RMDemo_Python/RMDemo_DualArmConcept/src

# 1. C14 — what tool frame is ACTIVE right now? (read-only, nothing moves)
RM_ARM=left  python3 test_frame_alignment.py --mode REAL
RM_ARM=right python3 test_frame_alignment.py --mode REAL
#    read tool_frame.name in the C14CAP line. The 2026-08-08 run failed to
#    restore it (F15), so the arms may be sitting on 'tip'.

# 2. C14 — rewrite the frames with the CORRECTED 15.3 mm constant
RM_ARM=right python3 test_frame_alignment.py --mode REAL --create-frames
RM_ARM=left  python3 test_frame_alignment.py --mode REAL --create-frames
#    now verifies the restore by read-back and refuses to write if it
#    cannot identify what to restore to.

# 3. C11 — one SIM capture (no physical motion, no RM_HOST_IP needed)
RM_ARM=right python3 test_rehearsal_validate.py

# 4. C2 / C3 — sync steps, now unblocked by canfd
python3 test_dual_locked.py  --mode REAL
python3 test_dual_chained.py --mode REAL
```

Then, on any machine with the ROS workspace:

```bash
python3 test_rehearsal_validate.py --replay rehearsal_right.json
python3 run_hinge_verify.py --margin <the number it prints>
```

Stop at the first red gate — later gates assume earlier ones.

**C11 is a capture, not a verdict.** It runs in SIMULATION (no physical
motion) and writes `rehearsal_right.json`; the verdict comes from
`--replay`, which needs no hardware. Its output is the **FCL margin** C12
must then carry.

**Portability:** the plan JSON lives in the ROS workspace, which the lab
laptop may not have — so a copy is bundled at `plans/` and
`segment_verifier.resolve_plan()` prefers the workspace, falls back to the
bundle, and both scripts PRINT which source they used. Only the plan needs
to travel: the capture half reads joint positions and nothing else, while
the analysis half (URDF, SRDF, commode meshes) runs on the machine that
already has the workspace.

**The UDP push target is no longer an operator input.** `host_ip_for()`
asks the kernel which local address routes to that arm, so C5 and C11 work
from any machine or interface without `RM_HOST_IP` (which still pins it if
routing is unusual). This matters because a wrong target is accepted with
`ret=0` and delivers nothing — indistinguishable from "the arm never
moved".

---

## 8. Risks and open questions

| # | Risk / question | Impact | Mitigation |
|---|---|---|---|
| ~~R1~~ | ~~Sync fix unverified~~ | — | **Closed by C16**: planned-domain sync is impossible, not unverified. Superseded by R1b |
| **R1b** | **CANFD sync backend unverified on hardware** | Blocks C17, C9, C2 | Implemented and emulator-proven; first item in the remaining 1B session. Fallback: strict sequencing (pole, then arm) — costs cycle time, not feasibility |
| R2 | The controller's own motion between sparse targets collides | Architecture invalid as specified | C12 answers it offline for the real hinge path. Ladder: intermediate targets → pin stroke entry → canfd for that segment |
| **R2c** | ~~Frame misalignment~~ — **largely RESOLVED offline 2026-08-08**: the kinematic trees agree exactly; the only difference is the constant 32.5 mm Z (now compensated in the generated tool frames). The GUI's old hand-entered tool frame (−35, 10, 260) matches none of the derived frames — replace it via `--create-frames`, don't trust it | residual: one hardware confirmation | C14 hardware half: create frames, verify a pose, restore |
| R3 | `connect=1` chaining differs from the docs | Dispatcher redesign | C10 measures it |
| ~~R4~~ | ~~Firmware mismatch~~ | — | **Closed 2026-08-07**: both arms V1.7.4 / algo 1.5.9; lift gearing auto-detected |
| R5 | **C3's 1.6 s follower-gate latency** unexplained | Unknown chained-mode timing | One dedicated look; chained is the likely Phase 2 execution mode |
| R6 | Dual-arm concurrent collision validity | Locked/free modes could collide | Phase 2 scoped to one arm; `dual_arm_plan_collision.py` exists |
| **R7** | ~~Awaiting RealMan reply~~ — **ANSWERED 2026-08-08**: the F9 conflict is a *documented Gen-3 defect*, engineers on it, **no ETA**; workaround = hoist in blocking mode. The `expand`-field and clear-error questions remain open with them | The planned-sync option stays closed until they ship a fix | **Planned backend LOCKED** (`RM_UNLOCK_PLANNED_SYNC=1` is the re-test hatch); no schedule dependency on RealMan — canfd is the path |
| **R8** | `rm_clear_system_err` does not clear joint 16384 (F13) | Recovery needs the GUI | Add `rm_set_joint_clear_err` per joint to `clear_errors()` |
| R9 | Right arm is 1.8× slower than left for identical moves (F14) | Per-arm timing models | Unexplained; matters if either arm's duration is ever estimated |

**Answered:** hinge pose is **fixture-taught** (no perception) · the glove is
worn on the hand, so **hand pose = glove state**, cleaning with the **back of
the hand** · **no force control** for now, but force feedback is available
from the **wrist 6-axis sensor** (`tool_zero_force_data`), not the hand.

---

## 9. Next actions

**Offline (no hardware):**
- [x] WP1: segment verifier — `src/segment_verifier.py` (reuses
      butterfli_workspace FK/FCL, studio fixture meshes, rm_algo)
- [x] WP2 / C12: clearance map for `hinge_area_right` — **all stages OK,
      Mode B not needed**; transits 19.9 mm, contact stages judged
      touch-fraction vs the MoveIt reference
- [x] C14 draft: `test_frame_alignment.py` — captures C14CAP records
      (joints, controller pose, tool/work frame, offline FK) for the
      offline URDF comparison; flags a configured tool-frame offset on the
      spot
- [x] C10 draft: `test_waypoint_chain.py` — W1 chain/events, W2 queue
      depth 2/5/10/20, W3 blend r=0 vs 50, W4 mid-chain invalid target
      (skippable), W5 rm_moves spline
- [x] R8: `clear_errors()` now clears flagged joints PER JOINT via
      `rm_set_joint_clear_err` (the Web GUI button's API); emulator models
      the rm_clear_system_err gap faithfully
- [x] C14 offline: `frame_alignment_offline.py` — **trees identical**, one
      constant 32.5 mm; compensated tool-frame table generated, and
      `--create-frames` added to the hardware test
- [x] WP4 / C11 draft: `test_rehearsal_validate.py` — SIM capture +
      `--replay` analysis; per-stage residual → the C12 FCL margin

**Hardware — the remaining ~30 min (see §7.1):**
- [x] C17 — canfd sync, **both arms 7/7, 2026-08-08**
- [x] C10 — waypoint chaining, **7/7, 2026-08-08**
- [ ] C14 — read the active tool frame, then rewrite the frames at 15.3 mm
- [ ] C11 — one SIM capture; the verdict is produced offline afterwards
- [ ] C2 / C3 — re-run now that the sync steps work on canfd

**Next build (no hardware needed):** WP3 — the MTC task builder for
`hinge_area_right`, the first piece of Phase 2 proper. It is not blocked
by any of the above: C12 has already declared the sparse-target execution
collision-safe, and C11 only tightens the margin it is checked against.

**External:**
- [x] RealMan replied (2026-08-08): defect confirmed, no ETA, blocking-mode
      workaround. Follow-ups still open: `expand` field purpose, clear-error
      gap, and a timeline (asked)
