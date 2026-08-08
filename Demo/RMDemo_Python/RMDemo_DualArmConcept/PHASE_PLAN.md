# Butterfli — Phase Plan: ROS 2 planning + controller execution

*Living document. Newton owns the schedule and the orchestrator; this file is
the shared contract for what "feasible" means and what closes each gate.*
*Created 2026-08-07 · **substantially revised 2026-08-07 evening** after the
C15/C16 hardware sessions overturned the arm–pole synchronization design.*

---

## 0. Status at a glance

*(Fully revised 2026-08-08 late — all Phase 1 blocking gates are closed;
Phase 2 is APPROVED and in progress.)*

| | |
|---|---|
| **Architecture** | ROS 2 plans/validates; the RealMan controller executes. **Confirmed end to end**: canfd sync (C17 both arms), chained movel to depth 100 (C10), movej joint-linear to 0.004° (C11), frame chain vs both controllers 0.0 mm / 0.07° (C14). |
| **Phase 1** | **All blocking gates ✅** — C1 C2 C3 C5 C6 C7 C8 C10 C11 C12 C14 C15 C16(🔵) C17. C4/C13 stay open by design (non-blocking). |
| **Phase 2 build** | **Delivered and emulator-verified**: `task_config.py` (two-file config merge, serialization enforced via `RM_SERIALIZE`), `cleaning_path.py` (config-resolved Cartesian path, mount-corrected, hover A/B), `stage_runner.py` (blocking dispatcher, SIM assumes pole/hand — F3), `speed_limits.py`, `power_probe.py`, `recover_joints.py`. Four tasks bundled and verified: hinge_area_{right,left}, toplid_{right,left}. Dry run **208/208**, emulated suite **19/19** (all four tasks end to end). |
| **Speeds** | MoveIt's scaling == controller % (F20). Policy: **cleaning 100 %, everything else 50 %**; pole saturates against the drive first. `RM_SPEED_DERATE` for bring-up. |
| **Open item (NOT a gate)** | **F21 / R10**: J6/J7 Under-Voltage during the first free-space run at the new speeds; recovery + probe ladder defined. ZIGZAG01 — our own RealMan-level MOVEL program — already executes this motion class on this hardware, so this is an execution debug, not a feasibility question (Newton). |
| **Vendor** | F9 (arm+pole planned-domain conflict) vendor-confirmed, no ETA; sync stays canfd-only, planned backend LOCKED. Under-voltage evidence (power_probe CSVs) goes to the same support thread. |

---|---|
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

**And it is more than that (Newton, 2026-08-08): ZIGZAG01 is the
RealMan-level version of our task files** — the same motion class Phase 2
executes (a MOVEL waypoint program with blend, pole staged between arm
motion) written directly against the controller, and it runs on this very
hardware. That is why the F21 under-voltage incident is classified as an
execution bug to introspect and fix — speeds, transients, supply margin —
and **not** a gate on Phase 2: the capability itself is already
demonstrated on this machine.

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
| **F17** | **The cleaning path is a CONFIG artifact, not a trajectory artifact.** `TaskBase::resolveCleaningWaypoints` resolves `cleaning_points` (translation + rotation deltas) against an anchor — declared `start_pose`, else a lookback to the preceding stage's FK — with **both deltas expressed in `butterfli_ref_frame`**: position added in that frame's axes, `R_delta = Rx·Ry·Rz` (degrees) applied by LEFT-multiplication. `cleaning_sequence` is a chained polyline (only each segment's `.second` is emitted), giving 44 waypoints for each hinge task and 28 per toplid. The stage then computes ONE continuous Cartesian path at `MaxEEFStep(0.01)` and hands it all to a single Ruckig Pro call | Reproduced offline to **0.3–0.6 mm mean** against all four saved plans, both arms. So the movel program is the config's own strokes — no subsampling of the saved JSON, and the "how many targets" question dissolves. `cleaning_path.py` emits it; every segment is queued (43 / 27), which **exceeds the 20 C10 verified** |
| **F18** | **Two silent-substitution bugs, both caught before hardware.** (a) `rm_set_manual_tool_frame` CREATES — it returns ret=1 on an existing name, which is why the second C14 run failed on all five frames; existing frames need `rm_update_tool_frame`. (b) The commode tasks use hand poses `open_tenth` (1.17 rad) and `quarter_grasp` (0.336) that are not concept states, and substituting the nearest-named state sent `open_tenth` to **993 counts instead of ~130** — the hand flying open mid-task while gripping a cloth against the fixture | Frames are now created-or-updated with a read-back match table. Hand targets come from the **plan's own joint angles** through `hand_rad_to_hw()`, a fit derived from the three states we hold in both radians (SRDF) and counts (bench) — it reproduces all three to within 1 count. A pose with neither source is SKIPPED, never guessed |
| **F19** | **The controller accepts at least 100 chained `movel` segments** (hardware, 2026-08-08: depth 100, no rejection, 65.4 s). Hinge needs 43, toplid 27 | **No batching needed** — a cleaning path is one chain, as designed. This was the last result that could have forced a dispatcher redesign. Caveat: blend `r=50` saved only 0.01 s over `r=0` on that run (an earlier probe showed 0.64 s), so the 10 % blend radius is not yet justified by measurement |
| **F20** | **MoveIt's velocity scaling IS the controller's percentage.** `joint_limits.yaml` gives 3.14 / 3.92 rad/s = **179.9 / 224.6 deg/s** against the controller's measured **180 / 225** (F10) — the same ceiling. So `max_velocity_scaling: 1.0` means v=100 %. The POLE does not line up: MoveIt allows 160 mm/s, the drive does 104.2 (F12), so scaling 1.0 asks 154 % and saturates | Speeds now come from the config, not a hard-coded cap (an earlier version multiplied by 20 — a silent 5× slowdown). Operating policy (Newton): **cleaning 100 %, everything else 50 %**; the pole saturates against the DRIVE before the halving. `start_pose_velocity_scaling` is a MULTIPLIER on the task scaling, not a scaling of its own. Acceleration and jerk scaling cannot be mapped — `movej`/`movel` take only `v` and `r`, so the task's jerk 0.3 never reaches the arm |
| **F21** | **J6/J7 "Under Voltage" incident (left arm, 2026-08-08 ~23:00).** During the first session at the new speeds (transit 50 %, cleaning 100 % — all prior sessions ran 20 %), the movel chain aborted ~4.8 s in (controller pushed trajectory_state=False); GUI showed **Joint6/Joint7 Under Voltage, de-enabled**. Every later motion died in ~0.07 s (dead joints reject all commands — including movej, so NOT a frame issue). **Teach mode was violent because pressing it releases ALL brakes and unpowered wrist joints fall under gravity**; releasing re-clamps mid-fall. Fault recurred after reboot. Install pose ruled out as cause: our code has NO setter call (grep), and the 18:58 capture read install_pose **[0, 90, 0]** back from both arms. **The motion was FREE SPACE — no contact with the commode occurred (Newton, 2026-08-08), so contact torque is ruled out for this incident.** Working hypothesis, re-ranked: (1) **supply/harness margin** — free-space motion at the first-ever 50/100 % speeds draws accel-transient current the bus should absorb but didn't, and J6/J7 sit at the far end of the internal power chain where sag lands first; recurrence at boot (no motion) is the strongest hardware signal. (2) wrist-drive demand — Cartesian tracking can load the small wrist drives specifically; the probe distinguishes by WHICH joint's current spikes at the dip. A healthy arm sustains rated free-space motion, so if dips track motion this is a hardware/support case for RealMan, not a design limit | **Do not use teach mode with disabled joints — support the wrist.** Recovery is PER JOINT: clear error + re-enable (`recover_joints.py --enable 6,7`, or the GUI's Clear Error + Enable). Gate hardened: `error_state()` now reads `rm_get_joint_en_state` and REFUSES with disabled joints; the connect test no longer reports "clean" over dead joints (it did exactly that during this incident). Our one real state leak fixed: the runner left the active tool frame on `L_glove_4` after the abort (visible in the GUI screenshot) — now restored in a `finally`. Correlation test when recovered: rerun at `RM_SPEED_DERATE=0.4` |
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
| C2 | `test_dual_locked` | Locked dual-arm, barrier per step, dispatch skew | yes | ✅ **PASSED 2026-08-08 (7/7)** — full sequence incl. BOTH sync steps on canfd, both arms: max dispatch skew 27 ms, pole outlasts the arm +1.93 s, arrival events for every device |
| C3 | `test_dual_chained` | Chained advance-on-finish ordering invariant | yes | ✅ **CLOSED on the 2026-08-08 data** — ordering invariant passed, both chains completed; event-gated follower latencies 0.2–10.8 ms. The 4.7–5.3 s figures were OUR polling on hand steps (no device-2 events — F4), not the arms; the check now scopes to event-gated steps. Newton: not an anomaly — arms from the same pose to the same target finish together |
| C4 | `test_dual_free` | Free-running mode, independent completion | no | 🟡 Arm-only 4/4; sync untested — non-blocking, the cleaning tasks do not use free mode |

**Honest read:** every gate that does not involve *simultaneous* arm+pole
motion has passed on hardware. The sync gates are blocked not by a bug in our
code but by a controller limitation we have now characterised precisely. The
remaining work on them is to verify the **canfd** backend on hardware.

### 4.2 Bridge gates — ROS 2 ↔ controller

None of C1–C16 sends a target that came from MoveIt. These close that gap.

| ID | Test | Question | Blocking? | STATUS |
|---|---|---|---|---|
| C12 | **Segment collision verifier** (offline) | Predict the controller's motion per segment, verify against the scene | yes | ✅ **REBUILT and re-closed 2026-08-08**: base state now comes from the PLAN (`plan_state_upto` — the pole carries the arm base; SRDF home had it 215 mm high, voiding all earlier numbers), fixture filtered to the task's declared articulation (`commode_fixture_type: closed` — checking against lid_open put phantom contacts in clear stages), and contact stages judged by the **config contract** (declared links × declared surface objects, `contact_links.yaml`) instead of touch fraction. All four tasks verify clean at the C11 margin (5 mm). Note: link groups are currently the full hand set per ik_frame (Newton: to be narrowed — the verdict reads the file, so it sharpens automatically) |
| C14 | **Frame alignment** | URDF `*_ConnectorLink` vs RealMan `Arm_Tip`; recreate the glove/ik frames in the controller tree | yes | ✅ **CLOSED 2026-08-08 18:58, both arms.** Trees kinematically identical; constant = **15.3 mm** (ISF model — F15). All six URDF-named frames (`R_glove_1..4`, `R_tip`, `R_index_tip` + left mirrors) written with create-or-update, **read-back match table 0.00 mm on every row**, superseded `glove1..tip` deleted, active frames restored and verified (`Arm_Tip`/`Hand`). Model check: |p| agrees 0.00–0.01 mm, full vector 0.0 mm with install pose (0, 90, 0) mirrored. Frame chain further validated end to end: commanded-pose math vs both controllers' reported poses **0.0 mm / 0.07°** |
| C10 | Chained-target execution (hardware) | `connect=1` queue depth, one arrival event vs N, blend-% reference, mid-chain failure behaviour, `rm_moves` spline | yes | ✅ **PASSED, and re-probed 2026-08-08: depth 100 accepted with no rejection.** Mode B is unblocked and a cleaning path can be queued whole (43 / 27 segments). Mid-chain invalid target returns ret=1 for THAT SEGMENT ONLY while the chain still completes — the dispatcher must check every return code. Blend effect was unmeasurable on the re-probe (F19) |
| C11 | Rehearsal-validation loop (hardware) | SIM-execute → UDP capture → FCL; does the capture match the C12 predictor? | yes | ✅ **PASSED 2026-08-08, 7/7 capture + 3/3 analysis.** Right arm, SIM, the repo's plan, 26/26 targets, 10177 frames over 50.9 s. **Residual: 0.003–0.004° joint, 0.01 mm at the tool, on EVERY stage including the 20-target cleaning chain. Clearance optimism 0.00 mm.** `rm_movej` between two joint targets is joint-linear to four thousandths of a degree — the C12 model is not an approximation, it is exact for movej. Applied margin: 5 mm (measured 0 + safety) |
| C17 | **CANFD sync** (hardware) | Does the shipped `RM_SYNC_BACKEND=canfd` path reproduce `bench_sync` — arm streamed while the pole runs, both complete, no faults? | yes | ✅ **PASSED BOTH ARMS 2026-08-08, 7/7 each.** Sync steps: lift 5.96 s / arm 4.01 s, dispatch skew 6–17 ms (budget 50), pole outlasts the arm by +1.90 s, arrival event for every device, zero latched faults. **R1b closed — the architecture is confirmed end to end** |
| C13 | Fence characterization | What the fence bounds (TCP vs body), reject-vs-stop | **no** | ⬜ Dropped from blocking — safety rests on primitives + offline FCL |

### 4.3 Approval rule

> **APPROVED 2026-08-08: every blocking gate is ✅/🔵** — C1 C2 C3 C5 C6
> C7 C8 C10 C11 C12 C14 C15 C16 C17 (C9 closed with C17).
> C4 (free mode) and C13 (fence) stay open by design — the cleaning tasks
> use neither. Phase 2 is in progress; its remaining risk is R10
> (under-voltage debug), which per Newton is NOT a gate.

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

### 6.3 Work packages — re-scoped 2026-08-08

The build took a shorter path than planned: the TASK CONFIG is the
specification (F17), so no MTC task builder is needed for these tasks —
`stage_runner.py` executes the config directly, taking named-pose joint
values from the saved plans and the cleaning path from the config.

| WP | Deliverable | Status |
|---|---|---|
| WP1 | Segment verifier | ✅ `segment_verifier.py` |
| WP2 | Clearance map on the real path | ✅ `run_hinge_verify.py`, all four tasks, contract verdict |
| ~~WP3~~ | ~~MTC task builder~~ | **obsolete** — the config is the source; revisit only if a task ever needs MoveIt-side replanning |
| WP4 | Rehearsal validation | ✅ as C11 tooling (`test_rehearsal_validate.py`, capture + replay) |
| WP5 | Dispatcher (serialized, invariants, speeds, frames) | ✅ **built + emulator-verified** — `stage_runner.py`; first clean hardware run pending R10 |
| WP6 | End-to-end runs + acceptance | ⬜ **the remaining milestone** — blocked only by the R10 debug ladder |

### 6.4 Definition of done

> Each of the four tasks (hinge_area_right/left, toplid_right/left) runs
> end-to-end from its task config — serialized stages, chained movel at
> the policy speeds — **5 consecutive times** with zero faults, every
> waypoint within tolerance, and contact only where the config allows.

"Implemented" is not done; 5 clean consecutive runs is done.

---

## 7. Schedule

| Phase | Content | Status |
|---|---|---|
| **1A** | Architecture, offline verification | ✅ |
| **1B** | Hardware gates | ✅ **closed 2026-08-08** — every blocking gate passed (see §4) |
| **2 build** | Config loader, path resolver, dispatcher, probes | ✅ dry run 208/208, emulated 19/19, four tasks end to end under emulation |
| **2 execution** | R10 debug → first clean run → 4-task rollout → acceptance (5× clean each) | 🟡 **current** |

### 7.1 Next hardware session — R10 debug ladder, then the first task

```bash
cd RM_API2/Demo/RMDemo_Python/RMDemo_DualArmConcept/src

# 0. recover the wrist joints (support the wrist; NO teach mode while disabled)
RM_ARM=left python3 recover_joints.py                 # report
RM_ARM=left python3 recover_joints.py --enable 6,7    # clear + enable
python3 test_dual_connect.py                          # now catches dead joints

# 1. idle power baseline — if V(J6/J7) is low at rest, stop: hardware/support
RM_ARM=left python3 power_probe.py --seconds 10 --label idle

# 2. reproduce the free-space chain with the probe attached, derate ladder
RM_ARM=left python3 power_probe.py --seconds 90 --label derate04 &
RM_SPEED_DERATE=0.4 python3 stage_runner.py --task hinge_area_left --mode REAL
#    clean? step up: 0.7, then 1.0 — the CSV shows dip depth, joint, current
#    (which joint's CURRENT spikes at the dip: J2/J4 -> supply margin;
#     J6/J7 -> wrist-drive demand)

# 3. whatever derate is clean = the operating speed until R10 is resolved;
#    then the rollout: all four tasks, then 5x acceptance runs each
```

## 8. Risks and open questions

| # | Risk / question | Impact | Mitigation |
|---|---|---|---|
| ~~R1~~ | ~~Sync fix unverified~~ | — | **Closed by C16**: planned-domain sync is impossible, not unverified. Superseded by R1b |
| ~~R1b~~ | ~~CANFD sync backend unverified on hardware~~ | — | **Closed by C17 (2026-08-08, both arms 7/7)** |
| ~~R2~~ | ~~Controller motion between sparse targets collides~~ | — | **Closed**: C12 clearance maps on all four tasks + C11 measured the movej model EXACT (0.004°, optimism 0.00 mm) |
| ~~R2c~~ | ~~Frame misalignment~~ | — | **Closed by C14 hardware (2026-08-08)**: frames written at the corrected 15.3 mm, read-back 0.00 mm, and the commanded-pose chain validated against both controllers to 0.0 mm / 0.07° |
| ~~R3~~ | ~~`connect=1` chaining differs from the docs~~ | — | **Closed by C10**: depth 100 accepted; per-segment ret checking required (bad segment fails alone, chain completes) |
| ~~R4~~ | ~~Firmware mismatch~~ | — | **Closed 2026-08-07**: both arms V1.7.4 / algo 1.5.9; lift gearing auto-detected |
| R6 | Dual-arm concurrent collision validity | Locked/free modes could collide | Phase 2 scoped to one arm; `dual_arm_plan_collision.py` exists |
| **R7** | ~~Awaiting RealMan reply~~ — **ANSWERED 2026-08-08**: the F9 conflict is a *documented Gen-3 defect*, engineers on it, **no ETA**; workaround = hoist in blocking mode. The `expand`-field and clear-error questions remain open with them | The planned-sync option stays closed until they ship a fix | **Planned backend LOCKED** (`RM_UNLOCK_PLANNED_SYNC=1` is the re-test hatch); no schedule dependency on RealMan — canfd is the path |
| **R8** | `rm_clear_system_err` does not clear joint 16384 (F13) | Recovery needs the GUI | Add `rm_set_joint_clear_err` per joint to `clear_errors()` |
| R9 | Right arm is 1.8× slower than left for identical moves (F14) | Per-arm timing models | Unexplained; matters if either arm's duration is ever estimated |
| **R10** | **J6/J7 Under-Voltage during free-space motion at the 50/100 % speeds** (F21) — recurred at boot once | Blocks WP6 acceptance runs until debugged; **explicitly NOT a Phase 2 gate (Newton)**: ZIGZAG01 — our own RealMan-level MOVEL program — already executes this motion class on this hardware, so this is introspection + debugging, not feasibility | Ladder in §7.1: recover joints → idle probe (hardware verdict) → derate ladder with `power_probe` CSVs (which joint's current spikes at the dip: J2/J4 = supply margin, J6/J7 = wrist demand) → operate at the clean derate meanwhile; CSVs to the open RealMan thread |

**Answered:** hinge pose is **fixture-taught** (no perception) · the glove is
worn on the hand, so **hand pose = glove state**, cleaning with the **back of
the hand** · **no force control** for now, but force feedback is available
from the **wrist 6-axis sensor** (`tool_zero_force_data`), not the hand.

---

## 9. Next actions

**Hardware (the R10 ladder + rollout — §7.1 has the commands):**
- [ ] Recover J6/J7, verify with the hardened connect test
- [ ] Idle power baseline (decides hardware-vs-load in one measurement)
- [ ] Derate ladder 0.4 → 0.7 → 1.0 with `power_probe` attached; CSVs to
      the RealMan thread; adopt the clean derate as the operating speed
- [ ] First clean `hinge_area_left` run, then all four tasks
- [ ] WP6 acceptance: 5 consecutive clean runs per task

**Offline (opportunistic):**
- [ ] Narrow `contact_link_groups` per ik_frame when Newton revises the
      contract (the verdict sharpens automatically)
- [ ] Blend radius: r=10 is unjustified by measurement (C10 re-probe saw
      no effect); measure or drop to 0 during the rollout
- [ ] R9 / F14: right arm 1.8× slower than left for identical moves —
      still unexplained, matters for cycle-time estimates

**External:**
- [x] F9 vendor-confirmed (no ETA) — sync stays canfd-only
- [ ] RealMan follow-ups: under-voltage CSVs (R10), `expand` field,
      `rm_clear_system_err` gap, fix timeline
