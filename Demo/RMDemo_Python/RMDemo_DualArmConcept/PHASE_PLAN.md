# Butterfli — Phase Plan: ROS 2 planning + controller execution

*Living document. Newton owns the schedule and the orchestrator; this file is
the shared contract for what "feasible" means and what closes each gate.*
*Created 2026-08-07 · **substantially revised 2026-08-07 evening** after the
C15/C16 hardware sessions overturned the arm–pole synchronization design.*

---

## 0. Status at a glance

*(Revised 2026-08-10 after a full ledger audit. Several earlier claims are
corrected or retracted here — see F22-F25 and the gate notes.)*

| | |
|---|---|
| **Architecture** | ROS 2 plans and validates; the RealMan controller executes. Frame chain now proven against both controllers: **0.0 mm / 0.07 deg**. |
| **Phase 1** | All blocking gates ✅/🔵. Caveat recorded: C2/C3/C9/C17 were measured at **arm v=20 %**, while the tasks now run 50/100 % — the evidence is at a setting that no longer ships. |
| **Phase 2 build** | `task_config.py`, `cleaning_path.py`, `stage_runner.py`, `controller_caps.py`, `speed_limits.py`, `power_probe.py`, `recover_joints.py`. Four tasks: hinge_area_{right,left}, toplid_{right,left}. Dry run **216/216**, emulated suite **20/20**. |
| **Hardware status** | **No task has yet completed on hardware.** `execute_path` failed in REAL *and* in SIMULATION (F21b). J6/J7 latched Under-Voltage in a separate, free-space incident (F21a). |
| **Root cause** | **NOT established.** An earlier claim that the movel chain is Cartesian-infeasible was **RETRACTED** — it was an artifact of global tool-frame state (F24). |
| **Prepared for the next run** | Mount angle read from `rm_get_install_pose()`; **singularity avoidance enabled** (was supported-but-OFF, F23); failure diagnostics that name the reason; the global-state trap guarded. |
| **Open risks** | R10 under-voltage (needs the probe ladder) · R11 C12 verifies a different path than we execute, and neither side is hardware truth (F22) · R12 blend r=10 unmeasured. |

---

## 1. The architecture

**ROS 2 decides WHERE to move; the RealMan controller decides HOW to get
there.** MoveIt/MTC are a **task planner and a validator — not a path source
to be reproduced**. They own the scene, URDF/TF, IK, reachability and the
collision verdict; they do *not* hand over a dense trajectory. Execution
sends the controller a **short list of sparse targets**.

The safety question is therefore **not** "does the executed path match
MoveIt's path?" — nothing is trying to match it. It is "**is the motion the
controller actually produces collision-free in this scene?**"

> **AUDIT 2026-08-10 — this question is NOT currently answered.** C12
> sweeps the saved MoveIt trajectory joint-linearly; the dispatcher sends
> Cartesian `movel` through the config's waypoints with the controller
> solving IK. The two differ by **57 mm**, and per F22 the saved plan was
> never executed either — so the comparison is model-vs-model. See R11 for
> the two ways to close it. Self-collision detection (now enabled) is the
> only online backstop meanwhile.

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
| **F19** | **The controller accepts at least 100 chained `movel` segments** (hardware, 2026-08-08: depth 100, no rejection, 65.4 s) | **No batching needed** — a cleaning path is one chain. *Blend sub-claim CORRECTED (audit 2026-08-10): the two probes disagree — r=50 saved 0.64 s in one run and 0.01 s in another on a different 3-corner path. Blending is therefore UNMEASURED, and the `r=10` in the movel program is unjustified (R12).* Note `test_waypoint_chain.DEPTHS` must use `max()` for its verdict — a non-ascending list reported "deepest 13" after depth 100 had passed |
| **F20** | **MoveIt's velocity scaling IS the controller's percentage.** `joint_limits.yaml` gives 3.14 / 3.92 rad/s = **179.9 / 224.6 deg/s** against the controller's measured **180 / 225** (F10) — the same ceiling. So `max_velocity_scaling: 1.0` means v=100 %. The POLE does not line up: MoveIt allows 160 mm/s, the drive does 104.2 (F12), so scaling 1.0 asks 154 % and saturates | Speeds now come from the config, not a hard-coded cap (an earlier version multiplied by 20 — a silent 5× slowdown). Operating policy (Newton): **cleaning 100 %, everything else 50 %**; the pole saturates against the DRIVE before the halving. `start_pose_velocity_scaling` is a MULTIPLIER on the task scaling, not a scaling of its own. Acceleration and jerk scaling cannot be mapped — `movej`/`movel` take only `v` and `r`, so the task's jerk 0.3 never reaches the arm |
| **F21a** | **J6/J7 "Under Voltage", left arm, 2026-08-08 ~23:00.** Latched during a FREE-SPACE run at the first 50/100 % session; recurred after a power cycle with the arm at rest. Teach mode with a disabled wrist produced violent motion on release | Recovery: `recover_joints.py` (report-only by default). Ranking after Newton's correction that no contact occurred: **supply/harness margin** (recurrence at rest needs no motion story) > wrist-drive demand > pure speed. `power_probe.py` records per-joint V/I at 100 Hz; the idle baseline decides hardware-vs-load in one measurement. CSVs go to the RealMan thread |
| **F21b** | **`execute_path` ALSO fails in SIMULATION** (stage_runner.log:686, hinge_area_left, 23:20, with the mount fix present). SIM moves nothing and draws no meaningful current | **This decouples the chain failure from the under-voltage entirely — they are two separate problems.** All 43 segments are ACCEPTED (no rejections); the chain then fails during execution. Cause still unknown: the runner reported only `arrived and ok`. `diagnose_failure()` now reads errors, joint enables, run mode, collision stage, singularity mode, tool frame and joints at the moment of failure |
| **F22** | **The saved `*_ruckig_pro_only.json` is PLANNER OUTPUT, never executed.** `savePlannedTrajectory(*solution)` writes the MTC `SolutionBase` (task_base.cpp:4898 and :4979 — *both* paths save the plan, not feedback), and all four of our plans carry `plan_only=True` | Determines what it can justify. **Sound uses:** named-pose joint values (we need one validated IK branch out of the 7-DOF continuum — being flown is irrelevant, we fly it ourselves and the arrival event confirms it) and hand targets. **Unsound use:** C12 labels it "dense (MoveIt ref)" and judges candidates against it — it is a SECOND MODEL, not ground truth, and its authority is a plan-time collision check computed in a scene whose pole was 215 mm wrong until 2026-08-09. The only execution-grounded artifact we hold is C11's 10 177-frame capture |
| **F23** | **V1.7.4 capabilities exist and were never switched on.** Probe of both arms: singularity avoidance *supported, OFF* (7-axis support landed V1.7.3); static-state collision, payload self-collision, electronic fence, manual collision-release all *supported, OFF*; dynamics `collision_stage` 4 | Singularity avoidance is the controller's own answer to resolving 7-DOF redundancy along a Cartesian path — precisely the regime `execute_path` runs in. `controller_caps.py` now censuses all six, enables singularity avoidance + self-collision for the run, and restores the arm as found |
| **F24** | **RETRACTED: "the movel chain is Cartesian-infeasible".** An analysis claimed straight-line segments force ~180 deg J7 flips and 100 % singular configurations. It was an ARTIFACT: `rm_algo_set_toolframe` is GLOBAL C state, and constructing a `SegmentVerifier` (which builds its own `Algo`) silently reset it, so every IK target sat 227 mm beyond the real tool point — forcing a straight elbow (J4=0.000 in every solution) which then read as singular | Redone with the tool frame set last and a guard asserting FK of a known configuration: **hinge_area_left is completely clean (0/430 IK failures, 0 singular, 11 deg max joint step) — and hinge_area_left is the task that failed in SIM.** The mechanism is therefore NOT established. Lesson recorded in `segment_verifier._load_algo` and enforced by `cleaning_path.assert_toolframe_intact()` |
| **F25** | **The emulator can certify bugs the hardware rejects.** Three reached the arms this way: `rm_set_hand_angle` missing its positional `timeout`, `rm_movel` handed an `rm_pose_t` when it indexes `pose[:3]`, and a V1.7.1-era block of hard-coded capability getters defined AFTER the stateful ones — so `avoid_singularity` always read 0 and a setter appeared to work forever | The emulator now mirrors the real signatures exactly, and the dry run audits **every SDK call site (125) against the real signature arity**. A capability drill in the suite asserts read/apply/restore round-trips against the EMULATED SDK, not a mock — the dry run's mock agreed with reality while the emulator did not, and the suite runs the emulator |
| **F26** | **`execute_path` aborts on the FIRST movel segment with controller system error `0x100D` (4109) — the arm is NOT the problem.** First run with the 100 Hz recorder (`runs/20260810T160921_hinge_area_left_left`, SIMULATION mode, hinge_area_left, left arm). What the stream shows, sample by sample: all 43 `rm_movel` calls returned **ret=0** (the dispatcher checks every one and reports rejects — it reported none); the arm then sat **IDLE for 5.44 s** before moving at all; the first segment ran for **0.14 s**, moving the joints ~1.4 deg, before `arm_current_status` went to **9 = `RM_STOP_E` (急停 / emergency-stop state)**. At the abort: **no joint error codes, no disabled joints**, `avoid_singularity=1`, `self_collision=True`, `collision_stage=3`, tool `L_glove_4`, joints `[-4.6, -33.9, 14.3, 100.9, 21.4, -40.2, 159.3]`. Total travel **49.9 mm of a 4763 mm program — 1.0 %**. **`0x100D` is the same code that latched on 2026-08-08** (`test_dual_connect.log` `active=['4109']`, `stage_runner.log` `LATCHED ERRORS — system 4109`), so this is a reproduction of the original failure, now instrumented. Distinguish it from `0x1002`/4098, which appeared WITH joint errors `16384` during the F9 pole/arm truncation — a drive fault. `0x100D` arrives with the joints clean, in SIMULATION, so it is a **controller planning/kinematics rejection, not a drive or power event** | **Two things it is NOT.** Not F21a under-voltage: no joint disabled, no joint error, and SIM mode draws no load. Not a bad start pose: `move_to_start` left the TCP **0.0 mm** from waypoint 0 once measured in the correct tool frame (the apparent 44.5 mm gap is a reporting artifact — the joints are bit-identical across it, so it is the tool-frame change, not motion). **`0x100D` IS NOW DECODED** (RealMan JSON-protocol error appendix, supplied by Newton 2026-08-10): **`0x100D` = 机械臂发生碰撞, "arm collision detected"**. Note it is NOT `0x1012` (自碰撞错误, self-collision) and NOT `0x1013` (electronic fence) — the controller reports a COLLISION verdict. Since the run was in SIMULATION, no physical contact existed, so this is the controller's own MODEL talking. **A blend-radius hypothesis raised before the code was decoded is RETRACTED** — the 180-deg reversals are real geometry (23 of 42 corners) and R12 still wants measuring, but they are not what this error says. What the offline model says about the very same path: `movel AS EXECUTED`, 976 samples at 5 mm — **919 in contact with the commode (intended: `execute_path` is a contact stage, only declared objects touched by declared links) and `self=0`, zero self-collision** on `rm_algo_safety_robot_self_collision_detection`, the controller's own algorithm. The controller has NO model of the commode, so its verdict cannot be the fixture. That leaves its self-collision model and — the untested one — **`endeff_collision`, which is evaluated against the ACTIVE TOOL FRAME, and ours is `L_glove_4`, far off the flange, where our offline check models no tool at all.** `controller_caps.py` now accepts `RM_SELF_COLLISION` / `RM_ENDEFF_COLLISION` / `RM_COLLISION_STAGE` so each check can be switched off in isolation and the abort attributed — see §7.1 |
| **F27** | **ONE segment fails identically — it is not the chain, and it is not the blend.** `--max-segments 1`, 2026-08-10 16:39 (`runs/20260810T163929_hinge_area_left_left`): a **single** `rm_movel` from `point1` to `point2`, 94.3 mm straight in −Y. A single segment IS the last segment, so it was dispatched with **blend=0 and connect=0** — no chaining, no blend arc, nothing queued. Same result: **system 4109 / 0x100D, joints clean, joint error codes NONE**, joints at failure `[-5.0, -33.8, 14.8, 100.9, 21.8, -40.2, 159.1]` (within 1 deg of the 43-segment run's). Same shape too: **4.56 s IDLE**, then **0.05 s of motion (~1 deg)**, then `RM_STOP_E`. **This also corrects the F26 dispatch reading**: the ~5 s of stillness is NOT 43 calls at 126 ms — one call shows 4.56 s of it. It is a fixed pre-motion latency | **Ruled out by this one run: the 43-segment chain, `connect=1` queueing, the blend radius, and R12's 180-deg reversals.** Also ruled out earlier: F21a under-voltage (SIM, no joint errors, nothing disabled) and a bad start pose (0.0 mm from waypoint 0 in the correct frame). **What is left is the one thing `movej` does not use and `movel` does: the TOOL FRAME.** Both `move_to_pre_start` and `move_to_start` are `movej` and both PASS in the same run; `movel` resolves its target through `L_glove_4`. Run the §7.1 attribution set with `--max-segments 1` — each run is ~20 s |
| **F28** | **The BLOCKLY program hits the same collision — our SDK layer is eliminated.** Newton, 2026-08-10: the hand-built pendant program, running the points `blockly_points.py` generated, in SIMULATION, aborts with a collision error too. A controller-native program shares none of our dispatch code, so this **rules out** `rm_movel` invocation, `connect=1` chaining, blend, the speed policy, serialization, and the Python SDK entirely. What our runs and the Blockly run still share is exactly three things: **the points, the tool frame, and the physical arm** | Every remaining hypothesis must explain a failure that happens with NO code of ours involved. That is a much smaller space, and it is where §7.1 now points |
| **F29** | **J3-J7 UNDERVOLTAGE, and the arm will not re-enable.** `recover_joints.log` 2026-08-10 16:56, left arm: `J3..J7 DISABLED err_flag=4`. Decoded against the newly transcribed table (`ERROR_CODES.md`), **joint bit `0x0004` = 欠压, undervoltage** — five joints, every one distal of J2. `clear_err ret=0` cleared the flags (`err_flag` 4 -> 0) but **`enable ret=1` was REFUSED on all five** and they stayed DISABLED; per the API2 table `ret=1` is "controller returns false — parameters wrong or arm state wrong". **Timeline: 16:08 connect CLEAN, all joints enabled -> 16:09 collision abort -> 16:39 collision abort -> Blockly collision abort -> 16:53 five joints down.** This is F21a/R10 escalated from J6/J7 to J3-J7; the J3-and-outward pattern is a shared-supply/harness signature, not five independent faults | **BLOCKING — the left arm cannot run anything until this clears.** The recovery script's own advice stands: a joint that will not re-enable means the CAUSE is still present (e-stop fully released, supply, harness) and that is RealMan-support territory. **The right arm is CLEAN and its pole is at the commanded 75 mm**, so testing can continue there — see F30 for why that is more than a workaround |
| **F30** | **Two measurements that reframe the collision, both from the recordings.** (1) **`joint_voltage` is unusable**: a flat **22.00 V on all 7 joints, zero variance across 2085 samples**, in both failing runs. It is a nominal placeholder, not a measurement — so no recording can ever show the undervoltage sag, and `power_probe.py`'s voltage premise needs revisiting before the R10 ladder is trusted. (2) **Current IS live and J4 is working hard**: peaks of **2150 mA / 2541 mA** on J4 and ~860 mA on J2, *in SIMULATION where the arm never moves* — that is pure holding torque of an extended arm. Meanwhile the tool frames carry a payload of **0.706 kg on the LEFT with centroid (0, 0, 0)** (copied from the pre-existing `Hand` frame by C14) and **0.0 kg on the RIGHT** | **Hypothesis, not yet a finding:** RealMan collision detection compares MEASURED joint current against its MODEL's prediction. A payload declared at centroid (0,0,0) puts the mass at the flange and understates the moment arm, so predicted current sits below the 2.5 A J4 actually draws — and the controller calls the difference an external collision. It fits the SIM occurrence (holding current is real), the Blockly reproduction (same physical arm), and `collision_stage=3`. **The left/right payload difference makes this free to test: 0.706 kg vs 0.0 kg on otherwise identical arms.** Run `hinge_area_right` — if it also aborts, payload is NOT the cause |
| F14 | Right arm

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
| C12 | **Segment collision verifier** (offline) | Predict the controller's motion per segment, verify against the scene | yes | 🟡 **REBUILT 2026-08-08** (plan-derived pole state, articulation filter, config contact contract; all four tasks clean at 5 mm) **but AUDIT-DOWNGRADED**: it sweeps `stage_maps(st)` — the SAVED PLAN interpolated joint-linearly — while `stage_runner` sends Cartesian `movel` through the CONFIG's waypoints. Measured separation between verified and executed tool paths: **57.1 mm** (hinge) / **46.8 mm** (toplid). And per F22 neither side is hardware truth: this is model-vs-model. It does NOT currently answer §1's question, "is the motion the controller produces collision-free" (R11) |
| C14 | **Frame alignment** | URDF `*_ConnectorLink` vs RealMan `Arm_Tip`; recreate the glove/ik frames in the controller tree | yes | ✅ **CLOSED 2026-08-08 18:58, both arms.** Trees kinematically identical; constant = **15.3 mm** (ISF model — F15). All six URDF-named frames (`R_glove_1..4`, `R_tip`, `R_index_tip` + left mirrors) written with create-or-update, **read-back match table 0.00 mm on every row**, superseded `glove1..tip` deleted, active frames restored and verified (`Arm_Tip`/`Hand`). Model check: |p| agrees 0.00–0.01 mm, full vector 0.0 mm with install pose (0, 90, 0) mirrored. Frame chain further validated end to end: commanded-pose math vs both controllers' reported poses **0.0 mm / 0.07°** |
| C10 | Chained-target execution (hardware) | `connect=1` queue depth, one arrival event vs N, blend-% reference, mid-chain failure behaviour, `rm_moves` spline | yes | ✅ **PASSED, and re-probed 2026-08-08: depth 100 accepted with no rejection.** Mode B is unblocked and a cleaning path can be queued whole (43 / 27 segments). Mid-chain invalid target returns ret=1 for THAT SEGMENT ONLY while the chain still completes — the dispatcher must check every return code. Blend effect was unmeasurable on the re-probe (F19)  **AUDIT NOTE:** blend finding is unsound (F19) |
| C11 | Rehearsal-validation loop (hardware) | SIM-execute → UDP capture → FCL; does the capture match the C12 predictor? | yes | ✅ **PASSED 2026-08-08, 7/7 + 3/3.** 26/26 targets, 10 177 frames, residual 0.003–0.004 deg / 0.01 mm on every stage. **AUDIT CORRECTION: this validates `rm_movej` ONLY.** It was cited as validating "the execution model"; the cleaning stroke executes `rm_movel`, which C11 never exercised. Its real result — movej is joint-linear to four thousandths of a degree — is solid and is the ONLY execution-grounded data we hold (F22) |
| C17 | **CANFD sync** (hardware) | Does the shipped `RM_SYNC_BACKEND=canfd` path reproduce `bench_sync` — arm streamed while the pole runs, both complete, no faults? | yes | ✅ **PASSED BOTH ARMS 2026-08-08, 7/7 each.** Sync steps: lift 5.96 s / arm 4.01 s, dispatch skew 6–17 ms (budget 50), pole outlasts the arm by +1.90 s, arrival event for every device, zero latched faults. **R1b closed — the architecture is confirmed end to end**  **AUDIT NOTE:** measured at arm v=20 %; the cleaning tasks run 50/100 % and, being serialized, never use canfd sync at all — so C17 closes the architecture question, not the cleaning-path critical path |
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
| WP3 | MTC task builder | **NOT BUILT — and "obsolete" was the wrong word (audit).** The orchestrator ALREADY runs MTC; building a second builder was redundant. MTC output is a HARD RUNTIME DEPENDENCY: `stage_runner.py:387` loads the saved plan and `_arm_target_deg` takes every named-pose joint solution from it (proven when a task with no bundled plan died before any stage ran). What the shortcut moved: IK and 7-DOF redundancy → the controller; time parameterization/jerk → lost (F20); reachability-before-motion → largely lost (a bad segment fails alone while the chain completes, C10); collision validation → offline C12, which verifies a different path (R11) |
| WP4 | Rehearsal validation | ✅ as C11 tooling (`test_rehearsal_validate.py`, capture + replay) |
| WP5 | Dispatcher (serialized, invariants, speeds, frames) | 🟡 **built; emulator-verified is weaker than it sounds** — the emulator has no IK and models `movel_chain` as a timed no-op, so it cannot reproduce anything about Cartesian feasibility. It verifies dispatch order, invariants, frames, speeds and call shapes. **No hardware run has completed** |
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
| **2 build** | Config loader, path resolver, dispatcher, probes, run recorder, Blockly point generator | ✅ dry run 236/236, emulated 20/20, four tasks end to end under emulation |
| **2 execution** | R10 debug → first clean run → 4-task rollout → acceptance (5× clean each) | 🟡 **current** |

### 7.1 Next hardware session — diagnose, then run

Everything below is prepared: mount angle read from the controller,
singularity avoidance enabled and restored, failure diagnostics that name
the reason, and a capability census at connect.

```bash
cd RM_API2/Demo/RMDemo_Python/RMDemo_DualArmConcept/src

# 0. health + capability census (read-only, nothing moves)
python3 test_dual_connect.py
RM_ARM=left python3 recover_joints.py                # report; --enable 6,7 to act

# 1. RUN 2026-08-10 16:09 — FAILED, and the recorder named the reason.
#    system 0x100D on the FIRST segment, joints clean, SIM mode (F26).
python3 stage_runner.py --task hinge_area_left --mode SIM

# 1a. THE LEFT ARM IS DOWN. J3-J7 undervoltage, enable REFUSED (F29).
#     Nothing below runs on the left arm until this clears. Confirm, do
#     not keep re-clearing:
python3 test_dual_connect.py
RM_ARM=left python3 recover_joints.py --enable 3,4,5,6,7
#     If enable is refused again: e-stop fully released? supply? harness?
#     That is RealMan support, not a state to keep clearing.
#     Also note: the LEFT pole reads 223 mm while every task path is
#     computed for 75 mm (F30). No REAL left-arm run until it is at 75.

# 1b. MEANWHILE, USE THE RIGHT ARM — it is clean, its pole is at 75 mm,
#     and its tool frames carry payload 0.0 kg against the left's
#     0.706 kg. That difference is the free test of the F30 hypothesis.
python3 stage_runner.py --task hinge_area_right --mode SIM --max-segments 1
#     -> aborts with 0x100D too?  payload is NOT the cause; it is the
#        points or the tool frame, on both arms.
#     -> passes?  the payload/centroid model is implicated, and the fix
#        is to declare the hand's real mass and centroid, not to switch
#        collision detection off.

# 1c. ATTRIBUTE THE COLLISION VERDICT. 0x100D = "arm collision
#     detected", raised in SIMULATION where nothing can be touched, so a
#     MODEL raised it. Turn the models off one at a time; all are
#     restored on exit. Stop at the first run that PASSES — that names
#     the check that owns the abort.
TASK=hinge_area_right       # the healthy arm; hinge_area_left once J3-J7 are back
RM_ENDEFF_COLLISION=0 python3 stage_runner.py --task $TASK --mode SIM --max-segments 1
#     -> END-EFFECTOR self-collision, evaluated against the active tool
#        frame (L_glove_4, far off the flange). START HERE: it is the one
#        check our offline sweep does NOT model, since rm_algo's
#        self-collision takes joint angles and knows no tool.
RM_SELF_COLLISION=0 python3 stage_runner.py --task $TASK --mode SIM --max-segments 1
#     -> the arm's own self-collision model. Offline says self=0 on all
#        976 movel samples, so a PASS here would mean our model and the
#        controller's disagree — which is a finding in itself.
RM_COLLISION_STAGE=0 python3 stage_runner.py --task $TASK --mode SIM --max-segments 1
#     -> the 0..8 detection sensitivity (as-found 3).
RM_ENDEFF_COLLISION=0 RM_SELF_COLLISION=0 RM_COLLISION_STAGE=0 \
    python3 stage_runner.py --task $TASK --mode SIM --max-segments 1
#     -> all three off. If this STILL fails, the abort is not the
#        collision system and the chain itself is next (--max-segments).

# 1d. TOOL-FRAME attribution — movej passes and movel fails in the SAME
#     run, and the tool frame is the only thing movel uses that movej
#     does not (F27). SIM, single segment, nothing moves for real.
python3 test_frame_alignment.py --mode REAL --poses
#     -> read back the glove frame as the controller holds it. C14 wrote it and
#        verified 0.00 mm, but that was 2026-08-08 and a payload or a
#        re-write since would not have been noticed.
#     Then, in the Web GUI with the arm idle: select tool frame
#     L_glove_4 and jog -Y by 90 mm. If the GUI ALSO reports a collision,
#     the frame/payload is the defect and no amount of SDK work fixes it.

# 1e. only if the above do not explain it — chain shape (R12 still
#     unmeasured, but F27 shows it is not what 0x100D reports)
python3 stage_runner.py --task $TASK --mode SIM --blend 0

# 2. the under-voltage ladder (only once step 1 passes)
RM_ARM=left python3 power_probe.py --seconds 10 --label idle
RM_ARM=left python3 power_probe.py --seconds 90 --label derate04 &
RM_SPEED_DERATE=0.4 python3 stage_runner.py --task hinge_area_left --mode REAL
#    step 0.4 -> 0.7 -> 1.0; the clean derate is the operating speed

# 3. rollout: the other three tasks, then WP6 acceptance (5x clean each)
```

**If step 1 fails, do not proceed to step 2** — a chain that cannot
complete in simulation will not be fixed by changing speed.

Every run above writes `runs/<runid>/{stream.csv,run.json}` — 100 Hz arm +
pole + the controller's own TCP pose, stage marks on the same `t_mono`
clock. That directory is the evidence for whatever happens next, and it is
committed, not ignored.

### 7.2 Blockly track — runs in PARALLEL, independent of §7.1

The 1:1 Blockly comparison Newton asked for (R11 cross-check): the same
cleaning path, entered by hand on the pendant, so a controller-native
program and our dispatcher can be compared on identical geometry.

```bash
cd RM_API2/Demo/RMDemo_Python/RMDemo_DualArmConcept/src

# offline — regenerate the hand-off files, no hardware
python3 blockly_points.py --task hinge_area_left --out ../blockly/hinge_area_left.json
python3 blockly_points.py --task toplid_right   --out ../blockly/toplid_right.json

# hardware, nothing moves — the tool frames must EXIST before the points
RM_ARM=left  python3 test_frame_alignment.py --mode REAL --create-frames
RM_ARM=right python3 test_frame_alignment.py --mode REAL --create-frames

# push into the controller's global-waypoint store
python3 blockly_points.py --task hinge_area_left --push
python3 blockly_points.py --task toplid_right   --push
```

The hand-off JSONs are self-describing (setup block, per-point pose +
joints + FK residual, numbered move sequence), so whoever builds the
program needs nothing else. Accuracy is verified, not asserted: FK of each
point's joint solution vs the point's own pose, worst residual **0.012 mm**
(hinge_area_left, 23 points) and **0.007 mm** (toplid_right, 15 points).

Two things the program must get right, both easy to miss:
**lift to 75 mm before any arm motion**, and `tool_frame` =
`L_glove_4` / `R_glove_2` — *not* ZIGZAG01's `Hand`.

---

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
| **R10** | **J6/J7 Under-Voltage (F21a)** — free-space motion, recurred at rest after a power cycle | Blocks acceptance runs. **Not a Phase 2 gate** (Newton): ZIGZAG01 executes this motion class on this hardware. NOTE the supporting argument is weaker than first written — ZIGZAG01 uses HAND-TAUGHT points in one arm configuration at `tool_frame=Hand`, whereas we generate computed Cartesian targets and let the controller re-solve each. It proves MOVEL chains work on this arm; it does not prove OUR points do | Ladder in §7.1: recover joints → idle probe (decides hardware-vs-load in one measurement) → derate ladder with `power_probe` CSVs (which joint's CURRENT spikes at the dip: J2/J4 = supply margin, J6/J7 = wrist demand) → operate at the clean derate meanwhile |
| **R11** | **C12 verifies a path we do not execute, and neither side is hardware truth** (F22). Verified: saved-plan waypoints, joint-linear. Executed: config waypoints, Cartesian movel, controller IK. Separation **57.1 mm** / **46.8 mm** | The clearance map does not currently support the safety claim in §1. Self-collision detection (now ON) is the only online backstop | **Newton chose (a) 2026-08-10 — "this is what will be executed, so we need to check on this motion".** IMPLEMENTED: `movel_joint_timeline()` (cleaning_path.py) interpolates position linearly and orientation by SLERP, re-solving seeded IK per sample, and `run_hinge_verify.py` now drives its verdict from that sweep rather than from the saved plan. `--movel-step` sets the resolution (default 10 mm). **Not yet run on all four tasks:** hinge_area_left is clean (0/430 IK failures); hinge_area_right 55/430, toplid_right 31/270, toplid_left 20/270 need reading before those tasks execute. Option (b) remains the fallback if the failures are real rather than seeding artifacts |
| **R12** | **Blend radius `r=10` is unjustified** — C10's two probes disagree (0.64 s vs 0.01 s benefit) | Unknown effect on corner geometry during contact | Measure it, or set r=0 until measured |
| **R13** | **Nothing pins the 7-DOF arm angle ALONG the cleaning stroke.** `move_to_start` pins the entry; the 43 movel targets are then re-solved independently by the controller | A branch change mid-stroke would move the elbow through space no offline check has seen. NOTE: the claim that this DOES happen was RETRACTED (F24) — it is an open question, not an established failure | Singularity avoidance (now ON, F23) is the controller's own mitigation; option (b) of R11 removes the ambiguity entirely |

**Answered:** hinge pose is **fixture-taught** (no perception) · the glove is
worn on the hand, so **hand pose = glove state**, cleaning with the **back of
the hand** · **no force control** for now, but force feedback is available
from the **wrist 6-axis sensor** (`tool_zero_force_data`), not the hand.

---

## 9. Next actions

**Hardware — §7.1 has the commands, in order:**
- [ ] Connect test: capability census + joint health
- [ ] Recover J6/J7 if still disabled
- [ ] **`hinge_area_left` in SIM** — the chain failure without power in the
      equation; singularity avoidance now ON, diagnostics now name the cause
- [ ] Only then: the under-voltage probe ladder (idle → 0.4 → 0.7 → 1.0)
- [ ] Rollout to all four tasks, then WP6 acceptance (5× clean each)

**In parallel (§7.2), no dependency on the above:**
- [ ] Write the frames, push the Blockly points, build the program
- [ ] Read the R11(a) movel sweep on the three tasks that showed IK
      failures — before those tasks are executed, not after

**Decision needed (R11) — the verification gap:**
- [ ] (a) extend C12 to sweep the actual movel program, **or**
      (b) execute the stroke as chained `movej` on MoveIt's joint solution.
      (b) is already modelled and validated by C11/C12 and pins the arm
      angle along the stroke; (a) keeps Cartesian straightness. This is an
      architecture choice, not a patch.

**Offline, smaller:**
- [ ] Measure the blend radius or set r=0 (R12)
- [ ] Narrow `contact_link_groups` per ik_frame when the contract is revised
      — the verdict reads the file, so it sharpens automatically
- [ ] R9 / F14: right arm 1.8× slower than left, still unexplained

**External:**
- [x] F9 vendor-confirmed (no ETA) — sync stays canfd-only
- [ ] RealMan: under-voltage CSVs (R10), `expand` field, `rm_clear_system_err`
      gap, fix timeline
