# Butterfli — Phase Plan: ROS 2 planning + controller-planned execution

*Living document. Newton owns the schedule and the orchestrator; this file is
the shared contract for what "feasible" means and what closes each gate.*
*Created 2026-08-07. Update the STATUS column as gates close.*

---

## 0. The architecture in one paragraph

**ROS 2 decides WHERE to move; the RealMan controller decides HOW to get
there.** MoveIt/MTC are a **task planner and a validator — not a path source
to be reproduced**. They own the scene, URDF/TF, IK, reachability and the
collision verdict; they do *not* hand over a dense trajectory. Execution
sends the controller a **short list of sparse targets**: joint targets for
named poses (`rest`, `ready`, stroke entry) and Cartesian targets for the
cleaning strokes. Peripherals (pole, hand+glove) are sequenced around arm
motion under a strict ordering rule.

The safety question is therefore **not** "does the executed path match
MoveIt's path?" — nothing is trying to match it. It is "**is the motion the
controller actually produces collision-free in this scene?**", answered on
two independent layers:

```
  MTC task ──▶ MoveIt: reachability, IK,  ──▶ sparse targets ──▶ controller-planned
  (stages)     goal-state collision, order     movej / movel      execution
                        │                            │                  │
                        │                   OFFLINE verify:      ONLINE backstop:
                        │                   predict the           self-collision
                        │                   controller's own      detection
                        │                   motion (rm_algo),     (free, always on)
                        │                   FCL vs PRIMITIVES     [fence NOT used]
                        │                            │
                        └──── planning scene ────────┘
                              URDF · TF · SRDF
                     (C11 rehearsal capture validates the predictor once)
```

**Why this shape:** it keeps MoveIt's collision reasoning and MTC's task
structure while execution stays on the vendor's own motion profiles — which
are deterministic and repeatable, so a segment verified once stays valid.
MoveIt's *timing* is discarded; the controller re-times everything. That is
acceptable for cleaning: geometry and coverage matter, velocity profiles do
not.

---

## 1. Existence proof: ZIGZAG01

Newton's hand-written Web-GUI program (`ZIGZAG01`, 2026-08-07) is
structurally the same thing this architecture generates automatically:

| ZIGZAG01 | The architecture |
|---|---|
| 21 taught waypoints ZIG0…ZIG20 | 20 hinge cleaning points from `cleaning_path_gen` |
| `MOVEJ` then `MOVEL`, World frame, blending radius 10 | `movej` to pin configuration, `movel` for strokes (§4.1) |
| lift commanded **non-blocking first**, then the arm moves | the ordering rule (§4.3) |
| runs, pole and arm move together | the behaviour Phase 2 reproduces from a task spec |

**So the target execution model already runs on this hardware.** Phase 2 does
not invent it — it generates it from MTC instead of teaching it by hand. This
materially de-risks the schedule and should be treated as the reference
behaviour whenever a design question comes up.

---

## 2. What the bench tests have established

Findings that now constrain the design (all hardware-observed unless noted):

| # | Finding | Consequence for Phase 2 |
|---|---|---|
| F1 | **A lift command issued during an in-flight planned arm move ABORTS it** — arm stops short, no arrival event, and it **latches joint errors** that block all later motion until cleared | Hard dispatch invariant (§4.3). Must be enforced in code, not by convention |
| F2 | Sim mode **executes the arm and is fully observable** (UDP push, arrival events, TCP polls) | The rehearsal-validation loop is viable — this is what makes controller-planned execution collision-safe |
| F3 | Sim mode does **not** execute lift or hand | Rehearsals cover arm geometry only; peripherals need separate reasoning |
| F4 | Hand: device-2 arrival events never fire; blocking `rm_set_hand_angle` fails (−4) during arm motion | Hand completion is duration-based (`acked_angle`); modbus RTU is the true-feedback path |
| F5 | Planned `rm_movej` + concurrent hand works (C6, 7/7 ×3) | Tool/hand actuation during arm motion is safe |
| F6 | Lift is a profiled positioner; **re-targeting mid-motion restarts the profile** | One target per pole goal; no lookahead re-targeting |
| F7 | Per-arm lift gearing differs (left 1:1 V1.7.4, right 2:3 V1.7.1) | Firmware alignment is a Phase 2 prerequisite (§7 R4) |
| F8 | canfd `expand` cannot drive the lift (butterfli_hw `bench_lift_expand`) | No single-command 8-axis sync; pole is always a separate command |

---

## 3. PHASE 1 GATE CHECKLIST — the approval table for Phase 2

Legend: ✅ passed on hardware · 🟡 partially passed · ⛔ blocked/not run ·
⬜ not started. **Phase 2 starts when every BLOCKING row is ✅.**

### 3.1 Execution-layer gates (C1–C9) — RM_API2 command semantics

| ID | Test | What it proves | Blocking? | STATUS | Evidence / what's left |
|---|---|---|---|---|---|
| C1 | `test_dual_connect` | Both arms reachable, state readable, handles distinct | yes | ✅ | 9/9, 2026-08-06 |
| C5 | `test_sim_motion_visibility` | Simulated motion is observable → **rehearsal loop viable** | yes | ✅ | 5/5 + YES verdict on all three channels |
| C7 | `test_hand_only` | Hand controllable with measured feedback (both paths) | yes | ✅ | 8/8 both arms |
| C6 | `test_single_arm_planned` | Planned moves + pole homing + **concurrent hand**; Cartesian accuracy | yes | ✅ | 7/7 ×3 runs; `dx=+0.196 m` vs +0.20 target |
| C8 | `test_pole_only` | Pole acceptance probe + fault diagnosis + recovery | yes | ⛔ | **Never run on hardware.** ~5 min |
| C9 | `test_single_arm_locked` | Full concept sequence incl. **arm+pole sync** on one arm | yes | 🟡 | Reproduced the freeze (diagnostic success); passed arm+hand. **Sync untested since the lift-first fix** |
| C2 | `test_dual_locked` | Locked dual-arm, barrier per step, dispatch skew | yes | 🟡 | Arm-only 6/6. Froze with poles; **retest with fix** |
| C3 | `test_dual_chained` | Chained advance-on-finish ordering invariant | yes | 🟡 | Arm-only 4P/1F — **1.6 s gate-latency anomaly unexplained** (§7 R5) |
| C4 | `test_dual_free` | Free-running mode, independent completion | no | 🟡 | Arm-only 4/4; poles untested |

**Honest read:** the arm-only and peripheral-only halves are solid. The
**arm+pole concurrent (sync) case has never passed on hardware** — the fix is
implemented and proven in the emulator but is unverified on the arms. That is
the single most important open gate.

### 3.2 Bridge gates (C10–C14) — RECOMMENDED ADDITION

C1–C9 validate *command semantics*. None of them sends a target that came
from MoveIt, and none exercises the safety layers this architecture leans on.
These five close that gap; C12 needs no hardware.

| ID | Test | Question it answers | Blocking? | STATUS | Where |
|---|---|---|---|---|---|
| C12 | **Segment collision verifier** (offline) | For a commanded segment (`movej` joint target / `movel` Cartesian target), predict the controller's own motion with `rm_algo` and FCL-sweep it against the scene + `rm_algo_safety_robot_self_collision_detection`. Is the hinge path clean? Where is it tight? | yes | ⬜ | Emulator + real `rm_algo`; **today, no hardware** |
| C14 | **Frame alignment** (hardware) | Does a pose expressed in URDF/TF land where the controller thinks it does? Controller work frame `World` and tool frame (ZIGZAG01 used `Hand`) vs URDF `butterfli_ref_frame` / `R_glove_frame_4` | yes | ⬜ | New `test_frame_alignment.py`; ~15 min. **Silent offsets here corrupt every cleaning point** |
| C13 | Fence / online-safety characterization (hardware) | What the electronic fence bounds (TCP vs arm body), reject-vs-stop, planned-move coverage | **no** — *dropped from blocking 2026-08-07 (Newton): collision safety rests on scene PRIMITIVES + offline FCL, not the fence* | ⬜ | Optional. Keep `rm_set_self_collision_enable` ON as a free backstop |
| C10 | **Chained-target execution** (hardware) | Mechanism confirmed in the docs (§4.1a); C10 measures the numbers: **max queue depth**, one arrival event vs N, what the blend **percentage** is relative to, whether a mid-chain planning failure rejects the whole chain, and `rm_moves` spline as an alternative | yes | ⬜ | New `test_waypoint_chain.py`; ~15 min. **Gates Mode B** |
| C11 | Rehearsal-validation loop (hardware) | SIM-execute → UDP capture → FCL check; and does the capture match the C12 predictor? | yes | ⬜ | New `test_rehearsal_validate.py`; ~20 min |

### 3.3 Approval rule

> **Phase 2 is approved when C1, C5, C6, C7, C8, C9, C2, C3 and C10, C11,
> C12, C14 are ✅.** C4 (free mode) and C13 (fence) may stay 🟡/⬜ — the
> cleaning task needs neither. Collision safety rests on scene **primitives +
> offline FCL** (C12), with controller self-collision detection as a free
> backstop.

---

## 4. The bridge design (what Phase 2 builds)

### 4.1 Target selection — which command for which segment

No dense path is reproduced, so the choice is per *segment*, by what that
segment functionally needs:

| Segment | Command | Why |
|---|---|---|
| Named poses (`rest`, `ready`, transit) | `rm_movej` with **MoveIt's joint values** | Unambiguous; MoveIt has already validated the goal state |
| **Stroke entry** (approach to the first cleaning point) | `rm_movej` with MoveIt's joint solution for that pose | **Pins the arm configuration** — see the rule below |
| **Cleaning strokes** (point→point on the surface) | `rm_movel` | Cartesian straightness along the surface is the functional requirement; dense `movel` would stutter, sparse is exactly what the 20-point path is |

### 4.1a `trajectory_connect` — CONFIRMED (SDK docs, 2026-08-07)

Newton's proposal is exactly what the API provides. From the `rm_movej` /
`rm_movel` / `rm_moves` docstrings:

> `connect` (轨迹连接标志)
> - `0`：立即规划并执行轨迹，不与后续轨迹连接 — *plan and execute now, not
>   connected to what follows*
> - `1`：将当前轨迹与下一条轨迹一起规划，但不立即执行。**阻塞模式下，即使发送成功也会立即返回** —
>   *plan this trajectory together with the next, do not execute yet; in
>   blocking mode it returns immediately even on success*

So a reduced path is queued and then fired as **one** connected trajectory:

```python
for p in path[:-1]:
    rm_movel(p, v, r, connect=1, block=0)   # queued, planned with the next
rm_movel(path[-1], v, r, connect=0, block=0) # connect=0 plans + EXECUTES the chain
# then wait for ONE arrival event
```

Three consequences worth pinning down now:

1. **`connect=1` never blocks and never completes.** Completion only exists
   for the closing `connect=0`. A dispatcher that waits on a `connect=1`
   segment waits forever.
2. **Our `ArrivalMonitor` already implements this** — it treats an event with
   `trajectory_connect == 1` as *not* a completion and only latches on
   `trajectory_connect == 0` ([`dual_arm_common.py:483`](src/dual_arm_common.py#L483)).
   Chain semantics are already dry-run tested.
3. **The blend radius `r` is a PERCENTAGE (0–100), not millimetres**
   (`交融半径百分比系数`). ZIGZAG01's "10" is 10 %, not 10 mm — §4.2's rule
   must therefore be expressed as a fraction of segment length, not an
   absolute distance. What the percentage is *of* is not documented → C10.

Still unknown, and the reason C10 exists: **how many segments the controller
will queue**, whether a mid-chain planning failure rejects the whole chain,
and what the blend percentage is relative to. `rm_moves` (spline) is a third
option — it needs ≥ 3 consecutive `connect=1` points or it degrades to a
straight line, and could give smoother surface following than chained
`movel`. Worth one extra minute in C10.

### 4.1b Two path modes — pick per segment, not per project

| | **Mode A — sparse targets** | **Mode B — reduced MoveIt path** |
|---|---|---|
| Source | named poses + the 20 cleaning points | MoveIt path (geometry only, timing discarded), simplified to few points |
| Command | `movej` / `movel`, executed one at a time | chained `movel`/`movej` with `connect=1`, closed by `connect=0` |
| Use when | open space; the direct move is obviously safe | **constrained geometry — arm reaching inside the commode**, around the lid, any place where the direct move would cut a corner through the fixture |
| Cost | trivial | path simplification + more verification surface |
| Verified by | C12 segment verifier | C12 on every sub-segment of the chain |

**Selection rule:** start every segment in Mode A; escalate to Mode B when the
C12 verifier reports the direct move is not clean, or clearance falls below
the margin. That way complexity is paid only where geometry demands it, and
the verifier decides — not a guess.

Taking MoveIt's **path** and discarding its timing is right: the controller
re-times everything anyway, so any time-parameterization effort is wasted
work. Read the joint positions out of the plan and ignore `time_from_start`.

**The one guard on `rm_movel`: pin the configuration before the stroke.**
`rm_movel` solves IK seeded from the *current* configuration, so a stroke is
deterministic and repeatable — but only relative to where it started. If the
arm enters the same stroke in a different configuration on a different run,
the verified result does not transfer. Entering every stroke via a `movej` to
MoveIt's joint solution removes that freedom, and *that* is what makes
"verify once offline, trust thereafter" sound. Branch flips within a
continuous seeded stroke are unlikely; branch flips **between runs** are the
real risk, and pinning kills them.

C12 verifies the resulting motion either way; C11 confirms the predictor
matches reality once on hardware.

### 4.2 Blend radius must be bounded by clearance

Blending cuts corners — the executed path bows **inside** the commanded
corner. The bow must not exceed the local collision margin:

```
r_i  ≤  k · min_clearance(waypoint_i)      k ≈ 0.5 as a starting margin
```

Clearance comes from the planning scene MoveIt already has. A fixed `r=10`
(the ZIGZAG01 value) is fine in open space and wrong near the fixture.

### 4.3 Dispatch ordering invariant (from F1)

> **Never issue a pole command while that arm's planned trajectory is in
> flight.** Per arm, per segment: pole first (non-blocking), then the arm
> move; or pole strictly between arm segments.

This is a safety invariant, not a style preference — violating it aborts the
trajectory and latches joint errors. It must be enforced with a runtime
assertion in the dispatcher (reject the command, log loudly) so it cannot be
reintroduced by a future refactor. Hand/glove actuation is exempt (F5).

### 4.4 Rehearsal validation (the collision-safety closure)

Because the controller re-plans between waypoints, MoveIt's collision proof
does not automatically transfer to the executed path. C5 proved the fix:

1. put the arm in SIM mode, execute the waypoint chain;
2. capture the actual joint trajectory over UDP push;
3. FCL-check that capture against the same planning scene;
4. only then execute for real.

This runs per task, not per cycle. It is also the natural artifact to store
alongside each cleaning plan (Phase 0 of `STUDIO_LINK_RESEARCH.md` — the plan
recorder — feeds the studio visualizer from the same capture).

### 4.5 Interfaces (proposed — Newton owns the orchestrator internals)

| Component | Owner | Responsibility |
|---|---|---|
| `cleaning_path_gen` | existing | fixture → cleaning points (already produces the hinge task) |
| MTC task builder | Phase 2 | cleaning points → MTC stages → validated sparse targets (reachability, IK, goal-state collision, ordering) |
| **Segment verifier** | Phase 2 | predict the controller's own motion per segment (`rm_algo`) → FCL sweep + self-collision → verdict |
| **Rehearsal validator** | Phase 2 | SIM execute + UDP capture + FCL; also calibrates the verifier against reality |
| **RM dispatcher** | Newton | `rm_movej`/`rm_movel` targets, peripherals, ordering invariant, arrival events |

The segment verifier is a pure function over (start config, command) — no
hardware, no ROS runtime, so it belongs in CI and runs on every path change.

**Predictor caveat:** the offline `rm_algo` is v1.6.0 while the controllers
run 1.5.5 (right) / 1.5.9 (left). IK solutions may differ slightly. C11 is
what settles it — capture the real motion once and measure the predictor's
error, then carry that as a margin in the FCL check.

---

## 5. PHASE 2 — hinge-area cleaning

### 5.1 The task already exists

`cleaning_tasks/config/commode_cleaning/commode_c/hinge_area_right_cleaning_points.yaml`
is fully specified: **20 cleaning points** (8 lid-sides-back + 12 hinge area),
expressed as start-pose-origin translation deltas + Euler RPY deltas in
`butterfli_ref_frame`, `ik_frame: R_glove_frame_4`, articulation "lid closed,
body static", surface "concave_interior + near_planar". Left-side and
lid/seat/body variants exist too.

**Scope Phase 2 to `hinge_area_right`, right arm only, sequential execution.**
20 points is the same order as ZIGZAG01's 21 — a like-for-like target.

### 5.2 Integration point

The file sets `cleaning_path_mode: ruckig_pro_only` — the current pipeline
time-parameterizes and streams. Phase 2 adds a **new mode**
(`controller_planned`) rather than replacing the existing one, so both paths
stay runnable and directly comparable on the same task. That comparison is
the strongest possible evidence for or against the architecture.

### 5.3 Work packages

| WP | Deliverable | Depends on | Hardware? | Est. |
|---|---|---|---|---|
| WP1 | Segment verifier: `rm_algo` motion predictor + FCL sweep + self-collision check | — | no | 0.5 d |
| WP2 | C12 report: run the real 20-point hinge path through WP1; clearance map, tight spots | WP1 | no | 0.5 d |
| WP3 | MTC task builder for `hinge_area_right` | — | no | 1 d |
| WP4 | Rehearsal validator (SIM + UDP capture + FCL) | C5 | yes (C11) | 1 d |
| WP5 | Dispatcher integration + ordering assertion + wrist-force logging | C10, C14 | yes | 1 d |
| WP6 | End-to-end hinge run + acceptance | WP1–WP5 | yes | 1 d |

### 5.4 Definition of done (acceptance test)

> The `hinge_area_right` task runs end-to-end from an MTC plan — reduced,
> rehearsal-validated, executed with chained planned moves and concurrent
> peripherals — **5 consecutive times** with: zero freezes, zero latched
> faults, every waypoint reached within tolerance, and measured deviation
> from the MoveIt path within the clearance budget.

"Implemented" is not done; 5 clean consecutive runs is done.

---

## 6. Schedule

### 6.1 Correction to the original plan

Phase 1 **cannot close today**: four blocking gates (C8, C9-sync, C2, C3) plus
C10/C11 need the hardware, which is not available. Splitting it keeps today
productive and makes the hardware session short and scripted.

| Phase | Content | Needs hardware | When |
|---|---|---|---|
| **1A** | Architecture + this plan + WP1 + WP2 (C12) | no | **today** |
| **1B** | Hardware gate session (C8, C9, C2, C3, C10, C11) | **yes** | first lab access — **~2 h, scripted** |
| **2** | WP3–WP6, hinge task | partly | 5 working days after 1B |

**End of next week is achievable if and only if 1B happens by Monday/Tuesday.**
Each day 1B slips, Phase 2's acceptance slips with it — the offline WPs
(WP1–WP3) can absorb about two days of that before becoming blocked.

### 6.2 The 1B hardware session script (in order, ~2 h)

```bash
# 0. recover from the last frozen run (joint errors are almost certainly latched)
RM_ARM=left python3 test_pole_only.py --mode REAL --clear-errors          # C8
# 1. the fix, one arm, no hands — the critical gate
RM_ARM=left python3 test_single_arm_locked.py --mode REAL --no-hands      # C9-sync
RM_ARM=right python3 test_single_arm_locked.py --mode REAL --no-hands
# 2. full sequence, one arm
RM_ARM=left python3 test_single_arm_locked.py --mode REAL                 # C9 full
# 3. dual-arm, full sequence
python3 test_dual_locked.py --mode REAL                                   # C2
python3 test_dual_chained.py --mode REAL                                  # C3
# 4. bridge gates (new tests, to be written in 1A/early 2)
RM_ARM=right python3 test_frame_alignment.py --mode REAL                  # C14
python3 test_waypoint_chain.py --mode REAL                     # C10 — gates Mode B
python3 test_rehearsal_validate.py                                        # C11
```

Stop at the first red gate and diagnose — later gates assume earlier ones.

---

## 7. Risks and open questions

| # | Risk / question | Impact | Mitigation / needed decision |
|---|---|---|---|
| R1 | **Sync fix unverified on hardware** | Blocks everything | First item in 1B. If it fails, fall back to strictly sequential pole/arm (no concurrency) — costs cycle time, not feasibility |
| R2 | **The controller's own motion between sparse targets collides** (it is not MoveIt's path and nothing constrains it to be) | Architecture invalid as specified | C12 answers it offline **today** for the real hinge path. Fallback ladder: add intermediate targets at the tight spots → constrain the stroke entry configuration → canfd streaming for that segment only (butterfli_hw already proves streaming works) |
| R2b | ~~Fence does not bound the arm body~~ — **retired 2026-08-07**: the fence is not in the safety case. Collision safety = scene primitives + offline FCL (C12) | — | Keep `rm_set_self_collision_enable` ON; free and independent |
| R2c | **Frame misalignment** between URDF/TF and the controller's work/tool frames | Every cleaning point silently offset — worst failure mode, looks like it works | C14, before any cleaning path runs. ZIGZAG01 shows `World` + `Hand` frames exist on the controller; whether `Hand` equals `R_glove_frame_4` is unverified |
| R3 | `connect=1` chaining behaves differently than documented (e.g. one arrival event vs N, or no true blending) | Dispatcher redesign | C10 measures it directly; ZIGZAG01 suggests it works |
| R4 | **Firmware mismatch** (right V1.7.1 vs left V1.7.4) | Per-arm behaviour differences, different lift gearing | Upgrade right arm before Phase 2 hardening; scope Phase 2 to the right arm means its firmware matters most |
| R5 | **C3's 1.6 s follower-gate latency** unexplained | Unknown timing behaviour in chained mode | One dedicated look during 1B; chained mode is the likely Phase 2 execution mode, so this matters |
| R6 | Dual-arm **concurrent** collision validity | Locked/free modes could collide despite per-arm plans | Phase 2 scoped to one arm. `dual_arm_plan_collision.py` exists for when it isn't |
| ~~Q1~~ | **ANSWERED 2026-08-07: fixture-taught.** No perception in the loop | scope held | No perception work package. Makes C14 (frame alignment) critical: a taught fixture means every cleaning point is only as good as the frame agreement |
| ~~Q2~~ | **ANSWERED: the glove is worn on the dexterous hand — hand pose determines glove state.** Cleaning uses the **BACK of the hand** | — | The cleaning hand pose is part of the path spec: command it before the stroke and hold it (proven safe during arm motion, F5). Tool frame must sit on the back-of-hand contact patch (`R_glove_frame_4`) — see C14 |
| ~~Q3~~ | **ANSWERED: no force control for now, but force feedback wanted.** Newton's concern: the back of the hand cannot sense force | — | **It can — from the arm, not the hand.** The RM75-6**FB** has a 6-axis wrist F/T sensor; `rm_get_force_data` returns `tool_zero_force_data` (external force in the TOOL frame, gravity/tool compensated) — exactly right for back-of-hand contact, independent of which surface touches. Available now, read-only, no force control needed |

---

## 8. Today's checklist (1A — no hardware required)

- [x] Review + correct this plan (Newton, 2026-08-07 — architecture reframed:
      MoveIt validates, it does not supply a path to reproduce)
- [x] Answer Q1–Q3 (fixture-taught · glove = hand pose, back-of-hand contact ·
      no force control, force feedback via the wrist 6-axis sensor)
- [ ] Verify the ordering invariant assertion in the dispatcher (Newton, today)
- [ ] WP1: segment verifier (`rm_algo` predictor + FCL + self-collision)
- [ ] WP2: run C12 on the real 20-point hinge path → clearance map
- [ ] Draft `test_frame_alignment.py` (C14) and `test_waypoint_chain.py`
      (C10) so 1B is pure execution
- [ ] Confirm the 1B session slot
