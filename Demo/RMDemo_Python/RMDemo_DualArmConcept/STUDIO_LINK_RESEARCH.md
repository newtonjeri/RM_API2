# Research: Linking the RM Emulator to Cleaning Path Studio

*Feasibility study, 2026-08-06. Research only — no code has been written.
Goal: use `cleaning_path_studio` (in `butterfli_ws/src/butterfli-ai-ros-jazzy/
cleaning_path_gen`) as the 3D visualizer / simulator front-end for
`src/rm_emulator.py` (and, later, the real arms).*

## Verdict

**Highly feasible; the architecture is unusually favorable.** The studio is a
zero-dependency local HTTP server (stdlib `ThreadingHTTPServer` backend,
vanilla three.js frontend) that renders the full butterfli URDF, and every
robot pose it ever displays flows through **one function**:
`applyRobotPose({link: 4x4})` at `web_editor/editor.js:6164` — already fed by
four independent producers (IK preview, sequence animation, plan replay,
dual-arm collision playback). Its FK entry point
(`butterfli_workspace.urdf_kinematics.UrdfModel.link_world_transforms`)
accepts a plain `{joint_name: radians}` dict and resolves hand mimics itself.

The whole integration therefore reduces to: **produce a URDF-named joint map
from emulator state, and get it to the server.** The studio handles
world-frame placement, mirrored arm mounting, hand mimic chains, and scene
context — which neatly sidesteps the emulator's one display weakness (its
`rm_algo` FK uses default mounting in the arm-base frame, not the butterfli
world frame).

**The single structural gap:** the studio has **no live-update channel** —
no WebSocket, no SSE, no polling, no file watching. All animation is
precomputed frames played client-side. That gap defines the work.

## Key facts (with anchors, from the survey)

| Studio fact | Where |
|---|---|
| Entry point `cleaning_path_studio` → `server.py:main()`, port 8765, loopback | `pyproject.toml:42`, `server.py:3051-3081` |
| Universal pose sink `applyRobotPose(poseMap)` | `editor.js:6164-6174` |
| Robot geometry endpoint (URDF meshes, link-local frames) | `server.py:1263-1321`, `/api/robot-geometry` |
| FK from joint map + mimic resolution | `butterfli_workspace/urdf_kinematics.py:237, :217` |
| Route tables = plain dicts (one-line endpoint additions) | `server.py:2919-2957` |
| Plan-JSON replay (whole-robot FK at dt=0.1 s) | `server.py:2467-2551` → `dual_arm_plan_collision.py:231-327` |
| Client playback loops (`setInterval` / rAF) to copy for a Live mode | `editor.js:6551-6567`, `:7180-7229` |
| Optional-import idiom for sibling packages (graceful degrade) | `server.py:730-732` etc. |
| Plan roots the replay reads | `cleaning_tasks/plans/`, `Resource/plans/` (`server.py:2088`) |
| No auth off-loopback; `/api/save` path-whitelisted | `server.py:3058-3063`, `:1433-1452` |

## The unit/name adapter (fully determined — a table, not research)

- **Arms**: emulator `joints_deg[0..6]` (degrees) → `L_joint1..7` /
  `R_joint1..7` in **radians**. Plain conversion, no sign flips (verified
  against butterfli_hw conversions).
- **Lifts**: per-side gearing → `L_/R_sliding_plate_joint` in **metres**:
  left (1:1, V1.7.4) `m = hw/1000`; right (2:3, V1.7.1) `m = hw*1.5/1000`.
  Both inside the URDF 0–0.3 m limit (left full 290 → 0.29).
- **Hands**: SDK order `[little, ring, middle, index, thumb_flex, thumb_rot]`,
  1000 = open → `ILH_left_/IRH_right_{*_1, thumb_2, thumb_1}_joint` radians via
  `rad = (1 - hw/1000) * max_rad[channel]` — already implemented in Python at
  `butterfli_hw/scripts/hand_units.py`. Mimics resolved studio-side.
- Unmodeled joints (UGV, bin, glove arm) simply stay at home values — the
  studio's joint layering handles absent joints.

## Recommended phases

### Phase 0 — works today, ZERO studio changes: record emulated runs as plan JSON
The studio's replay mode consumes the orchestrator plan-JSON schema
(`sub_trajectories[].{stage_name, joint_names, waypoints[{time_from_start,
positions}]}`) from `Resource/plans/` / `cleaning_tasks/plans/`. Add a small
**recorder to the emulator harness** (sample emulator state at fixed dt during
a run; emit that schema with URDF joint names). Every emulated C2/C6 run
becomes replayable in the studio — full robot, fixture scene, hinge
articulation, and the dual-arm collision playback (red-flash pairs) applied to
*emulated* trajectories. Highest value-to-effort; doubles as the archive
format for emulated rehearsals.

### Phase 1 — Live mode (~1 day, 4 touch points)
1. `POST /api/live-joints` — store latest joint map + timestamp in memory
   (one entry in `ROUTES_POST`).
2. `GET /api/live-pose` — FK the latest map via the existing `_ik_context` /
   `UrdfModel` path; return `{link: 4x4}`.
3. UI "Live" toggle — `setInterval` fetch → `applyRobotPose` (same loop shape
   as `schPlaySegment`).
4. Emulator-side bridge thread (opt-in, e.g. `RM_EMU_VIZ=http://127.0.0.1:8765`)
   pushing the converted joint map at 10–20 Hz. Push-from-source keeps the
   studio generic: it gains a "live joint feed" with no emulator knowledge.

Perf is a non-issue (numpy FK sub-ms; 10–20 Hz loopback polling; `_serialized`
lock idiom already available).

### Phase 2 — polish
SSE (`EventSource`) instead of polling (stdlib-compatible, unlike WebSockets
which would break the zero-dependency ethos); arrival-event markers; a
record-to-plan button capturing a live session into Phase-0 JSON.

### Phase 3 — strategic payoff
The live channel is **source-agnostic**: fed from the real arms' UDP push
(5 ms, both arms) it becomes a live digital twin — a lightweight RViz
alternative with the actual scene/regions/collision tooling. Inverted, a
"dispatch to emulator" panel makes the studio an interactive simulator
front-end. Both fall out of the Phase-1 plumbing.

## Risks / caveats

- No plugin abstraction exists in the studio — follow its duck-typed
  optional-import idiom; don't invent a framework.
- Visualization fidelity is **command-level, not dynamics**: the emulator
  interpolates linearly per device (matches `movej` joint-space geometry), so
  paths and timing are faithful, physics is not. Correct for rehearsal; don't
  oversell.
- Collision meshes are undecimated (`_OVERLAY_MAX_FACES = 0`, ~12 MB
  payloads) — the mesh-density reduction identified earlier (2.94 M triangles
  across the workspace) would speed both studio loading and FCL sweeps.
- Keep everything on 127.0.0.1 (no auth off-loopback).
- The studio's `STUDIO_IMPROVEMENT_PLAN.md` / `TODO.md` mention no live-state
  plans — no collision with in-flight work.
