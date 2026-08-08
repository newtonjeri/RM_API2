# Bundled orchestrator plans

Copies of saved plans from the ROS workspace, checked in so the tests can
run on a machine that has **no ROS workspace** — specifically the lab
laptop, where the C11 capture runs.

| file | source | generated |
|---|---|---|
| `hinge_area_right_ruckig_pro_only.json` | `butterfli-ai-ros-jazzy/Resource/plans/commode_c/hardware/` | 2026-08-01 00:13:34, `cleaning_path_mode: ruckig_pro_only` |

## Why only the plan, and not the rest

The two halves of C11 need different things:

| half | needs | runs on |
|---|---|---|
| **capture** (`test_rehearsal_validate.py`) | this JSON + the SDK | lab laptop |
| **analysis** (`--replay`) | the full workspace — `butterfli.urdf`, SRDF, commode meshes via `scene_manifest_cached` | dev machine |

The workspace can't travel in this repo, but it doesn't have to: the
capture only reads joint positions out of the plan, and the machine that
does the geometry is the one that already has the workspace.

## Resolution — this folder, full stop

`segment_verifier.resolve_plan()` returns the copy **here**. It does not
search the ROS workspace. Override deliberately with `--plan PATH`, never
by accident and never by which machine you happen to be on.

An earlier version preferred the workspace copy, and on 2026-08-08 that
produced precisely the failure this folder exists to prevent: the lab
machine's workspace held a *different* plan under the same filename —
stroke stage `execute_cleaning_path` with 2012 waypoints plus an extra
retreat stage — so the rehearsal captured one plan while the dev machine
had verified another. Both runs reported success.

The plan is a versioned artifact of this repo, like the code. Whatever is
committed here is what every machine runs, and both scripts print the path
they used.

## Refreshing

Re-plan in the workspace, then copy the file here and say so in the
commit — the C12 clearance map and the C11 residual are both tied to a
specific plan, and both must be re-run when it changes.

```bash
cp ~/butterfli_ws/src/butterfli-ai-ros-jazzy/Resource/plans/commode_c/hardware/\
hinge_area_right_ruckig_pro_only.json plans/
```
