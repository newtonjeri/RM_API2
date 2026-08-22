# motions/ — drop cleaning-motion files here

This folder is yours. `run_cleaning_motion.py --list` enumerates whatever is
in it; `--motion <file>` runs one. Nothing here is imported as code — a `.py`
motion file is parsed with `ast`, never executed.

## The format is not decided yet

Newton is supplying the trajectory format (2026-08-22). Rather than guess it
and bake the guess through the pipeline, **everything downstream consumes one
in-memory type**, `cm_loader.MotionProgram`, and knows nothing about files.

Teaching this bed a new format is therefore exactly one function:

```python
@register("newtons_format")
def _load_newtons_format(path, text) -> MotionProgram:
    ...
```

and nothing else changes — frames, speed scaling, screens, dispatch and
recording are already built and tested against the seam.

## What an adapter must return

Each field below has a failure mode that is **silent at dispatch**, which is
why every one of them is validated rather than trusted.

| field | meaning | why it is checked |
|---|---|---|
| `poses` | `[[x, y, z, rx, ry, rz]]`, **metres and radians** | a program in millimetres is accepted by `rm_movel` and lands 1000× away |
| `source_frame` | `"arm_base"` if already controller-ready, else the URDF frame the poses are authored in | resolved at the **minimum pole height**; the wrong height displaces every waypoint |
| `side` | `"left"` or `"right"` | picks the arm and the base link |
| `tool_frame` | controller name, **≤ 11 chars** | the controller field is `c_char_Array_12` and **truncates silently**, selecting a different frame |
| `v_list`, `r_list` | per-**move**, so length is `len(poses) - 1` | move `i` blends at the vertex it *ends* on; an off-by-one shifts every blend by one corner |

The tool-frame name is the URDF link with the `_frame` token removed
(`L_glove_frame_4` → `L_glove_4`), per `FRAME_MAP.md`. The `json` adapter
applies that rule for you.

## Adapters that already exist

- **`task_yaml`** — a cleaning-points YAML: reference frame, `start_pose`
  relative to it, cleaning points as **deltas from the start pose**, the
  `cleaning_sequence`, and `ik_frame`. **This is the live format.** It is not
  re-parsed here — the adapter delegates to `TaskConfig` + `CleaningPath` in
  the concept tree, the faithful port of `TaskBase::resolveCleaningWaypoints`
  (verified against the saved plans to 0.3–0.4 mm), and pins the pole at its
  minimum. Reach it by path, or by name with `--motion task:<name>`.

  Three details that resolver gets right and a fresh parser would not:
  translations are start-pose-origin deltas **in the reference frame's axes**
  (not rotated into the start pose's frame); rotations compose the other way,
  `R_final = R_delta · R_start`, with `R_delta = Rx·Ry·Rz` in **degrees**; and
  `start_pose` is a **quaternion** whose files carry both `xyzw` and `wxyz`
  orderings across live and commented lines.

- **`json`** — a JSON object of waypoints. *Provisional*: it accepts a
  superset of the obvious spellings (`poses`/`waypoints`/`points`,
  `side`/`arm`, `tool_frame`/`ik_frame`/`tool`) and converts `units: "mm"`
  and `angle_units: "deg"` for you. It is the template for the real adapter.
- **`concept_path`** — a `paths/*.py` module from `RMDemo_DualArmConcept`,
  parsed with `ast`. Poses there are `POSES_MM`, already arm-base. Not a
  guess: it lets the bed be exercised today against paths that have real
  hardware recordings behind them.

A minimal JSON example:

```json
{
  "name": "my_motion",
  "side": "left",
  "tool_frame": "L_glove_frame_4",
  "frame": "arm_base",
  "units": "mm",
  "speed_ladder": [0.20, 0.25],
  "r_list": [10, 10],
  "waypoints": [
    {"name": "p0", "xyz": [515.0, 32.0, -323.6], "rpy": [-3.117, -0.400, 0.077]},
    {"name": "p1", "xyz": [580.0, 32.0, -323.6], "rpy": [-3.117, -0.400, 0.077]},
    {"name": "p2", "xyz": [645.0, 32.0, -323.6], "rpy": [-3.117, -0.400, 0.077]}
  ]
}
```

## If the poses are not already arm-base

Set `frame` to the URDF frame they are authored in (e.g. the fixture frame).
`cm_frames` resolves it with the **pole pinned at its minimum**, because the
pole is a prismatic joint that carries the arm base — so the transform is a
function of pole height, not a constant. The log always states which of the
two happened: an identity pass-through or a real transform, with the start
pose printed before and after.
