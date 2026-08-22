# RMDemo_CleaningMotion — how the hardware executes a cleaning motion

An **arm-only** test bed for watching what the controller actually does with
a cleaning trajectory. No pole stage, no hand stage.

```
RMDemo_CleaningMotion/
├── motions/      <- drop cleaning-motion files here (see motions/README.md)
├── runs/         <- recordings; emulated runs redirect to /tmp/emu_runs
└── src/
    ├── cm_common.py           bridge to RMDemo_DualArmConcept + arm-only invariants
    ├── cm_loader.py           motion file -> MotionProgram   <-- THE SEAM
    ├── cm_frames.py           source frame -> arm base, pole pinned at minimum
    ├── cm_speed.py            speed args, the scaling law, the offline screens
    ├── cm_arm.py              the controller: v / r / connect / block per move
    └── run_cleaning_motion.py the driver
```

## The sequence is four steps

```
movej    -> rest
movej_p  -> start pose
cleaning path (the cleaning sequence)
movej    -> rest
```

That is the whole motion. No prestart hop, no approach `movel`. The arm
reaches the start pose with `movej_p`, which interpolates in **joint** space —
so the tool does not travel a commanded straight line and cannot be dragged
through anything on the way.

## Connect and blend radius are the point

They are ordinary arguments you set per run, not buried constants, because
they are what changes the answer this bed is asking for.

| flag | meaning |
|---|---|
| `--blend R` | blend radius: a **percentage 0–100** of the shorter adjoining segment (vendor: 交融半径百分比系数), **not millimetres** |
| `--connect 0\|1` | `1` = the moves join into one continuous trajectory; `0` = every move is discrete and the arm rests at each waypoint |
| `--block 0\|1` | `1` = the SDK blocks per move; `0` = non-blocking, arrival observed on the event stream |
| `--primitive movel\|moves` | Cartesian-linear between waypoints (the faithful mapping of what the planner intends), or a spline through them |
| `--cleaning-v`, `--transit-v` | v % on the cleaning moves and on the transits |

Two rules the dispatcher enforces, both of which bite silently otherwise:

- **The last move always closes the chain** with `r=0, connect=0`. A chain
  whose final move still says `connect=1` never closes — the controller waits
  for a continuation that never arrives and the run *hangs* rather than failing.
- **`--connect 0` forces `r=0` on every move.** A discrete move cannot blend
  into anything. Asking for both prints a note saying so; it is the control,
  not a bug.

Move `i` blends at the vertex it **ends** on, so per-move lists are
`len(poses) - 1` long — an off-by-one shifts every blend by one corner, which
is invisible in a plot.

## Running it

```bash
cd src

# offline — no arm, no network
python3 run_cleaning_motion.py --motion task:toplid_left --dry-run
python3 run_cleaning_motion.py --selftest --dry-run

# on the arm
python3 run_cleaning_motion.py --motion task:toplid_left --mode SIM  --speed 0.25
python3 run_cleaning_motion.py --motion task:toplid_left --mode REAL --speed 0.25 \
        --blend 10 --connect 1

# the two controls worth running back to back
python3 run_cleaning_motion.py --motion task:toplid_left --mode REAL --speed 0.25 --connect 0
python3 run_cleaning_motion.py --motion task:toplid_left --mode REAL --speed 0.25 --blend 25
```

`--motion` takes a file in `motions/`, or `task:<name>` to run a cleaning
config from the concept tree directly (`--fixture` selects the fixture,
default `commode_c`).

Speeds: `--speed` (one) or `--speeds a,b,c` (a ladder, run in order, stopping
on the first failure). With neither, the motion's own ladder; with neither of
those, the **factory 0.250 m/s** — an unspecified speed must not become a fast
one.

## The config drives everything — [cm_config.py](src/cm_config.py)

The generated config carries the frames, the start pose, the points and the
sequence. **Motion parameters are not in it** — they are passed at test-run
time, and anything nobody asks for keeps the value the **arm's controller
already holds**. An empty limit set means *change nothing*, which is not the
same as writing back what is already there: the TCP limits ratchet and are
shared with the Web GUI.

```yaml
task_parameters:
  ik_frame: R_arm_tip
  reference_frame: arm_world
cartesian_poses:
  start_pose: [0.5828, -0.1012, -0.1021, 2.9145, 0.4291, -3.0507]
cleaning_points:
  point1: {translation: [-0.1577, 0.2055, 0.0132], rotation: [-12.2, -10.2, 64.5]}
cleaning_sequence:
  - [point14, point15]
```

`reference_frame` and `ik_frame` are read from `task_parameters:` or the top
level. A `motion:` block is optional and overrides the controller for that run;
so does any CLI flag.

### The start-pose format depends on the reference frame

| `reference_frame` | `start_pose` | transform |
|---|---|---|
| `arm_world` / `arm_base` | `[x, y, z, rx, ry, rz]` — 6 values, Euler RPY **radians** (`rm_pose_t`, `R = Rz·Ry·Rx`) | identity |
| `alix_ref_frame` (a URDF frame) | `[x, y, z, qx, qy, qz, qw]` — 7 values, quaternion **w LAST** | from the URDF at the pinned minimum pole height |

`alix_ref_frame` and `butterfli_ref_frame` are the same frame under two
workspace names; either resolves. The **length** decides which form is parsed
(length cannot be misread), then the frame name is cross-checked and a mismatch
is *reported* rather than guessed at.

`rotation_units` governs the **point deltas** (degrees), not the RPY start pose
— that is a controller-convention pose in radians. Conflating them turned
`3.114 rad` into `0.054` in an early draft and silently re-oriented the whole
path.

### How the deltas resolve

```
position     p = p_start + translation      <- in REFERENCE FRAME axes,
                                               not rotated into the start pose
orientation  R = R_delta @ R_start          <- LEFT-multiplied
             R_delta = Rx(roll) Ry(pitch) Rz(yaw), degrees
```

### Frames

`ik_frame` becomes the controller tool frame through the lookup table: the URDF
link name with `_frame` removed, so `L_glove_frame_2` → `L_glove_2`. One
special case — **`R_arm_tip` / `L_arm_tip` → `Arm_Tip`**: the URDF names the
arm tip per side because one model carries two arms, but a controller knows
only its own and calls it `Arm_Tip`, its built-in default tool (no tool
offset). Selecting it by the URDF name fails with a bare `ret=1` that reads
like a missing frame.

Names over 11 characters are refused — the controller field truncates silently
and would select a different frame.

### Where movej_p goes

The `movej_p` target is the config's declared **`start_pose`**, which is not
always one of the cleaning points: in the generated configs no point has a zero
delta, so `start_pose` sits ~44 mm before the first stroke and the move onto it
is a real `movel`. Where `start_pose` *is* the first point (older configs,
`point1` at a zero delta) the duplicate is dropped rather than commanded as a
zero-length move. The dry-run prints which case applies.

### Sequences need not join

`cleaning_sequence` is a list of segments. Where consecutive segments do not
join, the move across the gap is implied but never written down — it is
traversed and **reported**, never silently inserted.

### Separate from `point_sequence.py`

`RMDemo_PointSequence` is the opposite tool by design: it takes **absolute**
world poses and dispatches through whatever tool frame the controller already
holds — it sets nothing, and computes no transform. This package reads the
frames, computes the transform itself, and sets the controller up first.

## Three rules this bed is built on

**1. Arm only. The pole is assumed at its minimum and never commanded.** The
pole is a prismatic joint that *carries the arm base*, so its height is part
of the transform that turns an authored waypoint into a pose the controller
understands. Resolving at the wrong height displaces every waypoint (215 mm
between the SRDF home 0.29 and the tasks' 0.075) and the arm reports nothing —
it just cleans the wrong place. `--no-pole`, `--no-hands` and `--pole` are
**refused**, not ignored. `cm_arm.CleaningArmController` has no lift or hand
method at all, so no run out of this bed can move either device by accident.

**2. Reuse, never fork.** The arrival semantics, the `acc ≥ 3 × speed`
rejection rule, the 15.3 mm `Arm_Tip → ConnectorLink` offset, the `Ry(+90°)`
mount, the error tables, the recorder, the J4 screen and the waypoint resolver
all come from `../RMDemo_DualArmConcept/src` **by import**. If a calibration
lands there, this bed gets it with no edit here.

**3. The mode is yours to set.** `--mode SIM` simulates, `--mode REAL` runs on
metal, and the script never changes it behind you. With neither, the arm runs
in whatever mode it was found in and the log says so.

## What the offline checks mean

| check | may refuse? | why |
|---|---|---|
| **J4 screen** | **yes** | On an S-R-S arm the elbow is fixed by the commanded pose regardless of how the 7-DOF redundancy resolves, so J4 is the one joint predictable offline. It refuses only when even the *most optimistic* reading (×1.10, the lowest measured under-read) still exceeds the 95 % abort. **Necessary, not sufficient** — the other six joints need a saved plan. |
| **junction audit** | never | Advisory. The predictor over-flags — run verbatim it refuses rungs hardware has completed — so it prints and the recording judges. |
| **unit sanity** | warns | Catches millimetres-as-metres and degrees-as-radians before dispatch. |

`--allow-over-limit` dispatches a rung the J4 screen refused. E-stop in hand.

## The speed scaling law is in effect

The controller **rejects** a `(speed, acceleration)` pair whose acceleration is
under 3× the speed — with a bare `ret=1`, after which the run proceeds at
whatever was already configured and reads as *"the speed made no difference"*.
Every speed goes through `speed_limits.scale_for()`. Two inherited traps:
`3 * 0.70` is `2.0999999999999996` in binary floating point (hence a 0.01
margin, the smallest `"%.2f"` can express), and **`v` is a percentage**, not
m/s — a rung is set by *configuring* `line_speed` and commanding `v` as a
percent of it.

TCP limits **ratchet** and are global controller state shared with the Web GUI.
The driver restores them, the tool frame and the run mode in a `finally`; after
a session above the factory defaults run `reset_limits.py --apply`.

## Status

Verified on the emulator (no hardware on this machine), on the **real**
`toplid_left` config — 28 waypoints, 27 segments, tool frame `L_glove_2`:

| | |
|---|---|
| chained, `--blend 10 --connect 1` | 27 segments in **32.09 s**, 3742 samples @ 97.6 Hz |
| discrete, `--connect 0` | 27 segments in **40.21 s**, `r` forced to 0 |
| config-driven, `loops: 2` | 2 × 27 segments, 27.00 s and 26.99 s, one rest at the end |
| J4 screen | 75 % → 82 % RANKED at 0.25 m/s; refused rung 0.35 on the selftest path (91 % → 100 %) |
| teardown | tool frame, run mode, limits all restored |

**Resolver cross-check.** `cm_config`'s independent resolution of the real
`toplid_left` config was compared against `CleaningPath`'s, all 28 waypoints,
reference frame given as `alix_ref_frame`: **max position difference 0.000000
mm**, max rotation difference 5.1×10⁻⁷ rad, tool frame and waypoint names
identical. That validates the quaternion order, the delta composition, the
mount angle and the pole pinning in one shot.

Adapters exercised: `cleaning_config`, `task_yaml`, `concept_path`, `json`.
**Not yet exercised:** any run on metal.

### Hardware runbook

1. `--dry-run` first. Read the frame-conversion block: the **start pose in arm
   coordinates** is where `movej_p` drives the arm blind. Confirm it before
   anything moves.
2. Confirm the tool frame exists on the controller. If not, the driver refuses
   rather than dispatching through the wrong frame — create it with
   `RMDemo_DualArmConcept/src/test_frame_alignment.py --mode REAL --create-frames`.
3. `--mode SIM` at the intended speed. A SIM truncation blocks the REAL run.
4. `--mode REAL --speed <lowest rung> --connect 0` first: discrete is the
   gentler motion and isolates whether a problem is the path or the blending.
   Then repeat with `--connect 1` and the blend you want.
5. Climb one rung at a time. The recording carries the commanded program
   verbatim, so the analysis checks the controller against what was *sent*.

⚠ The arm does **not** always self-stop: a controller abort can be GUI-only
(`4103 Joint4overspeed`) with `err1..err7`, `lift_err` and `arm_status` all
clean. The machine-readable evidence is truncated traced distance plus
non-arrival at the final waypoint. **Keep the E-stop in hand.**
