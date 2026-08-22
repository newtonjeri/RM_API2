# RMDemo_PointSequence

Drive a list of points, in a given sequence, on one arm. **Standalone** —
`point_sequence.py` imports the RealMan SDK and PyYAML and nothing else.
Copy it anywhere with a points file and it runs.

```
movej_p  -> the first point of the sequence
movel    -> through the sequence, connect and blend applied per segment
movej    -> rest
```

`--loops N` repeats the motion N times before the single return to rest.

```
RMDemo_PointSequence/
├── points/example_points.yaml
└── src/point_sequence.py
```

## Running it

```bash
cd src

# resolve and print; never touches the arm
python3 point_sequence.py --points ../points/example_points.yaml --dry-run

python3 point_sequence.py --points ../points/my.yaml --ip 192.168.1.18
python3 point_sequence.py --points ../points/my.yaml --blend 20 --connect 1
python3 point_sequence.py --points ../points/my.yaml --loops 5 --v 30
```

| flag | default | meaning |
|---|---|---|
| `--points` | — | the YAML points file |
| `--ip`, `--port` | `192.168.1.18`, `8080` | the arm |
| `--v` | 20 | speed %, every move |
| `--blend`, `-r` | 0 | blend radius **% 0–100**, default for every move |
| `--connect` | 1 | `1` chains the moves, `0` makes each discrete |
| `--block` | 1 | blocking SDK calls |
| `--loops` | 1 | repeat the motion N times |
| `--rest` | arm-model table | rest joints, comma-separated degrees |
| `--dry-run` | — | resolve and print, never contact the arm |

## The points file

The generated cleaning config. **Two unit conventions live in one file** and
they are not the same:

| block | units | composition |
|---|---|---|
| `cartesian_poses` | metres and **RADIANS** | `R = Rz·Ry·Rx` (rm_pose_t) |
| `cleaning_points.rotation` | **DEGREES** | `R_delta = Rx·Ry·Rz`, left-multiplied |

Reading either in the other's units is silent — degrees-as-radians re-orients
the whole path, radians-as-degrees flattens it — so the defaults are separate
keys (`pose_units` rad, `rotation_units` deg) matching the generated files.

The cleaning points are **deltas from `start_pose`**:

```
position     p = p_start + translation   <- in arm_world AXES, not rotated
                                            into p_start
orientation  R = R_delta @ R_start       <- LEFT-multiplied
```

```yaml
cartesian_poses:
  start_pose: [0.5828, -0.1012, -0.1021, 2.9145, 0.4291, -3.0507]
cleaning_points:
  point14:
    translation: [0.0403, 0.0115, 0.0147]
    rotation: [-12.2, -10.2, 64.5]
cleaning_sequence:
  - [point14, point15]
```

`movej_p` goes to `start_pose` — which is usually **not** one of the cleaning
points, since no generated point has a zero delta — then the movel chain
follows the sequence. A file with no `start_pose` is read as absolute poses
instead. Optional: `rest_pose:`, `pose_units:`, `rotation_units:`.

**This program composes poses; it does not transform frames.** A config whose
`reference_frame` is not the arm's own needs the URDF and the pole height —
that is `RMDemo_CleaningMotion`'s job, and such a file is refused here rather
than resolved in the wrong frame.

## The sequence is a list of segments, and they need not join

In the example above segment 1 ends at `point2` while segment 2 starts at
`point3` — so there is a `point2 → point3` move the sequence never names.
Those **discontinuities are found and reported with their distance** before
anything is dispatched, then traversed as ordinary moves. Never silently: an
unannounced straight line across the workspace is exactly the move you did
not intend.

```
  DISCONTINUITIES — the sequence does not join here, so these moves are implied,
  not written down. Each is an ordinary straight line the arm will travel:
    point3     <- point2       80.1 mm
```

## Connect and blend

`--connect` and `--blend` set the default for every move. A segment can
override them with an optional third element:

```yaml
  - [point3, point4, {r: 25, connect: 1, v: 40}]
```

The override attaches to the move that **lands on** that segment's second
point.

Two rules are enforced, because both fail silently otherwise:

- **The last move always closes the chain** with `r=0, connect=0`. A chain
  whose final move still says `connect=1` never closes — the arm waits for a
  continuation that never arrives and the program hangs rather than failing.
- **`connect=0` forces `r=0`** on that move: a discrete move has nothing to
  blend into. Asking for both prints a note saying so.

`r` is a **percentage 0–100** of the shorter adjoining segment, not
millimetres. Passing `--blend 250` is rejected rather than sent.

A bare `ret=1` on a blended move usually means the blend could not carry the
speed step into that corner — lower `r`, or lower the speed. The program says
so when it sees one.

## Verified

`--dry-run` on the example above, and the loop path against a stubbed SDK:

```
--loops 3  ->  movej_p ×3, movel ×12 (4 per loop), movej rest ×1, disconnect
               every 4th movel closes the chain (r=0, connect=0)
```

Checked: the two unit conventions, discontinuity reporting with distances,
per-segment overrides landing on the right move (including the +1 shift the
prepended anchor introduces), `--connect 0` zeroing every `r`, unknown point
names refused with the known list, and `--blend 250` refused as a percentage.

**Cross-checked against `RMDemo_CleaningMotion`.** Both tools resolve the same
config through completely separate code, and
`RMDemo_CleaningMotion/src/verify_equivalence.py` diffs every dispatched call:
max pose difference **0.000e+00** on the `movej_p` and all 25 `movel` moves,
with identical per-move `(v, r, connect)`. Rotating one point by 1° flips it to
`THEY DIFFER`, so the check has teeth.

**Not exercised:** any run on real hardware.

⚠ Dry-run first and read the resolved pose table — the arm is driven to
point 0 by `movej_p` before anything else, and those coordinates are yours to
confirm. Keep the E-stop in hand.

## Visualising the points — `points/plot_points.py`

```bash
cd points
python3 plot_points.py example_points.yaml
python3 plot_points.py a.yaml b.yaml c.yaml -o compare.png
python3 plot_points.py a.yaml b.yaml --show
```

The **3D view takes two thirds** of the figure; **TOP (XY)**, **FRONT (XZ)**
and **SIDE (YZ)** share the remaining third, stacked. Up to three files
overlay, one colour each (blue / orange / aqua), named in the legend.

**Uniform scale on every axis, in every panel.** One millimetre of X is one
millimetre of Y is one millimetre of Z. Nothing is exaggerated or normalised,
so a flat surface looks flat and a slope has its real slope — which means the
elevations of a near-planar surface draw as thin strips, because that is what
they are. The points are plotted as given, in the traversal order the sequence
defines.
