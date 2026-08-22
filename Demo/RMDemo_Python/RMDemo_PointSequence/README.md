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

Points are **absolute poses in the arm's world frame**. `translation` is
`[x, y, z]` in **metres**; `rotation` is `[rx, ry, rz]` Euler RPY in
**degrees** (set `rotation_units: rad` if yours are already radians).

```yaml
rotation_units: deg

cleaning_points:
  point1:
    translation: [0, 0, 0]
    rotation: [0, 0, 0]
  point2:
    translation: [0.0405, 0.012, 0.1065]
    rotation: [-10, 0, 0]

cleaning_sequence:
  - [point1, point2]
  - [point3, point4]
  - [point4, point1]
```

Optional keys: `rest_pose:` (overrides the arm-model table), and
`rotation_units:`.

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

Checked: degrees → radians (−80° → −1.396 rad), the `point2 → point3`
discontinuity at 80.1 mm, per-segment overrides landing on the right move,
`--connect 0` zeroing every `r`, unknown point names refused with the known
list, and `--blend 250` refused as a percentage.

**Not exercised:** any run on real hardware.

⚠ Dry-run first and read the resolved pose table — the arm is driven to
point 0 by `movej_p` before anything else, and those coordinates are yours to
confirm. Keep the E-stop in hand.
