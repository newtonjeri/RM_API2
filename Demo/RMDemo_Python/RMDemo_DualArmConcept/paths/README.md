# Cartesian paths

One motion program per path, plus the points extracted from it. **These are
data — not gitignored.**

    <name>.py               the motion program, as authored
    <name>.csv              its unique points        (metres, radians)
    <name>_sequence.csv     its traversal, in order  (metres, radians)
    <name>.png              the plot, if --plot was used

Everything but the `.py` is generated, by one command:

```bash
cd src
python3 path_viz.py --source ../paths/<name>.py --tool <ik frame> --plot
```

## paths/ is not plans/

| | holds | produced by | consumed by |
|---|---|---|---|
| `plans/` | MoveIt joint trajectories (~1800 waypoints) | the ROS orchestrator | `stage_runner`, `predict_task` |
| `paths/` | Cartesian points a program commands directly | hand-authored or taught | `path_viz`, and the program itself |

A plan says *which of the infinitely many arm configurations to use*; a
path here says only *where the tool goes*. Nothing in this folder pins the
redundant DOF, so two runs of the same path can use different elbows.

## Units

The CSVs are **metres and radians** — the units `rm_movel` itself takes, so
a row can go straight back to the controller with no conversion. The
terminal report and the plots print mm because that is how every other
surface shows these numbers (`FRAME_MAP.md`, the Web GUI, `blockly_points`).
Multiply by 1000; nothing else differs.

## Why the program is checked in next to its CSV

The CSV is derived. Keeping only the CSV means the next person cannot tell
whether it matches the program that ran, which is the same failure
`plans/README.md` exists to prevent — on 2026-08-08 two machines verified
different plans under one filename and both runs reported success. The
`.py` here is the *source of the points* and tracks the version that last
ran on hardware; regenerate the CSV from it rather than editing the CSV.
Where a value has changed, the old one stays in a comment beside it with
the reason — a point that was moved for a kinematic reason is exactly the
thing someone will otherwise "clean up" back to the failing value.

⚠ **The programs in this folder are runnable and some carry
`SIMULATION = False`.** They live here as the point source, not as an
invitation. `path_viz.py` never imports them — it parses them with `ast`
and executes nothing.

## What is here

| path | arm | frame | points | notes |
|---|---|---|---|---|
| `test_motion_001` | left (192.168.1.10) | World, `L_glove_4` | 10 (9 distinct) | 3x3 grid on one plane, `Z = -323.628 mm`; 37-step raster, 5.12 m of travel, blend 25. **Runs at 100% on every segment since 2026-08-12** (see below) |

## The elbow decides how fast a segment can go — 2026-08-12

`test_motion_001` would not run at 100%: segments `9->8` and `8->7` had to
be hand-tuned down to 60 and 90. The cause was **joint 4**, and it was
predictable offline:

> On an S-R-S 7-DOF arm the elbow angle is a function of the shoulder-to-
> wrist distance alone. The commanded Cartesian pose fixes that distance,
> so **J4 is redundancy-invariant** — re-solving `9->8` from 12 randomised
> arm configurations gave 548.1 deg/m every time, 0.0 % spread. It is the
> one joint that can be predicted without the controller's own resolution
> scheme, which is the same reason `predict_task.py` found the plan named
> the REAL binding joint 13/13.

At 0.600 m/s that cost J4 **329 deg/s against its 225 limit (146 %)**.
Nothing else on the path exceeded 60 %.

**The counterintuitive part: `9->8` failed *because* it was the smoothest
segment.** Every other long segment demands more than the controller's
0.60 rad/s angular cap, so the controller time-scales it — `2->1` runs at
0.211 m/s, not 0.600 — and that incidentally keeps their elbows safe.
`9->8` asked for only 0.23 rad/s, so nothing throttled it. Worse, with the
tool tilt nearly constant across it, the 220 mm glove stayed rigid and the
wrist had to cover 202 of the 215 mm itself, from the most folded
configuration on the path (J4 = 121 deg).

The fix was one number — point 8's `ry`, `-0.218 -> -0.400` — chosen where
the kinematic sweep and the other two rows' own tilt pattern agreed. The
elbow now balances across both segments (55 % / 56 % instead of 146 % / 9 %),
the row runs at 100 %, and it is *faster in wall-clock*: 0.83 s against
1.34 s at the old hand-tuned speeds.

**Read a low `rot deg` in the segment table as a warning, not comfort.**
A segment that barely rotates is a segment nothing will slow down for you.

`test_motion_001`'s tool frame is not stated in the program — it was given
with the points (glove_4 on the arm at `192.168.1.10`, which is
`LEFT_IP`) and is recorded in the CSV's `tool_frame` column so it cannot be
lost. Everything else in the table is read out of the program.

## What these files do NOT tell you

Reachability, clearance, and joint speed. The points are as **written**,
not as reached: `path_viz` does no IK, so a point can plot perfectly and
still be unreachable or drive a joint over its limit on the way in. Screen
a path with `predict_task.py` / `orientation_cost.py` and a SIM run before
any REAL motion.

One thing worth reading twice in `test_motion_001`: its `TCP_LINEAR_*`
constants are **printed, never applied** — the program's own comment says
they are configured in the controller. So the limits it runs under are
whatever the controller was last left holding, and those **ratchet**
(`reset_limits.py` before and after, every session).

That the old `9->8` failed at 100 % and survived at 60 % brackets the
controller's configured line speed to `0.41 < v <= 0.68 m/s`, consistent
with the `0.60` the program documents and *not* with the `0.250` recorded
in `dual_arm_common.py` from an earlier Web GUI read. Read it from the arm
before trusting either.

A side effect of the 2026-08-12 fix is worth knowing: `9->8` is now
governed by the 0.60 rad/s **angular** cap (v_eff 0.552 m/s), not the
linear one. Its elbow demand therefore no longer moves if the line-speed
limit ratchets — the segment went from the most speed-sensitive on the path
to one of the least.
