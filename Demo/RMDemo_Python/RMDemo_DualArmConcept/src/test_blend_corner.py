"""C19 — does `connect=1` + blend radius actually hold speed through a corner?

THE CLAIM UNDER TEST (Newton, 2026-08-12): chained `rm_movel` decelerates
hard at every corner, even for turns well under 45 deg, so the tool never
holds cruise across a connected path. `stage_runner` dispatches every
cleaning stroke this way, and H43 measured 42 % of the stroke time going
into 11 % of the path — dips, not cruise. If blending does not work, that
number is the whole explanation and no amount of `line_acc` fixes it.

THE GEOMETRY LIVES IN `paths/`, NOT HERE. This file dispatches and measures;
`paths/blend_corner_001.py` holds the points, the traversal, the corner
angles, the blend radii to compare and the speed. Move a point or add a
corner by editing that file — no source change, and the same file can be
plotted with `path_viz.py` and screened with `orientation_cost.py
--segments`, because all three parse the same format with `ast`.

WHAT THE PATH FILE GUARANTEES, and why the test is meaningless without it:
orientation is IDENTICAL at every point, so omega is zero on every segment
and the angular cap (H67) cannot time-scale anything — otherwise a
throttled segment looks exactly like a corner deceleration; and segments
are several times the ramp distance v^2/2a, so there is a real cruise
plateau to lose at a corner. Its header states both.

WHAT IT MEASURES. From the 100 Hz UDP push, per corner: cruise speed on the
approach, the minimum through the corner, and the fraction retained.

    speed retained = v_min_at_corner / v_cruise_before_it

A corner that blends holds most of its cruise; a corner that stops shows
near zero. Corners are located by ARC LENGTH, not by time — the arm's speed
is the thing being measured, so finding the corner by time would assume the
answer.

DECISIVE COMPARISON: r=0 against the largest r. If they measure the same,
the blend radius is not being applied at all. `--connect0` adds a discrete
baseline that SHOULD stop dead at every corner; if it does not, the
measurement is wrong rather than the controller.

SPEED IS A LADDER, NOT A SETTING. The path file's `SPEED_LADDER` lists the
line_speeds to sweep; each runs in turn, ASCENDING, with its own recording
and its own `run.json`, and the climb STOPS at the first rung that fails. A
path with no `SPEED_LADDER` is a one-rung ladder at its own
`TCP_LINEAR_VELOCITY`, so nothing changes for it.

`--speed X` IS AN OFF-LADDER RUN and takes ANY value up to the 1.8 m/s
vendor ceiling — including speeds the ladder would refuse. The elbow screen
still runs and still prints, but over the limit it WARNS instead of
stopping, because the two situations are not the same: a ladder climbs on
its own and nobody chose its top rung, whereas `--speed` is an operator
naming one number for one run. Every requested speed is validated against
the ceiling BEFORE the arm is connected, so an impossible request costs
nothing and leaves no limits raised behind it.

`line_acc` is not a free parameter — `speed_limits.scale_for` derives it
from the rung under the vendor rules (max line_speed 1.8 m/s, line_acc >=
3 x line_speed). Setting only the speed is rejected with a bare ret=1, after
which the run proceeds at whatever was already configured and reads as "the
speed made no difference". The FIRST rung's limits are the ones restored at
exit — these limits ratchet, so `reset_limits.py` is still the closing step.

EVERY RUNG IS SCREENED BEFORE IT RUNS. `preflight_j4` predicts the elbow
demand offline and refuses any rung over 100 % of J4's limit, before the
limits are touched, so a refused rung leaves the controller untouched.
`--allow-over-limit` overrides it. This is not hypothetical: run
`20260813T183633` climbed to 0.45 m/s on `blend_corner_001` and returned
"segment 5: arrival event reports failure" — segment 5 is the one this
screen puts at 117 %. J4 is exact because the elbow angle is fixed by the
commanded pose regardless of how the redundancy is resolved; the other six
joints need a saved plan, so a clean screen is necessary, not sufficient.

WHY THE TWO PATHS HAVE DIFFERENT LADDERS. `test_motion_001` rotates on every
segment, so the angular cap time-scales it (H67) and its J4 demand SATURATES
at 59 % — it takes 0.25 through 0.80 safely. `blend_corner_001` holds
orientation constant, which is what makes a dip mean "corner"; nothing
throttles it, J4 scales linearly, and it stops at 0.35. The smooth path is
the dangerous one.

MODE IS YOURS TO SET, and this script never changes it behind you. `--mode
SIM` runs in simulation, `--mode REAL` runs on metal, and that is all that
happens — the request is engaged through `apply_run_mode`, which VERIFIES it
by readback and aborts if the controller refuses, then restores the previous
mode at exit. Drive it the way the rest of the suite is driven: SIM first,
then REAL once SIM has completed.

SIM is worth running first but is not a guarantee: `20260811T184017`
completed in SIM with J4 at 100 % while REAL at the same settings stopped
silently. It catches the planning class of failure, not the dynamic one.

SAFETY. Free space, no commode, E-stop in hand — the controller does not
reliably stop itself (H39/H45). `blend_corner_001` sits INSIDE the box
`test_motion_001` has already run on hardware at 100 % — x 515-788 mm,
y -66..40 mm, on its exact z plane -323.628 and its tool L_glove_4, with
12-74 mm of margin on every side. Its worst segment is 52 % of the J4
limit. It runs at 0.20 m/s, under the factory default, so no limit changes.
Verify the pendant's real-time speed slider reads 100 % (H65) or every
number here is scaled by an unknown factor.

MEASUREMENT ACCURACY IS VALIDATED, not assumed — `verify_blend_measure.py`
synthesises streams with a KNOWN retention and checks this code recovers it
(+-8 points, biased HIGH so it under-reports deceleration). Re-run it after
any change to the analysis.

    python3 test_blend_corner.py --side left --mode SIM
    python3 test_blend_corner.py --side left --mode REAL
    python3 test_blend_corner.py --side left --mode REAL --connect0
    python3 test_blend_corner.py --side left --mode REAL --reverse
    python3 test_blend_corner.py --side left --mode REAL --speed 0.25
    python3 test_blend_corner.py --side left --mode SIM --path ../paths/<other>.py
"""

import ast
import math
import os
import pathlib
import sys
import time

from dual_arm_common import (
    handle_cli, ArrivalMonitor, connect_both, teardown, host_ip_for,
    apply_run_mode, restore_run_modes, mode_label, parse_mode_arg,
    DEV_JOINT, LEFT_IP, RIGHT_IP, UDP_PORT,
)
from run_recorder import RunRecorder
import speed_limits

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "paths", "blend_corner_001.py")


def load_path(src):
    """Points, traversal and settings from a `paths/*.py` program.

    PARSED WITH `ast`, NEVER IMPORTED — the same rule `path_viz.py` and
    `orientation_cost.py` follow, because these files are runnable motion
    programs and some carry `SIMULATION = False`. Reading one must not be
    able to move an arm.

    Everything the test needs lives in the path file: the points, the
    sequence, the blend radii to compare, the corner angles to label the
    output with, and the speed. Adjusting the test means editing that file,
    not this one.
    """
    tree = ast.parse(open(src).read())
    got = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            nm = getattr(t, "id", "")
            if nm in ("POSES_MM", "SEQUENCE", "BLEND_SWEEP", "CORNER_ANGLES",
                      "TCP_LINEAR_VELOCITY", "SEGMENT_SPEEDS", "BLEND",
                      "SPEED_LADDER", "TOOL_FRAME"):
                try:
                    got[nm] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    if "POSES_MM" not in got:
        raise SystemExit("no POSES_MM in %s" % src)
    by = {str(k): v for k, v in got["POSES_MM"].items()}
    seq = [str(s) for s in got.get("SEQUENCE", list(by))]
    # A SEQUENCE entry with no Cartesian pose is a JOINT target — programs
    # open with a `startpose` reached by MOVEJ. It is a real step but not a
    # `movel` segment, so it is dropped from the Cartesian path and named,
    # never silently swallowed.
    skipped = [s for s in seq if s not in by]
    if skipped:
        print("  [INFO] not Cartesian waypoints, skipped: %s"
              % ", ".join(dict.fromkeys(skipped)))
    seq = [s for s in seq if s in by]
    if len(seq) < 2:
        raise SystemExit("fewer than 2 Cartesian waypoints in %s" % src)
    poses = [[by[s][0] / 1000.0, by[s][1] / 1000.0, by[s][2] / 1000.0,
              by[s][3], by[s][4], by[s][5]] for s in seq]
    speed = got.get("TCP_LINEAR_VELOCITY", 0.25)
    return {
        "poses": poses, "labels": seq,
        "blends": list(got.get("BLEND_SWEEP", [got.get("BLEND", 25)])),
        "angles": list(got.get("CORNER_ANGLES", [])),
        "speed": speed,
        # A path with no SPEED_LADDER is a one-rung ladder at its own speed —
        # so the ladder is the only code path and a file that predates the
        # feature still behaves exactly as it used to.
        "ladder": [float(v) for v in got.get("SPEED_LADDER", [speed])],
        "tool": got.get("TOOL_FRAME"),
        "seg_speeds": got.get("SEGMENT_SPEEDS", {}),
    }


J4_LIMIT_PCT = 100.0


def preflight_j4(poses, tool, speed):
    """(worst % of the J4 limit, segment index) predicted offline, or None.

    THE GATE THAT WOULD HAVE CAUGHT `20260813T183633`. That run climbed to
    0.45 m/s on `blend_corner_001` and came back "[FAIL] segment 5: arrival
    event reports failure" — segment 5 is `c5->end`, the segment this screen
    puts at 117 % of J4's limit at that speed. Nothing about the blend was
    wrong; the elbow was asked for more than it has.

    J4 is used because on an S-R-S arm the elbow angle is fixed by the
    commanded pose, INDEPENDENT of how the 7-DOF redundancy is resolved — so
    this number is exact without knowing which configuration the controller
    will pick. The other six joints are not predictable without a saved plan,
    so a clean result here is necessary and NOT sufficient.

    Returns None if the screen cannot run, and a caller must treat that as
    "unknown", never as "safe".
    """
    try:
        import orientation_cost as oc
        if tool not in oc.TOOL_OFFSETS:
            return None
        err = oc.selfcheck(tool)
        if err is not None and err > 0.001:
            return None                  # transform unverified: refuse to opine
        worst = None
        for r in oc.segment_report(poses, tool, speed):
            if r["j4"] is None:
                continue
            pct = 100.0 * r["j4"] / oc.JOINT_LIMIT[3]
            if worst is None or pct > worst[0]:
                worst = (pct, r["i"])
        return worst
    except Exception:
        return None


def read_tcp(run_dir):
    """[(t, x, y, z)] from a recorded `stream.csv`, BY COLUMN NAME.

    `RunRecorder.stop()` returns the run DIRECTORY, not the samples — so
    the analysis reads the file the recorder just wrote. Two benefits over
    holding the rows in memory: the numbers analysed are provably the
    numbers stored, and any past run can be re-analysed with
    `corner_speeds(read_tcp(<run dir>), ...)`.

    Columns are looked up by name because the stream carries 40-odd of them
    (7 joints x 5 channels, lift, TCP pose, status) and their order is not
    part of any contract.
    """
    import csv
    out = []
    with (pathlib.Path(run_dir) / "stream.csv").open() as fh:
        for r in csv.DictReader(fh):
            try:
                out.append((float(r["t_mono"]), float(r["tcp_x"]),
                            float(r["tcp_y"]), float(r["tcp_z"])))
            except (KeyError, ValueError, TypeError):
                continue
    return out


def read_joint_peaks(run_dir):
    """Peak |speed| per joint from a recorded stream, deg/s. [] if absent.

    SIMULATION populates positions but leaves the speed channel at zero, so
    an all-zero result means "not measured", never "nothing moved". The
    caller must not read it as a clean bill of health.
    """
    import csv
    peaks = [0.0] * 7
    with (pathlib.Path(run_dir) / "stream.csv").open() as fh:
        for r in csv.DictReader(fh):
            try:
                for j in range(7):
                    peaks[j] = max(peaks[j], abs(float(r["speed%d" % (j + 1)])))
            except (KeyError, ValueError, TypeError):
                continue
    return peaks if any(peaks) else []


def path_followed(rows, poses, tol=0.15):
    """(ok, recorded_m, commanded_m) — did the tool actually trace the path?

    EVERY corner number below is meaningless if it did not, and the two ways
    that happens both matter:

      SHORT — the arm stopped part-way. That is the H45 signature, and at
      `line_speed 0.60` it stopped at 77 % of the path while reporting
      nothing on any channel. A corner analysis of that run would quietly
      report the corners it did reach and look like a result.

      LONG — the stream is not the path. The emulator does this: its IK
      fails on every Cartesian sample of `blend_corner_001`, and it traces
      1.176 m against a commanded 0.600 m. Arc length is rotation-invariant,
      so this comparison holds whichever frame the controller reports in.
    """
    if len(rows) < 2:
        return False, 0.0, 0.0
    rec = sum(math.dist(rows[i][1:4], rows[i - 1][1:4])
              for i in range(1, len(rows)))
    cmd = sum(math.dist(poses[i][:3], poses[i + 1][:3])
              for i in range(len(poses) - 1))
    if cmd < 1e-9:
        return False, rec, cmd
    # THE END POSE IS CHECKED SEPARATELY, because arc length alone missed a
    # real abort. Run 20260813T205319 stopped 44.6 mm short of its final
    # waypoint after the controller refused the trajectory — and still
    # measured 101 % of commanded arc, because summing noisy displacements
    # inflates the recorded path by 7-10 % and that inflation cancelled the
    # shortfall almost exactly. A tool that ends 45 mm from where it was sent
    # did not follow the path, whatever the arc says.
    #
    # Only meaningful when the stream is in the PATH's frame, which is not
    # guaranteed — a controller reporting in the base frame ends nowhere near
    # the path's last waypoint while having followed it perfectly. Arc length
    # is rotation-invariant and this is not, so the frame is established from
    # the START sample first: if the run did not begin at the path's first
    # waypoint either, the two are in different frames and only the
    # arc-length test applies.
    same_frame = math.dist(rows[0][1:4], poses[0][:3]) <= 0.010
    end_off = math.dist(rows[-1][1:4], poses[-1][:3])
    if same_frame and end_off > 0.010:
        return False, rec, cmd
    return abs(rec - cmd) / cmd <= tol, rec, cmd


# 7 samples = 70 ms. Chosen by measurement, not taste: swept 3-11 samples
# against synthetic streams carrying the MEASURED aliasing and a prescribed
# retention of 100/70/50/30/20 %, 7 was the minimum-error window (9.2 points
# worst case, monotonic). 3 gives 23.7, 11 gives 10.5 and starts smearing the
# dip. `run_recorder` smooths 50 ms for the same reason; this is the same
# correction tuned to a narrower feature.
SPEED_WINDOW = 7


def corner_speeds(rows, poses, t_start, t_end):
    """(cruise, v_min, retained, n, n_near, flat) per interior corner.

    A corner is located by ARC LENGTH, not by time: the arm's speed is what
    we are measuring, so using time to find the corner would assume the
    answer. Cruise is the median speed over the middle half of the incoming
    segment; the corner window is +-30 % of a segment either side of the
    vertex.

    SPEED IS DIFFERENCED OVER `SPEED_WINDOW` SAMPLES, NOT ONE. The position
    field of the UDP push is NOT synchronous with the 100 Hz push rate — it
    advances unevenly, so consecutive-sample differencing aliases badly even
    though the timebase itself is clean (measured 2026-08-13 on run
    `20260813T183639`: dt = 10.0 +-0.14 ms, yet 24-36 % of moving intervals
    advance less than 40 % of the median step). Two independent proofs that
    the fault is in the position channel and not the motion:
      * the controller's OWN joint-speed channel climbs smoothly over the
        same window (J4: -47.9 -> -119.3 deg/s monotonic) while differencing
        the reported joint POSITION over the same samples gives -112, -68,
        -27, -164, -25 deg/s. Median ratio 1.00, p10 0.39, p90 1.63;
      * single-interval TCP speed reaches a p95 of 730 mm/s against a
        450 mm/s cap (162 %) — not physically possible. At a 30 ms window
        that falls to 484 mm/s = 108 % of cap, the H59 signature.
    `v_min` is a MINIMUM, so raw differencing lands on the worst alias and
    biases retention LOW — it would manufacture deceleration. This is the
    same 50 ms smoothing `run_recorder._speed_report` already applies; the
    two now agree by construction.
    """
    pts = [r for r in rows if t_start <= r[0] <= t_end]
    # The last sample can be a partial step — the recording stops mid-interval,
    # giving a short chord and a spuriously low speed that lands in the final
    # corner's window. One sample, dropped.
    pts = pts[:-1]
    if len(pts) < 40:
        return []
    xs = [(r[0], r[1], r[2], r[3]) for r in pts]          # t, x, y, z
    cum, prev = [0.0], xs[0]
    for p in xs[1:]:
        cum.append(cum[-1] + math.dist(p[1:4], prev[1:4]))
        prev = p
    half = SPEED_WINDOW // 2
    spd = []
    for i in range(len(xs)):
        a = max(0, i - half)
        b = min(len(xs) - 1, i + half)
        dt = xs[b][0] - xs[a][0]
        spd.append(math.dist(xs[b][1:4], xs[a][1:4]) / dt if dt > 1e-6
                   else 0.0)
    seg = [math.dist(poses[i][:3], poses[i + 1][:3])
           for i in range(len(poses) - 1)]
    # EACH VERTEX IS FOUND BY CLOSEST APPROACH IN SPACE, not by arc length.
    #
    # Arc length was the original choice, to avoid using time — the arm's
    # speed is what is being measured, so locating a corner by time would
    # assume the answer. But cumulative arc is a SUM OF NOISY DISPLACEMENTS
    # and it only ever runs long: on hardware the recorded arc came to
    # 107-110 % of the commanded path (0.46 mm median perpendicular deviation
    # from the commanded line, and 19 % of intervals stepping backwards),
    # while noiseless SIM streams read 97-100 %.
    #
    # An 8 % overestimate is not a small error here, because it ACCUMULATES:
    # measured on 20260813T205154, the arc locator placed the vertices 9, 22,
    # 28, 34 and 34 mm early. The corner window is +-30 % of a 65 mm segment
    # = +-19.5 mm, so vertices 2-5 fell entirely OUTSIDE their own windows and
    # were measured mid-segment, where the arm is cruising. That is where the
    # 222 %, 229 % and 426 % retentions came from, and it made the FIRST
    # corner — the least mislocated one — look uniquely bad.
    #
    # A waypoint is a known point in space, so closest approach is exact and
    # cannot drift: it recovered every vertex to within 0.1-4.8 mm, the
    # residual being the blend genuinely cutting the corner. It assumes
    # nothing about speed either.
    cmd_total = sum(seg)
    # Window widths are still measured in recorded arc, so they are stretched
    # by the same factor to cover the intended TRUE distance. Over a 40 mm
    # window this is a ~3 mm correction — small, but free.
    scale = (cum[-1] / cmd_total) if cmd_total > 1e-9 else 1.0
    scale = min(max(scale, 1.0), 1.5)      # never shrink; never trust a wild one

    idx = [min(range(len(xs)), key=lambda i: math.dist(xs[i][1:4],
                                                       poses[k + 1][:3]))
           for k in range(len(seg) - 1)]
    # CLOSEST APPROACH NEEDS THE STREAM TO BE IN THE PATH'S FRAME, and that is
    # not guaranteed — arc length was rotation-invariant and this is not. On
    # hardware the two agree (all 42 runs of 2026-08-13 started within 3.2 mm
    # of their commanded start pose), but a controller reporting in the base
    # frame, or an emulator whose IK diverges, would put every "closest"
    # sample somewhere meaningless while still returning an index.
    #
    # So it is CHECKED, not assumed: if the tool never comes near its own
    # waypoints, fall back to arc length with the inflation divided out. That
    # keeps the drift proportional instead of accumulating, which is most of
    # the fix, and it costs nothing when closest approach is available.
    miss = sorted(math.dist(xs[idx[k]][1:4], poses[k + 1][:3])
                  for k in range(len(idx)))
    if miss and miss[len(miss) // 2] > 0.25 * (cmd_total / len(seg)):
        acc = 0.0
        idx = []
        for s in seg[:-1]:
            acc += s * scale
            idx.append(min(range(len(cum)), key=lambda i: abs(cum[i] - acc)))
    # THE CHORD-AT-A-CORNER ARTIFACT, and why one sample per corner is
    # dropped. Speed here is |dp|/dt between consecutive samples — a CHORD.
    # Where an interval straddles a vertex the chord cuts the corner and is
    # shorter than the distance actually travelled, so the computed speed
    # reads LOW by a geometric amount that grows with the turn angle.
    # Measured on synthetic streams holding a known constant speed through
    # the corners: 15 deg read 2 % low, 90 deg read 25 % low. Uncorrected,
    # this test would manufacture the very deceleration it exists to detect,
    # worst exactly where the claim is most interesting.
    #
    # Only ONE interval per vertex can straddle it, so dropping that single
    # interval removes the artifact and keeps every real sample of the dip.
    # The vertex is located from CHORD-based cumulative arc, which lags the
    # true arc slightly — so the crossing index can be off by one and the
    # short interval survives. Excluding the immediate neighbourhood covers
    # that: measured, a sample landing exactly on a vertex still read 8 %
    # low with only the single interval removed.
    #
    # With a windowed speed the contamination spreads to every sample whose
    # window spans the vertex, so the exclusion widens by the window's half
    # width. This costs dip resolution: at 0.20 m/s an r=25 blend on a 65 mm
    # segment is only ~8 samples wide and this removes ~5 of them. Lower
    # `TCP_LINEAR_VELOCITY` in the path file to buy that resolution back —
    # halving the speed doubles the samples in the dip.
    straddle = set()
    for iv in idx:
        straddle.update(range(iv - 1 - half, iv + 2 + half))

    out = []
    for k, iv in enumerate(idx):
        v = cum[iv]                        # the vertex, in RECORDED arc
        # +-30 % of the segment. `near` feeds a MINIMUM, so widening it can
        # only add cruise samples that cannot lower the min — it costs
        # nothing and buys resolution. At +-15 % a 65 mm segment gave ~10
        # samples, and removing 3 for the straddle left 7, below
        # MIN_CORNER_SAMPLES: a perfectly flat run was being declined for
        # want of samples. The next vertex is a full segment away, so 30 %
        # cannot reach it.
        win = 0.30 * seg[k] * scale
        near = [spd[i] for i in range(len(cum))
                if abs(cum[i] - v) <= win and i not in straddle]
        mid = [spd[i] for i in range(len(cum))
               if v - 0.75 * seg[k] * scale <= cum[i]
               <= v - 0.25 * seg[k] * scale
               and i not in straddle]
        if not near or not mid:
            out.append(None)
            continue
        mid.sort()
        cruise = mid[len(mid) // 2]
        vmin = min(near)
        # IS THERE A CRUISE PLATEAU TO MEASURE AGAINST? Retention is a ratio
        # against the approach speed, so it needs the approach to actually
        # BE at cruise. Once the blend radius approaches half the segment the
        # dips from both ends meet and the "cruise" window is itself inside a
        # dip — cruise reads low and retention reads high. Verified: at a
        # blend of 50 % of a 65 mm segment the recovered value came out 8
        # points high with no other cause. A flat window varies by a few per
        # cent; anything worse is not a plateau and the corner is marked.
        flat = (mid[-1] - mid[0]) / mid[-1] if mid[-1] > 1e-9 else 1.0
        # The gate is 0.30, not 0.20, because 0.20 sat BELOW THE NOISE FLOOR.
        # A synthetic stream holding a dead-constant speed, carrying only the
        # position-field aliasing measured on hardware, scores flat = 0.22 —
        # so the old gate declined every corner of every run including
        # perfectly flat ones, and then blamed it on sample count. 0.30 still
        # catches the case it exists for: once the blend approaches half the
        # segment the dips from both ends meet, the "cruise" window is itself
        # inside a dip, and retention reads high against a depressed cruise.
        n_ok = len(near) if flat <= 0.30 else 0
        # `n` is how many samples the corner was measured from. Accuracy
        # tracks it directly — validated against synthetic streams holding a
        # KNOWN retention: 20 samples -> 2 pt error, 8 -> 5 pt, 4 -> 13 pt,
        # 2 -> 20 pt. Reported so a thin corner is visibly thin rather than
        # quietly wrong.
        #
        # `n_near` and `flat` are carried out too because a corner is
        # declined for TWO different reasons and they need different fixes:
        # too few samples wants a lower speed or a bigger blend, no cruise
        # plateau wants a longer segment or a lower speed. Reporting both as
        # "< 8 samples" sent the 2026-08-13 session looking for the wrong
        # thing — every corner there had 11-44 samples and was declined
        # purely on the plateau test.
        out.append((cruise, vmin, vmin / cruise if cruise > 1e-6 else 0.0,
                    n_ok, len(near), flat))
    return out


MIN_CORNER_SAMPLES = 8      # >= 8 kept the validated error under ~5 points


def goto_start(arm, poses, mon):
    """Put the tool on `poses[0]` and PROVE it got there. True on success.

    EVERY case must begin from the same pose or it is not the same
    experiment. This is called before each case, not after, because "after"
    is a place that can be skipped — and was.

    2026-08-13, what this repairs: `run_one` dispatches `poses[1:]` and never
    moved to `poses[0]`, so the first case ran from wherever the previous
    program left the arm — 47 mm away, and its first segment was a different
    geometry. The return-to-start then sat AFTER the analysis, behind a
    `continue` that fires whenever a case is rejected; so one bad first case
    left every later case starting from the previous case's END pose, 273 mm
    away. Nine of the fifteen recorded runs were lost that way, and the two
    failure modes compound silently: the arc-length guard rejected the ones
    that were obviously wrong, but `20260813T183639` drifted 0.2 mm and was
    ACCEPTED, so a wrong-start run can still print numbers.

    Arrival is confirmed by POSE READBACK, not by the event alone. The event
    says the controller finished planning its move; the readback says the
    tool is where the next measurement assumes it is.
    """
    off, st = None, None
    # TWO ATTEMPTS, each allowed to SETTLE. The arrival event fires when the
    # controller has finished its trajectory, which is not the same instant
    # the tool has stopped moving — reading the pose immediately after it
    # catches the arm mid-settle. On 2026-08-13 this cost five of seven rungs
    # of a `test_motion_001` ladder to a single 9.8 mm reading, on a return
    # the arm had in fact completed. Waiting for the pose to stop changing
    # removes the race; a second attempt covers a move that genuinely fell
    # short, which is worth one retry before abandoning the sweep.
    for attempt in (1, 2):
        mon.expect(arm.handle_id, DEV_JOINT)
        ret = arm.robot.rm_movel(poses[0], 30, 0, 0, 0)
        arrived, ok = mon.wait(arm.handle_id, DEV_JOINT, 90.0)
        if ret != 0 or not arrived or not ok:
            print("  [FAIL] move to the start pose: ret=%s arrived=%s ok=%s"
                  % (ret, arrived, ok))
            return False
        prev = None
        for _ in range(20):                       # up to ~2 s of settling
            sret, st = arm.robot.rm_get_current_arm_state()
            if sret != 0 or not isinstance(st, dict) or not st.get("pose"):
                print("  [FAIL] cannot read back the pose to confirm the "
                      "start position (ret=%s)" % sret)
                return False
            if prev is not None and math.dist(st["pose"][:3], prev) < 0.0002:
                break                             # stopped moving
            prev = list(st["pose"][:3])
            time.sleep(0.1)
        off = math.dist(st["pose"][:3], poses[0][:3])
        if off <= 0.005:
            return True
        if attempt == 1:
            print("  [INFO] start pose %.1f mm out after settling — "
                  "re-commanding it once before giving up." % (1000 * off))
    if off > 0.005:
        # Print BOTH poses, because the two ways this fails need different
        # responses and the number alone does not separate them. A few mm is
        # a genuine miss — the arm did not get there. Hundreds of mm means
        # the readback is not in the frame the path is written in, and the
        # check is wrong rather than the arm; the emulator does exactly this,
        # reporting 930 mm because its IK cannot resolve this path at all.
        print("  [FAIL] start pose is %.1f mm from where it should be, so "
              "every later case would run a different geometry.\n"
              "         commanded %s\n"
              "         read back %s\n"
              "         A few mm means the move fell short. Hundreds of mm "
              "means the readback is in a different frame than the path."
              % (1000 * off,
                 ["%.1f" % (1000 * v) for v in poses[0][:3]],
                 ["%.1f" % (1000 * v) for v in st["pose"][:3]]))
        return False
    return True


def run_one(arm, poses, blend, connect, mon):
    """Dispatch the polyline. True on success.

    A chained program produces ONE arrival event, from the closing
    `connect=0` segment; a discrete program produces one per move. Waiting
    on the wrong count is how a chained test hangs, so the expectation is
    registered only where an event is actually due.
    """
    for i, p in enumerate(poses[1:]):
        last = (i == len(poses) - 2)
        c = 0 if not connect else (0 if last else 1)
        r = 0 if (last or not connect) else blend
        if c == 0:
            mon.expect(arm.handle_id, DEV_JOINT)
        ret = arm.robot.rm_movel(p, 100, r, c, 0)
        if ret != 0:
            print(f"  [FAIL] segment {i} rm_movel ret={ret}")
            return False
        if c == 0:
            arrived, ok = mon.wait(arm.handle_id, DEV_JOINT, 90.0)
            if not arrived:
                print(f"  [FAIL] segment {i}: no arrival event in 90 s "
                      "(H45 signature — the chain was abandoned)")
                return False
            if not ok:
                print(f"  [FAIL] segment {i}: arrival event reports failure")
                return False
    return True


def main() -> int:
    handle_cli(__doc__, extra_flags=("--connect0", "--reverse"),
               value_flags=("--side", "--path", "--speed", "--line-acc"))
    forced = parse_mode_arg()
    side = "left"
    if "--side" in sys.argv:
        side = sys.argv[sys.argv.index("--side") + 1]
    src = DEFAULT_PATH
    if "--path" in sys.argv:
        src = sys.argv[sys.argv.index("--path") + 1]
    also_c0 = "--connect0" in sys.argv

    path = load_path(src)
    poses, labels = path["poses"], path["labels"]
    angles_file = path["angles"]
    if "--reverse" in sys.argv:
        # THE CONTROL FOR "IS IT THE ANGLE, OR IS IT THE FIRST CORNER?"
        # Every corner keeps its angle and its geometry; only the order
        # changes, so the sharpest corner is no longer the one the tool
        # reaches first out of the start pose. The 2026-08-13 runs need this:
        # the 90 deg corner read 5-14 % retained at every blend radius while
        # the other four ran 50-122 %, and it is also the corner reached
        # 65 mm after standing still. Those two explanations predict opposite
        # results here — if the angle is the cause the 90 deg column stays
        # low, if the start is the cause it recovers.
        poses = poses[::-1]
        labels = labels[::-1]
        angles_file = angles_file[::-1]
        print("  [NOTE] --reverse: traversing the path end-to-start. Same "
              "corners, same angles, opposite order — the control that "
              "separates a corner's ANGLE from its POSITION in the run.")
    path["angles"] = angles_file
    angles = path["angles"] or [None] * (len(poses) - 2)
    req_acc = None
    if "--line-acc" in sys.argv:
        req_acc = float(sys.argv[sys.argv.index("--line-acc") + 1])
    # THE LADDER. Every rung runs in turn, ASCENDING, each recorded on its
    # own, and the climb STOPS at the first rung that fails — continuing past
    # a stall is how the 0.80 run that reversed four joints in 80 ms happened.
    # `--speed` collapses the ladder to one rung, which is also what a path
    # file with no SPEED_LADDER gets.
    off_ladder = "--speed" in sys.argv
    if off_ladder:
        rungs = [float(sys.argv[sys.argv.index("--speed") + 1])]
        ladder_src = "--speed (off-ladder, single run)"
    else:
        rungs = sorted(set(path["ladder"]))
        ladder_src = ("SPEED_LADDER in %s" % os.path.basename(src)
                      if len(rungs) > 1 else
                      "TCP_LINEAR_VELOCITY in %s" % os.path.basename(src))
    tool = path["tool"]
    allow_over = "--allow-over-limit" in sys.argv

    # VALIDATE EVERY SPEED BEFORE TOUCHING THE ARM. `scale_for` enforces the
    # 1.8 m/s vendor ceiling, but it is called inside the rung loop — by then
    # the arm is connected, the mode is engaged and earlier rungs may already
    # have raised the limits, so a bad number would abort a run mid-flight
    # with the controller left reconfigured. Checked here, an impossible
    # request costs nothing and changes nothing.
    for v in rungs:
        if v <= 0:
            print("  [FAIL] speed %.3f m/s is not positive" % v)
            return 1
        if v > speed_limits.MAX_LINE_SPEED:
            print("  [FAIL] speed %.3f m/s is above the vendor maximum "
                  "%.3f m/s for line_speed. RealMan state this as a hard "
                  "ceiling; the controller would refuse it with a bare ret=1 "
                  "and the run would then proceed at whatever was already "
                  "configured, reading as \"the speed made no difference\"."
                  % (v, speed_limits.MAX_LINE_SPEED))
            return 1

    ip = LEFT_IP if side == "left" else RIGHT_IP
    total = sum(math.dist(poses[i][:3], poses[i + 1][:3])
                for i in range(len(poses) - 1))
    seg_mm = 1000 * total / max(1, len(poses) - 1)
    print("=" * 74)
    print("C19  blend / connect corner test   side=%s" % side)
    print("     path   %s" % os.path.relpath(src))
    print("     %d points, %d corners, %.2f m, mean segment %.0f mm"
          % (len(poses), len(poses) - 2, total, seg_mm))
    shown = [a for a in angles if a is not None]
    print("     corners %s   blends %s %%"
          % ("%s deg" % shown if shown else "%d (unlabelled)" % len(angles),
             path["blends"]))
    print("     speeds  %s m/s   (%s)"
          % (", ".join("%.2f" % v for v in rungs), ladder_src))
    print("     tool    %s" % (tool or "NOT DECLARED — elbow screen disabled"))
    print("=" * 74)

    # Wrap each Euler delta to the shortest arc before judging it: rz sits
    # at +-180 deg on the right arm, where a raw subtraction reads ~358 deg
    # for a move of a fraction of a degree.
    def _d(a, b):
        return abs((b - a + math.pi) % (2 * math.pi) - math.pi)
    rot = max((max(_d(poses[i][3 + k], poses[i + 1][3 + k])
                   for k in range(3))
               for i in range(len(poses) - 1)), default=0.0)
    if rot <= 1e-6:
        print("  orientation is CONSTANT on every segment, so the angular cap")
        print("  cannot time-scale anything (H67) — a dip IS the corner\n")
    else:
        print("  [WARN] this path CHANGES ORIENTATION (max %.2f deg on a "
              "segment). The angular cap can then time-scale a segment (H67), "
              "which is indistinguishable from a corner deceleration — the "
              "retained-speed column below is NOT a clean corner measurement. "
              "The recording and log are still valid.\n"
              % math.degrees(rot))

    left, right = connect_both()
    arm = left if side == "left" else right
    if arm is None:
        print("  [SKIP] hardware not reachable at %s" % ip)
        return 0
    limits_before = None
    originals = {}
    try:
        # THE MODE IS WHATEVER YOU ASKED FOR, and nothing else. `--mode SIM`
        # runs in simulation, `--mode REAL` runs on metal; this script never
        # switches between them on your behalf. `apply_run_mode` engages the
        # request and VERIFIES it by readback, aborting if the controller
        # refuses — a SIM request that silently stayed REAL would move real
        # metal. Run SIM first, then REAL if it completed, exactly as the
        # rest of the suite is driven.
        #
        # Set ONCE, above the ladder: the mode is a property of the session,
        # not of a rung, and re-engaging it per rung would be seven more
        # chances for a SIM request to land on metal.
        originals = apply_run_mode(forced, arm)
        if originals is None:
            print("  [FAIL] could not engage %s on %s — refusing to dispatch"
                  % (mode_label(forced), side))
            return 1
        _r, mode_now = arm.robot.rm_get_arm_run_mode()
        real = (forced == 1) or (forced is None and mode_now == 1)
        print("  run mode: %s%s" % (
            mode_label(forced) if forced is not None else
            "%s (as found — pass --mode to be explicit)" % mode_label(mode_now),
            "" if real else "   SIMULATION: nothing physical moves"))
        lim0 = speed_limits.read(arm.robot)
        print("  limits found:  line_speed %.3f  line_acc %.3f"
              % (lim0.get("line_speed", -1), lim0.get("line_acc", -1)))

        mon = ArrivalMonitor()
        mon.register(arm.robot)
        cases = [(b, True) for b in path["blends"]]
        if also_c0:
            cases.append((0, False))
        hdr = "  ".join("%5s deg" % ("?" if a is None else a) for a in angles)

        prev_meas = None        # (line_speed, worst joint fraction, joint no)
        for ri, rung in enumerate(rungs):
            speed, line_acc, notes = speed_limits.scale_for(rung, acc=req_acc)
            print("\n" + "=" * 74)
            print("  RUNG %d of %d   line_speed %.3f m/s   line_acc %.3f m/s2"
                  % (ri + 1, len(rungs), speed, line_acc))
            print("=" * 74)
            for n in notes:
                print("  [NOTE] %s" % n)

            # THE MEASURED GATE, which outranks the offline one. Joint rate
            # scales LINEARLY with commanded speed — measured on
            # test_motion_001, J4 peaked at 134 deg/s at 0.25 m/s and
            # 191 deg/s at 0.35 (ratio 1.43 against a speed ratio of 1.40) —
            # so the rung just completed predicts the next one directly, from
            # this arm on this path, with no model in the way.
            #
            # It is here because the OFFLINE screen was badly optimistic on a
            # rotating path: it called 0.25 m/s 35 % of J4's limit where the
            # arm measured 59 %, and 0.35 m/s 49 % where the arm measured
            # 85 % — a factor of 1.7. Left to the offline number alone this
            # ladder would have climbed to 0.80 m/s, which the measurement
            # puts at 194 % of J4. The 0.45 rung failing in SIM is the same
            # arithmetic landing at 109 %.
            if prev_meas is not None:
                pv, frac, jn = prev_meas
                proj = frac * speed / pv
                if proj > 1.0 and not allow_over:
                    print("  [STOP] the rung just completed measured J%d at "
                          "%.0f %% of its limit at %.2f m/s. Joint rate scales "
                          "linearly with speed, so THIS rung projects to "
                          "%.0f %% — measured on this arm, on this path, not "
                          "modelled.\n"
                          "         Not running it. Remaining: %s\n"
                          "         --allow-over-limit overrides, E-stop in "
                          "hand."
                          % (jn, 100 * frac, pv, 100 * proj,
                             ", ".join("%.2f" % v for v in rungs[ri:])))
                    break
                print("  measured last rung: J%d at %.0f %% of its limit at "
                      "%.2f m/s -> this rung projects to %.0f %%"
                      % (jn, 100 * frac, pv, 100 * proj))

            # THE PRE-FLIGHT ELBOW GATE — before the limits are raised, so a
            # refused rung never touches the controller's configuration.
            worst = preflight_j4(poses, tool, speed)
            if worst is None:
                print("  [WARN] the offline elbow screen could not run (no "
                      "TOOL_FRAME in the path file, an unknown frame, or a "
                      "failed transform self-check). This rung is UNSCREENED "
                      "— its joint demand is unknown, not known to be safe.")
            elif worst[0] > J4_LIMIT_PCT and not (allow_over or off_ladder):
                # THE HARD STOP IS FOR THE LADDER, and only for the ladder. A
                # ladder climbs on its own: nobody chose 0.80 for this path,
                # the list did, so it must not wander past what the elbow can
                # give. A `--speed` run is the opposite — an operator naming
                # one number for one run — so it is warned, loudly, not
                # refused. Both keep the same screen and the same numbers; only
                # who made the decision differs.
                print("  [STOP] segment %d needs %.0f %% of J4's limit at this "
                      "speed. The controller does NOT reliably refuse this: it "
                      "attempts it, and 20260813T183633 came back \"segment 5: "
                      "arrival event reports failure\" from exactly this "
                      "cause.\n"
                      "         Not running this rung, and not climbing "
                      "further — every rung above is worse. Remaining: %s\n"
                      "         Override with --allow-over-limit, or name the "
                      "speed directly with --speed, in either case with the "
                      "E-stop in hand."
                      % (worst[1], worst[0],
                         ", ".join("%.2f" % v for v in rungs[ri:])))
                break
            elif worst[0] > J4_LIMIT_PCT:
                print("  " + "!" * 70)
                print("  [WARN] OVER THE ELBOW LIMIT — segment %d needs %.0f %% "
                      "of J4's %.0f deg/s." % (worst[1], worst[0], 225.0))
                print("         Running anyway because you named this speed "
                      "explicitly (%s)."
                      % ("--speed" if off_ladder else "--allow-over-limit"))
                print("         The arm does NOT reliably stop itself: at "
                      "line_speed 0.80 it reversed four joints in 80 ms at "
                      "16.7 A and reported nothing on any channel.")
                print("         E-STOP IN HAND. Run SIM first.")
                print("  " + "!" * 70)
            else:
                print("  elbow: worst is segment %d at %.0f %% of the J4 limit "
                      "(exact — J4 is redundancy-invariant; the other six "
                      "joints need a saved plan and are NOT screened)"
                      % (worst[1], worst[0]))

            if (abs(speed_limits.read(arm.robot).get("line_speed", -1) - speed)
                    > 1e-6
                    or abs(speed_limits.read(arm.robot).get("line_acc", -1)
                           - line_acc) > 1e-6):
                # Set BOTH together. Raising only the speed is rejected with a
                # bare ret=1 when acc < 3x speed, and the run then silently
                # proceeds at whatever was already configured.
                prev = speed_limits.apply(
                    arm.robot, allow_raise=True,
                    line_speed=speed, line_acc=line_acc)
                # Capture only the FIRST rung's previous values — that is what
                # the arm had before this program touched it, and what the
                # restore at exit must return it to. Capturing per rung would
                # leave the arm at rung N-1's settings.
                if limits_before is None:
                    limits_before = prev
                print("  limits set:    line_speed %.3f  line_acc %.3f  "
                      "(originals restored at exit)" % (speed, line_acc))
            ramp = speed ** 2 / (2 * max(line_acc, 1e-6))
            print("  ramp %.0f mm each way against a %.0f mm mean segment "
                  "(%.1fx)"
                  % (1000 * ramp, seg_mm, seg_mm / max(1000 * ramp, 1e-9)))
            spacing_mm = 1000.0 * speed / 100.0        # 100 Hz UDP push
            print("  sample spacing %.2f mm at %.3f m/s — a corner needs >= %d "
                  "samples in its dip for the validated +-5 pt accuracy, so a "
                  "blend of at least ~%.0f mm"
                  % (spacing_mm, speed, MIN_CORNER_SAMPLES,
                     MIN_CORNER_SAMPLES * spacing_mm / 2))
            if 2 * ramp > 0.5 * (seg_mm / 1000.0):
                print("  [WARN] segments barely reach cruise — there is no "
                      "plateau to lose at a corner. Lengthen them in the path "
                      "file or lower the speed.")
            wide = [b for b in path["blends"] if b >= 40]
            if wide:
                print("  [NOTE] blend %s %% is >= 40 %% of a segment: the dips "
                      "from a segment's two ends MEET, so the approach never "
                      "returns to cruise and retention is measured against a "
                      "depressed reference. Those columns read ~12 points HIGH "
                      "(measured) — compare them with r=0, not with 100 %%."
                      % wide)
            print()
            print("%-14s %-10s %s" % ("case", "corner", hdr))
            print("-" * 74)

            rung_ok = True
            rung_frac = None                  # worst measured joint fraction
            jlim = (speed_limits.read(arm.robot).get("joint_speed")
                    or [180.0, 180.0] + [225.0] * 5)
            for blend, connect in cases:
                # BEFORE the recorder, every time. Putting this after the
                # analysis meant a rejected case skipped it and poisoned all
                # the cases that followed — see goto_start's docstring.
                if not goto_start(arm, poses, mon):
                    rung_ok = False
                    break
                # The run directory carries the RUNG in its name. Without it a
                # seven-rung sweep writes seven `blend_r25_left` directories
                # distinguishable only by timestamp, and the 2026-08-13 session
                # showed how easily the wrong one gets read.
                rec = RunRecorder(arm.robot,
                                  "blend_r%d%s_v%03d"
                                  % (blend, "" if connect else "_c0",
                                     round(speed * 1000)),
                                  side, host_ip_for(ip), UDP_PORT)
                if not rec.start():
                    print("  [WARN] recorder did not start; this case is not "
                          "introspectable afterwards")
                t0 = time.perf_counter()
                ok = run_one(arm, poses, blend, connect, mon)
                t1 = time.perf_counter()
                # Metadata in the same shape stage_runner writes, so a blend
                # run is introspectable with the same tooling as a task run.
                rec.meta.update({
                    "mode": mode_label(1 if real else 0),
                    "sim": not real,
                    "path_file": os.path.relpath(src),
                    "speeds": {"cleaning_pct": 100, "blend_pct": blend,
                               "connect": int(connect)},
                    "limits_in_force": speed_limits.read(arm.robot),
                    "line_speed_cap_m_s": speed,
                    "ladder": {"rung": ri + 1, "of": len(rungs),
                               "line_speed": speed, "line_acc": line_acc,
                               "all_rungs": rungs,
                               "predicted_j4_pct": None if worst is None
                               else round(worst[0], 1)},
                    "commanded": {
                        "tool_frame": tool, "blend_pct": blend,
                        "connect": int(connect),
                        "num_waypoints": len(poses),
                        "segments": len(poses) - 1,
                        "waypoint_names": labels,
                        "poses": poses,
                        "corner_angles_deg": angles,
                    },
                })
                run_dir = rec.stop()
                # WHAT THE JOINTS ACTUALLY DID, from the same recording. This
                # is the input to the gate on the next rung, so it is taken
                # from every case whether or not the case produced usable
                # corner numbers — a failed case is exactly the one whose
                # joint rates matter most.
                pk = read_joint_peaks(run_dir)
                if pk:
                    f, jn = max((pk[j] / jlim[j], j + 1) for j in range(7))
                    if rung_frac is None or f > rung_frac[0]:
                        rung_frac = (f, jn)
                    if f > 1.0:
                        print("  [WARN] J%d reached %.0f %% of its limit "
                              "(%.0f deg/s) during this case."
                              % (jn, 100 * f, pk[jn - 1]))
                if not ok:
                    rung_ok = False
                    continue
                # t_mono is zeroed when recording starts, so the motion window
                # is measured from the recorder's clock, not perf_counter's.
                tcp = read_tcp(run_dir)
                followed, rec_m, cmd_m = path_followed(tcp, poses)
                if not followed:
                    print("%-14s %s" % (
                        ("r=%d%%" % blend) if connect else "connect=0",
                        "NO RESULT — the tool traced %.3f m against a "
                        "commanded %.3f m (%.0f %%). %s" % (
                            rec_m, cmd_m, 100 * rec_m / max(cmd_m, 1e-9),
                            "It stopped short — check for the H45 stall."
                            if rec_m < cmd_m else
                            "The stream is not this path; corner numbers "
                            "would be fiction.")))
                    # A short trace is a STALL and must stop the climb; a long
                    # one is a bookkeeping fault in this harness and does not
                    # say anything about the arm's ability to take the next
                    # rung. Treating them alike would either hide a stall or
                    # abandon a sweep for no reason.
                    if rec_m < cmd_m:
                        rung_ok = False
                    continue
                res = corner_speeds(tcp, poses, 0.0, t1 - t0)
                label = ("r=%d%%" % blend) if connect else "connect=0"
                # A corner measured from too few samples is marked "?" — its
                # number would be unsupported, not merely imprecise.
                print("%-14s %-10s %s" % (label, "retained", "  ".join(
                    "     -   " if r is None else
                    ("%5.0f%%%s " % (100 * r[2],
                                     "?" if r[3] < MIN_CORNER_SAMPLES else " "))
                    for r in res)))
                print("%-14s %-10s %s" % ("", "v_min m/s", "  ".join(
                    "     -   " if r is None else "%6.3f   " % r[1]
                    for r in res)))
                # A declined corner is declined for one of TWO reasons and they
                # want opposite fixes, so say which. Reporting both as "too few
                # samples" cost a session: every corner of the 2026-08-13 runs
                # had 11-44 samples and was declined purely because the
                # approach never held a plateau.
                few = [i + 1 for i, r in enumerate(res)
                       if r is not None and r[3] < MIN_CORNER_SAMPLES
                       and r[4] < MIN_CORNER_SAMPLES]
                noflat = [i + 1 for i, r in enumerate(res)
                          if r is not None and r[3] < MIN_CORNER_SAMPLES
                          and r[4] >= MIN_CORNER_SAMPLES]
                if few:
                    print("%-14s %-10s corners %s: only %d-%d samples in the "
                          "dip (need %d) — lower the speed or widen the blend"
                          % ("", "", few,
                             min(res[i - 1][4] for i in few),
                             max(res[i - 1][4] for i in few),
                             MIN_CORNER_SAMPLES))
                if noflat:
                    print("%-14s %-10s corners %s: NO CRUISE PLATEAU on the "
                          "approach (speed varies %d-%d %% across the middle "
                          "of the segment) — the segment never settles, so "
                          "there is no cruise to retain. Lengthen it or lower "
                          "the speed."
                          % ("", "", noflat,
                             int(100 * min(res[i - 1][5] for i in noflat)),
                             int(100 * max(res[i - 1][5] for i in noflat))))

            if rung_frac is not None:
                prev_meas = (speed, rung_frac[0], rung_frac[1])
                print("  measured this rung: worst J%d at %.0f %% of its limit"
                      % (rung_frac[1], 100 * rung_frac[0]))
            elif real:
                print("  [WARN] no joint-speed telemetry in this rung's "
                      "recordings, so the next rung cannot be gated on "
                      "measurement — only on the offline screen, which has "
                      "run 1.7x optimistic on a rotating path.")

            if not rung_ok:
                # THE CLIMB STOPS HERE. Every rung above this one asks more of
                # the arm than the one that just failed, so "try the next one"
                # is never the right response to a stall or a failure event.
                print("\n  [STOP] rung %.3f m/s did not complete. Not climbing "
                      "further — rungs %s were NOT run."
                      % (speed, ", ".join("%.2f" % v for v in rungs[ri + 1:])
                         or "(none remaining)"))
                break

        print("\n  retained = corner minimum / cruise on the approach.")
        print("  A working blend holds most of its cruise; ~0 % is a full stop.")
        print("  DECISIVE: compare r=0 with the largest r. If they match, the")
        print("  blend radius is not being applied at all.")
    finally:
        restore_run_modes(originals)
        if limits_before:
            speed_limits.restore(arm.robot, limits_before)
            print("  limits restored to %s" % limits_before)
        teardown(left, right)
    return 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    raise SystemExit(main())
