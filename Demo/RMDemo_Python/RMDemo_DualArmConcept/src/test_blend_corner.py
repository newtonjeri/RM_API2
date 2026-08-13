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

SPEED. `--speed` scales BOTH limits together through
`speed_limits.scale_for`, which enforces the vendor rules: max line_speed
1.8 m/s, and line_acc >= 3 x line_speed. Raising only the speed is rejected
by the controller with a bare ret=1, after which the run proceeds at
whatever was already configured and reads as "the speed made no
difference". Whatever it sets is RESTORED at exit — these limits ratchet.

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
    python3 test_blend_corner.py --side left --mode REAL --speed 0.45
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
                      "TCP_LINEAR_VELOCITY", "SEGMENT_SPEEDS", "BLEND"):
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
    return {
        "poses": poses, "labels": seq,
        "blends": list(got.get("BLEND_SWEEP", [got.get("BLEND", 25)])),
        "angles": list(got.get("CORNER_ANGLES", [])),
        "speed": got.get("TCP_LINEAR_VELOCITY", 0.25),
        "seg_speeds": got.get("SEGMENT_SPEEDS", {}),
    }


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
    return abs(rec - cmd) / cmd <= tol, rec, cmd


def corner_speeds(rows, poses, t_start, t_end):
    """(cruise, v_min, retained) per interior corner, from the TCP stream.

    A corner is located by ARC LENGTH, not by time: the arm's speed is what
    we are measuring, so using time to find the corner would assume the
    answer. Cruise is the median speed over the middle half of the incoming
    segment; the corner window is +-15 % of a segment either side of the
    vertex.
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
    spd = [0.0]
    for i in range(1, len(xs)):
        dt = xs[i][0] - xs[i - 1][0]
        spd.append(math.dist(xs[i][1:4], xs[i - 1][1:4]) / dt if dt > 1e-6
                   else 0.0)
    seg = [math.dist(poses[i][:3], poses[i + 1][:3])
           for i in range(len(poses) - 1)]
    vert, acc = [], 0.0
    for s in seg[:-1]:
        acc += s
        vert.append(acc)
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
    straddle = set()
    for v in vert:
        for i in range(1, len(cum)):
            if cum[i - 1] <= v <= cum[i]:
                straddle.update((i - 1, i, i + 1))
                break

    out = []
    for k, v in enumerate(vert):
        # +-30 % of the segment. `near` feeds a MINIMUM, so widening it can
        # only add cruise samples that cannot lower the min — it costs
        # nothing and buys resolution. At +-15 % a 65 mm segment gave ~10
        # samples, and removing 3 for the straddle left 7, below
        # MIN_CORNER_SAMPLES: a perfectly flat run was being declined for
        # want of samples. The next vertex is a full segment away, so 30 %
        # cannot reach it.
        win = 0.30 * seg[k]
        near = [spd[i] for i in range(len(cum))
                if abs(cum[i] - v) <= win and i not in straddle]
        mid = [spd[i] for i in range(len(cum))
               if v - 0.75 * seg[k] <= cum[i] <= v - 0.25 * seg[k]
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
        n_ok = len(near) if flat <= 0.20 else 0
        # `n` is how many samples the corner was measured from. Accuracy
        # tracks it directly — validated against synthetic streams holding a
        # KNOWN retention: 20 samples -> 2 pt error, 8 -> 5 pt, 4 -> 13 pt,
        # 2 -> 20 pt. Reported so a thin corner is visibly thin rather than
        # quietly wrong.
        out.append((cruise, vmin, vmin / cruise if cruise > 1e-6 else 0.0,
                    n_ok))
    return out


MIN_CORNER_SAMPLES = 8      # >= 8 kept the validated error under ~5 points


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
    handle_cli(__doc__, extra_flags=("--connect0",),
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
    speed = path["speed"]
    if "--speed" in sys.argv:                 # override the file, deliberately
        speed = float(sys.argv[sys.argv.index("--speed") + 1])
    angles = path["angles"] or [None] * (len(poses) - 2)
    req_acc = None
    if "--line-acc" in sys.argv:
        req_acc = float(sys.argv[sys.argv.index("--line-acc") + 1])
    speed, line_acc, notes = speed_limits.scale_for(speed, acc=req_acc)

    ip = LEFT_IP if side == "left" else RIGHT_IP
    total = sum(math.dist(poses[i][:3], poses[i + 1][:3])
                for i in range(len(poses) - 1))
    seg_mm = 1000 * total / max(1, len(poses) - 1)
    print("=" * 74)
    print("C19  blend / connect corner test   side=%s  speed=%.3f m/s" % (side, speed))
    print("     path   %s" % os.path.relpath(src))
    print("     %d points, %d corners, %.2f m, mean segment %.0f mm"
          % (len(poses), len(poses) - 2, total, seg_mm))
    shown = [a for a in angles if a is not None]
    print("     corners %s   blends %s %%"
          % ("%s deg" % shown if shown else "%d (unlabelled)" % len(angles),
             path["blends"]))
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
        for n in notes:
            print("  [NOTE] %s" % n)
        lim = speed_limits.read(arm.robot)
        print("  limits found:  line_speed %.3f  line_acc %.3f"
              % (lim.get("line_speed", -1), lim.get("line_acc", -1)))
        if (abs(lim.get("line_speed", -1) - speed) > 1e-6
                or abs(lim.get("line_acc", -1) - line_acc) > 1e-6):
            # Set BOTH together. Raising only the speed is rejected with a
            # bare ret=1 when acc < 3x speed, and the run then silently
            # proceeds at whatever was already configured.
            limits_before = speed_limits.apply(
                arm.robot, allow_raise=True,
                line_speed=speed, line_acc=line_acc)
            print("  limits set:    line_speed %.3f  line_acc %.3f  "
                  "(restored at exit)" % (speed, line_acc))
        ramp = speed ** 2 / (2 * max(line_acc, 1e-6))
        print("  ramp %.0f mm each way against a %.0f mm mean segment (%.1fx)"
              % (1000 * ramp, seg_mm, seg_mm / max(1000 * ramp, 1e-9)))
        spacing_mm = 1000.0 * speed / 100.0        # 100 Hz UDP push
        print("  sample spacing %.2f mm at %.3f m/s — a corner needs >= %d "
              "samples in its dip for the validated +-5 pt accuracy, so a "
              "blend of at least ~%.0f mm"
              % (spacing_mm, speed, MIN_CORNER_SAMPLES,
                 MIN_CORNER_SAMPLES * spacing_mm / 2))
        if 2 * ramp > 0.5 * (seg_mm / 1000.0):
            print("  [WARN] segments barely reach cruise — there is no plateau "
                  "to lose at a corner. Lengthen them in the path file or "
                  "lower the speed.")

        # THE MODE IS WHATEVER YOU ASKED FOR, and nothing else. `--mode SIM`
        # runs in simulation, `--mode REAL` runs on metal; this script never
        # switches between them on your behalf. `apply_run_mode` engages the
        # request and VERIFIES it by readback, aborting if the controller
        # refuses — a SIM request that silently stayed REAL would move real
        # metal. Run SIM first, then REAL if it completed, exactly as the
        # rest of the suite is driven.
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
        print()

        mon = ArrivalMonitor()
        mon.register(arm.robot)
        cases = [(b, True) for b in path["blends"]]
        if also_c0:
            cases.append((0, False))
        hdr = "  ".join("%5s deg" % ("?" if a is None else a) for a in angles)
        print("%-14s %-10s %s" % ("case", "corner", hdr))
        print("-" * 74)
        for blend, connect in cases:
            rec = RunRecorder(arm.robot,
                              "blend_r%d%s" % (blend, "" if connect else "_c0"),
                              side, host_ip_for(ip), UDP_PORT)
            if not rec.start():
                print("  [WARN] recorder did not start; this case is not "
                      "introspectable afterwards")
            t0 = time.perf_counter()
            ok = run_one(arm, poses, blend, connect, mon)
            t1 = time.perf_counter()
            # Metadata in the same shape stage_runner writes, so a blend run
            # is introspectable with the same tooling as a task run.
            rec.meta.update({
                "mode": mode_label(1 if real else 0),
                "sim": not real,
                "path_file": os.path.relpath(src),
                "speeds": {"cleaning_pct": 100, "blend_pct": blend,
                           "connect": int(connect)},
                "limits_in_force": speed_limits.read(arm.robot),
                "line_speed_cap_m_s": speed,
                "commanded": {
                    "tool_frame": None, "blend_pct": blend,
                    "connect": int(connect),
                    "num_waypoints": len(poses),
                    "segments": len(poses) - 1,
                    "waypoint_names": labels,
                    "poses": poses,
                    "corner_angles_deg": angles,
                },
            })
            run_dir = rec.stop()
            if not ok:
                continue
            # t_mono is zeroed when recording starts, so the motion window is
            # measured from the recorder's clock, not perf_counter's.
            tcp = read_tcp(run_dir)
            followed, rec_m, cmd_m = path_followed(tcp, poses)
            if not followed:
                print("%-14s %s" % (
                    ("r=%d%%" % blend) if connect else "connect=0",
                    "NO RESULT — the tool traced %.3f m against a commanded "
                    "%.3f m (%.0f %%). %s" % (
                        rec_m, cmd_m, 100 * rec_m / max(cmd_m, 1e-9),
                        "It stopped short — check for the H45 stall."
                        if rec_m < cmd_m else
                        "The stream is not this path; corner numbers would be "
                        "fiction.")))
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
                "     -   " if r is None else "%6.3f   " % r[1] for r in res)))
            thin = [i + 1 for i, r in enumerate(res)
                    if r is not None and r[3] < MIN_CORNER_SAMPLES]
            if thin:
                print("%-14s %-10s corners %s measured from < %d samples — "
                      "treat as indicative only"
                      % ("", "", thin, MIN_CORNER_SAMPLES))
            # Back to the start so every case runs the identical geometry from
            # the identical configuration — and CHECK it got there. A silent
            # failure here leaves every later case starting from wherever the
            # last one ended, which is a different arm posture and therefore a
            # different measurement; the emulator run showed exactly that,
            # with cases 2-4 tracing 0.000 m.
            mon.expect(arm.handle_id, DEV_JOINT)
            rret = arm.robot.rm_movel(poses[0], 30, 0, 0, 0)
            rarr, rok = mon.wait(arm.handle_id, DEV_JOINT, 90.0)
            if rret != 0 or not rarr or not rok:
                print("  [FAIL] could not return to the start pose "
                      "(ret=%s arrived=%s ok=%s). Later cases would run from "
                      "a different configuration, so stopping here."
                      % (rret, rarr, rok))
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
