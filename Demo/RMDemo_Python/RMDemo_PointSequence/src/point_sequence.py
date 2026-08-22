#!/usr/bin/env python3
"""Drive a list of points, in a given sequence, on one arm. Standalone.

Independent of the test bed: this file imports the RealMan SDK and PyYAML
and nothing else. Copy it anywhere with a points file and it runs.

    movej_p  -> the first point of the sequence
    movel    -> through the sequence, connect and blend applied per segment
    movej    -> rest

`--loops N` repeats the motion N times before the final return to rest.

THE POINTS FILE IS THE GENERATED CLEANING CONFIG, and it carries TWO UNIT
CONVENTIONS which are not the same:

    cartesian_poses:  [x, y, z, rx, ry, rz]  metres and RADIANS
                      (rm_pose_t, R = Rz.Ry.Rx), in arm_world
    cleaning_points:  translation metres, rotation DEGREES

Reading either in the other's units is silent — degrees-as-radians
re-orients the whole path, radians-as-degrees flattens it — so the two
defaults are separate (`pose_units`, `rotation_units`) and match the
generated files.

The cleaning points are DELTAS from `start_pose`:

    position     p = p_start + translation   <- in arm_world AXES, not
                                                rotated into p_start
    orientation  R = R_delta @ R_start       <- LEFT-multiplied,
                                                R_delta = Rx.Ry.Rz

    cartesian_poses:
      start_pose: [0.5828, -0.1012, -0.1021, 2.9145, 0.4291, -3.0507]
    cleaning_points:
      point14:
        translation: [0.0403, 0.0115, 0.0147]
        rotation: [-12.2, -10.2, 64.5]
    cleaning_sequence:
      - [point14, point15]

`movej_p` goes to `start_pose`; the movel chain follows the sequence. A
file with no `start_pose` is read as absolute poses instead.

THIS PROGRAM COMPOSES POSES, IT DOES NOT TRANSFORM FRAMES. A config whose
`reference_frame` is not the arm's own needs the URDF and the pole height;
that is RMDemo_CleaningMotion's job, and such a file is refused here rather
than resolved in the wrong frame.

THE SEQUENCE IS A LIST OF SEGMENTS, and consecutive segments need not
join. In the example above segment 1 ends at `point2` while segment 2
starts at `point3`, so there is a `point2 -> point3` move the sequence
never names. Those DISCONTINUITIES are found, reported with their distance
and traversed as ordinary moves — never silently, because an unannounced
straight line across the workspace is exactly the move you did not intend.

CONNECT AND BLEND. `--connect` and `--blend` set the default for every
move; a segment can override them with an optional third element:

      - [point1, point2, {r: 25, connect: 1, v: 40}]

    r        blend radius: a PERCENTAGE 0-100 of the shorter adjoining
             segment, NOT millimetres
    connect  1 = this move joins the next into one continuous trajectory
             0 = discrete; the arm comes to rest at the end of the move

Two rules are enforced because both fail silently otherwise:
  * the LAST move always closes the chain with r=0, connect=0 — a chain
    whose final move still says connect=1 never closes, and the program
    hangs waiting for a continuation that never comes;
  * connect=0 forces r=0 on that move, because a discrete move has
    nothing to blend into.

USAGE
    python3 point_sequence.py --points ../points/example_points.yaml --dry-run
    python3 point_sequence.py --points ../points/my.yaml --ip 192.168.1.18
    python3 point_sequence.py --points ../points/my.yaml --blend 20 --connect 1
    python3 point_sequence.py --points ../points/my.yaml --loops 5 --v 30
"""

import argparse
import math
import pathlib
import sys

import yaml

# The canonical SDK lives at the repository root, not in this folder.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))

from Robotic_Arm.rm_robot_interface import *          # noqa: E402,F403

# Rest pose per arm model, in degrees — same values the vendor demo uses.
# `--rest` overrides, and a `rest_pose:` in the points file overrides that.
arm_models_to_rest = {
    "RM_65":  [0, 0, 0, 0, 0, 0],
    "RM_75":  [0, 20, 0, 70, 0, 90, 0],
    "RML_63": [0, 20, 70, 0, 90, 0],
    "ECO_65": [0, 20, 70, 0, -90, 0],
    "ECO_63": [0, 20, 70, 0, -90, 0],
    "GEN_72": [0, 0, 0, -90, 0, 0, 0],
}


# ── rotation helpers (no numpy — this file stays standalone) ────────────
ARM_FRAMES = ("arm_world", "arm_base", "base", "base_link", "world", "arm")


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _pose_to_R(rx, ry, rz):
    """R = Rz(rz) Ry(ry) Rx(rx) — the CONTROLLER's pose convention.

    Used for `cartesian_poses`, which are rm_pose_t values in RADIANS.
    """
    return _mul(_Rz(rz), _mul(_Ry(ry), _Rx(rx)))


def _delta_to_R(r, p, y):
    """R = Rx(r) Ry(p) Rz(y) — the DELTA convention for cleaning points.

    NOTE THE OPPOSITE COMPOSITION ORDER to `_pose_to_R`. Both live here
    because the file genuinely uses both: a `cartesian_poses` entry is a
    controller pose, while a cleaning point's rotation is a delta the
    runtime builds as Rx*Ry*Rz and LEFT-multiplies onto the anchor.
    Collapsing them into one helper is the bug waiting to happen, and
    neither mistake fails loudly — the arm just cleans somewhere else.
    """
    return _mul(_Rx(r), _mul(_Ry(p), _Rz(y)))


def _R_to_pose(R):
    """R -> (rx, ry, rz) with R = Rz(rz) Ry(ry) Rx(rx). Inverse of
    `_pose_to_R`, so what goes to `rm_movel` is in the controller's own
    convention."""
    ry = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(R[2][0]) < 0.99999:
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    else:                                     # gimbal lock
        rx = math.atan2(-R[1][2], R[1][1])
        rz = 0.0
    return rx, ry, rz


# ── the points file ─────────────────────────────────────────────────────
def load_points(path):
    """(points, sequence, doc) — absolute arm-world poses, validated.

    TWO UNIT CONVENTIONS LIVE IN ONE FILE, and they are not the same:

        cartesian_poses:   [x, y, z, rx, ry, rz]  metres and RADIANS
                           (rm_pose_t, R = Rz.Ry.Rx)
        cleaning_points:   translation metres, rotation DEGREES

    That is the format as generated, and reading either one in the other's
    units is silent: degrees-as-radians re-orients the whole path, and
    radians-as-degrees flattens it. `rotation_units` / `pose_units`
    override, but the defaults match the generated files.

    RESOLUTION. When the file declares a `start_pose`, the cleaning points
    are DELTAS from it:

        position     p = p_start + translation   <- in arm_world AXES,
                                                    not rotated into p_start
        orientation  R = R_delta @ R_start       <- LEFT-multiplied

    With no `start_pose` the translations are read as absolute poses
    directly. The file says which it is; nothing is guessed.
    """
    p = pathlib.Path(path)
    if not p.is_file():
        raise SystemExit("no such points file: %s" % p)
    doc = yaml.safe_load(p.read_text()) or {}
    if not isinstance(doc, dict):
        raise SystemExit("%s: top level must be a mapping" % p)

    points = doc.get("cleaning_points") or doc.get("points")
    sequence = doc.get("cleaning_sequence") or doc.get("sequence")
    if not points:
        raise SystemExit("%s: no `cleaning_points` block" % p)
    if not sequence:
        raise SystemExit("%s: no `cleaning_sequence` block" % p)

    # This program composes poses; it does NOT transform between frames.
    # A config authored against a fixture frame needs the URDF and the pole
    # height, which is a different tool's job — say so instead of resolving
    # it in the wrong frame.
    tp = doc.get("task_parameters") or {}
    ref = (doc.get("reference_frame") or doc.get("ref_frame")
           or tp.get("reference_frame"))
    if ref and str(ref).lower() not in ARM_FRAMES:
        raise SystemExit(
            "%s declares reference_frame %r, which is not the arm's own "
            "frame.\n"
            "  Resolving it needs the URDF and the pole height, which this "
            "standalone program deliberately does not do.\n"
            "  Run it with the tool that does:\n"
            "    cd ../../RMDemo_CleaningMotion/src\n"
            "    python3 run_cleaning_motion.py --motion %s --dry-run"
            % (p, ref, p))

    rot_units = str(doc.get("rotation_units", "deg")).lower()
    if not rot_units.startswith(("deg", "rad")):
        raise SystemExit("%s: rotation_units must be deg or rad, got %r"
                         % (p, rot_units))
    to_rad = math.pi / 180.0 if rot_units.startswith("deg") else 1.0

    # ── the anchor ──
    anchor = (doc.get("start_pose")
              or (doc.get("cartesian_poses") or {}).get("start_pose"))
    if anchor is not None:
        vals = [float(v) for v in anchor]
        if len(vals) != 6:
            raise SystemExit(
                "%s: start_pose has %d values; this program wants 6, "
                "[x y z rx ry rz] in the arm's own frame." % (p, len(vals)))
        pu = str(doc.get("pose_units", "rad")).lower()
        if not pu.startswith(("deg", "rad")):
            raise SystemExit("%s: pose_units must be deg or rad" % p)
        # RADIANS by default, and deliberately NOT governed by
        # `rotation_units`: that key describes the point DELTAS, which the
        # generated files write in degrees, while cartesian_poses are
        # rm_pose_t values in radians.
        ps = math.pi / 180.0 if pu.startswith("deg") else 1.0
        p0 = vals[:3]
        R0 = _pose_to_R(vals[3] * ps, vals[4] * ps, vals[5] * ps)
    else:
        p0, R0 = None, None

    resolved = {}
    for name, body in points.items():
        if not isinstance(body, dict):
            raise SystemExit("%s: point %r must be a mapping with "
                             "`translation` and `rotation`" % (p, name))
        t, r = body.get("translation"), body.get("rotation")
        if t is None or r is None:
            raise SystemExit("%s: point %r needs both `translation` and "
                             "`rotation`" % (p, name))
        if len(t) != 3 or len(r) != 3:
            raise SystemExit("%s: point %r wants 3 values each, got %d and %d"
                             % (p, name, len(t), len(r)))
        t = [float(v) for v in t]
        r = [float(v) * to_rad for v in r]
        if not all(math.isfinite(v) for v in t + r):
            raise SystemExit("%s: point %r has a non-finite value" % (p, name))
        if p0 is None:
            resolved[name] = t + r                      # absolute already
        else:
            R = _mul(_delta_to_R(r[0], r[1], r[2]), R0)  # LEFT-multiplied
            rx, ry, rz = _R_to_pose(R)
            resolved[name] = [p0[0] + t[0], p0[1] + t[1], p0[2] + t[2],
                              rx, ry, rz]

    if p0 is not None:
        # The anchor is a waypoint in its own right: `movej_p` goes there
        # first, and it is usually NOT one of the cleaning points.
        rx, ry, rz = _R_to_pose(R0)
        resolved.setdefault("start_pose", p0 + [rx, ry, rz])

    far = max(max(abs(v) for v in q[:3]) for q in resolved.values())
    if far > 5.0:
        print("  [WARN] largest |coordinate| is %.1f. Translations are "
              "METRES here — a file written in millimetres would be 1000x "
              "out." % far)
    return resolved, sequence, doc


def build_traversal(points, sequence):
    """[(name, from_jump)] — the waypoints to visit, in order.

    The first entry is the `movej_p` target; every entry after it is a
    `movel` target. `from_jump` marks a waypoint reached across a
    discontinuity, i.e. one the sequence implies but never writes down.
    """
    traversal, prev_end = [], None
    for i, seg in enumerate(sequence):
        if not isinstance(seg, (list, tuple)) or len(seg) < 2:
            raise SystemExit("sequence entry %d is %r; expected "
                             "[from, to] with an optional third {params} "
                             "element" % (i, seg))
        a, b = seg[0], seg[1]
        for n in (a, b):
            if n not in points:
                raise SystemExit("sequence entry %d names %r, which is not "
                                 "in cleaning_points. Known: %s"
                                 % (i, n, ", ".join(points)))
        if prev_end is None:
            traversal.append((a, False))
        elif a != prev_end:
            traversal.append((a, True))          # discontinuity
        traversal.append((b, False))
        prev_end = b
    return traversal


def segment_params(sequence):
    """Per-segment overrides, indexed by the MOVE they belong to.

    Returns {move_index: {...}}. A segment's params attach to the move that
    lands on its `.second`, which is the move the segment describes.
    """
    params, tlen, prev_end = {}, 0, None
    for seg in sequence:
        a, b = seg[0], seg[1]
        if prev_end is None or a != prev_end:
            tlen += 1                # `a` was appended (start, or a jump)
        tlen += 1                    # `b` was appended
        # `b` sits at traversal index tlen-1, and traversal[0] is the
        # movej_p target, so the movel that lands on it is index tlen-2.
        if len(seg) > 2 and isinstance(seg[2], dict):
            params[tlen - 2] = seg[2]
        prev_end = b
    return params


def build_program(n_moves, v=20, r=0, connect=1, overrides=None):
    """[(v, r, connect)] per move, built once and printed once.

    Rules: the last move closes the chain (r=0, connect=0); connect=0
    forces r=0, because a discrete move has nothing to blend into.
    """
    overrides = overrides or {}
    prog = []
    for i in range(n_moves):
        o = overrides.get(i, {})
        mv = int(o.get("v", v))
        mr = int(o.get("r", o.get("blend", r)))
        mc = int(o.get("connect", connect))
        last = (i == n_moves - 1)
        if last:
            mr, mc = 0, 0
        elif not mc:
            mr = 0
        prog.append((mv, mr, mc))
    return prog


# ── the arm ─────────────────────────────────────────────────────────────
class RobotArmController:
    def __init__(self, ip, port=8080, level=3, mode=2):
        self.thread_mode = rm_thread_mode_e(mode)              # noqa: F405
        self.robot = RoboticArm(self.thread_mode)              # noqa: F405
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)
        if self.handle.id == -1:
            print("\nFailed to connect to the robot arm\n")
            sys.exit(1)
        print("\nSuccessfully connected to the robot arm: %d\n"
              % self.handle.id)

    def get_arm_model(self):
        res, model = self.robot.rm_get_robot_info()
        if res == 0:
            return model["arm_model"]
        print("\nFailed to get robot arm model\n")
        return None

    def disconnect(self):
        h = self.robot.rm_delete_robot_arm()
        print("\nSuccessfully disconnected from the robot arm\n" if h == 0
              else "\nFailed to disconnect from the robot arm\n")

    def movej(self, joint, v=20, r=0, connect=0, block=1):
        ret = self.robot.rm_movej(joint, v, r, connect, block)
        print("movej succeeded" if ret == 0
              else "movej FAILED, error code: %s" % ret)
        return ret == 0

    def movej_p(self, pose, v=20, r=0, connect=0, block=1):
        ret = self.robot.rm_movej_p(pose, v, r, connect, block)
        print("movej_p succeeded" if ret == 0
              else "movej_p FAILED, error code: %s" % ret)
        return ret == 0

    def movel(self, pose, v=20, r=0, connect=0, block=1):
        ret = self.robot.rm_movel(pose, v, r, connect, block)
        if ret != 0:
            print("movel FAILED, error code: %s" % ret)
        return ret == 0

    def run_sequence(self, poses, program, names=None, block=1):
        """movel through `poses`, one (v, r, connect) per pose."""
        for i, pose in enumerate(poses):
            v, r, c = program[i]
            label = names[i] if names else "move %d" % i
            ret = self.robot.rm_movel(list(pose), v, r, c, block)
            if ret != 0:
                print("\nmovel FAILED at %s (v=%d r=%d connect=%d), "
                      "error code: %s\n" % (label, v, r, c, ret))
                if ret == 1 and r:
                    print("  a bare ret=1 on a blended move often means the "
                          "blend could not carry the speed step into that "
                          "corner — lower r, or lower the speed.\n")
                return False
        print("\nsequence succeeded\n")
        return True


# ── reporting ───────────────────────────────────────────────────────────
def describe(traversal, poses, program, names):
    out = ["points   %d in the traversal (%d movel moves after the movej_p)"
           % (len(traversal), len(traversal) - 1)]
    jumps = [(i, n) for i, (n, j) in enumerate(traversal) if j]
    if jumps:
        out.append("")
        out.append("  DISCONTINUITIES — the sequence does not join here, so "
                   "these moves are implied,")
        out.append("  not written down. Each is an ordinary straight line "
                   "the arm will travel:")
        for i, n in jumps:
            d = math.dist(poses[i - 1][:3], poses[i][:3])
            out.append("    %-10s <- %-10s   %.1f mm"
                       % (n, traversal[i - 1][0], 1000 * d))
    out.append("")
    out.append("  %-4s %-12s %-34s %s" % ("#", "point", "pose [m, rad]",
                                          "v /  r / connect"))
    for i, (name, jump) in enumerate(traversal):
        q = poses[i]
        pose = ("%8.4f %8.4f %8.4f  %7.3f %7.3f %7.3f"
                % (q[0], q[1], q[2], q[3], q[4], q[5]))
        if i == 0:
            tag = "movej_p"
        else:
            v, r, c = program[i - 1]
            tag = "%3d / %2d / %d" % (v, r, c)
        out.append("  %-4s %-12s %s   %s%s"
                   % (i, name, pose, tag, "   <- JUMP" if jump else ""))
    return "\n".join(out)


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__.split("USAGE")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="USAGE" + __doc__.split("USAGE")[1])
    ap.add_argument("--points", required=True, help="YAML points file")
    ap.add_argument("--ip", default="192.168.1.18", help="arm IP")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--v", type=int, default=20,
                    help="speed %%, all moves (default 20)")
    ap.add_argument("--blend", "-r", type=int, default=0,
                    help="blend radius %%%% 0-100, default for every move")
    ap.add_argument("--connect", type=int, default=1, choices=(0, 1),
                    help="1 = chain the moves, 0 = discrete (default 1)")
    ap.add_argument("--block", type=int, default=1, choices=(0, 1),
                    help="1 = blocking SDK calls (default 1)")
    ap.add_argument("--loops", type=int, default=1,
                    help="repeat the motion N times (default 1)")
    ap.add_argument("--rest", default=None,
                    help="rest joint pose, comma-separated degrees")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and print; never touch the arm")
    args = ap.parse_args()
    if not 0 <= args.blend <= 100:
        ap.error("--blend is a PERCENTAGE 0-100 (not mm), got %d" % args.blend)
    if not 1 <= args.v <= 100:
        ap.error("--v is a percentage 1-100, got %d" % args.v)
    if args.loops < 1:
        ap.error("--loops must be at least 1, got %d" % args.loops)
    return args


def main():
    args = parse_args()

    points, sequence, doc = load_points(args.points)
    traversal = build_traversal(points, sequence)
    # THE ANCHOR IS THE movej_p TARGET. `start_pose` is usually NOT one of
    # the cleaning points — in the generated files no point has a zero
    # delta — so it is prepended as waypoint 0 and the move onto the first
    # stroke is a real movel. Where it DOES coincide with the first point,
    # the duplicate is dropped rather than commanded as a zero-length move.
    if "start_pose" in points and traversal and traversal[0][0] != "start_pose":
        if math.dist(points["start_pose"][:3],
                     points[traversal[0][0]][:3]) > 1e-6:
            traversal.insert(0, ("start_pose", False))
        else:
            traversal[0] = ("start_pose", traversal[0][1])
    poses = [points[n] for n, _ in traversal]
    names = [n for n, _ in traversal]
    program = build_program(len(poses) - 1, v=args.v, r=args.blend,
                            connect=args.connect,
                            overrides=segment_params(sequence))

    print()
    print(describe(traversal, poses, program, names))
    print()
    if args.blend and not args.connect:
        print("  [NOTE] --blend %d with --connect 0: a discrete move cannot "
              "blend into anything, so r is 0 on every move.\n" % args.blend)

    if args.dry_run:
        print("dry run — the arm was never contacted.\n")
        return 0

    controller = RobotArmController(args.ip, args.port)
    print("API Version: ", rm_api_version(), "\n")        # noqa: F405

    if args.rest:
        rest = [float(x) for x in args.rest.split(",")]
    elif doc.get("rest_pose"):
        rest = [float(x) for x in doc["rest_pose"]]
    else:
        rest = arm_models_to_rest.get(controller.get_arm_model())
    if not rest:
        controller.disconnect()
        raise SystemExit(
            "no rest pose: the arm model is not in `arm_models_to_rest`. "
            "Pass --rest, or add a `rest_pose:` to the points file.")

    ok = True
    for loop in range(args.loops):
        print("--- loop %d/%d ---" % (loop + 1, args.loops))
        if not controller.movej_p(poses[0], v=args.v, block=args.block):
            ok = False
            break
        if not controller.run_sequence(poses[1:], program, names[1:],
                                       block=args.block):
            ok = False
            break

    # Rest is attempted even after a failure: an arm left mid-path is worse
    # than one parked, and the failure is already reported above.
    controller.movej(rest, v=args.v, block=args.block)
    controller.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
