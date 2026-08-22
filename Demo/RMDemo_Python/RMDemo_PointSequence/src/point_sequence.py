#!/usr/bin/env python3
"""Drive a list of points, in a given sequence, on one arm. Standalone.

Independent of the test bed: this file imports the RealMan SDK and PyYAML
and nothing else. Copy it anywhere with a points file and it runs.

    movej    -> rest          establishes the arm configuration
    movej_p  -> start_pose
    movel    -> through the sequence   (v, blend radius, connect)
    movej    -> rest          parks

The run OPENS at rest as well as closing there. `movej_p` names a pose, not
a configuration, and a 7-axis arm reaches one pose from many — which branch
it picks depends on where it started. Beginning from a known joint pose is
what makes the whole path repeatable instead of dependent on wherever the
previous run happened to stop.

POINTS ARE ABSOLUTE poses in the arm's own frame. `translation` is the
position, `rotation` is the orientation; nothing is added to `start_pose`
and nothing is composed onto it. The values are dispatched as given.

A POINT'S ROTATION IS INTRINSIC XYZ, R = Rx.Ry.Rz — the convention
`task_base.cpp` applies and the one the generator writes. The controller
wants `rm_pose_t`, which is EXTRINSIC xyz, R = Rz.Ry.Rx. Those are opposite
compositions, so converting is a re-decomposition and NOT a reordering of
the triple: build the matrix, then read the other triple off it. Passing
the angles through unconverted is 155 deg out on a typical cleaning point,
and it looks like plausible angles the whole way.

TWO UNIT CONVENTIONS live in one file and they are not the same:

    cartesian_poses:  [x, y, z, rx, ry, rz]  metres and RADIANS (rm_pose_t)
    cleaning_points:  translation metres,    rotation DEGREES

Reading either in the other's units is silent — degrees-as-radians
re-orients the whole path, radians-as-degrees flattens it. `pose_units` and
`rotation_units` override them, and when a file declares neither the
assumption is PRINTED rather than made quietly.

THE TOOL FRAME IS NOT SET HERE. The poses mean nothing without it, and the
controller uses whatever tool is currently selected on the pendant. The
file's `ik_frame` is printed for that reason; select it before running.

USAGE
    python3 point_sequence.py --points ../points/example_points_4.yaml --dry-run
    python3 point_sequence.py --points ../points/my.yaml --ip 192.168.1.18
    python3 point_sequence.py --points ../points/my.yaml --v 30 --blend 20
    python3 point_sequence.py --points ../points/my.yaml --v 30,30,10,10 \
                              --blend 0,20,20,0 --connect 1,1,1,0
"""

import argparse
import math
import pathlib
import sys

import yaml

# The canonical SDK lives at the repository root, not in this folder.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))

from Robotic_Arm.rm_robot_interface import *          # noqa: E402,F403

# Rest pose, in DEGREES, from the SRDF this fleet actually plans against:
# alix_moveit_config/config/alix.srdf, group_state name="rest_pose".
# The vendor demo's [0, 20, 0, 70, 0, 90, 0] is a different pose entirely —
# 102 deg away at joint2 — so parking with it leaves the arm somewhere the
# rest of the stack does not expect.
# The two arms are NOT mirror images in every joint: 3 and 5 flip sign and
# joint 7 differs outright, so they are written out rather than negated.
REST_POSE_DEG = {
    "right": [0.0, -81.9903, -26.0008, 97.9930, 5.9989, 61.9998, 69.0013],
    "left":  [0.0, -81.9903, 26.0008, 97.9930, -5.9989, 61.9998, 110.9995],
}

ARM_FRAMES = ("arm_world", "arm_base", "base", "base_link", "world", "arm")


# ── rotation (no numpy — this file stays standalone) ─────────────────────
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


def _intrinsic_xyz_to_pose(rx, ry, rz):
    """A point's rotation (intrinsic XYZ, R = Rx.Ry.Rz) -> `rm_pose_t`
    (extrinsic xyz, R = Rz.Ry.Rx). Radians in, radians out.

    Opposite compositions, so this is a re-decomposition and not a reorder.
    Checked against `alix_dispatch` on the toplid chain: with it all 25
    moves agree to 7e-5 deg, without it they are 158 deg out.

    `start_pose` does NOT come through here — it is already `rm_pose_t`.
    """
    m = _mul(_Rx(rx), _mul(_Ry(ry), _Rz(rz)))
    out_ry = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(m[2][0]) < 0.99999:
        return math.atan2(m[2][1], m[2][2]), out_ry, math.atan2(m[1][0], m[0][0])
    return math.atan2(-m[1][2], m[1][1]), out_ry, 0.0          # gimbal lock


def load_points(path):
    """(points, sequence, doc, notes) — absolute poses in the arm's frame."""
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

    # This program dispatches poses in the arm's own frame; it does NOT
    # transform between frames. A config authored against a fixture frame
    # needs the URDF and the pole height, which is a different tool's job.
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

    # UNITS. Assuming is fine; assuming SILENTLY is not — the two mistakes
    # this guards against both look like a working run that cleans the wrong
    # place, so the assumption is reported with the numbers it produced.
    notes = []
    rot_units = str(doc.get("rotation_units", "deg")).lower()
    pose_units = str(doc.get("pose_units", "rad")).lower()
    for key, value in (("rotation_units", rot_units), ("pose_units", pose_units)):
        if not value.startswith(("deg", "rad")):
            raise SystemExit("%s: %s must be deg or rad, got %r"
                             % (p, key, value))
        if key not in doc:
            notes.append("%s not declared — assuming %s"
                         % (key, "DEGREES" if value.startswith("deg") else "RADIANS"))
    to_rad = math.pi / 180.0 if rot_units.startswith("deg") else 1.0
    pose_scale = math.pi / 180.0 if pose_units.startswith("deg") else 1.0

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
        q = [float(v) for v in t] + list(_intrinsic_xyz_to_pose(
            *(float(v) * to_rad for v in r)))
        if not all(math.isfinite(v) for v in q):
            raise SystemExit("%s: point %r has a non-finite value" % (p, name))
        resolved[name] = q

    anchor = (doc.get("start_pose")
              or (doc.get("cartesian_poses") or {}).get("start_pose"))
    if anchor is not None:
        vals = [float(v) for v in anchor]
        if len(vals) != 6:
            raise SystemExit(
                "%s: start_pose has %d values; this program wants 6, "
                "[x y z rx ry rz] in the arm's own frame." % (p, len(vals)))
        resolved.setdefault("start_pose",
                            vals[:3] + [v * pose_scale for v in vals[3:]])

    far = max(max(abs(v) for v in q[:3]) for q in resolved.values())
    if far > 5.0:
        notes.append("largest |coordinate| is %.1f — translations are METRES "
                     "here, so a file written in millimetres is 1000x out" % far)
    return resolved, sequence, doc, notes


def traversal(points, sequence):
    """The waypoint names to visit, in order.

    The first is the `movej_p` target; the rest are `movel` targets.

    Consecutive segments need not join: where one ends at `point2` and the
    next starts at `point3` there is a `point2 -> point3` move the sequence
    never writes down. It is traversed as an ordinary move and reported,
    never taken silently — an unannounced straight line across the
    workspace is exactly the move you did not intend.
    """
    out, prev_end = [], None
    for i, seg in enumerate(sequence):
        if not isinstance(seg, (list, tuple)) or len(seg) < 2:
            raise SystemExit("sequence entry %d is %r; expected [from, to]"
                             % (i, seg))
        a, b = seg[0], seg[1]
        for n in (a, b):
            if n not in points:
                raise SystemExit("sequence entry %d names %r, which is not in "
                                 "cleaning_points. Known: %s"
                                 % (i, n, ", ".join(points)))
        if prev_end is None or a != prev_end:
            out.append(a)
        out.append(b)
        prev_end = b

    # The anchor is the movej_p target. In the generated files the first
    # cleaning point sits exactly ON start_pose, and commanding both would
    # be a zero-length move — so that ONE duplicate is dropped. Checked
    # only against the sequence's first point, which is the only place it
    # can occur.
    if "start_pose" in points and out and out[0] != "start_pose":
        if math.dist(points["start_pose"][:3], points[out[0]][:3]) > 1e-6:
            out.insert(0, "start_pose")
        else:
            out[0] = "start_pose"
    return out


def per_move(text, n_moves, name, low, high):
    """One value for every movel, from `--v 20` or `--v 20,20,30,...`.

    A single number applies to all of them; a list gives each move its own.
    The length must be exactly 1 or `n_moves` — accepting a short list and
    padding it would silently run the tail of the path at a speed nobody
    chose, which is the failure this refuses rather than guesses through.
    """
    try:
        vals = [int(x) for x in str(text).replace(" ", "").split(",") if x != ""]
    except ValueError:
        raise SystemExit("%s: %r is not a number or a comma-separated list"
                         % (name, text))
    if not vals:
        raise SystemExit("%s: no value given" % name)
    for v in vals:
        if not low <= v <= high:
            raise SystemExit("%s must be %d-%d, got %d" % (name, low, high, v))
    if len(vals) == 1:
        return vals * n_moves
    if len(vals) != n_moves:
        raise SystemExit("%s has %d values but there are %d movel moves — "
                         "give one value or exactly %d"
                         % (name, len(vals), n_moves, n_moves))
    return vals


def build_program(n_moves, v, r, connect):
    """[(v, r, connect)] per move, from three per-move lists.

    Two rules, because both fail silently otherwise: the LAST move closes
    the chain (r=0, connect=0) — one that still says connect=1 never closes
    and the program waits for a continuation that never comes — and
    connect=0 forces r=0, because a discrete move has nothing to blend into.
    """
    prog = []
    for i in range(n_moves):
        if i == n_moves - 1:
            prog.append((v[i], 0, 0))
        else:
            prog.append((v[i], r[i] if connect[i] else 0, connect[i]))
    return prog


class RobotArmController:
    def __init__(self, ip, port=8080, level=3, mode=2):
        self.robot = RoboticArm(rm_thread_mode_e(mode))        # noqa: F405
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)
        if self.handle.id == -1:
            raise SystemExit("\nFailed to connect to the robot arm\n")
        print("\nSuccessfully connected to the robot arm: %d\n" % self.handle.id)

    def disconnect(self):
        h = self.robot.rm_delete_robot_arm()
        print("\nSuccessfully disconnected from the robot arm\n" if h == 0
              else "\nFailed to disconnect from the robot arm\n")

    def movej(self, joint, v=20, block=1):
        ret = self.robot.rm_movej(joint, v, 0, 0, block)
        print("movej succeeded" if ret == 0
              else "movej FAILED, error code: %s" % ret)
        return ret == 0

    def movej_p(self, pose, v=20, block=1):
        ret = self.robot.rm_movej_p(pose, v, 0, 0, block)
        print("movej_p succeeded" if ret == 0
              else "movej_p FAILED, error code: %s" % ret)
        return ret == 0

    def run_sequence(self, poses, program, names, block=1):
        """movel through `poses`, one (v, r, connect) per pose.

        A connect=1 move is QUEUED, not executed — the SDK returns 0 without
        the controller having planned anything. Only the final connect=0
        move plans and runs the chain, so a failure reported there is a
        failure of the CHAIN and says nothing about that point in
        particular.
        """
        for i, pose in enumerate(poses):
            v, r, c = program[i]
            ret = self.robot.rm_movel(list(pose), v, r, c, block)
            if ret != 0:
                print("\nmovel FAILED at %s (v=%d r=%d connect=%d), "
                      "error code: %s" % (names[i], v, r, c, ret))
                if c == 0 and i == len(poses) - 1:
                    print("  This is the move that CLOSES the chain, so the "
                          "controller planned every queued move here — the "
                          "fault may lie anywhere in the sequence.")
                print("  ret=1 is the controller returning false: bad "
                      "parameters, or the arm is already in an error state.\n")
                return False
        print("\nsequence succeeded\n")
        return True


def describe(names, poses, program, doc, notes, rest):
    tp = doc.get("task_parameters") or {}
    out = ["plan     movej rest -> movej_p start_pose -> %d x movel -> movej rest"
           % (len(names) - 1),
           "rest     %s" % " ".join("%.1f" % v for v in rest),
           "points   %d in the traversal (%d movel moves after the movej_p)"
           % (len(names), len(names) - 1),
           "tool     %s   <- SELECT THIS ON THE PENDANT; it is not set here"
           % (tp.get("ik_frame") or "NOT DECLARED in the file")]
    for note in notes:
        out.append("note     %s" % note)
    out.append("")
    out.append("  %-4s %-12s %-34s %s" % ("#", "point", "pose [m, rad]",
                                          "v /  r / connect"))
    prev = None
    for i, name in enumerate(names):
        q = poses[i]
        pose = ("%8.4f %8.4f %8.4f  %7.3f %7.3f %7.3f"
                % (q[0], q[1], q[2], q[3], q[4], q[5]))
        tag = "movej_p" if i == 0 else "%3d / %2d / %d" % program[i - 1]
        jump = ""
        if i and prev is not None:
            d = math.dist(poses[i - 1][:3], q[:3])
            if d > 0.15:
                jump = "   <- %.0f mm" % (1000 * d)
        out.append("  %-4s %-12s %s   %s%s" % (i, name, pose, tag, jump))
        prev = name
    return "\n".join(out)


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__.split("USAGE")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="USAGE" + __doc__.split("USAGE")[1])
    ap.add_argument("--points", required=True, help="YAML points file")
    ap.add_argument("--ip", default="192.168.1.18", help="arm IP")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--v", default="20",
                    help="speed %%%% 1-100: one value for every movel, or a "
                         "comma-separated value per move (default 20)")
    ap.add_argument("--blend", "-r", default="0",
                    help="blend radius %%%% 0-100: one value, or one per move "
                         "(default 0)")
    ap.add_argument("--connect", default="1",
                    help="1 = chain the move into the next, 0 = discrete: one "
                         "value, or one per move (default 1)")
    ap.add_argument("--block", type=int, default=1, choices=(0, 1),
                    help="1 = blocking SDK calls (default 1)")
    ap.add_argument("--side", choices=("right", "left"), default=None,
                    help="which arm, for the rest pose (default: from ik_frame)")
    ap.add_argument("--rest", default=None,
                    help="rest joint pose, comma-separated degrees")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and print; never touch the arm")
    return ap.parse_args()


def rest_pose(args, doc, notes):
    """Joint degrees to park at. `--rest`, then the file, then the SRDF."""
    if args.rest:
        return [float(x) for x in args.rest.split(",")]
    if doc.get("rest_pose"):
        return [float(x) for x in doc["rest_pose"]]
    side = args.side
    if side is None:
        ik = str((doc.get("task_parameters") or {}).get("ik_frame") or "")
        side = "left" if ik.startswith("L_") else "right"
        if not ik:
            notes.append("no ik_frame and no --side — resting the RIGHT arm")
    return REST_POSE_DEG[side]


def main():
    args = parse_args()
    points, sequence, doc, notes = load_points(args.points)
    names = traversal(points, sequence)
    poses = [points[n] for n in names]
    n_moves = len(poses) - 1
    program = build_program(
        n_moves,
        per_move(args.v, n_moves, "--v", 1, 100),
        per_move(args.blend, n_moves, "--blend", 0, 100),
        per_move(args.connect, n_moves, "--connect", 0, 1))
    speed = program[0][0] if program else 20
    rest = rest_pose(args, doc, notes)

    print()
    print(describe(names, poses, program, doc, notes, rest))
    print()

    if args.dry_run:
        print("dry run — the arm was never contacted.\n")
        return 0

    controller = RobotArmController(args.ip, args.port)
    print("API Version: ", rm_api_version(), "\n")        # noqa: F405
    ok = (controller.movej(rest, v=speed, block=args.block)
          and controller.movej_p(poses[0], v=speed, block=args.block)
          and controller.run_sequence(poses[1:], program, names[1:],
                                      block=args.block))
    # Rest is attempted even after a failure: an arm left mid-path is worse
    # than one parked, and the failure is already reported above.
    controller.movej(rest, v=speed, block=args.block)
    controller.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
