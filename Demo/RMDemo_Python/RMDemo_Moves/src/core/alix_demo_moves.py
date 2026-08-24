#!/usr/bin/env python3
"""alix moves demo — rm_moves through a point sequence, RM_75 only.

    movej    -> rest pose        establishes the arm configuration
    movej_p  -> start pose
    moves    -> the sequence     x --loops
    movej    -> rest pose        parks

The run OPENS at rest as well as closing there. `movej_p` names a pose, not
a configuration, and a 7-axis arm reaches one pose from many — which branch
it picks depends on where it started. Beginning from a known joint pose is
what makes the run repeatable instead of dependent on wherever the previous
one happened to stop.

POINTS AND SEQUENCE COME FROM A YAML FILE, in rm_pose_t form: every pose is
[x, y, z, rx, ry, rz], METRES and RADIANS, R = Rz.Ry.Rx, in the arm's own
frame. They are handed to the SDK exactly as written — no unit conversion,
no composition, nothing added to anything. That is the point of this format:
there is no step between the file and the controller where a convention can
be got wrong.

USAGE
    python3 alix_demo_moves.py --dry-run
    python3 alix_demo_moves.py --ip 192.168.1.103
    python3 alix_demo_moves.py --ip 192.168.1.103 --v 30 --blend 20 --loops 3
    python3 alix_demo_moves.py --ip 192.168.1.10 --side left
    python3 alix_demo_moves.py --v 20,20,30,30 --blend 0,20,20,0
"""

import argparse
import math
import os
import sys

import yaml

# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.Robotic_Arm.rm_robot_interface import *          # noqa: E402,F403

# This demo is RM_75 only. The arm is asked what it is and the run is
# refused otherwise: the rest poses below are 7-joint values, and sending
# them to a 6-axis arm is not a mismatch the SDK reports usefully.
ARM_MODEL = "RM_75"

DEFAULT_POINTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'points', 'alix_points.yaml'))

# rm_moves is a SPLINE. The vendor documentation is explicit that it needs
# at least THREE points queued with connect=1 or the controller falls back
# to a straight line — so a two-point "spline" silently is not one.
MOVES_MIN_CHAINED = 3


def load_points(path, side_override=None):
    """(rest, start_pose, points, sequence, side) — all rm_pose_t, as given."""
    if not os.path.isfile(path):
        raise SystemExit("no such points file: %s" % path)
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    if not isinstance(doc, dict):
        raise SystemExit("%s: top level must be a mapping" % path)

    # The other format in this repo splits a pose into translation/rotation
    # with the rotation in DEGREES and puts the anchor under
    # `cartesian_poses`. It shares the `cleaning_points` key, so say which
    # file this is rather than failing later on a missing sub-key.
    if "cartesian_poses" in doc:
        raise SystemExit(
            "%s is the translation/rotation format (it has "
            "`cartesian_poses`), not rm_pose_t.\n"
            "  This demo wants every pose as a flat [x, y, z, rx, ry, rz] in "
            "metres and RADIANS.\n"
            "  Convert it, or run it with the tool that reads that format:\n"
            "    cd ../../../RMDemo_PointSequence/src\n"
            "    python3 point_sequence.py --points %s --dry-run" % (path, path))

    points = doc.get("points") or doc.get("cleaning_points")
    sequence = doc.get("sequence") or doc.get("cleaning_sequence")
    start = doc.get("start_pose")
    if not points:
        raise SystemExit("%s: no `points` block" % path)
    if not sequence:
        raise SystemExit("%s: no `sequence` block" % path)
    if start is None:
        raise SystemExit("%s: no `start_pose`" % path)

    # A pose here is SIX numbers. The other format in this repo splits them
    # into translation/rotation with the rotation in degrees; catching that
    # shape by its structure is better than letting degrees be dispatched
    # as radians, which is a working run that cleans the wrong place.
    def pose(name, value):
        if isinstance(value, dict):
            raise SystemExit(
                "%s: %s is written as translation/rotation. This file wants "
                "rm_pose_t: a flat [x, y, z, rx, ry, rz] in metres and "
                "RADIANS." % (path, name))
        vals = [float(v) for v in value]
        if len(vals) != 6:
            raise SystemExit("%s: %s has %d values, expected 6 "
                             "[x, y, z, rx, ry, rz]" % (path, name, len(vals)))
        if not all(math.isfinite(v) for v in vals):
            raise SystemExit("%s: %s has a non-finite value" % (path, name))
        if max(abs(v) for v in vals[3:]) > 2 * math.pi + 1e-6:
            raise SystemExit(
                "%s: %s has a rotation above 2*pi — this file is RADIANS, so "
                "a value in degrees would be dispatched as a much larger "
                "angle." % (path, name))
        return vals

    resolved = {n: pose(n, v) for n, v in points.items()}
    start_pose = pose("start_pose", start)

    # THE FILE'S `side` DECLARES WHICH ARM ITS POINTS BELONG TO, and that is
    # not a preference. Each arm has its own base frame, so a pose authored
    # for the right arm names a different place on the left one — the run
    # would be accepted and the arm would go somewhere else entirely.
    # `--side` therefore selects the REST pose and must AGREE with the file;
    # driving the other arm needs points authored for it.
    declared = str(doc.get("side") or "").lower()
    side = (side_override or declared or "right").lower()
    if side not in ("right", "left"):
        raise SystemExit("side must be right or left, got %r" % side)
    if declared and side_override and side_override != declared:
        raise SystemExit(
            "--side %s, but %s declares `side: %s` — its points are authored "
            "in the %s arm's frame.\n"
            "  Each arm has its own base frame, so these poses name a "
            "different place on the %s arm: the run would be accepted and go "
            "somewhere else.\n"
            "  Use a points file authored for the %s arm."
            % (side_override, path, declared, declared, side, side_override))
    rest_block = doc.get("rest_pose")
    if isinstance(rest_block, dict):
        rest = rest_block.get(side)
        if rest is None:
            raise SystemExit("%s: rest_pose has no %r entry" % (path, side))
    elif rest_block:
        rest = rest_block                      # a single list, one arm
    else:
        raise SystemExit("%s: no `rest_pose`" % path)
    # Joint values are given as whole degrees on purpose; keep them int so
    # what is printed is exactly what is sent.
    rest = [int(round(float(v))) for v in rest]

    return rest, start_pose, resolved, sequence, side


def traversal(points, sequence):
    """The waypoint names to visit, in order.

    The sequence is a list of SEGMENTS. Consecutive ones need not join:
    where one ends at `pointA` and the next starts at `pointB` there is an
    `A -> B` move the sequence never writes down. It is traversed as an
    ordinary point and reported, never taken silently — an unannounced move
    across the workspace is exactly the one you did not intend.
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
                                 "`points`. Known: %s"
                                 % (i, n, ", ".join(points)))
        if prev_end is None or a != prev_end:
            out.append(a)
        out.append(b)
        prev_end = b
    return out


def check_loop_closes(names, points, start_pose, loops):
    """When looping, the sequence must END where it BEGINS.

    `--loops` repeats the moves sequence without returning to rest in
    between, so if the last point is not the start pose the arm jumps
    straight from wherever the path ended back to the first point — a long
    unplanned move, at cleaning speed, once per repeat. Refused rather than
    run.
    """
    if loops <= 1:
        return
    last = points[names[-1]]
    gap = math.dist(last[:3], start_pose[:3])
    if gap > 1e-3:
        raise SystemExit(
            "--loops %d needs the sequence to end where it starts.\n"
            "  Its last point is %r, %.1f mm from start_pose.\n"
            "  Looping would send the arm on an unplanned %.1f mm move back "
            "to the beginning, once per repeat.\n"
            "  End the sequence on a point that sits at start_pose, or run "
            "with --loops 1."
            % (loops, names[-1], 1000 * gap, 1000 * gap))


def per_move(text, n_moves, name, low, high):
    """One value for every move, from `--v 20` or `--v 20,20,30,...`.

    A single number applies to all of them; a list gives each move its own.
    The length must be exactly 1 or `n_moves` — accepting a short list and
    padding it would silently run the tail of the path at a value nobody
    chose, which this refuses rather than guesses through.
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
        raise SystemExit("%s has %d values but there are %d moves — give one "
                         "value or exactly %d" % (name, len(vals), n_moves,
                                                  n_moves))
    return vals


class RobotArmController:
    def __init__(self, ip, port, level=3, mode=2):
        """
        Initialize and connect to the robotic arm.

        Args:
            ip (str): IP address of the robot arm.
            port (int): Port number.
            level (int, optional): Connection level. Defaults to 3.
            mode (int, optional): Thread mode (0: single, 1: dual, 2: triple). Defaults to 2.
        """
        self.thread_mode = rm_thread_mode_e(mode)          # noqa: F405
        self.robot = RoboticArm(self.thread_mode)          # noqa: F405
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)

        if self.handle.id == -1:
            print("\nFailed to connect to the robot arm\n")
            exit(1)
        else:
            print(f"\nSuccessfully connected to the robot arm: {self.handle.id}\n")

    def get_arm_model(self):
        """Get robotic arm mode."""
        res, model = self.robot.rm_get_robot_info()
        if res == 0:
            return model["arm_model"]
        print("\nFailed to get robot arm model\n")
        return None

    def disconnect(self):
        """
        Disconnect from the robot arm.

        Returns:
            None
        """
        handle = self.robot.rm_delete_robot_arm()
        if handle == 0:
            print("\nSuccessfully disconnected from the robot arm\n")
        else:
            print("\nFailed to disconnect from the robot arm\n")

    def movej(self, joint, v=20, r=0, connect=0, block=1):
        """
        Perform movej motion.

        Args:
            joint (list of float): Joint positions.
            v (float, optional): Speed of the motion. Defaults to 20.
            r (float, optional): Blending radius. Defaults to 0.
            connect (int, optional): Trajectory connection flag. Defaults to 0.
            block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.

        Returns:
            bool: True on success.
        """
        movej_result = self.robot.rm_movej(joint, v, r, connect, block)
        if movej_result == 0:
            print("\nmovej motion succeeded\n")
        else:
            print("\nmovej motion failed, Error code: ", movej_result, "\n")
        return movej_result == 0

    def movej_p(self, pose, v=20, r=0, connect=0, block=1):
        """
        Perform movej_p motion.

        Args:
            pose (list of float): Position [x, y, z, rx, ry, rz].
            v (float, optional): Speed of the motion. Defaults to 20.
            r (float, optional): Blending radius. Defaults to 0.
            connect (int, optional): Trajectory connection flag. Defaults to 0.
            block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.

        Returns:
            bool: True on success.
        """
        movej_p_result = self.robot.rm_movej_p(pose, v, r, connect, block)
        if movej_p_result == 0:
            print("\nmovej_p motion succeeded\n")
        else:
            print("\nmovej_p motion failed, Error code: ", movej_p_result, "\n")
        return movej_p_result == 0

    def moves(self, move_positions, speed=20, blending_radius=0, block=1,
              names=None):
        """
        Perform a sequence of move operations.

        Args:
            move_positions (list): positions to move to, each [x, y, z, rx, ry, rz].
            speed (int or list, optional): speed % — one value, or one per move.
            blending_radius (int or list, optional): blend % — one value, or one per move.
            block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.
            names (list, optional): point names, for readable failures.

        Returns:
            bool: True on success.

        A connect=1 move is QUEUED, not executed — the SDK returns 0 without
        the controller having planned anything. Only the closing connect=0
        move plans and runs the whole chain, so a failure reported there is
        a failure of the CHAIN and says nothing about that point in
        particular.
        """
        n = len(move_positions)
        v = speed if isinstance(speed, list) else [speed] * n
        r = blending_radius if isinstance(blending_radius, list) \
            else [blending_radius] * n
        for i, pos in enumerate(move_positions):
            last = (i == n - 1)
            # The last move closes the chain; a chain whose final move still
            # says connect=1 never closes, and the run waits for a
            # continuation that never comes. A closing move cannot blend
            # into anything either, so its r is 0.
            current_connect = 0 if last else 1
            moves_result = self.robot.rm_moves(
                pos, v[i], 0 if last else r[i], current_connect, block)
            if moves_result != 0:
                label = names[i] if names else "index %d" % i
                print("\nmoves operation failed, error code: %s, at %s: %s\n"
                      % (moves_result, label, pos))
                if last:
                    print("  This is the move that CLOSES the chain, so the "
                          "controller planned every queued move here — the "
                          "fault may lie anywhere in the sequence.\n")
                return False

        print("\nmoves operation succeeded\n")
        return True


def describe(names, poses, rest, start_pose, side, v, r, loops, path):
    out = ["plan     movej rest -> movej_p start_pose -> %d x (moves x %d) -> movej rest"
           % (len(names), loops),
           "points   %s" % path,
           "arm      %s, %s" % (ARM_MODEL, side),
           "rest     %s   [deg]" % " ".join("%d" % j for j in rest),
           "start    %s" % " ".join("%.4f" % q for q in start_pose),
           "moves    %d points per loop, %d loop(s)" % (len(names), loops),
           ""]
    out.append("  %-4s %-10s %-40s %s" % ("#", "point",
                                          "pose [m, rad]  (rm_pose_t)",
                                          "v /  r / connect"))
    for i, name in enumerate(names):
        q = poses[i]
        last = (i == len(names) - 1)
        out.append("  %-4d %-10s %s   %3d / %2d / %d%s"
                   % (i, name,
                      "%8.4f %8.4f %8.4f  %7.3f %7.3f %7.3f" % tuple(q),
                      v[i], 0 if last else r[i], 0 if last else 1,
                      "   <- closes the chain" if last else ""))
    return "\n".join(out)


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__.split("USAGE")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="USAGE" + __doc__.split("USAGE")[1])
    ap.add_argument("--ip", default="192.168.1.103", help="arm IP address")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--points", default=DEFAULT_POINTS,
                    help="YAML points file (rm_pose_t)")
    ap.add_argument("--side", choices=("right", "left"), default=None,
                    help="which arm's rest pose (default: from the file)")
    ap.add_argument("--v", default="20",
                    help="speed %% 1-100: one value for every move, or a "
                         "comma-separated value per move (default 20)")
    ap.add_argument("--blend", "-r", default="0",
                    help="blend radius %% 0-100: one value, or one per move "
                         "(default 0)")
    ap.add_argument("--loops", type=int, default=1,
                    help="repeat the moves sequence N times (default 1)")
    ap.add_argument("--block", type=int, default=1, choices=(0, 1),
                    help="1 = blocking SDK calls (default 1)")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and print; never touch the arm")
    args = ap.parse_args()
    if args.loops < 1:
        ap.error("--loops must be at least 1, got %d" % args.loops)
    return args


def main():
    args = parse_args()

    rest, start_pose, points, sequence, side = load_points(args.points,
                                                           args.side)
    names = traversal(points, sequence)
    poses = [points[n] for n in names]
    n_moves = len(poses)

    v = per_move(args.v, n_moves, "--v", 1, 100)
    r = per_move(args.blend, n_moves, "--blend", 0, 100)
    check_loop_closes(names, points, start_pose, args.loops)

    print()
    print(describe(names, poses, rest, start_pose, side, v, r, args.loops,
                   args.points))
    print()
    if n_moves < MOVES_MIN_CHAINED:
        print("  [WARN] rm_moves is a spline and needs at least %d chained "
              "points; with %d the controller falls back to a straight "
              "line.\n" % (MOVES_MIN_CHAINED, n_moves))

    if args.dry_run:
        print("dry run — the arm was never contacted.\n")
        return 0

    # Create a robot arm controller instance and connect to the robot arm
    robot_controller = RobotArmController(args.ip, args.port, 3)

    # Get API version
    print("\nAPI Version: ", rm_api_version(), "\n")       # noqa: F405

    arm_model = robot_controller.get_arm_model()
    if arm_model != ARM_MODEL:
        robot_controller.disconnect()
        raise SystemExit(
            "this demo is %s only; the arm at %s reports %r.\n"
            "  The rest pose is a 7-joint value and would be refused, or "
            "worse accepted, on a different arm."
            % (ARM_MODEL, args.ip, arm_model))

    ok = robot_controller.movej(rest, v=v[0], block=args.block)
    if ok:
        ok = robot_controller.movej_p(start_pose, v=v[0], block=args.block)
    if ok:
        for loop in range(args.loops):
            if args.loops > 1:
                print("--- loop %d/%d ---" % (loop + 1, args.loops))
            ok = robot_controller.moves(poses, speed=v, blending_radius=r,
                                        block=args.block, names=names)
            if not ok:
                break

    # Rest is attempted even after a failure: an arm left mid-path is worse
    # than one parked, and the failure is already reported above.
    robot_controller.movej(rest, v=v[0], block=args.block)

    # Disconnect the robot arm
    robot_controller.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
