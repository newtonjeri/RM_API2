"""Source frame -> ARM BASE frame, with the POLE PINNED AT ITS MINIMUM.

"Take the start pose and convert it to what the arm understands" (Newton,
2026-08-22). That conversion is the one genuinely new piece of geometry in
this folder, and it is not a constant offset — it is a function of the pole.

WHY THE POLE IS IN A FRAME TRANSFORM AT ALL. The pole is a prismatic joint
that CARRIES THE ARM BASE. Every cleaning waypoint is authored relative to
the fixture (the commode), which does not move; the arm base does. So

    T(arm_base -> reference) = Ry(mount) @ inv(T_world_baselink(pole))
                                         @ T_world_reference

and resolving at the wrong pole height displaces EVERY waypoint by the
difference — 215 mm between the SRDF home 0.29 and the tasks' 0.075. That
is not an error the arm reports; it just cleans the wrong place.

This bed is arm-only, so the pole never moves and its height is a constant:
`cm_common.POLE_M`, the measured minimum. Pinning it is what makes an
arm-only test well-defined. It is threaded through explicitly rather than
defaulted deep in a call chain, because a silently-defaulted pole height is
exactly the bug above.

THE MOUNT ANGLE COMES FROM THE ARM WHEN THERE IS AN ARM. Controller World
is the base rotated Ry(+90). A hard-coded 90 goes stale the day an arm is
re-mounted, and the failure mode is not an error — it is every target
rotated, which is what broke the first REAL run in the concept tree. So
`adopt_mount_from_controller()` asks the controller and only falls back to
the constant with a loud warning.

The FK itself is not reimplemented here: it comes from `SegmentVerifier` +
`UrdfModel`, the same objects `cleaning_path._ref_to_arm_base` uses, so
this tree and that one cannot disagree about where a frame is.
"""

import math

import numpy as np

from cm_common import POLE_M

# The house convention, imported rather than rewritten. `_Ry` is private
# next door on purpose; reusing it (instead of writing three lines of
# numpy) is what guarantees this file and `cleaning_path` share ONE
# rotation convention. `R_to_euler` matters even more: controller poses are
# Euler RPY (Rz*Ry*Rx), NOT a rotation vector, and reading them the other
# way produced a confidently wrong tilt table once already.
from cleaning_path import _Ry, R_to_euler          # noqa: E402

MOUNT_RY_DEG_DEFAULT = 90.0
ARM_BASE_ALIASES = ("arm_base", "base", "base_link", "controller", "arm")


class FrameResolver:
    """Resolves a source frame to the arm base at a fixed pole height."""

    def __init__(self, side, fixture="commode_c", pole_m=POLE_M, quiet=True):
        self.side = side
        self.fixture = fixture
        self.pole_m = float(pole_m)
        self.quiet = quiet
        self.mount_ry_deg = MOUNT_RY_DEG_DEFAULT
        self.mount_source = "constant (%.1f deg, not yet asked of the arm)" \
            % MOUNT_RY_DEG_DEFAULT
        self._verifier = None

    # ── mount ──
    def adopt_mount_from_controller(self, robot):
        """Take the mounting angle from the ARM. Returns the angle adopted.

        Only a pure Y rotation is modelled. A non-Y install pose is a hard
        stop rather than a warning: the transform below would be wrong in a
        way no downstream check can see.
        """
        try:
            ip = robot.rm_get_install_pose()
            if not isinstance(ip, dict) or ip.get("return_code") != 0:
                raise RuntimeError("ret=%s" % ip)
            rx, ry, rz = float(ip["x"]), float(ip["y"]), float(ip["z"])
        except Exception as exc:
            print("  [WARN] install pose unreadable (%r) — falling back to "
                  "the measured constant %.1f deg about Y. If this arm has "
                  "been re-mounted this is WRONG."
                  % (exc, MOUNT_RY_DEG_DEFAULT))
            return None
        if abs(rx) > 1e-6 or abs(rz) > 1e-6:
            raise SystemExit(
                "install pose (%g, %g, %g) is not a pure Y rotation. This "
                "resolver models the mounting as Ry only; extend "
                "FrameResolver._mount() before running this arm."
                % (rx, ry, rz))
        if abs(ry - self.mount_ry_deg) > 0.5:
            print("  [WARN] install pose Y=%.2f deg differs from %.1f — "
                  "using the CONTROLLER's value (it is the authority)"
                  % (ry, self.mount_ry_deg))
        self.mount_ry_deg = ry
        self.mount_source = "controller install pose (%.2f deg)" % ry
        return ry

    def _mount(self):
        T = np.eye(4)
        T[:3, :3] = _Ry(math.radians(self.mount_ry_deg))
        return T

    # ── the transform ──
    def ref_to_arm_base(self, reference_frame):
        """4x4 T(arm base <- reference_frame) at this resolver's pole height."""
        if self._verifier is None:
            from segment_verifier import SegmentVerifier
            self._verifier = SegmentVerifier(fixture=self.fixture,
                                             side=self.side, quiet=True)
        v = self._verifier
        pref = "R_" if self.side == "right" else "L_"
        state = dict(v.home)
        key = "%ssliding_plate_joint" % pref
        if key not in state:
            raise SystemExit(
                "the URDF home state has no %r, so the pole height cannot "
                "be pinned. Refusing to resolve a cleaning motion against an "
                "unknown pole position." % key)
        state[key] = self.pole_m
        tw = v.model.link_world_transforms(state)
        if reference_frame not in tw:
            near = [k for k in tw if reference_frame.lower() in k.lower()][:6]
            raise SystemExit(
                "unknown reference frame %r.\n  %s"
                % (reference_frame,
                   ("did you mean: %s" % ", ".join(near)) if near else
                   "not a link in the URDF; %d links known" % len(tw)))
        base = "%sbase_link" % pref
        return self._mount() @ np.linalg.inv(tw[base]) @ tw[reference_frame]


def to_arm_base(program, pole_m=POLE_M, robot=None, fixture="commode_c"):
    """Return (poses_in_arm_base, note). Never mutates `program`.

    A program already authored in the arm base frame passes through
    UNCHANGED — the identity is stated explicitly and reported, rather than
    being a transform that happens to be identity, so a log always says
    which of the two happened.
    """
    if program.source_frame.lower() in ARM_BASE_ALIASES:
        return ([list(q) for q in program.poses],
                "source frame is the arm base — no transform applied "
                "(pole irrelevant to the poses; assumed at %.3f m)" % pole_m)

    res = FrameResolver(program.side, fixture=fixture, pole_m=pole_m)
    if robot is not None:
        res.adopt_mount_from_controller(robot)
    T = res.ref_to_arm_base(program.source_frame)

    out = []
    for q in program.poses:
        M = np.eye(4)
        M[:3, :3] = _rpy_to_R(q[3], q[4], q[5])
        M[:3, 3] = q[:3]
        W = T @ M
        rx, ry, rz = R_to_euler(W[:3, :3])
        out.append([float(W[0, 3]), float(W[1, 3]), float(W[2, 3]),
                    rx, ry, rz])
    note = ("%s -> arm base, pole PINNED at %.3f m (minimum), mount from %s"
            % (program.source_frame, pole_m, res.mount_source))
    return out, note


def _rpy_to_R(rx, ry, rz):
    """R = Rz(rz) @ Ry(ry) @ Rx(rx) — the controller's Euler RPY convention.

    Matches `orientation_cost._Rmat`. Written out here (rather than reusing
    a private helper for this one direction) so the composition ORDER is
    visible at the point where a source pose becomes a rotation matrix —
    that order is the single most expensive thing to get wrong in this file.
    """
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], float)
    Ry_ = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], float)
    return Rz @ Ry_ @ Rx


def start_pose(poses):
    """The pose the entry sequence must reach — waypoint 0, in arm base."""
    if not poses:
        raise SystemExit("no poses — nothing to start from")
    return list(poses[0])


def describe_conversion(program, poses, note):
    """Human-readable before/after for the log. The start pose is printed in
    full because it is the one the arm is actually driven to blind."""
    a, b = program.poses[0], poses[0]
    return "\n".join([
        "  frame conversion: %s" % note,
        "    start (source) %s" % _fmt_pose(a),
        "    start (arm)    %s" % _fmt_pose(b),
        "    displacement   %.1f mm" % (1000 * math.dist(a[:3], b[:3])),
    ])


def _fmt_pose(q):
    return ("xyz %8.1f %8.1f %8.1f mm   rpy %7.3f %7.3f %7.3f rad"
            % (1000 * q[0], 1000 * q[1], 1000 * q[2], q[3], q[4], q[5]))
