"""One cleaning motion, read from ONE config: frames, start pose, deltas,
sequence, and every motion parameter. Nothing is defaulted silently.

This is separate from `RMDemo_PointSequence/src/point_sequence.py` on
purpose. That program takes ABSOLUTE poses in the arm's world frame and
dispatches through whatever tool frame the controller already holds — it
sets nothing. This one is the opposite: it reads the frames from the
config, computes the transformation into the arm's frame itself, and SETS
what the controller needs (tool frame, speed limits) before moving.

THE POINTS ARE ABSOLUTE POSES, NOT DELTAS (Newton, 2026-08-22).
`translation` is the position in the reference frame and `rotation` is the
orientation; nothing is added to `start_pose` and nothing is composed onto
it. In the generated files the first cleaning point sits exactly ON
`start_pose`, which is how you can tell at a glance.

ROTATIONS ARE EXTRINSIC EULER XYZ, in DEGREES. Extrinsic XYZ — rotating
about the FIXED x, then y, then z — composes as R = Rz.Ry.Rx, which is
exactly the controller's own `rm_pose_t` convention, so the angles carry
through unchanged apart from the unit conversion.

THE START POSE FORMAT DEPENDS ON THE REFERENCE FRAME (Newton, 2026-08-22):

    reference_frame: alix_ref_frame     -> [x, y, z, qx, qy, qz, qw]
                     (a URDF frame)        7 values, QUATERNION, w LAST

    reference_frame: arm_base | world   -> [x, y, z, rx, ry, rz]
                     (the arm's own)       6 values, Euler RPY

The length decides which is parsed, because length cannot be misread; the
frame name is then cross-checked against it and a mismatch is reported
rather than guessed at. `w LAST` is the config convention (`quat_to_R`
takes `[qx, qy, qz, qw]`) — reading it as `wxyz` yields a different,
entirely plausible-looking orientation, which is the worst kind of bug.

FRAMES ARE RESOLVED, NOT ASSUMED. `ik_frame` becomes the controller tool
frame through the lookup table (FRAME_MAP.md: the URDF link name with the
`_frame` token removed). `reference_frame` is located in the URDF and the
transform into the arm base is computed at the PINNED MINIMUM POLE HEIGHT,
because the pole carries the arm base.
"""

import math
import pathlib

import numpy as np
import yaml

from cm_common import POLE_M

# `alix_ref_frame` and `butterfli_ref_frame` are the same frame under two
# workspace names. Try both, and use whichever the loaded URDF actually has,
# so a config written against either name resolves.
REF_FRAME_ALIASES = {
    "alix_ref_frame": ("alix_ref_frame", "butterfli_ref_frame"),
    "butterfli_ref_frame": ("butterfli_ref_frame", "alix_ref_frame"),
}

ARM_BASE_NAMES = ("arm_world", "arm_base", "base", "base_link", "world",
                  "controller", "arm")

# The four CONTROLLER LIMITS. These are NOT defaulted here: when nothing
# asks for a value the arm keeps whatever it already holds, which is the
# controller's own default setting (Newton, 2026-08-22 — "the defaults used
# are the ones in the arm's controller"). Inventing a value here would
# silently reconfigure the machine every run and make "the default" mean
# whatever this file last said.
CONTROLLER_LIMITS = ("line_speed", "line_acc", "angular_speed", "angular_acc")

# Per-command dispatch parameters. These are arguments to `rm_movel` and
# friends, not controller state, so they DO need a value every time and it
# is stated here.
DISPATCH_DEFAULTS = {
    "v": 100,                  # %     commanded per cleaning move
    "blend_r": 0,              # %     per move, 0-100
    "connect": 1,
    "block": 0,
    "transit_v": 20,           # %     movej / movej_p
    "loops": 1,
    "primitive": "movel",
}


class CleaningConfig:
    """Everything one cleaning motion needs, from one YAML file."""

    def __init__(self, doc, path=""):
        self.doc = doc or {}
        self.path = str(path)
        self._validate()

    @classmethod
    def load(cls, path):
        p = pathlib.Path(path)
        if not p.is_file():
            raise SystemExit("no such cleaning config: %s" % p)
        return cls(yaml.safe_load(p.read_text()), p)

    # ── frames ──────────────────────────────────────────────────────────
    @property
    def _task_parameters(self):
        return self.doc.get("task_parameters") or {}

    @property
    def reference_frame(self):
        # Generated configs put it under `task_parameters:`; a hand-written
        # one may put it at the top level. Both are accepted, top level
        # first, so a file can override its own generated block.
        f = (self.doc.get("reference_frame") or self.doc.get("ref_frame")
             or self._task_parameters.get("reference_frame"))
        if not f:
            raise SystemExit(
                "%s: no `reference_frame`, at the top level or under "
                "`task_parameters`. It decides both the transform into the "
                "arm's frame AND how `start_pose` is read, so it cannot be "
                "defaulted." % self.path)
        return str(f)

    @property
    def ik_frame(self):
        f = (self.doc.get("ik_frame") or self.doc.get("tool_frame")
             or self._task_parameters.get("ik_frame"))
        if not f:
            raise SystemExit(
                "%s: no `ik_frame`. The commanded poses are meaningless "
                "without the frame they are measured to." % self.path)
        return str(f)

    # The URDF names the arm tip PER SIDE (`L_arm_tip` / `R_arm_tip`)
    # because one model carries two arms. A controller only knows its own
    # arm, and calls that frame `Arm_Tip` — its built-in default tool, i.e.
    # NO tool offset at all. So an ik_frame of `*_arm_tip` is a request for
    # the flange, not for a tool frame someone forgot to create; selecting
    # it by the URDF name fails with a bare ret=1 that reads like a missing
    # frame.
    CONTROLLER_ARM_TIP = "Arm_Tip"

    @property
    def tool_frame(self):
        """The CONTROLLER tool-frame name, via the frames lookup table."""
        name = self.ik_frame
        bare = name.replace("_frame", "")
        if bare.lower().lstrip("lr_").replace("_", "") == "armtip" \
                or bare.lower() in ("arm_tip", "l_arm_tip", "r_arm_tip"):
            return self.CONTROLLER_ARM_TIP
        try:
            from frame_alignment_offline import controller_frame_name
            return controller_frame_name(name)
        except Exception:
            return bare

    @property
    def reference_is_arm_base(self):
        return self.reference_frame.lower() in ARM_BASE_NAMES

    @property
    def side(self):
        s = self.doc.get("side")
        if s:
            return str(s).lower()
        # The ik_frame names the arm: L_* / R_*.
        head = self.ik_frame[:2].upper()
        if head == "L_":
            return "left"
        if head == "R_":
            return "right"
        raise SystemExit(
            "%s: no `side`, and it cannot be read from ik_frame %r. State "
            "`side: left` or `side: right`." % (self.path, self.ik_frame))

    # ── the anchor ──────────────────────────────────────────────────────
    def start_pose_matrix(self):
        """4x4 start pose IN THE REFERENCE FRAME, from either format."""
        sp = (self.doc.get("start_pose")
              or (self.doc.get("cartesian_poses") or {}).get("start_pose"))
        if sp is None:
            raise SystemExit(
                "%s: no `start_pose`. The cleaning points are deltas from "
                "it, so without it all %d of them are undefined."
                % (self.path, len(self.cleaning_points)))
        vals = [float(v) for v in sp]
        if not all(math.isfinite(v) for v in vals):
            raise SystemExit("%s: start_pose has a non-finite value" % self.path)

        expects_quat = not self.reference_is_arm_base
        if len(vals) == 7:
            if not expects_quat:
                print("  [WARN] start_pose has 7 values (a quaternion) while "
                      "reference_frame %r is the arm's own frame, where the "
                      "expected form is [x y z rx ry rz]. Parsing it as a "
                      "quaternion — check the file." % self.reference_frame)
            R = _quat_to_R(vals[3:7])
            form = "quaternion [x y z qx qy qz qw], w last"
        elif len(vals) == 6:
            if expects_quat:
                print("  [WARN] start_pose has 6 values (Euler RPY) while "
                      "reference_frame %r is a URDF frame, where the "
                      "expected form is a quaternion. Parsing it as RPY — "
                      "check the file." % self.reference_frame)
            # RADIANS, and deliberately NOT governed by `rotation_units`.
            # That key describes the cleaning-point DELTAS, which the
            # config writes in degrees. A 6-value start_pose is a
            # controller-convention pose in the arm's own frame — the same
            # thing `rm_movel` takes — and the controller works in radians.
            # Applying the delta units here turned 3.114 rad into 0.054 and
            # silently re-oriented the whole path. `start_pose_units:`
            # overrides if a file really does write it in degrees.
            su = str(self.doc.get("start_pose_units", "rad")).lower()
            if su.startswith("deg"):
                scale, unit = math.pi / 180.0, "degrees"
            elif su.startswith("rad"):
                scale, unit = 1.0, "radians"
            else:
                raise SystemExit("%s: start_pose_units must be deg or rad, "
                                 "got %r" % (self.path, su))
            if scale == 1.0 and max(abs(v) for v in vals[3:6]) > 2 * math.pi:
                print("  [WARN] start_pose rotations exceed 2*pi but are "
                      "being read as RADIANS. If that pose is in degrees, "
                      "set `start_pose_units: deg`.")
            R = _rpy_to_R(*[v * scale for v in vals[3:6]])
            form = "Euler RPY [x y z rx ry rz], %s" % unit
        else:
            raise SystemExit(
                "%s: start_pose has %d values. Expected 7 "
                "[x y z qx qy qz qw] for a URDF reference frame, or 6 "
                "[x y z rx ry rz] for the arm's own frame."
                % (self.path, len(vals)))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = vals[:3]
        self.start_pose_form = form
        return T

    # ── points and sequence ─────────────────────────────────────────────
    @property
    def _to_rad(self):
        u = str(self.doc.get("rotation_units", "deg")).lower()
        if u.startswith("deg"):
            return math.pi / 180.0
        if u.startswith("rad"):
            return 1.0
        raise SystemExit("%s: rotation_units must be deg or rad, got %r"
                         % (self.path, u))

    @property
    def cleaning_points(self):
        pts = self.doc.get("cleaning_points") or self.doc.get("points")
        if not pts:
            raise SystemExit("%s: no `cleaning_points`" % self.path)
        return pts

    @property
    def cleaning_sequence(self):
        seq = self.doc.get("cleaning_sequence") or self.doc.get("sequence")
        if not seq:
            raise SystemExit("%s: no `cleaning_sequence`" % self.path)
        return seq

    def traversal(self):
        """[(name, from_jump)] — the waypoints to visit, in order.

        The sequence is a list of SEGMENTS and consecutive segments need
        not join. Where they do not, the move across the gap is implied but
        never written down; it is included and MARKED, never dropped and
        never silent. (The concept tree's resolver refuses such a sequence
        outright because its runtime emits only each segment's second
        point — here the gap is traversed explicitly instead.)
        """
        pts = self.cleaning_points
        out, prev_end = [], None
        for i, seg in enumerate(self.cleaning_sequence):
            if not isinstance(seg, (list, tuple)) or len(seg) < 2:
                raise SystemExit("%s: sequence entry %d is %r; expected "
                                 "[from, to] with an optional third "
                                 "{params}" % (self.path, i, seg))
            a, b = seg[0], seg[1]
            for n in (a, b):
                if n not in pts:
                    raise SystemExit(
                        "%s: sequence entry %d names %r, which is not in "
                        "cleaning_points. Known: %s"
                        % (self.path, i, n, ", ".join(pts)))
            if prev_end is None:
                out.append((a, False))
            elif a != prev_end:
                out.append((a, True))
            out.append((b, False))
            prev_end = b
        return out

    def segment_overrides(self):
        """{move_index: {...}} from an optional third element per segment."""
        params, tlen, prev_end = {}, 0, None
        for seg in self.cleaning_sequence:
            a, b = seg[0], seg[1]
            if prev_end is None or a != prev_end:
                tlen += 1
            tlen += 1
            if len(seg) > 2 and isinstance(seg[2], dict):
                params[tlen - 2] = seg[2]
            prev_end = b
        return params

    # ── resolution ──────────────────────────────────────────────────────
    def waypoints_ref(self):
        """[(name, 4x4 in the REFERENCE frame)] — the points, as given."""
        to_rad = self._to_rad
        out = []
        for name, _ in self.traversal():
            pt = self.cleaning_points[name]
            if not isinstance(pt, dict) or "translation" not in pt \
                    or "rotation" not in pt:
                raise SystemExit("%s: point %r needs `translation` and "
                                 "`rotation`" % (self.path, name))
            t = np.asarray([float(v) for v in pt["translation"]], float)
            r = np.asarray([float(v) * to_rad for v in pt["rotation"]], float)
            if t.shape != (3,) or r.shape != (3,):
                raise SystemExit("%s: point %r wants 3 values each"
                                 % (self.path, name))
            T = np.eye(4)
            # ABSOLUTE, not a delta (Newton, 2026-08-22). The point IS the
            # pose: nothing is added to the anchor and nothing is composed
            # onto it. Rotations are EXTRINSIC EULER XYZ, which is the same
            # composition as the controller's own pose convention
            # (R = Rz.Ry.Rx), so the angles carry straight through.
            T[:3, :3] = _rpy_to_R(r[0], r[1], r[2])
            T[:3, 3] = t
            out.append((name, T))
        return out

    def poses_arm_base(self, robot=None):
        """(poses, note) — the cleaning path in the arm's own frame.

        When the reference frame IS the arm's frame this is an explicit
        identity and says so. Otherwise the transform is computed from the
        URDF at the PINNED MINIMUM POLE HEIGHT, because the pole carries
        the arm base — resolving at the wrong height displaces every
        waypoint and the arm reports nothing.
        """
        from cm_frames import FrameResolver, _rpy_to_R as _unused  # noqa: F401
        from cleaning_path import R_to_euler

        wp = self.waypoints_ref()
        if self.reference_is_arm_base:
            T = np.eye(4)
            note = ("reference frame %r is the arm's own — no transform "
                    "applied (pole assumed at %.3f m)"
                    % (self.reference_frame, POLE_M))
        else:
            res = FrameResolver(self.side, pole_m=POLE_M)
            if robot is not None:
                res.adopt_mount_from_controller(robot)
            T = res.ref_to_arm_base(self._urdf_reference_frame(res))
            note = ("%s -> arm base, pole PINNED at %.3f m (minimum), "
                    "mount from %s"
                    % (self.reference_frame, POLE_M, res.mount_source))

        poses, names = [], []
        for name, M in wp:
            W = T @ M
            rx, ry, rz = R_to_euler(W[:3, :3])
            poses.append([float(W[0, 3]), float(W[1, 3]), float(W[2, 3]),
                          rx, ry, rz])
            names.append(name)

        # THE ENTRY POSE — the declared `start_pose` itself, carried through
        # the same transform. It is the `movej_p` target, and it is NOT
        # always one of the cleaning points: in the generated configs no
        # point has a zero delta, so start_pose is a distinct pose 44 mm
        # off the first stroke. In the older configs point1 IS the anchor
        # (zero delta), and there the driver sees the duplicate and skips
        # it rather than commanding a zero-length move.
        W0 = T @ self.start_pose_matrix()
        rx, ry, rz = R_to_euler(W0[:3, :3])
        self.entry_pose = [float(W0[0, 3]), float(W0[1, 3]), float(W0[2, 3]),
                           rx, ry, rz]
        return poses, names, note

    def _urdf_reference_frame(self, resolver):
        """The reference frame's name AS THE LOADED URDF spells it.

        `alix_ref_frame` and `butterfli_ref_frame` are one frame under two
        workspace names; a config written against either must resolve.
        """
        want = self.reference_frame
        from segment_verifier import SegmentVerifier
        if resolver._verifier is None:
            resolver._verifier = SegmentVerifier(side=self.side, quiet=True)
        links = resolver._verifier.model.link_world_transforms(
            dict(resolver._verifier.home))
        for cand in REF_FRAME_ALIASES.get(want, (want,)):
            if cand in links:
                if cand != want:
                    print("  [INFO] reference_frame %r resolved to %r, the "
                          "name this URDF uses for it." % (want, cand))
                return cand
        return want          # let the resolver raise with its own near-miss list

    # ── motion parameters ───────────────────────────────────────────────
    def motion(self, overrides=None):
        """Motion parameters. A `motion:` block is OPTIONAL.

        The generated configs do not carry one — the parameters are passed
        at test-run time, and anything nobody asks for keeps the value the
        ARM'S CONTROLLER already holds (Newton, 2026-08-22). So the four
        controller limits come back as None when unset, and None here means
        "do not touch it", never "use some number this file chose".

        The per-command dispatch parameters are different: they are
        arguments to `rm_movel`, not controller state, so they always have
        a value and it is stated in DISPATCH_DEFAULTS.
        """
        block = dict(self.doc.get("motion") or self.doc.get("parameters")
                     or {})
        out = {}
        for k in CONTROLLER_LIMITS:
            v = block.get(k)
            if overrides and overrides.get(k) is not None:
                v = overrides[k]
            out[k] = None if v is None else float(v)
        for k, default in DISPATCH_DEFAULTS.items():
            v = block.get(k, default)
            if overrides and overrides.get(k) is not None:
                v = overrides[k]
            out[k] = v
        for k in ("v", "blend_r", "transit_v"):
            if not 0 <= int(out[k]) <= 100:
                raise SystemExit("%s: motion.%s is a percentage 0-100, got %r"
                                 % (self.path, k, out[k]))
        if int(out["connect"]) not in (0, 1):
            raise SystemExit("%s: motion.connect must be 0 or 1" % self.path)
        if int(out["loops"]) < 1:
            raise SystemExit("%s: motion.loops must be >= 1" % self.path)
        if out["primitive"] not in ("movel", "moves"):
            raise SystemExit("%s: motion.primitive must be movel or moves"
                             % self.path)
        return out

    @property
    def rest_pose(self):
        return self.doc.get("rest_pose")

    @property
    def name(self):
        return str(self.doc.get("name")
                   or pathlib.Path(self.path).stem or "cleaning_motion")

    # ── checks ──────────────────────────────────────────────────────────
    def _validate(self):
        if not isinstance(self.doc, dict):
            raise SystemExit("%s: top level must be a mapping" % self.path)

    def describe(self):
        T0 = self.start_pose_matrix()
        out = [
            "config    %s" % (self.path or "(inline)"),
            "  name    %s      side %s" % (self.name, self.side),
            "  frames  reference %s%s" % (
                self.reference_frame,
                "  (the arm's own frame)" if self.reference_is_arm_base
                else ""),
            "          ik_frame  %s  ->  controller tool frame %s"
            % (self.ik_frame, self.tool_frame),
            "  start   %s" % self.start_pose_form,
            "          xyz %8.4f %8.4f %8.4f  [reference frame]"
            % (T0[0, 3], T0[1, 3], T0[2, 3]),
            "  points  %d declared, %d in the traversal"
            % (len(self.cleaning_points), len(self.traversal())),
        ]
        jumps = [(i, n) for i, (n, j) in enumerate(self.traversal()) if j]
        if jumps:
            out.append("  ⚠ the sequence does not join at %d place(s); those "
                       "moves are implied, not written down" % len(jumps))
        return "\n".join(out)


# ── rotation helpers ────────────────────────────────────────────────────
def _quat_to_R(q):
    """[qx, qy, qz, qw] -> R. W LAST — the config convention.

    Reading this as `wxyz` produces a different but entirely
    plausible-looking orientation, which is why the order is stated here
    and not inferred.
    """
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise SystemExit("start_pose quaternion has zero norm")
    if abs(n - 1.0) > 1e-3:
        print("  [WARN] start_pose quaternion norm is %.4f, not 1 — "
              "normalising." % n)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rpy_to_R(rx, ry, rz):
    """R = Rz(rz) Ry(ry) Rx(rx) — the CONTROLLER's pose convention."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], float)
    return Rz @ Ry @ Rx


def _rpy_to_R_xyz(rx, ry, rz):
    """R = Rx(rx) Ry(ry) Rz(rz) — INTRINSIC xyz, i.e. the delta composition
    the older configs used.

    UNUSED as of 2026-08-22: the points are absolute and their rotations are
    EXTRINSIC XYZ, which is `_rpy_to_R` above. Kept only so the difference
    stays written down — extrinsic XYZ and intrinsic xyz are the same three
    angles composed in opposite orders, and swapping them silently
    re-orients every waypoint.
    """
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], float)
    return Rx @ Ry @ Rz
