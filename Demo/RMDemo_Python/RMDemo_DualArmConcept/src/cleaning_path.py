"""Phase 2 — the Cartesian cleaning path, resolved from the task config.

A faithful port of `TaskBase::resolveCleaningWaypoints`
(cleaning_tasks/src/common/task_base.cpp:3862), which is the authority on
what a cleaning path IS. Verified against all three saved plans: the
resolved waypoints sit on the executed tool path to a mean of 0.3-0.4 mm
(max 4.3 mm on hinge_area_right).

    anchor      cartesian_poses["start_pose"], or — when the task declares
                none — `path_anchor_override_`, the FK of the preceding
                stage's target (Newton, 2026-08-08: "a lookback")
    position    T_anchor.translation + (dx, dy, dz)     <- reference_frame
                axes, NOT rotated into the anchor's frame
    orientation R_delta * R_anchor, with
                R_delta = Rx(roll) * Ry(pitch) * Rz(yaw), degrees
    waypoints   cleaning_sequence[0].first, then every segment's .second
                — a chained polyline, guaranteed chained for any task with
                a saved plan (Newton). Asserted anyway; a break would
                silently drop a stroke.

WHAT THE CONTROLLER SHOULD DO WITH IT. The MTC stage computes ONE
continuous Cartesian path through every waypoint
(`CartesianInterpolator::computeCartesianPath(..., MaxEEFStep(0.01), ...)`)
and hands all of it to a single Ruckig Pro call, so the intended motion is
jerk-continuous end to end — not 43 discrete strokes with a stop at each.
The geometry between consecutive waypoints is Cartesian-linear, which is
exactly `rm_movel` semantics, so the faithful mapping is a CHAINED movel
program: `connect=1` on every segment, `connect=0` on the last, with a
blend radius so corners are rounded rather than stopped at.

Queue the whole path (Newton, 2026-08-08: "for all tasks, it should be
queue all segments") — 43 segments for hinge_area_right, 27 per toplid.
C10 probed queue depth only to 20, so depth beyond that is UNVERIFIED;
`movel_program()` reports the depth it needs so the dispatcher can gate.

Usage:
    from task_config import TaskConfig
    from cleaning_path import CleaningPath
    p = CleaningPath(TaskConfig.load("hinge_area_right"))
    p.waypoints_ref()      # [(name, 4x4 in reference_frame)]
    p.movel_program(pole_m=0.075)   # poses in the ARM BASE frame
"""

import math
import sys

import numpy as np

from segment_verifier import SegmentVerifier

BLEND_PCT_DEFAULT = 10       # ZIGZAG01 used 10 %; r is a PERCENTAGE (F/C10)


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_to_R(q):
    """[qx, qy, qz, qw] -> rotation matrix (the config's pose convention)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_euler(R):
    """Rotation matrix -> (rx, ry, rz) with R = Rz(rz) Ry(ry) Rx(rx).

    That is the convention `rm_pose_t.euler` uses and the one
    frame_alignment_offline builds poses with — keep the two in step.
    """
    ry = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.99999:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    else:                                     # gimbal lock
        rx = math.atan2(-R[1, 2], R[1, 1])
        rz = 0.0
    return rx, ry, rz


class CleaningPath:
    """Resolved Cartesian path for one task."""

    def __init__(self, cfg, anchor=None, quiet=False):
        self.cfg = cfg
        self.quiet = quiet
        self._anchor = anchor          # 4x4 in reference_frame, or None
        self._verifier = None

    # ── anchor ──
    def anchor(self):
        """The pose the relative cleaning points hang off.

        Declared `start_pose` when the task has one; otherwise the caller
        must supply the lookback pose (FK of the preceding stage's
        target). Failing loudly beats resolving 44 waypoints against the
        wrong origin — every one of them would be silently displaced.
        """
        if self._anchor is not None:
            return self._anchor
        sp = self.cfg.cartesian_poses.get("start_pose")
        if sp is None:
            raise SystemExit(
                f"task {self.cfg.task!r} declares no start_pose: it needs "
                "the path_anchor_override (FK of the preceding stage's "
                "target). Pass anchor=<4x4 in reference_frame>.")
        T = np.eye(4)
        T[:3, :3] = quat_to_R(sp[3:7])
        T[:3, 3] = np.asarray(sp[:3], dtype=float)
        return T

    # ── waypoints ──
    def waypoint_names(self):
        seq = self.cfg.cleaning_sequence
        if not seq:
            raise SystemExit(f"task {self.cfg.task!r} has no "
                             "cleaning_sequence")
        breaks = [i for i, (a, b) in enumerate(zip(seq, seq[1:]))
                  if a[1] != b[0]]
        if breaks:
            raise SystemExit(
                f"cleaning_sequence is NOT chained at segment(s) {breaks}: "
                "the runtime emits only each segment's second point, so a "
                "break would silently drop a stroke. Handle the retract / "
                "re-approach explicitly before running this task.")
        return [seq[0][0]] + [s[1] for s in seq]

    def waypoints_ref(self):
        """[(name, 4x4 pose in reference_frame)] — the resolved path."""
        T_anchor = self.anchor()
        p0, R0 = T_anchor[:3, 3], T_anchor[:3, :3]
        out = []
        for name in self.waypoint_names():
            pt = self.cfg.cleaning_points[name]
            t = np.asarray(pt["translation"], dtype=float)
            r = np.radians(np.asarray(pt["rotation"], dtype=float))
            R_delta = _Rx(r[0]) @ _Ry(r[1]) @ _Rz(r[2])
            T = np.eye(4)
            T[:3, :3] = R_delta @ R0          # LEFT-multiplied, as in C++
            T[:3, 3] = p0 + t                 # reference_frame axes
            out.append((name, T))
        return out

    # ── into the arm's own frame ──
    # The controller knows nothing of the UGV body, the pole, the hand or
    # butterfli_ref_frame — its world starts at its own base. It also sits
    # MOUNTED: measured against both controllers (C14 capture, 2026-08-08),
    #
    #     right  URDF base_link (-0.0788, -0.0021,  0.5131)
    #            controller     ( 0.5131, -0.0021,  0.0788)
    #     left   URDF base_link ( 0.1630, -0.0128,  0.4473)
    #            controller     ( 0.4472, -0.0128, -0.1630)
    #
    # i.e. controller = Ry(+90 deg) . urdf_base_link  —  (x,y,z)->(z,y,-x),
    # the same rotation the controller reports as install pose (0, 90, 0).
    # Sending URDF-base poses straight to the SDK put every cleaning target
    # 90 deg out; the chain was accepted and then failed in execution.
    MOUNT_RY_DEG = 90.0

    def _mount(self):
        a = math.radians(self.MOUNT_RY_DEG)
        T = np.eye(4)
        T[:3, :3] = _Ry(a)
        return T

    def _ref_to_arm_base(self, pole_m):
        """T(arm base -> reference_frame) at a given pole height.

        The pole is prismatic and CARRIES THE ARM BASE, so this transform
        is a function of pole height. Resolving the path at the wrong pole
        height displaces every waypoint by the difference (215 mm between
        the SRDF home 0.29 and the tasks' commanded 0.075).
        """
        if self._verifier is None:
            self._verifier = SegmentVerifier(
                fixture=self.cfg.fixture, side=self.cfg.side, quiet=True)
        v = self._verifier
        pref = "R_" if self.cfg.side == "right" else "L_"
        state = dict(v.home)
        state[f"{pref}sliding_plate_joint"] = pole_m
        tw = v.model.link_world_transforms(state)
        # ...expressed in the CONTROLLER's base frame, not the URDF's
        return (self._mount()
                @ np.linalg.inv(tw[f"{pref}base_link"])
                @ tw[self.cfg.reference_frame])

    def movel_program(self, pole_m, blend_pct=BLEND_PCT_DEFAULT,
                      hover_mm=0.0):
        """The chained movel program, in the ARM BASE frame.

        hover_mm lifts EVERY waypoint off the surface along the TASK'S OWN
        RETREAT VECTOR — normalize(pre_start_pose - start_pose) in the
        reference frame. Not the waypoint's local +Z: the yaml header
        claims pre_start is "start retreated along local +Z", but the
        DATA says otherwise — hinge's declared retreat aligns with local
        +Z at only 0.198 (the values are hand-edited), and toplid's at
        -0.991, i.e. the OPPOSITE sign. The declared vector is what the
        task itself uses to approach, so it is the direction that is
        guaranteed to leave the surface. A hover run executes the exact
        same chain with no contact: the A/B that separates "contact
        torque trips the power rail" from "speed does".

        Returns a dict with the poses, the tool frame to select, and the
        queue depth needed.
        """
        from frame_alignment_offline import controller_frame_name
        T = self._ref_to_arm_base(pole_m)
        lift_ref = np.zeros(3)
        if hover_mm:
            sp = self.cfg.cartesian_poses.get("start_pose")
            pp = self.cfg.cartesian_poses.get("pre_start_pose")
            if not (sp and pp):
                raise SystemExit(
                    f"task {self.cfg.task!r} declares no pre_start_pose — "
                    "no retreat vector to hover along. Run without --hover "
                    "or add the pose to the task config.")
            u = np.asarray(pp[:3], float) - np.asarray(sp[:3], float)
            n = np.linalg.norm(u)
            if n < 1e-6:
                raise SystemExit("start and pre_start coincide — no "
                                 "retreat direction")
            lift_ref = u / n * (float(hover_mm) / 1000.0)
        poses, names = [], []
        for name, T_ref in self.waypoints_ref():
            T_ref = T_ref.copy()
            T_ref[:3, 3] = T_ref[:3, 3] + lift_ref   # away from the surface
            M = T @ T_ref
            rx, ry, rz = R_to_euler(M[:3, :3])
            poses.append([float(M[0, 3]), float(M[1, 3]), float(M[2, 3]),
                          rx, ry, rz])
            names.append(name)
        return {
            "task": self.cfg.task,
            "side": self.cfg.side,
            "tool_frame": controller_frame_name(self.cfg.ik_frame),
            "pole_m": pole_m,
            "blend_pct": blend_pct,
            "hover_mm": float(hover_mm),
            "waypoint_names": names,
            "poses": poses,
            "segments": len(poses) - 1,     # movel commands to issue
            "queue_depth": len(poses) - 1,  # all queued (Newton)
        }


def _describe(task, pole_m):
    from task_config import TaskConfig
    cfg = TaskConfig.load(task)
    p = CleaningPath(cfg)
    prog = p.movel_program(pole_m)
    print(f"  {task}  [{prog['side']}]  tool frame {prog['tool_frame']}  "
          f"(ik {cfg.ik_frame})")
    print(f"  {len(prog['poses'])} waypoints -> {prog['segments']} chained "
          f"movel segments, blend r={prog['blend_pct']}%, pole "
          f"{prog['pole_m']:.3f} m")
    print(f"  {'#':>3s} {'point':10s} {'x':>9s} {'y':>9s} {'z':>9s}   "
          f"{'rx':>7s} {'ry':>7s} {'rz':>7s}   (m, rad, arm base frame)")
    for i, (n, q) in enumerate(zip(prog["waypoint_names"], prog["poses"])):
        if i < 4 or i == len(prog["poses"]) - 1:
            print(f"  {i:3d} {n:10s} {q[0]:9.4f} {q[1]:9.4f} {q[2]:9.4f}   "
                  f"{q[3]:7.3f} {q[4]:7.3f} {q[5]:7.3f}")
        elif i == 4:
            print(f"  {'...':>3s}")
    if prog["queue_depth"] > 20:
        print(f"  NOTE queue depth {prog['queue_depth']} exceeds the 20 "
              "verified by C10 — probe before trusting a single chain")


if __name__ == "__main__":
    tasks = sys.argv[1:] or ["hinge_area_right", "toplid_right",
                             "toplid_left"]
    for t in tasks:
        print("=" * 74)
        _describe(t, pole_m=0.075)
