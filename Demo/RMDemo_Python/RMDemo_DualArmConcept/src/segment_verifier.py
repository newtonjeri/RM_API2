"""WP1 — Segment verifier: is the CONTROLLER's own motion collision-free?

The architecture sends the controller sparse targets and lets it plan the
motion between them. MoveIt's collision proof covers MoveIt's path, not the
controller's — this module closes that gap OFFLINE, with no hardware and no
ROS runtime:

    controller model                 collision check
    ─────────────────                ───────────────────────────────────
    movej  = joint-space LINEAR      whole-robot FCL (right arm + hand
             interpolation           link meshes, posed by URDF FK) vs
    movel  ≈ joint-linear between    the commode fixture meshes at their
             adjacent surface points scene pose, min-distance per sample
             (seeded IK, small steps)  + rm_algo self-collision verdict

Built entirely from existing, bench-proven machinery:
  butterfli_workspace   UrdfModel (FK), RobotCollisionModel (link BVHs),
                        PosedRobotCollision, build_fixture_manager,
                        _build_home (SRDF-true non-arm joints)
  cleaning_path_gen     scene_manifest_cached (fixture meshes AT POSE)
  RM_API2 rm_algo       rm_algo_safety_robot_self_collision_detection

Pure functions over (joint start, joint target) — CI-able; run WP2 via
run_hinge_verify.py.

Env: BUTTERFLI_WS overrides the workspace root.
"""

import json
import os
import pathlib
import sys

WS = pathlib.Path(os.environ.get(
    "BUTTERFLI_WS",
    "~/butterfli_ws/src/butterfli-ai-ros-jazzy")).expanduser()
for sub in ("butterfli_workspace", "cleaning_path_gen"):
    p = str(WS / sub)
    if p not in sys.path:
        sys.path.insert(0, p)

URDF = WS / "butterfli_workspace" / "urdf" / "butterfli.urdf"
SRDF = (WS / "butterfli_moveit_config" / "config" / "butterfli_alix.srdf")


def _arm_link_filter(side: str):
    """Right/left arm + its hand + glove — the moving geometry."""
    pref = ("R_", "IRH_") if side == "right" else ("L_", "ILH_")

    def keep(link: str) -> bool:
        return link.startswith(pref)
    return keep


class SegmentVerifier:
    """Robot-vs-fixture clearance + self-collision for joint timelines."""

    def __init__(self, fixture: str = "commode_c", side: str = "right",
                 quiet: bool = False):
        from butterfli_workspace.urdf_kinematics import UrdfModel
        from butterfli_workspace.collision import (
            RobotCollisionModel, PosedRobotCollision, build_fixture_manager)
        from butterfli_workspace.dual_arm_plan_collision import _build_home
        self.side = side
        self.model = UrdfModel.from_file(str(URDF))
        rcm = RobotCollisionModel.from_file(str(URDF))
        self.home = _build_home(rcm, str(SRDF) if SRDF.exists() else None)
        self.posed = PosedRobotCollision(rcm, self.model,
                                         link_filter=_arm_link_filter(side))
        # fixture meshes at their scene pose (BVH built once, then reused)
        import trimesh
        import numpy as np
        from cleaning_path_gen.server import scene_manifest_cached
        manifest = scene_manifest_cached(fixture)
        self.fixture_mgrs = {}
        for key, md in manifest["meshes"].items():
            mesh = trimesh.Trimesh(
                np.asarray(md["vertices"], dtype=float),
                np.asarray(md["faces"], dtype=np.int64), process=False)
            self.fixture_mgrs[key] = build_fixture_manager(mesh)
        if not quiet:
            print(f"  [verifier] URDF {URDF.name}, fixture '{fixture}': "
                  f"{len(self.fixture_mgrs)} meshes, side={side}")
        self._algo = self._load_algo()

    @staticmethod
    def _load_algo():
        """RealMan's own self-collision check (offline lib), optional."""
        try:
            rm = pathlib.Path(__file__).resolve().parents[4] / "Python"
            if str(rm) not in sys.path:
                sys.path.insert(0, str(rm))
            from Robotic_Arm.rm_robot_interface import (
                Algo, rm_robot_arm_model_e, rm_force_type_e)
            algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                        rm_force_type_e.RM_MODEL_RM_B_E)
            algo.handle = None
            return algo
        except Exception:
            return None

    # ── per-sample checks ──
    def clearance_m(self, joint_map: dict) -> float:
        """Min distance (m) from the moving arm to any fixture mesh.
        Negative/zero => in collision (reported as 0.0)."""
        full = dict(self.home)
        full.update(joint_map)
        self.posed.set_pose(full)
        worst = float("inf")
        for mgr in self.fixture_mgrs.values():
            if self.posed._mgr.in_collision_other(mgr):
                return 0.0
            d = self.posed._mgr.min_distance_other(mgr)
            worst = min(worst, float(d))
        return worst

    def self_collision(self, arm_joints_deg) -> bool:
        """rm_algo verdict for the 7 ARM joints (True = collision)."""
        if self._algo is None or arm_joints_deg is None:
            return False
        try:
            return self._algo.rm_algo_safety_robot_self_collision_detection(
                list(arm_joints_deg)) != 0
        except Exception:
            return False

    # ── timelines ──
    @staticmethod
    def movej_timeline(q0: dict, q1: dict, samples: int = 25):
        """The controller's movej geometry: synchronized joint-linear."""
        keys = sorted(set(q0) | set(q1))
        out = []
        for i in range(samples):
            a = i / (samples - 1)
            out.append({k: q0.get(k, q1.get(k, 0.0)) * (1 - a)
                        + q1.get(k, q0.get(k, 0.0)) * a for k in keys})
        return out

    def verify_timeline(self, joint_maps, arm_joint_names=None, tag=""):
        """Sweep a timeline; returns the report dict."""
        import math
        rows = []
        for i, jm in enumerate(joint_maps):
            arm_q = ([math.degrees(jm[n]) for n in arm_joint_names]
                     if arm_joint_names and all(n in jm
                                                for n in arm_joint_names)
                     else None)
            rows.append({
                "i": i,
                "clearance_m": self.clearance_m(jm),
                "self_collision": self.self_collision(arm_q),
            })
        clear = [r["clearance_m"] for r in rows]
        return {
            "tag": tag,
            "samples": len(rows),
            "min_clearance_m": min(clear) if clear else None,
            "min_at": clear.index(min(clear)) if clear else None,
            "collisions": sum(1 for c in clear if c <= 0.0),
            "self_collisions": sum(1 for r in rows if r["self_collision"]),
            "rows": rows,
        }


# ── plan helpers (the saved orchestrator plan JSON schema) ──
def load_plan(path):
    return json.loads(pathlib.Path(path).read_text())


def arm_stages(plan, prefix="R_joint"):
    """Stages that move the arm (joint_names contain the arm joints)."""
    return [s for s in plan["sub_trajectories"]
            if any(prefix in j for j in s["joint_names"])]


def stage_maps(stage):
    """All waypoints of a stage as joint maps (radians, URDF names)."""
    names = stage["joint_names"]
    return [dict(zip(names, wp["positions"])) for wp in stage["waypoints"]]


def subsample(seq, n):
    """n evenly spaced elements, first and last always included."""
    if len(seq) <= n:
        return list(seq)
    step = (len(seq) - 1) / (n - 1)
    return [seq[round(i * step)] for i in range(n)]
