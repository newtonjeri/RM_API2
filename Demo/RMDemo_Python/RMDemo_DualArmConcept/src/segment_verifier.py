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

# The offline rm_algo must be told WHICH RM75 variant it is modelling — the
# force-sensor version changes the wrist length, and therefore every FK
# result and the self-collision geometry.
#
# MEASURED 2026-08-08 against both controllers (C14 capture, joints ~0):
#   controller Arm_Tip  z = 0.867699 m   (right, tool=Arm_Tip, offset 0)
#   controller Arm_Tip  z = 0.867698 m   (left,  via tool 'Hand' z=0.259999)
#   rm_algo ISF         z = 0.867700 m   <-- matches to 1 um
#   rm_algo B (no F/T)  z = 0.850500 m   <-- 17.2 mm SHORT (was used here)
# RM75-6FB carries the integrated six-axis force sensor, so ISF is correct.
# RM_FORCE_MODEL overrides it if a variant is ever swapped.
FORCE_MODEL_NAME = os.environ.get("RM_FORCE_MODEL", "RM_MODEL_RM_ISF_E")


BUNDLED_PLANS = pathlib.Path(__file__).resolve().parent.parent / "plans"


def resolve_plan(name: str) -> pathlib.Path:
    """THE plan is the copy in this repo's ../plans/. No search, no fallback.

    An earlier version preferred whatever sat in the ROS workspace, and on
    2026-08-08 that produced exactly the failure the bundling was meant to
    prevent: the lab machine's workspace held a DIFFERENT plan under the
    same filename (stroke stage `execute_cleaning_path`, 2012 wp, plus an
    extra retreat stage), so the capture rehearsed one plan while the dev
    machine had verified another. Both runs looked fine.

    The plan is a versioned artifact of this repo, like the code: whatever
    is committed here is what every machine runs. Point somewhere else
    deliberately with `--plan PATH` — never by accident, never by which
    machine you happen to be on.
    """
    return BUNDLED_PLANS / name


def _arm_link_filter(side: str):
    """Right/left arm + its hand + glove — the moving geometry."""
    pref = ("R_", "IRH_") if side == "right" else ("L_", "ILH_")

    def keep(link: str) -> bool:
        return link.startswith(pref)
    return keep


class SegmentVerifier:
    """Robot-vs-fixture clearance + self-collision for joint timelines."""

    def __init__(self, fixture: str = "commode_c", side: str = "right",
                 quiet: bool = False, articulation: str = None):
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
        # The commode is ARTICULATED: scene.yaml gives lid and seat a joint
        # with closed=0 deg and open=-110 deg, and the manifest exposes BOTH
        # posed variants. Only one exists at a time. Every commode task
        # declares `commode_fixture_type: closed`, so checking against
        # lid_open / seat_open collides the arm with geometry rotated 110
        # deg out of the way — which is what put phantom contacts in
        # move_to_pre_start and move_to_rest, stages that run OUTSIDE the
        # allow_collision bracket and must be clear.
        keys = list(manifest["meshes"])
        if articulation:
            other = "open" if articulation == "closed" else "closed"
            keys = [k for k in keys if not k.endswith(f"_{other}")]
            if not quiet:
                dropped = set(manifest["meshes"]) - set(keys)
                print(f"  [verifier] articulation {articulation!r}: using "
                      f"{sorted(keys)}, ignoring {sorted(dropped)}")
        self.fixture_mgrs = {}
        for key, md in manifest["meshes"].items():
            if key not in keys:
                continue
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
        """RealMan's own solver/self-collision check (offline lib).

        Shared by every offline consumer so the modelled arm variant is
        defined in exactly ONE place — see FORCE_MODEL_NAME above.
        """
        try:
            rm = pathlib.Path(__file__).resolve().parents[4] / "Python"
            if str(rm) not in sys.path:
                sys.path.insert(0, str(rm))
            from Robotic_Arm.rm_robot_interface import (
                Algo, rm_robot_arm_model_e, rm_force_type_e)
            algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                        getattr(rm_force_type_e, FORCE_MODEL_NAME))
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

    def contact_report(self, joint_map: dict) -> dict:
        """{fixture mesh key -> set of ROBOT LINKS touching it}.

        The task sequence brackets its cleaning with
        `allow_collision(contact_surface=...)` / `forbid_collision`, so
        "is anything in collision" is the wrong question there — contact
        is the job. The answerable question is the one contact_links.yaml
        poses: WHICH links touch WHICH objects. FCL already knows; the
        objects are registered as `link#geom_index`, so the link name
        falls out of the contact pair.
        """
        full = dict(self.home)
        full.update(joint_map)
        self.posed.set_pose(full)
        out = {}
        for key, mgr in self.fixture_mgrs.items():
            hit, data = self.posed._mgr.in_collision_other(
                mgr, return_data=True)
            if not hit:
                continue
            links = set()
            for c in data:
                for nm in getattr(c, "names", ()):
                    if "#" in nm:                 # ours: 'link#geom'
                        links.add(nm.split("#", 1)[0])
            out[key] = links
        return out

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


def plan_state_upto(plan, stage_index):
    """Joint state established by every stage BEFORE `stage_index`.

    The task sequence moves the pole and the hand in their own stages, and
    the pole is PRISMATIC AND CARRIES THE ARM BASE — hinge_area_right runs
    `move_pole_to_quarter_height` (0.075 m) before the stroke, while the
    SRDF home state has the pole at 0.29 m. Verifying an arm stage against
    the home state therefore placed the whole arm 215 mm too high relative
    to the fixture, and every clearance figure computed that way was wrong
    (found 2026-08-08 while validating the config-derived Cartesian path).

    So: the base state for an arm stage is the SRDF home overridden by the
    final value of each joint any earlier stage commanded.
    """
    state = {}
    for st in plan["sub_trajectories"][:stage_index]:
        if not st.get("waypoints"):
            continue
        last = st["waypoints"][-1]["positions"]
        state.update(dict(zip(st["joint_names"], last)))
    return state


def subsample(seq, n):
    """n evenly spaced elements, first and last always included."""
    if len(seq) <= n:
        return list(seq)
    step = (len(seq) - 1) / (n - 1)
    return [seq[round(i * step)] for i in range(n)]
