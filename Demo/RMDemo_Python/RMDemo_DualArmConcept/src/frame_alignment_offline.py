"""C14 (offline half) — URDF ConnectorLink vs RealMan Arm_Tip.

Newton's design: the URDF's `*_ConnectorLink` is the parent of every glove /
ik frame; the controller's kinematic tree ends at `Arm_Tip`. If the two
coincide (same transform from the arm base at the same joints), the glove
frames can be RECREATED on the controller as tool frames using the exact
parent->child offsets from `ik_frames.xacro` — and every cleaning point can
then be commanded directly in the controller's tree.

This half needs NO hardware:

    URDF  FK   R_base_link -> R_ConnectorLink   (butterfli tree, which
               inserts R_CameraHolderLink between J6 and J7)
    rm_algo FK base -> Arm_Tip                  (RealMan's own solver =
               the controller's kinematic model, default tool)

compared at several joint configurations. Three possible outcomes:
  IDENTICAL          -> create the tool frames with the xacro offsets as-is
  CONSTANT offset    -> compose the residual into the tool-frame offsets
                        (printed ready to use)
  CONFIG-DEPENDENT   -> the trees genuinely differ (e.g. the camera holder
                        changes the joint axes) — do NOT create frames;
                        resolve the model first.

The verdict table at the end is the input for `test_frame_alignment.py
--create-frames` (the hardware half).

Usage: python3 frame_alignment_offline.py [--side right|left]
"""

import math
import pathlib
import sys

import numpy as np

from segment_verifier import WS  # workspace path + sys.path setup

RM_PY = pathlib.Path(__file__).resolve().parents[4] / "Python"
if str(RM_PY) not in sys.path:
    sys.path.insert(0, str(RM_PY))

URDF = WS / "butterfli_workspace" / "urdf" / "butterfli.urdf"

# ConnectorLink -> ik/glove frames (verbatim from ik_frames.xacro)
GLOVE_FRAMES = {
    "right": {
        "glove1": ((0.05, 0.0, 0.145), (0.0, 0.0, 0.0)),
        "glove2": ((0.0135, 0.0, 0.165), (0.0, 0.0, 0.0)),
        "glove3": ((0.075, 0.007, 0.17), (0.0, 0.0, 0.0)),
        "glove4": ((0.055, 0.007, 0.205), (0.0, 0.0, 0.0)),
        "tip": ((0.015, 0.005, 0.23), (0.0, 0.0, 0.0)),
    },
    "left": {
        "glove1": ((-0.05, 0.0, 0.14), (0.0, 0.0, 0.0)),
        "glove2": ((-0.02, 0.0, 0.165), (0.0, 0.0, 0.0)),
        "glove3": ((-0.075, 0.007, 0.17), (0.0, 0.0, 0.0)),
        "glove4": ((-0.055, 0.007, 0.205), (0.0, 0.0, 0.0)),
        "tip": ((-0.015, 0.005, 0.23), (0.0, 0.0, 0.0)),
    },
}

# Joint test set (degrees): named states + exercising every joint
CONFIGS = {
    "zero": [0, 0, 0, 0, 0, 0, 0],
    "ready": [0, -108.0, 0, 103.0, 0, 79.0, 0],
    "rest": [0, -82.0, -26.0, 98.0, 6.0, 62.0, 69.0],
    "j1_j5": [30, -60, 0, 60, 45, 30, 0],
    "j3_j7": [0, -45, 40, 80, 0, -30, 90],
}


def _euler_zyx_to_R(rx, ry, rz):
    """rm_algo pose euler (rx,ry,rz) -> rotation matrix (URDF rpy order)."""
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                              math.sin(ry), math.cos(rz), math.sin(rz))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rot_angle_deg(Ra, Rb):
    tr = np.trace(Ra.T @ Rb)
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def urdf_base_to_connector(model, side, joints_deg):
    """T(base_link_arm -> ConnectorLink) from the butterfli URDF."""
    pref = "R_" if side == "right" else "L_"
    jm = {f"{pref}joint{i + 1}": math.radians(q)
          for i, q in enumerate(joints_deg)}
    tw = model.link_world_transforms(jm)
    return np.linalg.inv(tw[f"{pref}base_link"]) @ tw[f"{pref}ConnectorLink"]


def algo_base_to_tip(algo, joints_deg):
    """T(base -> Arm_Tip) from RealMan's own solver (default tool)."""
    pose = list(algo.rm_algo_forward_kinematics(list(joints_deg), 1))[:6]
    T = np.eye(4)
    T[:3, :3] = _euler_zyx_to_R(*pose[3:6])
    T[:3, 3] = pose[:3]
    return T


def main() -> int:
    if "-h" in sys.argv or "--help" in sys.argv:
        print(__doc__)
        return 0
    side = "left" if "--side" in sys.argv and \
        sys.argv[sys.argv.index("--side") + 1] == "left" else "right"

    from butterfli_workspace.urdf_kinematics import UrdfModel
    from Robotic_Arm.rm_robot_interface import (
        Algo, rm_robot_arm_model_e, rm_force_type_e)
    model = UrdfModel.from_file(str(URDF))
    algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                rm_force_type_e.RM_MODEL_RM_B_E)
    algo.handle = None

    print("=" * 70)
    print(f"C14-offline  URDF {side} ConnectorLink  vs  rm_algo Arm_Tip")
    print("=" * 70)
    print(f"{'config':8s} {'|dT| mm':>9s} {'dRot deg':>9s}   "
          f"dT in Arm_Tip frame (mm)")
    residuals = []
    for name, q in CONFIGS.items():
        Tu = urdf_base_to_connector(model, side, q)
        Ta = algo_base_to_tip(algo, q)
        # residual expressed in the TIP frame: what tool-frame offset would
        # make Arm_Tip coincide with ConnectorLink at this configuration
        R = np.linalg.inv(Ta) @ Tu
        dt = R[:3, 3]
        ang = _rot_angle_deg(np.eye(3), R[:3, :3])
        residuals.append((name, R))
        print(f"{name:8s} {np.linalg.norm(dt) * 1000:9.2f} {ang:9.3f}   "
              f"[{dt[0] * 1000:7.2f} {dt[1] * 1000:7.2f} {dt[2] * 1000:7.2f}]")

    # constant across configurations?
    base = residuals[0][1]
    spread_t = max(np.linalg.norm(R[:3, 3] - base[:3, 3])
                   for _, R in residuals)
    spread_r = max(_rot_angle_deg(base[:3, :3], R[:3, :3])
                   for _, R in residuals)
    dt0 = base[:3, 3]
    print("-" * 70)
    print(f"residual spread across configs: {spread_t * 1000:.2f} mm, "
          f"{spread_r:.3f} deg")

    if spread_t > 0.002 or spread_r > 0.2:
        print("\nVERDICT: CONFIG-DEPENDENT mismatch — the kinematic trees "
              "genuinely differ.\nDo NOT create tool frames; resolve the "
              "URDF/controller model first (camera-holder joint axes are "
              "the prime suspect).")
        return 1

    identical = np.linalg.norm(dt0) < 0.001 and \
        _rot_angle_deg(np.eye(3), base[:3, :3]) < 0.1
    if identical:
        print("\nVERDICT: IDENTICAL — ConnectorLink IS Arm_Tip. Create the "
              "tool frames with the xacro offsets verbatim:")
        residual = np.eye(4)
    else:
        print(f"\nVERDICT: CONSTANT offset Arm_Tip->ConnectorLink = "
              f"[{dt0[0] * 1000:.2f} {dt0[1] * 1000:.2f} "
              f"{dt0[2] * 1000:.2f}] mm, {_rot_angle_deg(np.eye(3), base[:3, :3]):.3f} deg."
              "\nCompose it into the tool frames (already done below):")
        residual = base

    print(f"\nTool frames to create on the {side.upper()} controller "
          "(rm_set_manual_tool_frame, pose relative to Arm_Tip):")
    print(f"{'name':10s} {'x mm':>8s} {'y mm':>8s} {'z mm':>8s}   rpy rad")
    table = {}
    for fname, (xyz, rpy) in GLOVE_FRAMES[side].items():
        Tf = np.eye(4)
        Tf[:3, :3] = _euler_zyx_to_R(*rpy)
        Tf[:3, 3] = xyz
        Tt = residual @ Tf                    # Arm_Tip -> glove frame
        x, y, z = Tt[:3, 3]
        # rpy back out (zyx euler, matching rm_pose_t euler convention)
        ry = math.asin(max(-1, min(1, -Tt[2, 0])))
        rz = math.atan2(Tt[1, 0], Tt[0, 0])
        rx = math.atan2(Tt[2, 1], Tt[2, 2])
        table[fname] = [round(float(v), 6) for v in
                        (x, y, z, rx, ry, rz)]
        print(f"{fname:10s} {x * 1000:8.2f} {y * 1000:8.2f} {z * 1000:8.2f}"
              f"   [{rx:+.4f} {ry:+.4f} {rz:+.4f}]")
    print("\nMachine-readable (for test_frame_alignment.py --create-frames):")
    import json
    print("C14FRAMES " + json.dumps({"side": side, "frames": table}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
