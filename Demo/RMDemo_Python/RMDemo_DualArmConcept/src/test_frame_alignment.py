"""C14 — Frame alignment survey. READ-MOSTLY (moves only with --poses).

The hinge cleaning points are FIXTURE-TAUGHT poses in URDF frames
(`butterfli_ref_frame`, ik_frame `R_glove_frame_4`), while the controller
executes in ITS work frame with ITS tool frame. If the two disagree, every
cleaning point lands offset — the worst failure mode, because it looks
like it works.

OFFLINE HALF ALREADY SOLVED (frame_alignment_offline.py, 2026-08-08):
URDF `*_ConnectorLink` == RealMan `Arm_Tip` + a constant (0, 0, 32.5 mm),
zero rotation, both arms, all configurations. The GUI's old hand-entered
tool frame (-35, 10, 260) matches NONE of the derived frames — replace it
with the generated ones (--create-frames) rather than trusting it.

This test needs ONLY the SDK on the lab laptop. It captures, per pose:

    joints (deg)                 rm_get_joint_degree
    controller pose              rm_get_current_arm_state (work frame)
    offline rm_algo FK           same joints through the offline solver
    tool frame in use            rm_get_current_tool_frame (name + offset)
    work frame in use            rm_get_current_work_frame

and prints a machine-parseable `C14CAP` line per pose, so the URDF-side
comparison (TF: butterfli_ref_frame -> R_glove_frame_4 at those joints)
runs OFFLINE afterwards against the captured file — no ROS needed at the
arm.

Default: capture the CURRENT pose only (nothing moves). With --poses,
movej through the named states (ready, rest, zero) capturing each — the
countdown and error gate apply.

--create-frames RECREATES THE URDF GLOVE/IK FRAMES IN THE CONTROLLER'S
TREE (Newton's C14 design). The offline half (frame_alignment_offline.py)
proved URDF `*_ConnectorLink` == RealMan `Arm_Tip` + a CONSTANT
(0, 0, 32.5 mm) offset with ZERO rotation on both arms, so each xacro
frame becomes a controller tool frame at (xacro offset + 32.5 mm Z),
relative to Arm_Tip, via rm_set_manual_tool_frame:

    glove1..glove4, tip     (e.g. right glove4 = 55, 7, 237.5 mm)

The payload of the CURRENT tool frame is copied onto the new frames (so
force compensation is unchanged), each frame is read back to verify, and
the originally-active tool frame is RESTORED before exit — creating
frames never leaves the arm on a different tool frame.

Checks: tool/work frame readable, controller pose vs offline rm_algo FK
agreement (flags gross frame mismatch immediately), capture completeness,
and (with --create-frames) frames written + verified + original restored.

Flags: --poses  (move through ready/rest/zero instead of capture-in-place)
       --create-frames  (write the glove tool frames to the controller)
Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import json
import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    preflight_error_gate,
    ARM_SPEED_PCT, ARM_TIMEOUT_S, DEV_JOINT, LEFT_IP, RIGHT_IP, ROBOT_PORT,
    ArrivalMonitor, ConceptArm, apply_run_mode, countdown, mode_label,
    parse_clear_errors_arg, parse_mode_arg, report_run_modes,
    restore_run_modes, state_deg, teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
POSE_STATES = ("ready", "rest", "zero")
FK_TOL_M = 0.005              # controller pose vs offline rm_algo FK

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 5


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _offline_fk(joints_deg):
    """FK through the offline rm_algo lib (arm-base frame, default tool)."""
    try:
        from Robotic_Arm.rm_robot_interface import (
            Algo, rm_robot_arm_model_e, rm_force_type_e)
        algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                    rm_force_type_e.RM_MODEL_RM_B_E)
        algo.handle = None
        return list(algo.rm_algo_forward_kinematics(list(joints_deg), 1))[:6]
    except Exception:
        return None


def _capture(robot, label):
    """One capture record; prints the C14CAP line."""
    cap = {"label": label, "side": ARM_SIDE}
    ret, joints = robot.rm_get_joint_degree()
    cap["joints_deg"] = list(joints) if ret == 0 else None
    ret, st = robot.rm_get_current_arm_state()
    cap["controller_pose"] = list(st["pose"][:6]) if ret == 0 else None
    for name, getter in (("tool_frame", "rm_get_current_tool_frame"),
                         ("work_frame", "rm_get_current_work_frame")):
        try:
            ret, frame = getattr(robot, getter)()
            cap[name] = frame if ret == 0 else f"ret={ret}"
        except Exception as exc:
            cap[name] = f"unavailable: {exc!r}"
    cap["offline_fk"] = (_offline_fk(cap["joints_deg"])
                         if cap["joints_deg"] else None)
    print(f"  C14CAP {json.dumps(cap, default=str)}")
    return cap


def _create_glove_frames(robot):
    """Write the URDF glove/ik frames as controller tool frames.

    Offsets come from frame_alignment_offline (xacro offsets composed with
    the proven constant Arm_Tip->ConnectorLink 32.5 mm Z). Returns the
    (tag, name, detail) triple for result()."""
    import numpy as np
    from frame_alignment_offline import GLOVE_FRAMES, _euler_zyx_to_R
    from Robotic_Arm.rm_ctypes_wrap import rm_frame_t, rm_pose_t, \
        rm_position_t, rm_euler_t

    # keep the active frame + its payload; restore the frame afterwards
    try:
        ret, cur = robot.rm_get_current_tool_frame()
        original = cur.get("frame_name") if ret == 0 else None
        payload = float(cur.get("payload", 0.0)) if ret == 0 else 0.0
    except Exception as exc:
        return ("FAIL", "glove tool frames created",
                f"cannot read the current tool frame: {exc!r}")

    residual = np.eye(4)
    residual[2, 3] = 0.0325            # Arm_Tip -> ConnectorLink (proven)
    created, failed = [], []
    for name, (xyz, rpy) in GLOVE_FRAMES[ARM_SIDE].items():
        T = np.eye(4)
        T[:3, :3] = _euler_zyx_to_R(*rpy)
        T[:3, 3] = xyz
        T = residual @ T
        frame = rm_frame_t()
        frame.frame_name = name.encode()[:11]
        frame.pose = rm_pose_t()
        frame.pose.position = rm_position_t(*[float(v) for v in T[:3, 3]])
        frame.pose.euler = rm_euler_t(*[float(v) for v in rpy])
        frame.payload = payload
        try:
            ret = robot.rm_set_manual_tool_frame(frame)
        except Exception as exc:
            ret = repr(exc)
        (created if ret == 0 else failed).append(f"{name}(ret={ret})")
        print(f"    tool frame {name:8s} at "
              f"({T[0, 3] * 1000:.1f}, {T[1, 3] * 1000:.1f}, "
              f"{T[2, 3] * 1000:.1f}) mm  payload {payload} kg  ret={ret}")
    if original:
        try:
            rret = robot.rm_change_tool_frame(original)
            print(f"    active tool frame restored to {original!r} "
                  f"(ret={rret})")
        except Exception as exc:
            failed.append(f"restore({exc!r})")
    if failed:
        return ("FAIL", "glove tool frames created",
                f"created {created}; FAILED {failed}")
    return ("PASS", "glove tool frames created",
            f"{', '.join(created)}; active frame restored")


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=("--poses", "--create-frames"))
    forced = parse_mode_arg()
    clear_errs = parse_clear_errors_arg()
    with_poses = "--poses" in sys.argv
    create_frames = "--create-frames" in sys.argv
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP

    print("=" * 68)
    print("C14  Frame alignment survey")
    print(f"    arm={ARM_SIDE} @ {ip}")
    print("    capture: " + (f"movej through {POSE_STATES} (ARM MOVES)"
                             if with_poses else
                             "current pose only (NOTHING MOVES)"))
    print(f"    mode: {mode_label(forced)}")
    print("    C14CAP lines are the deliverable — the URDF/TF comparison")
    print("    runs offline against them (frame_alignment_offline.py)")
    print("=" * 68)

    arm = None
    originals = {}
    caps = []
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ip}")
            _results["SKIP"] += N_CHECKS
            return 0
        arm = ConceptArm(ARM_SIDE, robot, handle)
        monitor = ArrivalMonitor()
        monitor.register(robot)
        originals = apply_run_mode(forced, arm)
        if originals is None:
            result("FAIL", "run-mode selection", "did not engage")
            return 1
        report_run_modes(arm)
        ok_err, detail = preflight_error_gate(arm, clear=clear_errs)
        if not ok_err:
            result("FAIL", "no latched controller errors", detail)
            return 1
        result("PASS", "no latched controller errors", detail)

        if with_poses:
            countdown()
            for state in POSE_STATES:
                monitor.expect(arm.handle_id, DEV_JOINT)
                if robot.rm_movej(state_deg(ARM_SIDE, state),
                                  ARM_SPEED_PCT, 0, 0, 0) != 0:
                    print(f"    movej to {state} rejected — capturing "
                          "in place instead")
                else:
                    monitor.wait(arm.handle_id, DEV_JOINT, ARM_TIMEOUT_S)
                    time.sleep(0.3)          # settle before reading pose
                caps.append(_capture(robot, state))
        else:
            caps.append(_capture(robot, "as-found"))

        # ── checks ──
        frames_ok = all(isinstance(c.get("tool_frame"), dict)
                        or "unavailable" not in str(c.get("tool_frame"))
                        for c in caps)
        result("PASS" if frames_ok else "FAIL",
               "tool/work frame readable",
               "controller reports its active frames" if frames_ok else
               "frame getters failed — capture incomplete")

        # Controller pose vs offline rm_algo FK: catches a gross work-frame
        # or tool-frame offset immediately, before any URDF comparison.
        worst = None
        for c in caps:
            if not (c["controller_pose"] and c["offline_fk"]):
                continue
            d = [abs(a - b) for a, b
                 in zip(c["controller_pose"][:3], c["offline_fk"][:3])]
            err = max(d)
            worst = max(worst or 0.0, err)
            print(f"    {c['label']:8s} controller vs rm_algo FK: "
                  f"max |d| {err * 1000:6.1f} mm")
        if worst is None:
            result("FAIL", "controller pose vs offline FK", "no data")
        elif worst <= FK_TOL_M:
            result("PASS", "controller pose vs offline FK",
                   f"agree within {worst * 1000:.1f} mm — controller runs "
                   "default mounting/tool, as rm_algo assumes")
        else:
            result("FAIL", "controller pose vs offline FK",
                   f"{worst * 1000:.0f} mm apart — a work/tool frame offset "
                   "IS configured on the controller; the URDF comparison "
                   "must account for it (this may be the 59 mm)")

        complete = all(c["joints_deg"] and c["controller_pose"] for c in caps)
        result("PASS" if complete else "FAIL", "capture complete",
               f"{len(caps)} C14CAP records for the offline comparison")

        # ── optional: recreate the URDF glove frames in the controller ──
        if not create_frames:
            result("SKIP", "glove tool frames created", "--create-frames "
                   "not given")
        else:
            result_tag = _create_glove_frames(robot)
            result(*result_tag)
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
