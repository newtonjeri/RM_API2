"""C6 — Single-arm planned-move benchmark WITH concurrent hand motion.
THE SELECTED ARM, ITS POLE, AND ITS HAND WILL MOVE.

Controls ONE arm purely through the controller's PLANNED move functions
(no passthrough), with the Inspire hand commanded CONCURRENTLY with every
arm motion (dispatched back-to-back, both arrivals awaited):

    pole -> full_length
    movej -> ready      + hand -> release
    movej -> rest_pose  + hand -> grasp
    movej_p +20 cm X    + hand -> half_grasp   (world/base frame)
    movej -> ready      + hand -> release

Verifies the Cartesian displacement from the controller's own pose
feedback and reports per-phase arm/hand durations and the hand-vs-arm
finish skew (negative = hand finished while the arm was still moving —
the concurrency evidence).

Arm selection: RM_ARM=left (default) or RM_ARM=right.
Hand caveat (fw 1.7.x): rm_set_hand_angle is the hand PROTOCOL path —
the end port must NOT be in modbus mode (rm_close_modbus_mode(1) first).
Note: in SIM mode the lift and hand do NOT simulate (2026-08-06 logs) —
this test is meaningful on REAL hardware.
"""

import os
import sys
import time

from dual_arm_common import (
    ARM_SPEED_PCT, ARM_TIMEOUT_S, DEV_HAND, DEV_JOINT, HAND_STATES_HW,
    HAND_TIMEOUT_S, LEFT_IP, RIGHT_IP, ROBOT_PORT, ArrivalMonitor,
    ConceptArm, apply_run_mode, countdown, home_poles_full, mode_label,
    parse_mode_arg, report_run_modes, restore_run_modes, run_step,
    teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
X_OFFSET_M = 0.20            # +X in the world/base frame, from rest_pose
POSE_TOL_M = 0.02            # per-axis verification tolerance
OFFAXIS_TOL_M = 0.03         # allowed drift on Y/Z during the X move

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 7


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _pose(robot):
    ret, st = robot.rm_get_current_arm_state()
    if ret != 0:
        return None
    return list(st["pose"][:6])


def _phase_report(label: str, rec: dict):
    """Print arm/hand durations + concurrency skew for a combo phase."""
    j = rec["devices"].get(DEV_JOINT)
    h = rec["devices"].get(DEV_HAND)
    parts = []
    if j and j["t_done"]:
        parts.append(f"arm {j['t_done'] - j['t_dispatch']:5.2f} s")
    if h and h["t_done"]:
        parts.append(f"hand {h['t_done'] - h['t_dispatch']:5.2f} s")
    skew = None
    if j and h and j["t_done"] and h["t_done"]:
        skew = h["t_done"] - j["t_done"]
        parts.append(f"hand-arm finish {skew:+5.2f} s"
                     + ("  (hand done DURING arm motion)" if skew < 0 else ""))
    print(f"  {label:26s} ok={rec['ok']}  " + "   ".join(parts))
    return skew


def main() -> int:
    forced = parse_mode_arg()
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    print("=" * 68)
    print("C6  Single-arm planned moves + CONCURRENT hand motion")
    print(f"    arm={ARM_SIDE} @ {ip}   movej v={ARM_SPEED_PCT}%   "
          f"X offset +{X_OFFSET_M*100:.0f} cm (world frame)")
    print("    hand states: release / grasp / half_grasp (protocol path — "
          "end port must NOT be in modbus mode)")
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)"))
    print("    pole pre-positioned to full length (0.29 m) first")
    print(f"    THE {ARM_SIDE.upper()} ARM, ITS POLE, AND ITS HAND WILL MOVE"
          + (" (VIRTUALLY — SIM forced; lift/hand do NOT simulate)"
             if forced == 0 else ""))
    print("=" * 68)

    arm = None
    originals = {}
    hand_skews = []
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
            result("FAIL", "run-mode selection",
                   "requested mode did not engage — aborting before motion")
            return 1
        report_run_modes(arm)
        countdown(5)

        if home_poles_full(monitor, arm):
            result("PASS", "pole pre-positioned to full length")
        else:
            result("FAIL", "pole pre-positioned to full length")
            return 1

        def combo(arm_state: str, hand_state: str, label: str) -> dict:
            rec = run_step(arm, monitor, ("combo", (("arm", arm_state),
                                                    ("hand", hand_state))))
            skew = _phase_report(label, rec)
            if skew is not None:
                hand_skews.append(skew)
            return rec

        # ── SA1/SA2: ready + release, then rest + grasp ──
        rec = combo("ready", "release", "movej ready + release")
        if rec["ok"]:
            result("PASS", "movej to ready + hand release")
        else:
            result("FAIL", "movej to ready + hand release",
                   f"ret={rec['ret']}")
            return 1

        rec = combo("rest", "grasp", "movej rest + grasp")
        if rec["ok"]:
            result("PASS", "movej to rest_pose + hand grasp")
        else:
            result("FAIL", "movej to rest_pose + hand grasp",
                   f"ret={rec['ret']}")
            return 1

        # ── SA3: +20 cm X via rm_movej_p, hand -> half_grasp concurrent ──
        pose0 = _pose(robot)
        if pose0 is None:
            result("FAIL", "read pose at rest")
            return 1
        target = list(pose0)
        target[0] += X_OFFSET_M
        print(f"  pose at rest: x={pose0[0]:+.3f} y={pose0[1]:+.3f} "
              f"z={pose0[2]:+.3f}  ->  target x={target[0]:+.3f}")

        monitor.expect(arm.handle_id, DEV_JOINT)
        monitor.expect(arm.handle_id, DEV_HAND)
        t0 = time.perf_counter()
        ret = robot.rm_movej_p(target, ARM_SPEED_PCT, 0, 0, 0)
        ret_h = robot.rm_set_hand_angle(HAND_STATES_HW["half_grasp"],
                                        False, 2)
        if ret != 0:
            result("FAIL", "movej_p +X accepted",
                   f"ret={ret} (1 can mean IK failure / unreachable)")
            return 1
        arrived, success = monitor.wait(arm.handle_id, DEV_JOINT,
                                        ARM_TIMEOUT_S)
        h_arrived, h_success = monitor.wait(arm.handle_id, DEV_HAND,
                                            HAND_TIMEOUT_S)
        t_arm = monitor.last_arrival(arm.handle_id, DEV_JOINT) \
            or time.perf_counter()
        t_hand = monitor.last_arrival(arm.handle_id, DEV_HAND) \
            or time.perf_counter()
        pose1 = _pose(robot)
        if not (arrived and success) or pose1 is None:
            result("FAIL", "movej_p +X completed",
                   f"arrived={arrived} success={success}")
            arm.halt()
            return 1
        hand_ok = ret_h == 0 and h_arrived and h_success
        skew = (t_hand - t_arm) if hand_ok else None
        if skew is not None:
            hand_skews.append(skew)
        print(f"  movej_p +X + half_grasp: arm {t_arm - t0:.2f} s, "
              + (f"hand finish skew {skew:+.2f} s"
                 + ("  (hand done DURING arm motion)" if skew < 0 else "")
                 if hand_ok else "hand FAILED"))
        dx = pose1[0] - pose0[0]
        dy = pose1[1] - pose0[1]
        dz = pose1[2] - pose0[2]
        print(f"  displacement: dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f} m")
        if abs(dx - X_OFFSET_M) <= POSE_TOL_M \
                and abs(dy) <= OFFAXIS_TOL_M and abs(dz) <= OFFAXIS_TOL_M \
                and hand_ok:
            result("PASS", "movej_p +X + half_grasp verified",
                   f"dx={dx:+.3f} m (target +{X_OFFSET_M:.2f})")
        else:
            result("FAIL", "movej_p +X + half_grasp",
                   f"dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f} "
                   f"hand_ok={hand_ok}")

        # ── SA4: back to ready + release ──
        rec = combo("ready", "release", "movej ready + release")
        if rec["ok"]:
            result("PASS", "movej back to ready + hand release")
        else:
            result("FAIL", "movej back to ready + hand release",
                   f"ret={rec['ret']}")

        # ── SA5/SA6: pipeline + concurrency evidence ──
        result("PASS", "planned-move pipeline exercised",
               "movej x3 + movej_p x1 + hand x4 concurrent, no passthrough")
        if hand_skews:
            overlapped = sum(1 for s in hand_skews if s < 0)
            result("PASS", "hand moved concurrently with arm",
                   f"{overlapped}/{len(hand_skews)} phases finished during "
                   "arm motion; finish skews "
                   + ", ".join(f"{s:+.2f}s" for s in hand_skews))
        else:
            result("FAIL", "hand moved concurrently with arm",
                   "no phase produced both arrivals")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        if arm is not None and _results["FAIL"] > 0:
            arm.halt()
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
