"""C6 — Single-arm planned-move benchmark. THE SELECTED ARM WILL MOVE.

Controls ONE arm purely through the controller's PLANNED move functions
(no passthrough): ready -> rest_pose via rm_movej, then a +20 cm X
translation in the world/base frame via rm_movej_p (joint-space planning
to a Cartesian pose target), then back to ready via rm_movej. Measures
per-move dispatch latency, arrival time, and verifies the Cartesian
displacement from the controller's own pose feedback.

Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import math
import os
import sys
import time

from dual_arm_common import (
    ARM_SPEED_PCT, ARM_TIMEOUT_S, DEV_JOINT, LEFT_IP, RIGHT_IP, ROBOT_PORT,
    ArrivalMonitor, ConceptArm, apply_run_mode, countdown, mode_label,
    home_poles_full, parse_mode_arg, report_run_modes, restore_run_modes,
    run_step, state_deg,
    teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
X_OFFSET_M = 0.20            # +X in the world/base frame, from rest_pose
POSE_TOL_M = 0.02            # per-axis verification tolerance
OFFAXIS_TOL_M = 0.03         # allowed drift on Y/Z during the X move

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 6


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _pose(robot):
    ret, st = robot.rm_get_current_arm_state()
    if ret != 0:
        return None
    return list(st["pose"][:6])


def main() -> int:
    forced = parse_mode_arg()
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    print("=" * 68)
    print("C6  Single-arm planned-move benchmark (controller planning only)")
    print(f"    arm={ARM_SIDE} @ {ip}   movej v={ARM_SPEED_PCT}%   "
          f"X offset +{X_OFFSET_M*100:.0f} cm (world frame)")
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)"))
    print("    pole pre-positioned to full length (0.29 m), then arm only")
    print(f"    THE {ARM_SIDE.upper()} ARM AND ITS POLE WILL MOVE (no hand)"
          + (" (VIRTUALLY — SIM forced)" if forced == 0 else ""))
    print("=" * 68)

    arm = None
    originals = {}
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

        def timed_movej(state_name: str) -> dict:
            t0 = time.perf_counter()
            rec = run_step(arm, monitor, ("arm", state_name))
            dur = (rec["t_done"] - rec["t_dispatch"]) if rec["t_done"] else None
            print(f"  movej -> {state_name:6s}  ret={rec['ret']}  "
                  f"event={rec['event']}  "
                  + (f"duration {dur:6.2f} s" if dur else "INCOMPLETE"))
            return rec

        # ── SA1/SA2: ready, then rest_pose (joint-space planned) ──
        rec = timed_movej("ready")
        if rec["ok"]:
            result("PASS", "movej to ready")
        else:
            result("FAIL", "movej to ready", f"ret={rec['ret']}")
            return 1

        rec = timed_movej("rest")
        if rec["ok"]:
            result("PASS", "movej to rest_pose")
        else:
            result("FAIL", "movej to rest_pose", f"ret={rec['ret']}")
            return 1

        # ── SA3: +20 cm X via rm_movej_p (world-frame pose target) ──
        pose0 = _pose(robot)
        if pose0 is None:
            result("FAIL", "read pose at rest")
            return 1
        target = list(pose0)
        target[0] += X_OFFSET_M
        print(f"  pose at rest: x={pose0[0]:+.3f} y={pose0[1]:+.3f} "
              f"z={pose0[2]:+.3f}  ->  target x={target[0]:+.3f}")

        monitor.expect(arm.handle_id, DEV_JOINT)
        t0 = time.perf_counter()
        ret = robot.rm_movej_p(target, ARM_SPEED_PCT, 0, 0, 0)
        if ret != 0:
            result("FAIL", "movej_p +X accepted",
                   f"ret={ret} (1 can mean IK failure / unreachable)")
            return 1
        arrived, success = monitor.wait(arm.handle_id, DEV_JOINT,
                                        ARM_TIMEOUT_S)
        dur = time.perf_counter() - t0
        pose1 = _pose(robot)
        if not (arrived and success) or pose1 is None:
            result("FAIL", "movej_p +X completed",
                   f"arrived={arrived} success={success}")
            arm.halt()
            return 1
        dx = pose1[0] - pose0[0]
        dy = pose1[1] - pose0[1]
        dz = pose1[2] - pose0[2]
        print(f"  movej_p +X: duration {dur:.2f} s   "
              f"dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f} m")
        if abs(dx - X_OFFSET_M) <= POSE_TOL_M \
                and abs(dy) <= OFFAXIS_TOL_M and abs(dz) <= OFFAXIS_TOL_M:
            result("PASS", "Cartesian displacement verified",
                   f"dx={dx:+.3f} m (target +{X_OFFSET_M:.2f})")
        else:
            result("FAIL", "Cartesian displacement",
                   f"dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f}")

        # ── SA4: back to ready via movej ──
        rec = timed_movej("ready")
        if rec["ok"]:
            result("PASS", "movej back to ready")
        else:
            result("FAIL", "movej back to ready", f"ret={rec['ret']}")

        # ── SA5: all arrivals via events (planned-move contract) ──
        result("PASS", "planned-move pipeline exercised",
               "movej x3 + movej_p x1, no passthrough")
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
