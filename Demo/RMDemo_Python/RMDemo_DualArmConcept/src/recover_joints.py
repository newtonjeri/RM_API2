"""Joint recovery — clear errors and RE-ENABLE disabled joints. Deliberate.

Why this exists (2026-08-08): after a trajectory abort at speed, the left
arm's J6 and J7 latched "Under Voltage" and were DE-ENABLED. In that state

  * every motion command fails within ~0.1 s (any move touching a dead
    joint is rejected), and
  * TEACH MODE IS DANGEROUS: pressing the teach button releases the
    brakes on ALL joints, and an unpowered joint has nothing holding it —
    the wrist and hand drop under gravity, violently. Releasing the
    button clamps the brakes again mid-fall.

`rm_clear_system_err` does not touch this (F13), and a de-enabled joint is
not an "error flag" either — recovery is PER JOINT: clear its error, then
re-enable it, then verify.

Default is REPORT-ONLY. Enabling drive power is an action:

    RM_ARM=left python3 recover_joints.py                 # report state
    RM_ARM=left python3 recover_joints.py --enable 6,7    # clear + enable

SUPPORT THE ARM while enabling — if the underlying cause (supply sag,
e-stop circuit, harness) is still present, the joint may fault again
immediately. If Under Voltage returns at idle or right at boot, that is a
hardware/power problem for RealMan, not a software state to clear.
"""

import os
import sys
import time

from dual_arm_common import (
    handle_cli, error_state, describe_error_state, error_state_clean,
    describe_joint_err,
    LEFT_IP, RIGHT_IP, ROBOT_PORT, ConceptArm,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()


def _joint_report(robot):
    rows = []
    try:
        ret, en = robot.rm_get_joint_en_state()
        en = list(en) if ret == 0 else [None] * 7
    except Exception:
        en = [None] * 7
    try:
        jd = robot.rm_get_joint_err_flag()
        flags = list(jd.get("err_flag", [])) if jd.get(
            "return_code") == 0 else [None] * 7
    except Exception:
        flags = [None] * 7
    for i in range(7):
        rows.append((i + 1, en[i] if i < len(en) else None,
                     flags[i] if i < len(flags) else None))
    return rows


def main() -> int:
    handle_cli(__doc__, value_flags=("--enable",))
    enable = []
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == "--enable" and i + 1 < len(argv):
            enable = [int(x) for x in argv[i + 1].split(",") if x.strip()]
        elif a.startswith("--enable="):
            enable = [int(x) for x in a.split("=", 1)[1].split(",")
                      if x.strip()]
    bad = [j for j in enable if not 1 <= j <= 7]
    if bad:
        raise SystemExit(f"joint numbers must be 1..7, got {bad}")

    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    print("=" * 68)
    print(f"Joint recovery — {ARM_SIDE} arm @ {ip}")
    print("    " + ("REPORT ONLY (use --enable N,N to act)" if not enable
                    else f"will clear + ENABLE joints {enable} — "
                    "SUPPORT THE ARM"))
    print("=" * 68)

    robot = None
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] hardware not reachable at {ip}")
            return 0

        print("  as-found:")
        for j, en, flag in _joint_report(robot):
            state = ("ENABLED" if en else
                     "DISABLED" if en is not None else "?")
            print(f"    J{j}: {state:9s} err_flag={describe_joint_err(flag)}")
        arm = ConceptArm(ARM_SIDE, robot, handle)
        st = error_state(arm)
        print(f"  gate: {describe_error_state(st)}")

        if not enable:
            return 0 if error_state_clean(st) else 1

        for j in enable:
            cret = robot.rm_set_joint_clear_err(j)
            eret = robot.rm_set_joint_en_state(j, 1)
            print(f"  J{j}: clear_err ret={cret}, enable ret={eret}")
            time.sleep(0.3)

        time.sleep(1.0)
        print("  after:")
        ok = True
        for j, en, flag in _joint_report(robot):
            state = ("ENABLED" if en else
                     "DISABLED" if en is not None else "?")
            mark = ""
            if j in enable:
                good = bool(en) and not flag
                ok &= good
                mark = "  <-- recovered" if good else "  <-- STILL FAULTED"
            print(f"    J{j}: {state:9s} err_flag={describe_joint_err(flag)}{mark}")
        if not ok:
            print("\n  A joint that re-faults immediately points at the "
                  "CAUSE still being present:\n  check the e-stop is fully "
                  "released, the supply, and the arm harness —\n  that is "
                  "RealMan-support territory, not a state to keep clearing.")
        return 0 if ok else 1
    finally:
        if robot is not None:
            try:
                robot.rm_delete_robot_arm()
            except Exception:
                pass
            try:
                RoboticArm.rm_destroy()
            except Exception:
                pass


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
