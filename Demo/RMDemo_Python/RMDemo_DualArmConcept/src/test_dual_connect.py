"""C1 — Dual-arm connectivity and state pre-check. NO MOTION IS COMMANDED.

Run this first: it validates both connections, arm identity, error state,
lift feedback, run mode, and event-callback registration for the two-handle
single-process topology the motion tests rely on.
"""

import sys

from dual_arm_common import (
    LEFT_IP, RIGHT_IP, ArrivalMonitor, connect_both, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 9


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def main() -> int:
    print("=" * 68)
    print("C1  Dual-arm connectivity pre-check  (no motion is commanded)")
    print(f"    left={LEFT_IP}  right={RIGHT_IP}")
    print("=" * 68)

    left = right = None
    try:
        left, right = connect_both()
        if left is None:
            print(f"  [SKIP] Hardware not reachable at {LEFT_IP} / {RIGHT_IP}")
            _results["SKIP"] += N_CHECKS
            return 0

        for arm in (left, right):
            ret, info = arm.robot.rm_get_robot_info()
            if ret == 0 and info.get("arm_dof") == 7 \
                    and str(info.get("arm_model")) == "RM_75":
                result("PASS", f"{arm.side}: robot info",
                       f"RM_75 7-DOF, controller gen {info.get('robot_controller_version')}")
            else:
                result("FAIL", f"{arm.side}: robot info", f"ret={ret} info={info}")

            ret, st = arm.robot.rm_get_current_arm_state()
            errs = st.get("err", {}) if ret == 0 else {}
            err_len = errs.get("err_len", 0) if isinstance(errs, dict) else 0
            if ret == 0 and err_len == 0:
                result("PASS", f"{arm.side}: arm state clean")
            else:
                result("FAIL", f"{arm.side}: arm state", f"ret={ret} err={errs}")

            ret, lift = arm.robot.rm_get_lift_state()
            if ret == 0 and lift.get("err_flag", 1) == 0 \
                    and 0 <= lift.get("pos", -1) <= 200:   # hw range: 0.3 m * 2/3
                result("PASS", f"{arm.side}: lift state",
                       f"pos={lift.get('pos')} hw-mm")
            else:
                result("FAIL", f"{arm.side}: lift state", f"ret={ret} {lift}")

            ret, mode = arm.robot.rm_get_arm_run_mode()
            label = {0: "SIMULATION", 1: "REAL"}.get(mode, f"? ({mode})")
            print(f"  [INFO] {arm.side}: run mode = {label}")
            if mode == 0:
                print(f"  [WARN] {arm.side} is in SIMULATION mode — motion "
                      "tests will not move hardware")

        if left.handle_id != right.handle_id:
            result("PASS", "handles distinct",
                   f"left id={left.handle_id} right id={right.handle_id}")
        else:
            result("FAIL", "handles distinct", f"both id={left.handle_id}")

        monitor = ArrivalMonitor()
        try:
            monitor.register(left.robot)
            result("PASS", "event callback registered",
                   "process-global, demuxed by handle_id")
        except Exception as exc:
            result("FAIL", "event callback registered", repr(exc))

        result("PASS", "pre-check complete", "no motion was commanded")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        teardown(left, right)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
