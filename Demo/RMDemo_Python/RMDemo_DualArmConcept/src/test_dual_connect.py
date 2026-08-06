"""C1 — Dual-arm connectivity and state pre-check. NO MOTION IS COMMANDED.

Run this first: it validates both connections, arm identity, error state,
lift feedback, run mode, and event-callback registration for the two-handle
single-process topology the motion tests rely on.
"""

import sys

from dual_arm_common import (
    handle_cli,
    LEFT_IP, LIFT_GEAR, RIGHT_IP, ArrivalMonitor, connect_both, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 9


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def main() -> int:
    for k in _results:                 # reset: the emulated suite calls
        _results[k] = 0                # main() more than once per process
    handle_cli(__doc__)
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
            codes = errs.get("err", []) if isinstance(errs, dict) else []

            def _nonzero(code):
                try:
                    return int(str(code), 0) != 0
                except (TypeError, ValueError):
                    return True      # unparseable -> treat as an error

            active = [c for c in codes if _nonzero(c)]
            # Real V1.7.1 fw pads a clean arm as err_len=1, err=['0']
            # (observed 2026-08-06) — only nonzero codes are actual errors.
            if ret == 0 and not active:
                result("PASS", f"{arm.side}: arm state clean",
                       f"err codes {codes or '[]'}")
            else:
                result("FAIL", f"{arm.side}: arm state",
                       f"ret={ret} active={active or errs}")

            ret, lift = arm.robot.rm_get_lift_state()
            hw_max = LIFT_GEAR[arm.side]["hw_max"]
            if ret == 0 and lift.get("err_flag", 1) == 0 \
                    and 0 <= lift.get("pos", -1) <= hw_max:
                result("PASS", f"{arm.side}: lift state",
                       f"pos={lift.get('pos')} of {hw_max} "
                       f"({LIFT_GEAR[arm.side]['name']})")
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
