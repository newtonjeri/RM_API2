"""C16 — Pure arm+pole concurrency baseline. NO MODELS, NO LAWS.

Every other concurrency test we have applies at least one of our own
models, so none of them is a clean base measurement:

  C9  sync steps   pole speed comes from matched_lift_speed_pct(), which
                   is itself driven by est_arm_duration_s() — an estimate
                   measured to be 24%-120% wrong. Two models between the
                   command and the observation.
  C15 phase B      law-free, but it never re-homes after an incomplete
                   leg (distances drift), never records the ARM duration
                   (no timing correlation), and stops at the first fault
                   (50/70/100% were never tested concurrently at all).

This test applies NOTHING. Both devices get a literal, fixed command:

    rm_set_lift_height(pole_speed, target_hw, 0)     # speed from the CLI
    rm_movej(pose, arm_speed, 0, 0, 0)               # speed from the CLI

and the only variables are the ones swept: the pole speed, the arm speed,
and the dispatch offset (negative = ARM first, positive = POLE first).
Every cell starts from a verified-identical state — pole re-homed to the
top, arm returned to ready, controller errors cleared and confirmed clean
— so cells are independent and can be compared directly.

Recorded per cell: both dispatch return codes, both true arrival times
(from the event push), both measured durations, the pole's final position
vs its target, and the controller/joint error state afterwards.

⚠ THIS TEST DELIBERATELY PROVOKES FAULTS. Each cell that halts an arm
mid-trajectory is an abrupt stop, so keep the matrix small and the e-stop
in reach. It clears latched errors between cells (and says so) because
otherwise the matrix could not continue past its first fault.

Flags: --pole-speeds 10,40,100  --arm-speed 20  --offsets -500,0,500
       --stroke MM  --cells-only (skip the phase-0 baselines)
Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    clear_errors, describe_error_state, error_state, error_state_clean,
    preflight_error_gate,
    ARM_TIMEOUT_S, DEV_JOINT, DEV_LIFT, LEFT_IP, LIFT_M, LIFT_TIMEOUT_S,
    RIGHT_IP, ROBOT_PORT, ArrivalMonitor, ConceptArm, apply_run_mode,
    countdown, lift_hw_mm, mode_label, parse_clear_errors_arg,
    parse_mode_arg, report_run_modes, restore_run_modes, state_deg,
    teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
DEFAULT_POLE_SPEEDS = [10, 40, 100]
DEFAULT_OFFSETS_MS = [0]
DEFAULT_ARM_SPEED = 20
DEFAULT_STROKE_MM = 100
POLE_TOL_HW = 3                # arrival position tolerance, hw-mm

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 5


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _arg(flag: str, default):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def _ints(text):
    return [int(t) for t in str(text).split(",") if t.strip()]


def _lift_pos(robot):
    try:
        ret, st = robot.rm_get_lift_state()
        return int(st.get("pos", -1)) if ret == 0 else None
    except Exception:
        return None


def _reset(arm, monitor, top_hw, ready_pose, pole_speed=50, arm_speed=20):
    """Return to the identical start state for every cell.

    arm -> ready, pole -> top, errors cleared and CONFIRMED clean.
    Returns (ok, note).
    """
    st = error_state(arm)
    if not error_state_clean(st):
        clear_errors(arm)
        st = error_state(arm)
        if not error_state_clean(st):
            return False, f"errors will not clear: {describe_error_state(st)}"
    # arm first, alone; then pole, alone — never together during a reset
    monitor.expect(arm.handle_id, DEV_JOINT)
    if arm.robot.rm_movej(ready_pose, arm_speed, 0, 0, 0) != 0:
        return False, "reset movej rejected"
    if not all(monitor.wait(arm.handle_id, DEV_JOINT, ARM_TIMEOUT_S)):
        return False, "reset movej did not arrive"
    monitor.expect(arm.handle_id, DEV_LIFT)
    if arm.robot.rm_set_lift_height(pole_speed, top_hw, 0) != 0:
        return False, "reset lift rejected"
    if not all(monitor.wait(arm.handle_id, DEV_LIFT, LIFT_TIMEOUT_S)):
        return False, "reset lift did not arrive"
    pos = _lift_pos(arm.robot)
    if pos is None or abs(pos - top_hw) > POLE_TOL_HW:
        return False, f"pole reset to {pos}, wanted {top_hw}"
    return True, "clean"


def _cell(arm, monitor, pole_speed, arm_speed, offset_ms, target_hw, pose):
    """One law-free concurrent dispatch. Returns a record dict."""
    rec = {"pole_pct": pole_speed, "arm_pct": arm_speed,
           "offset_ms": offset_ms}
    monitor.expect(arm.handle_id, DEV_LIFT)
    monitor.expect(arm.handle_id, DEV_JOINT)

    def send_pole():
        rec["t_pole_cmd"] = time.perf_counter()
        rec["pole_ret"] = arm.robot.rm_set_lift_height(
            int(pole_speed), int(target_hw), 0)

    def send_arm():
        rec["t_arm_cmd"] = time.perf_counter()
        rec["arm_ret"] = arm.robot.rm_movej(pose, int(arm_speed), 0, 0, 0)

    if offset_ms >= 0:                       # pole first (or together)
        send_pole()
        if offset_ms:
            time.sleep(offset_ms / 1000.0)
        send_arm()
    else:                                    # arm first
        send_arm()
        time.sleep(-offset_ms / 1000.0)
        send_pole()

    p_arr = monitor.wait(arm.handle_id, DEV_LIFT, LIFT_TIMEOUT_S)
    a_arr = monitor.wait(arm.handle_id, DEV_JOINT, ARM_TIMEOUT_S)
    t_pole = monitor.last_arrival(arm.handle_id, DEV_LIFT)
    t_arm = monitor.last_arrival(arm.handle_id, DEV_JOINT)
    rec["pole_ok"] = bool(p_arr[0] and p_arr[1])
    rec["arm_ok"] = bool(a_arr[0] and a_arr[1])
    rec["pole_s"] = (t_pole - rec["t_pole_cmd"]) if (t_pole and rec["pole_ok"]) else None
    rec["arm_s"] = (t_arm - rec["t_arm_cmd"]) if (t_arm and rec["arm_ok"]) else None
    rec["pole_end"] = _lift_pos(arm.robot)
    rec["pole_target"] = int(target_hw)
    rec["errors"] = describe_error_state(error_state(arm))
    return rec


def _print_cell(rec):
    order = ("together" if rec["offset_ms"] == 0 else
             f"pole+{rec['offset_ms']}ms" if rec["offset_ms"] > 0 else
             f"arm+{-rec['offset_ms']}ms")
    pole = (f"{rec['pole_s']:6.2f}s" if rec["pole_s"] else "  STALL ")
    armd = (f"{rec['arm_s']:6.2f}s" if rec["arm_s"] else "  STALL ")
    reach = ""
    if rec["pole_end"] is not None:
        short = rec["pole_target"] - rec["pole_end"]
        reach = f"  pole@{rec['pole_end']:3d}/{rec['pole_target']:3d}"
        if abs(short) > POLE_TOL_HW:
            reach += f" (short {abs(short)}mm)"
    print(f"    pole {rec['pole_pct']:3d}%  arm {rec['arm_pct']:3d}%  "
          f"{order:12s}  pole {pole}  arm {armd}{reach}"
          + ("" if rec["errors"] == "clean" else f"   [{rec['errors']}]"))


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=("--cells-only",),
               value_flags=("--pole-speeds", "--arm-speed", "--offsets",
                            "--stroke"))
    forced = parse_mode_arg()
    clear_errs = parse_clear_errors_arg()
    cells_only = "--cells-only" in sys.argv
    pole_speeds = _ints(_arg("--pole-speeds",
                             ",".join(map(str, DEFAULT_POLE_SPEEDS))))
    offsets = _ints(_arg("--offsets", ",".join(map(str, DEFAULT_OFFSETS_MS))))
    arm_speed = int(_arg("--arm-speed", DEFAULT_ARM_SPEED))
    stroke = int(_arg("--stroke", DEFAULT_STROKE_MM))
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    top_hw = lift_hw_mm(ARM_SIDE, LIFT_M["full"])
    bottom_hw = top_hw - stroke
    ready = state_deg(ARM_SIDE, "ready")
    zero = state_deg(ARM_SIDE, "zero")

    print("=" * 68)
    print("C16  Pure arm+pole concurrency baseline (NO models applied)")
    print(f"    arm={ARM_SIDE} @ {ip}   pole {top_hw} -> {bottom_hw} hw-mm "
          f"({stroke} mm)   arm ready -> zero")
    print(f"    pole speeds {pole_speeds}   arm speed {arm_speed}%   "
          f"offsets {offsets} ms  (-ve = arm first)")
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)")
          + "   [the lift does NOT execute in SIM]")
    print(f"    THE {ARM_SIDE.upper()} ARM AND ITS POLE WILL MOVE, "
          f"{len(pole_speeds) * len(offsets)} cells")
    print("    ⚠ this test PROVOKES faults by design; errors are cleared "
          "between cells")
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
            result("FAIL", "run-mode selection", "did not engage")
            return 1
        report_run_modes(arm)
        ok_err, detail = preflight_error_gate(arm, clear=clear_errs)
        if ok_err:
            result("PASS", "no latched controller errors", detail)
        else:
            result("FAIL", "no latched controller errors", detail)
            return 1
        countdown()

        ok, note = _reset(arm, monitor, top_hw, ready, arm_speed=arm_speed)
        if not ok:
            result("FAIL", "start state established", note)
            return 1
        result("PASS", "start state established", note)

        # ── Phase 0: each device ALONE (the control) ──
        if cells_only:
            result("SKIP", "singles baseline", "--cells-only")
        else:
            print("\n  Phase 0 — each device ALONE (the control)")
            print("  " + "─" * 64)
            singles_ok = True
            monitor.expect(arm.handle_id, DEV_JOINT)
            t0 = time.perf_counter()
            robot.rm_movej(zero, arm_speed, 0, 0, 0)
            a_ok = all(monitor.wait(arm.handle_id, DEV_JOINT, ARM_TIMEOUT_S))
            a_s = time.perf_counter() - t0
            print(f"    arm alone  ready->zero @ {arm_speed}%   "
                  f"{a_s:6.2f}s  arrived={a_ok}")
            singles_ok = singles_ok and a_ok
            for pct in pole_speeds:
                _reset(arm, monitor, top_hw, ready, arm_speed=arm_speed)
                monitor.expect(arm.handle_id, DEV_LIFT)
                t0 = time.perf_counter()
                robot.rm_set_lift_height(pct, bottom_hw, 0)
                p_ok = all(monitor.wait(arm.handle_id, DEV_LIFT,
                                        LIFT_TIMEOUT_S))
                p_s = time.perf_counter() - t0
                print(f"    pole alone {stroke} mm @ {pct:3d}%      "
                      f"{p_s:6.2f}s  arrived={p_ok}")
                singles_ok = singles_ok and p_ok
            if singles_ok:
                result("PASS", "singles baseline",
                       "arm and pole each complete alone")
            else:
                result("FAIL", "singles baseline",
                       "a device failed ALONE — fix that before reading "
                       "the matrix")
                return 1

        # ── Phase 1: the concurrency matrix ──
        print("\n  Phase 1 — CONCURRENT, literal commands only")
        print("  " + "─" * 64)
        cells, resets_failed = [], 0
        for offset in offsets:
            for pct in pole_speeds:
                ok, note = _reset(arm, monitor, top_hw, ready,
                                  arm_speed=arm_speed)
                if not ok:
                    print(f"    [ABORT] reset failed before pole {pct}% "
                          f"offset {offset}: {note}")
                    resets_failed += 1
                    break
                rec = _cell(arm, monitor, pct, arm_speed, offset,
                            bottom_hw, zero)
                cells.append(rec)
                _print_cell(rec)

        if not cells:
            result("FAIL", "matrix produced data", "no cell ran")
            return 1
        clean = [c for c in cells if c["pole_ok"] and c["arm_ok"]
                 and c["errors"] == "clean"]
        result("PASS", "matrix produced data",
               f"{len(cells)} cells, {resets_failed} aborted resets")

        print("\n  Verdict")
        print("  " + "─" * 64)
        print(f"    cells run                : {len(cells)}")
        print(f"    both devices arrived     : "
              f"{sum(1 for c in cells if c['pole_ok'] and c['arm_ok'])}")
        print(f"    pole stalled             : "
              f"{sum(1 for c in cells if not c['pole_ok'])}")
        print(f"    arm stalled              : "
              f"{sum(1 for c in cells if not c['arm_ok'])}")
        print(f"    left controller errors   : "
              f"{sum(1 for c in cells if c['errors'] != 'clean')}")
        if clean:
            print("    CLEAN combinations (both arrived, no errors):")
            for c in clean:
                print(f"      pole {c['pole_pct']}% / arm {c['arm_pct']}% / "
                      f"offset {c['offset_ms']} ms   "
                      f"pole {c['pole_s']:.2f}s vs arm {c['arm_s']:.2f}s")
            result("PASS", "a working concurrent combination exists",
                   f"{len(clean)}/{len(cells)} cells")
        else:
            result("FAIL", "a working concurrent combination exists",
                   "NONE — concurrent planned-arm + lift motion is "
                   "unsupported on this controller; sequence them instead")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        if arm is not None:
            arm.halt()
            try:
                clear_errors(arm)
            except Exception:
                pass
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
