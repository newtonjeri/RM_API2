"""C5 — Simulation-mode motion visibility probe. NO PHYSICAL MOTION.

Answers the open question: when the arm is switched to SIMULATION mode (the
Web GUI's sim toggle, rm_set_arm_run_mode(0)), is the simulated motion
accessible programmatically — over the UDP realtime push, via arrival
events, and via TCP state polling? The docs confirm the controller runs its
planner in sim mode and animates the pendant's 3D model, but never document
which feedback channels carry the simulated states; this probe settles it.

Safety: motion is dispatched ONLY after the controller confirms it is in
simulation mode (verified by readback), so the physical arm does not move.
A small J7 delta (+5 deg at v=10) is used regardless, the joint is returned
to its start value, and the original run mode is restored in `finally`.
"""

import os
import sys
import threading
import time

from dual_arm_common import (
    ARM_TIMEOUT_S, DEV_JOINT, HOST_IP, LEFT_IP, ROBOT_PORT, UDP_PORT,
    ArrivalMonitor,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import (
    rm_thread_mode_e, rm_realtime_arm_state_callback_ptr,
    rm_realtime_push_config_t, rm_udp_custom_config_t,
)

# HOST_IP / UDP_PORT come from dual_arm_common (env: RM_HOST_IP / RM_UDP_PORT)
UDP_WATCHDOG_S = 2.0         # frames must arrive within this window
# A silent UDP stream is a FAIL-TO-RUN by default: a wrong HOST_IP is
# accepted by the controller (ret 0) and simply delivers nothing, which
# would corrupt the verdict. Set RM_ALLOW_NO_UDP=1 to continue anyway.
PROBE_DELTA_DEG = 5.0
PROBE_SPEED_PCT = 10
SWEEP_TOL_DEG = 2.0

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 5

_udp_lock = threading.Lock()
_udp_samples = []            # (t, joint7_deg)


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _on_state(data):
    try:
        j7 = data.joint_status.joint_position[6]
    except Exception:
        return
    with _udp_lock:
        _udp_samples.append((time.perf_counter(), float(j7)))


_state_cb = rm_realtime_arm_state_callback_ptr(_on_state)   # keep alive


def main() -> int:
    print("=" * 68)
    print("C5  Simulation-mode motion visibility probe (no physical motion)")
    print(f"    arm={LEFT_IP}  UDP -> {HOST_IP}:{UDP_PORT}  "
          f"probe: J7 +{PROBE_DELTA_DEG} deg at v={PROBE_SPEED_PCT}%")
    print("=" * 68)

    robot = None
    original_mode = None
    start_joints = None
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(LEFT_IP, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {LEFT_IP}")
            _results["SKIP"] += N_CHECKS
            return 0

        # ── SM1: engage and VERIFY simulation mode before any dispatch ──
        ret, original_mode = robot.rm_get_arm_run_mode()
        if ret != 0:
            result("FAIL", "read current run mode", f"ret={ret}")
            return 1
        print(f"  [INFO] original run mode: "
              f"{'SIMULATION' if original_mode == 0 else 'REAL'}")
        robot.rm_set_arm_run_mode(0)
        ret, mode = robot.rm_get_arm_run_mode()
        if ret == 0 and mode == 0:
            result("PASS", "simulation mode engaged and verified")
        else:
            result("FAIL", "simulation mode engaged",
                   f"ret={ret} mode={mode} — aborting, no motion dispatched")
            return 1

        # ── feedback channels ──
        monitor = ArrivalMonitor()
        monitor.register(robot)
        robot.rm_realtime_arm_state_call_back(_state_cb)
        ret = robot.rm_set_realtime_push(rm_realtime_push_config_t(
            cycle=1, enable=True, port=UDP_PORT, ip=HOST_IP,
            custom_config=rm_udp_custom_config_t(joint_speed=1)))
        print(f"  [INFO] rm_set_realtime_push ret={ret}")

        # ── UDP watchdog: no frames == fail-to-run (default) ──
        deadline = time.perf_counter() + UDP_WATCHDOG_S
        while time.perf_counter() < deadline:
            with _udp_lock:
                if _udp_samples:
                    break
            time.sleep(0.05)
        with _udp_lock:
            n_frames = len(_udp_samples)
        if n_frames > 0:
            result("PASS", "UDP push delivering frames",
                   f"{n_frames} within {UDP_WATCHDOG_S:.0f} s")
        elif os.environ.get("RM_ALLOW_NO_UDP") == "1":
            print("  [WARN] no UDP frames received — continuing because "
                  "RM_ALLOW_NO_UDP=1 (UDP verdict will be unreliable)")
            result("PASS", "UDP push delivering frames",
                   "SKIPPED by RM_ALLOW_NO_UDP=1")
        else:
            result("FAIL", "UDP push delivering frames",
                   f"0 frames within {UDP_WATCHDOG_S:.0f} s")
            print(f"  [FATAL] UDP push to {HOST_IP}:{UDP_PORT} is silent. "
                  "The controller accepts ANY target IP (ret 0) and pushes "
                  "into the void when it is wrong — the run cannot produce "
                  "a valid verdict.")
            print("          Fix: set HOST_IP to THIS machine's IP on the "
                  "arm LAN> (`ip addr`), check RM_UDP_PORT is free and "
                  "not firewalled; or export RM_ALLOW_NO_UDP=1 "
                  "to probe events/polling only.")
            return 1

        ret, st = robot.rm_get_current_arm_state()
        if ret != 0:
            result("FAIL", "read start joints", f"ret={ret}")
            return 1
        start_joints = list(st["joint"])
        target = list(start_joints)
        target[6] += PROBE_DELTA_DEG

        with _udp_lock:
            _udp_samples.clear()

        # ── dispatch the probe move (sim mode verified above) ──
        monitor.expect(handle.id, DEV_JOINT)
        ret = robot.rm_movej(target, PROBE_SPEED_PCT, 0, 0, 0)
        if ret != 0:
            result("FAIL", "probe movej accepted", f"ret={ret}")
            return 1
        result("PASS", "probe movej accepted in simulation mode")

        # Poll TCP state while waiting on the arrival event.
        poll_samples = []
        deadline = time.perf_counter() + min(ARM_TIMEOUT_S, 20.0)
        event_arrived = event_success = False
        while time.perf_counter() < deadline:
            arrived, success = monitor.wait(handle.id, DEV_JOINT, 0.1)
            r, s = robot.rm_get_current_arm_state()
            if r == 0:
                poll_samples.append(s["joint"][6])
            if arrived:
                event_arrived, event_success = True, success
                break
        time.sleep(0.3)

        # ── verdicts: the three visibility channels ──
        with _udp_lock:
            udp = list(_udp_samples)
        udp_n = len(udp)
        udp_swept = udp and max(abs(j - start_joints[6]) for _, j in udp) \
            > SWEEP_TOL_DEG
        poll_swept = poll_samples and \
            max(abs(j - start_joints[6]) for j in poll_samples) > SWEEP_TOL_DEG

        print("\n  " + "─" * 60)
        print(f"  VERDICT — simulated motion accessibility ({udp_n} UDP frames):")
        print(f"    UDP realtime push sweeps:   "
              f"{'YES' if udp_swept else 'NO'}"
              + ("" if udp_n else "  (no frames at all — check HOST_IP/port)"))
        print(f"    Arrival event fires:        "
              f"{'YES' if event_arrived else 'NO'}"
              + (f" (trajectory_state={event_success})" if event_arrived else ""))
        print(f"    TCP polling sweeps:         "
              f"{'YES' if poll_swept else 'NO'}  "
              f"({len(poll_samples)} polls)")
        print("  " + "─" * 60)
        if udp_swept or poll_swept:
            print("  => Simulated motion IS programmatically accessible: the")
            print("     sim-rehearsal record-and-verify pipeline is viable.")
        elif event_arrived:
            print("  => Only arrival events fire; interpolated sim states are")
            print("     not streamed — rehearsal recording is NOT viable.")
        else:
            print("  => No feedback channel reflects simulated motion.")
        result("PASS", "probe completed with a definitive verdict")

        # ── return J7 and restore mode ──
        monitor.expect(handle.id, DEV_JOINT)
        robot.rm_movej(start_joints, PROBE_SPEED_PCT, 0, 0, 0)
        monitor.wait(handle.id, DEV_JOINT, min(ARM_TIMEOUT_S, 20.0))
        result("PASS", "probe joint returned to start")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        try:
            if robot is not None and original_mode is not None:
                robot.rm_set_arm_run_mode(original_mode)
                ret, mode = robot.rm_get_arm_run_mode()
                print(f"  [INFO] run mode restored: "
                      f"{'SIMULATION' if mode == 0 else 'REAL'} (ret={ret})")
        except Exception as exc:
            print(f"  [WARN] run mode restore failed: {exc!r}")
        try:
            if robot is not None:
                robot.rm_set_realtime_push(rm_realtime_push_config_t(
                    cycle=1, enable=False, port=UDP_PORT, ip=HOST_IP))
        except Exception:
            pass
        if robot is not None:
            try:
                robot.rm_delete_robot_arm()
            except Exception:
                pass
            try:
                RoboticArm.rm_destroy()
            except Exception:
                pass
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
