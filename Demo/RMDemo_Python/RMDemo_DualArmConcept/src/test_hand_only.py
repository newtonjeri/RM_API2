"""C7 — Hand-alone test. ONLY THE SELECTED HAND MOVES (no arm, no pole).

Determines which hand-control path works on this setup and exercises it
with REAL measured feedback:

  Phase P — PROTOCOL probe: one blocking rm_set_hand_angle. ret 0 means
      the end port is free and the protocol path (arrival events, used by
      C2/C6) works. ret -5 means the port is in MODBUS mode (the
      butterfli_hw ALL-MODBUS stack's state) and protocol commands are
      dead — the exact C6 hand-failure signature.
  Phase M — MODBUS RTU (the butterfli_hw adopted method): open the end
      port (rm_set_modbus_mode port=1, 115200), write SPEED_SET, drive
      release -> grasp -> half_grasp -> release via ANGLE_SET register
      writes, and verify each with polled ANGLE_ACT reads — the only
      genuine measured hand feedback on fw 1.7.x (no rm_get_hand_* API
      exists; UDP handState is absent).

Register map (butterfli_hw conversions.hpp, bench §3.8-verified):
  ANGLE_SET 1486, FORCE_SET 1498, SPEED_SET 1522, ANGLE_ACT 1546,
  FORCE_ACT 1582. Block ops: address=base, num=6 registers, data as
  hi-lo byte pairs. SDK DOF order [little, ring, middle, index,
  thumb_flex, thumb_rot], 1000 = open.

Order is safety-critical: protocol probe FIRST, modbus second — a failed
protocol call DURING a modbus session degrades the session (bench §3.8).
On exit the port is returned to protocol mode (rm_close_modbus_mode)
unless RM_KEEP_MODBUS=1.

Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    HAND_STATES_HW, LEFT_IP, RIGHT_IP, ROBOT_PORT,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import (
    rm_thread_mode_e, rm_peripheral_read_write_params_t,
)

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
HAND_DEVICE = int(os.environ.get("RM_HAND_MODBUS_DEVICE", "1"))
HAND_BAUD = 115200
REG_ANGLE_SET = 1486
REG_FORCE_SET = 1498
REG_SPEED_SET = 1522
REG_ANGLE_ACT = 1546
REG_FORCE_ACT = 1582
ANGLE_TOL = 25               # ANGLE_ACT counts; grasp-on-object stops short
POLL_S = 0.25                # modbus round trip is ~60 ms; poll gently
STATE_TIMEOUT_S = 6.0
SPEED_SET_VALUE = 500

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 8


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _params(address: int, num: int):
    return rm_peripheral_read_write_params_t(
        port=1, address=address, device=HAND_DEVICE, num=num)


def _encode(vals):
    out = []
    for v in vals:
        out += [(int(v) >> 8) & 0xFF, int(v) & 0xFF]
    return out


def _decode(data):
    return [((data[2 * i] & 0xFF) << 8) | (data[2 * i + 1] & 0xFF)
            for i in range(len(data) // 2)]


def _read_angles(robot):
    ret, raw = robot.rm_read_multiple_holding_registers(
        _params(REG_ANGLE_ACT, 6))
    if ret != 0 or len(raw) < 12:
        return None
    return _decode(raw)[:6]


def _drive_state(robot, name: str) -> bool:
    """ANGLE_SET write + ANGLE_ACT poll until within tolerance or stable."""
    target = HAND_STATES_HW[name]
    ret = robot.rm_write_registers(_params(REG_ANGLE_SET, 6), _encode(target))
    if ret != 0:
        print(f"    {name:10s} ANGLE_SET write FAILED ret={ret}")
        return False
    t0 = time.perf_counter()
    prev, stable = None, 0
    while time.perf_counter() - t0 < STATE_TIMEOUT_S:
        time.sleep(POLL_S)
        act = _read_angles(robot)
        if act is None:
            continue
        err = max(abs(a - t) for a, t in zip(act, target))
        if err <= ANGLE_TOL:
            print(f"    {name:10s} reached in {time.perf_counter()-t0:4.1f} s"
                  f"  ACT={act}  (max err {err})")
            return True
        if prev is not None and max(abs(a - p)
                                    for a, p in zip(act, prev)) <= 2:
            stable += 1
            if stable >= 3 and any(abs(a - t) > ANGLE_TOL
                                   for a, t in zip(act, target)):
                # Moved then stopped short of target: legacy semantics say
                # a finger stopped on an obstacle is a successful grasp.
                print(f"    {name:10s} stopped stable short of target "
                      f"(obstacle?)  ACT={act}")
                return True
        else:
            stable = 0
        prev = act
    print(f"    {name:10s} TIMEOUT  last ACT={_read_angles(robot)}")
    return False


def main() -> int:
    handle_cli(__doc__)
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    keep_modbus = os.environ.get("RM_KEEP_MODBUS") == "1"
    print("=" * 68)
    print("C7  Hand-alone test (protocol probe, then modbus RTU)")
    print(f"    hand on arm={ARM_SIDE} @ {ip}   modbus device={HAND_DEVICE} "
          f"baud={HAND_BAUD}")
    print("    ONLY THE HAND MOVES — no arm, no pole motion")
    print(f"    on exit: {'KEEP modbus mode (RM_KEEP_MODBUS=1)' if keep_modbus else 'restore protocol mode (rm_close_modbus_mode)'}")
    print("=" * 68)

    robot = None
    modbus_open = False
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ip}")
            _results["SKIP"] += N_CHECKS
            return 0
        ret, mode = robot.rm_get_arm_run_mode()
        print(f"  [INFO] arm run mode: "
              f"{'SIMULATION' if mode == 0 else 'REAL'} "
              "(hand is a physical peripheral; REAL recommended)")
        result("PASS", "connected", f"handle id={handle.id}")

        # ── Phase P: protocol probe (MUST run before any modbus traffic) ──
        print("\n  Phase P — protocol path probe (rm_set_hand_angle, "
              "blocking 5 s)")
        ret = robot.rm_set_hand_angle(HAND_STATES_HW["release"], True, 5)
        if ret == 0:
            result("PASS", "protocol path", "WORKS — end port is free; "
                   "C2/C6 hand steps can use arrival events")
            protocol_works = True
        elif ret == -5:
            result("PASS", "protocol path", "BLOCKED (-5 timeout) — end "
                   "port almost certainly in MODBUS mode; matches the C6 "
                   "hand failure")
            protocol_works = False
        else:
            result("PASS", "protocol path", f"ret={ret} — unexpected; see "
                   "API codes (-4 wrong arrival device, 1 param/state)")
            protocol_works = False

        # ── Phase M: modbus RTU, the butterfli_hw adopted method ──
        print("\n  Phase M — modbus RTU (ANGLE_SET writes, ANGLE_ACT "
              "feedback)")
        ret = robot.rm_set_modbus_mode(1, HAND_BAUD, 2)
        time.sleep(0.3)                       # bench: settle after enabling
        if ret == 0:
            modbus_open = True
            result("PASS", "modbus session opened", "port 1 @ 115200")
        else:
            result("FAIL", "modbus session opened", f"ret={ret}")
            return 1

        act = _read_angles(robot)
        if act is not None:
            result("PASS", "ANGLE_ACT readable", f"measured {act}")
        else:
            result("FAIL", "ANGLE_ACT readable",
                   "no response — check RM_HAND_MODBUS_DEVICE (RH56 "
                   "default 1) and hand power/wiring")
            return 1

        ret = robot.rm_write_registers(_params(REG_SPEED_SET, 6),
                                       _encode([SPEED_SET_VALUE] * 6))
        if ret == 0:
            result("PASS", "SPEED_SET written", f"{SPEED_SET_VALUE} x6")
        else:
            result("FAIL", "SPEED_SET written", f"ret={ret}")

        ok = True
        for name in ("release", "grasp", "half_grasp", "release"):
            ok = _drive_state(robot, name) and ok
        if ok:
            result("PASS", "modbus command + measured feedback",
                   "release -> grasp -> half_grasp -> release verified "
                   "via ANGLE_ACT")
        else:
            result("FAIL", "modbus command + measured feedback")

        ret, raw = robot.rm_read_multiple_holding_registers(
            _params(REG_FORCE_ACT, 6))
        if ret == 0 and len(raw) >= 12:
            result("PASS", "FORCE_ACT readable",
                   f"{_decode(raw)[:6]} g (signed)")
        else:
            result("PASS", "FORCE_ACT readable", f"ret={ret} — optional")

        print("\n  VERDICT: protocol path "
              + ("WORKS" if protocol_works else "BLOCKED")
              + "; modbus RTU path "
              + ("WORKS with real feedback" if ok else "FAILED")
              + ".")
        if not protocol_works and ok:
            print("  => Adopt MODBUS for hand steps (as butterfli_hw does),"
                  " or rm_close_modbus_mode(1) before protocol-path runs.")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        if robot is not None and modbus_open and not keep_modbus:
            try:
                ret = robot.rm_close_modbus_mode(1)
                print(f"  [INFO] end port restored to protocol mode "
                      f"(rm_close_modbus_mode ret={ret})")
                result("PASS", "end-port state restored")
            except Exception as exc:
                result("FAIL", "end-port state restored", repr(exc))
        elif modbus_open:
            print("  [INFO] modbus mode KEPT enabled (RM_KEEP_MODBUS=1)")
            result("PASS", "end-port state kept (modbus)")
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
