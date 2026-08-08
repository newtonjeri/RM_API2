"""Per-joint VOLTAGE / CURRENT recorder — turn the power question into data.

The UDP realtime push carries joint_voltage (0.001 V), joint_current
(mA), joint_temperature and joint_en_flag for all seven joints at up to
100 Hz — everything needed to say WHY J6/J7 report Under Voltage instead
of guessing between speed, contact and supply.

This tool only listens. Run it in one terminal while any test runs in
another, or alone for an idle baseline:

    RM_ARM=left python3 power_probe.py --seconds 10 --label idle
    RM_ARM=left python3 power_probe.py --seconds 120 --label chain_hover10

It prints a 1 Hz live line (min voltage / max current and which joint),
reports the collision-protection stage and joint enable states at start
(read-only), and on exit writes a CSV plus a per-joint summary.

HOW TO READ IT (the three candidate mechanisms):

  supply/harness   V(J6/J7) sits visibly below V(J1) ALREADY AT IDLE
                   — resistance in the arm's power chain, hardware.
  speed            V dips track motion in FREE SPACE as v% rises,
                   current high on the big joints (J2/J4).
  contact          V healthy in free space at any speed; the dip appears
                   only when the glove meets the fixture, current spiking
                   on the WRIST joints. NOTE: the 2026-08-08 incident was
                   FREE SPACE (Newton) — contact did not cause it; this
                   mechanism matters for the future, when contact begins.

WHICH JOINT'S CURRENT SPIKES AT THE DIP is the discriminator: J2/J4
(shoulder/elbow accel transients loading the shared bus) point at supply
margin; J6/J7 themselves point at wrist-drive demand. Under-voltage at
IDLE or straight after boot needs no motion story at all — that is
supply/harness/e-stop-circuit hardware, a RealMan support case.
"""

import csv
import os
import pathlib
import sys
import threading
import time

from dual_arm_common import (
    handle_cli, host_ip_for,
    LEFT_IP, RIGHT_IP, ROBOT_PORT, UDP_PORT,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import (
    rm_thread_mode_e, rm_realtime_arm_state_callback_ptr,
    rm_realtime_push_config_t, rm_udp_custom_config_t,
)

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
ARM_IP = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
HOST_IP = host_ip_for(ARM_IP)

_lock = threading.Lock()
_records = []          # (t, [V]*7, [mA]*7, [degC]*7, [en]*7)


def _on_state(data):
    try:
        js = data.joint_status
        v = [float(x) for x in js.joint_voltage]
        i = [float(x) for x in js.joint_current]
        tc = [float(x) for x in getattr(js, "joint_temperature", [0.0] * 7)]
        en = [int(bool(x)) for x in getattr(js, "joint_en_flag", [1] * 7)]
    except Exception:
        return
    with _lock:
        _records.append((time.perf_counter(), v, i, tc, en))


_cb = rm_realtime_arm_state_callback_ptr(_on_state)     # keep alive


def summarize(records):
    """Per-joint stats + dip detection. Pure — unit-tested offline.

    A 'dip' is a sample where a joint's voltage falls more than 2.0 V
    below that joint's own median — relative, so a chain-end joint that
    always runs slightly low is not itself flagged, but a sag event is.
    """
    if not records:
        return None
    n = 7
    out = {"frames": len(records), "joints": []}
    for j in range(n):
        vs = sorted(r[1][j] for r in records)
        cur = [r[2][j] for r in records]
        med = vs[len(vs) // 2]
        dips = [(r[0], r[1][j]) for r in records if med - r[1][j] > 2.0]
        out["joints"].append({
            "joint": j + 1,
            "v_min": vs[0], "v_median": med, "v_max": vs[-1],
            "i_max_ma": max(cur), "i_mean_ma": sum(cur) / len(cur),
            "dips": len(dips),
            "worst_dip_v": (med - min(v for _t, v in dips)) if dips else 0.0,
            "en_dropped": any(not r[4][j] for r in records),
        })
    return out


def main() -> int:
    handle_cli(__doc__, value_flags=("--seconds", "--label"))
    argv = sys.argv[1:]

    def arg(flag, default):
        for i, a in enumerate(argv):
            if a == flag and i + 1 < len(argv):
                return argv[i + 1]
            if a.startswith(flag + "="):
                return a.split("=", 1)[1]
        return default
    seconds = float(arg("--seconds", "30"))
    label = arg("--label", "probe")

    print("=" * 68)
    print(f"Power probe — {ARM_SIDE} arm @ {ARM_IP}   {seconds:.0f}s   "
          f"label={label!r}")
    print(f"    UDP -> {HOST_IP}:{UDP_PORT}   LISTEN-ONLY: no motion is "
          "commanded")
    print("=" * 68)

    robot = None
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ARM_IP, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] hardware not reachable at {ARM_IP}")
            return 0

        # context that changes how the numbers read — all read-only
        try:
            ret, stage = robot.rm_get_collision_stage()
            print(f"  collision protection stage: "
                  f"{stage if ret == 0 else f'unreadable ret={ret}'} "
                  "(0=off .. 8; a sensible level turns contact overload "
                  "into a graceful stop)")
        except Exception:
            pass
        try:
            ret, en = robot.rm_get_joint_en_state()
            if ret == 0:
                dead = [i + 1 for i, e in enumerate(en) if not e]
                print(f"  joint enable: "
                      + ("all enabled" if not dead else
                         f"DISABLED {dead} — recover before trusting "
                         "voltage readings under load"))
        except Exception:
            pass

        robot.rm_realtime_arm_state_call_back(_cb)
        robot.rm_set_realtime_push(rm_realtime_push_config_t(
            cycle=1, enable=True, port=UDP_PORT, ip=HOST_IP,
            custom_config=rm_udp_custom_config_t(joint_speed=1)))

        t0 = time.perf_counter()
        last_line = 0.0
        while time.perf_counter() - t0 < seconds:
            time.sleep(0.1)
            now = time.perf_counter()
            if now - last_line >= 1.0:
                with _lock:
                    recent = [r for r in _records if now - r[0] < 1.0]
                if recent:
                    vmin = min((min(r[1]), r[1].index(min(r[1])) + 1)
                               for r in recent)
                    imax = max((max(r[2]), r[2].index(max(r[2])) + 1)
                               for r in recent)
                    print(f"    t={now - t0:5.1f}s  frames={len(recent):3d}"
                          f"/s   Vmin={vmin[0]:6.2f} V (J{vmin[1]})   "
                          f"Imax={imax[0]:7.0f} mA (J{imax[1]})")
                else:
                    print(f"    t={now - t0:5.1f}s  NO FRAMES — check the "
                          f"push target {HOST_IP}:{UDP_PORT}")
                last_line = now

        with _lock:
            records = list(_records)
        rep = summarize(records)
        if rep is None:
            print("  no frames captured — nothing to report")
            return 1

        csv_path = pathlib.Path(__file__).resolve().parent / \
            f"power_{ARM_SIDE}_{label}.csv"
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"]
                       + [f"V{j}" for j in range(1, 8)]
                       + [f"mA{j}" for j in range(1, 8)]
                       + [f"degC{j}" for j in range(1, 8)]
                       + [f"en{j}" for j in range(1, 8)])
            for t, v, i, tc, en in records:
                w.writerow([f"{t - records[0][0]:.4f}"]
                           + [f"{x:.3f}" for x in v]
                           + [f"{x:.1f}" for x in i]
                           + [f"{x:.2f}" for x in tc]
                           + en)

        print(f"\n  {rep['frames']} frames -> {csv_path.name}")
        print(f"  {'J':>2s} {'Vmin':>7s} {'Vmed':>7s} {'Imax mA':>9s} "
              f"{'dips>2V':>8s} {'worst':>7s}  en")
        for j in rep["joints"]:
            print(f"  J{j['joint']} {j['v_min']:7.2f} {j['v_median']:7.2f} "
                  f"{j['i_max_ma']:9.0f} {j['dips']:8d} "
                  f"{j['worst_dip_v']:6.2f}V"
                  f"  {'DROPPED' if j['en_dropped'] else 'ok'}")
        chain_end = rep["joints"][5:]
        base = rep["joints"][0]
        gap = base["v_median"] - min(j["v_median"] for j in chain_end)
        print(f"\n  chain-end static gap (J1 median - min(J6,J7) median): "
              f"{gap:.2f} V")
        print("  read: idle gap large -> supply/harness; dips only under "
              "free-space motion -> speed;\n        dips only during "
              "contact (hover A/B) -> torque from pressing the surface")
        return 0
    finally:
        if robot is not None:
            try:
                robot.rm_set_realtime_push(rm_realtime_push_config_t(
                    cycle=1, enable=False, port=UDP_PORT, ip=HOST_IP))
            except Exception:
                pass
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
