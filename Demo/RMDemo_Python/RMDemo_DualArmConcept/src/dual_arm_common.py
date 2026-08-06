"""Shared constants and helpers for the dual-arm concept tests (Concept #1).

Joint states are verbatim (radians) from butterfli_moveit_config's
butterfli_alix.srdf; the RM controller takes degrees with no sign flips or
offsets (verified against butterfli_hw conversions.hpp / arm_channel.cpp).
Lift heights use the SRDF pole states in metres; hardware mm = m * 2000/3.
Hand states are the SRDF inspire_hand states converted through butterfli_hw's
hand_rad_to_hw into SDK order [little, ring, middle, index, thumb_flex,
thumb_rot], 1000 = open (cross-checked against the bench §3.8 ANGLE_SET echo:
grasp = 33/33/33/33/133/944).

Step kinds:
  ("arm",  state)                 rm_movej to an SRDF arm state
  ("lift", state)                 rm_set_lift_height to an SRDF pole state
  ("hand", state)                 rm_set_hand_angle to an SRDF hand state
  ("sync", (arm_state, lift_state))  ARM-POLE SYNCHRONIZATION: both devices
        dispatched back-to-back with the lift speed DURATION-MATCHED to the
        arm move (the butterfli_hw sync contract, command-level port:
        v = dist / (arm_duration - 0.38 s start latency), pct = ceil(v/1.85),
        round-UP quantization per bench_sync 2026-07-25). Start and finish
        skew between joint and lift arrivals are measured per step.

Execution modes: run_locked (barrier per step), run_chained (follower starts
step k after leader completes step k), run_free (independent advance).

Hardware caveat (fw 1.7.x): rm_set_hand_angle is the hand PROTOCOL path and
is mutually exclusive with end-port modbus mode — do not run these tests
while the butterfli_hw stack (ALL-MODBUS) is attached; if the end port was
left in modbus mode, call rm_close_modbus_mode(1) first.
"""

import math
import os
import pathlib
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))

from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e, rm_event_callback_ptr

# ─── Hardware / safety constants ────────────────────────────────────────────
# Endpoints are env-overridable (defaults = the network-verified butterfli
# setup) so address changes never require editing code:
#   RM_LEFT_IP / RM_RIGHT_IP   arm controller IPs
#   RM_ROBOT_PORT              arm TCP port
#   RM_HOST_IP                 THIS host's IP on the arm LAN (UDP push target)
#   RM_UDP_PORT                UDP push port used by the sim probe
LEFT_IP    = os.environ.get("RM_LEFT_IP", "192.168.1.10")
RIGHT_IP   = os.environ.get("RM_RIGHT_IP", "192.168.1.103")
ROBOT_PORT = int(os.environ.get("RM_ROBOT_PORT", "8080"))
HOST_IP    = os.environ.get("RM_HOST_IP", "192.168.1.235")
UDP_PORT   = int(os.environ.get("RM_UDP_PORT", "8095"))

ARM_SPEED_PCT  = 20           # movej speed percentage, conservative
LIFT_SPEED_PCT = 50           # measured ~56.4 phys mm/s (LiftBenchmark map)

ARM_TIMEOUT_S  = 40.0
LIFT_TIMEOUT_S = 25.0
HAND_TIMEOUT_S = 10.0         # full stroke ~0.8 s + generous margin
ARM_TOL_DEG    = 2.0          # fallback arrival tolerance per joint
LIFT_TOL_HW_MM = 10           # fallback arrival tolerance, hardware mm

DEV_JOINT = 0                 # rm_event_push_data_t.device values
DEV_HAND  = 2
DEV_LIFT  = 3

# Arm-pole sync model (butterfli_hw conversions.hpp, bench-measured):
LIFT_START_LATENCY_S   = 0.38     # kLiftStartLatencyS (up, worst case)
LIFT_MM_S_PER_PCT      = 1.85     # kLiftCruiseMpsPerPct * 1000
LIFT_MIN_MATCH_PCT     = 4        # lowered floor (TODO item 1, 2026-07-25)
ARM_MAX_DEG_S          = 180.0    # synchronized-profile joint speed at v=100

# ─── Joint states (radians, verbatim from butterfli_alix.srdf) ──────────────
STATES_RAD = {
    "left": {
        "zero":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.1416],
        "rest":  [0.0, -1.431, 0.4538, 1.7103, -0.1047, 1.0821, 1.9373],
        "ready": [0.0, -1.885, 0.0, 1.798, 0.0, 1.379, 3.1416],
    },
    "right": {
        "zero":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "rest":  [0.0, -1.431, -0.4538, 1.7103, 0.1047, 1.0821, 1.2043],
        "ready": [0.0, -1.885, 0.0, 1.798, 0.0, 1.379, 0.0],
    },
}

# ─── Lift heights (metres, SRDF pole_* group states) ────────────────────────
LIFT_M = {"minimum": 0.01, "quarter": 0.075, "half": 0.15, "full": 0.29}
LIFT_MIN_M   = 0.01
LIFT_MAX_M   = 0.29
LIFT_M_TO_HW = 2000.0 / 3.0   # butterfli_hw kLiftCmdUnitsPerMetre
HW_TO_PHYS   = 1.5            # hw mm -> physical mm

# ─── Hand states (SDK order [little,ring,middle,index,thumb_flex,thumb_rot],
#     0-1000, 1000 = open; SRDF inspire_hand states via hand_rad_to_hw) ─────
HAND_STATES_HW = {
    "release":    [993, 993, 993, 993, 981, 992],
    "half_grasp": [516, 516, 516, 516, 133, 944],
    "grasp":      [33, 33, 33, 33, 133, 944],
}

# The concept task: arm/lift/hand states plus two arm-pole SYNC steps.
CONCEPT_SEQUENCE = [
    ("arm", "ready"),
    ("hand", "release"),
    ("sync", ("zero", "half")),
    ("hand", "grasp"),
    ("sync", ("ready", "full")),
    ("hand", "half_grasp"),
    ("arm", "rest"),
]


def state_deg(side: str, name: str) -> list:
    return [math.degrees(q) for q in STATES_RAD[side][name]]


def lift_hw_mm(metres: float) -> int:
    assert LIFT_MIN_M <= metres <= LIFT_MAX_M, (
        f"lift target {metres} m outside safe range [{LIFT_MIN_M}, {LIFT_MAX_M}]")
    return int(round(metres * LIFT_M_TO_HW))


def est_arm_duration_s(current_deg, target_deg, v_pct: int) -> float:
    """Synchronized-profile duration estimate for a movej."""
    delta = max(abs(t - c) for t, c in zip(target_deg, current_deg))
    return max(delta / (ARM_MAX_DEG_S * v_pct / 100.0), 0.05)


def matched_lift_speed_pct(arm_duration_s: float, dist_phys_mm: float) -> int:
    """Duration-match the lift to the arm move (butterfli_hw sync contract).

    Speed such that start latency + cruise time == the arm's duration,
    quantized ROUND-UP (early finish is benign — the device waits; late is
    the failure mode; bench_sync 2026-07-25).
    """
    if dist_phys_mm <= 0.0:
        return LIFT_MIN_MATCH_PCT
    cruise_s = max(arm_duration_s - LIFT_START_LATENCY_S, 0.05)
    pct = math.ceil(dist_phys_mm / cruise_s / LIFT_MM_S_PER_PCT - 1e-9)
    return max(LIFT_MIN_MATCH_PCT, min(100, pct))


def countdown(seconds: int = 5):
    for s in range(seconds, 0, -1):
        print(f"  starting in {s} ...")
        time.sleep(1.0)


class ArrivalMonitor:
    """Demuxes the process-global arrival-event callback by (handle_id, device).

    The SDK registers ONE callback per process; events carry handle_id, so a
    single monitor serves both arms. Keep a reference to the ctypes callback
    object for the process lifetime (GC of it would crash the SDK thread).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._waiters = {}          # (handle_id, device) -> [Event, success]
        self._cb = rm_event_callback_ptr(self._on_event)
        self._registered = False

    def register(self, robot):
        if not self._registered:
            robot.rm_get_arm_event_call_back(self._cb)
            self._registered = True

    def expect(self, handle_id: int, device: int):
        with self._lock:
            self._waiters[(handle_id, device)] = [threading.Event(), False]

    def _on_event(self, data):
        # Runs on the SDK receive thread: keep minimal, no SDK calls here.
        if getattr(data, "event_type", 0) != 1:
            return
        with self._lock:
            waiter = self._waiters.get((data.handle_id, data.device))
        if waiter is None:
            return
        waiter[1] = bool(data.trajectory_state)
        if data.trajectory_connect == 0:
            waiter[0].set()

    def wait(self, handle_id: int, device: int, timeout: float):
        with self._lock:
            waiter = self._waiters.get((handle_id, device))
        if waiter is None:
            return False, False
        arrived = waiter[0].wait(timeout)
        return arrived, waiter[1]


class ConceptArm:
    """One arm-side wrapper: dispatch, arrival fallback verification, halt."""

    def __init__(self, side: str, robot, handle):
        self.side = side
        self.robot = robot
        self.handle_id = handle.id

    # ── step introspection ──
    def parts_for(self, step):
        """[(device, target)] making up this step."""
        kind, target = step
        if kind == "arm":
            return [(DEV_JOINT, target)]
        if kind == "lift":
            return [(DEV_LIFT, target)]
        if kind == "hand":
            return [(DEV_HAND, target)]
        if kind == "sync":
            return [(DEV_JOINT, target[0]), (DEV_LIFT, target[1])]
        raise ValueError(f"unknown step kind {kind}")

    def timeout_for(self, device: int) -> float:
        return {DEV_JOINT: ARM_TIMEOUT_S, DEV_LIFT: LIFT_TIMEOUT_S,
                DEV_HAND: HAND_TIMEOUT_S}[device]

    # ── dispatch ──
    def _dispatch_device(self, device: int, target, lift_speed=None) -> int:
        if device == DEV_JOINT:
            return self.robot.rm_movej(state_deg(self.side, target),
                                       ARM_SPEED_PCT, 0, 0, 0)
        if device == DEV_LIFT:
            return self.robot.rm_set_lift_height(
                lift_speed or LIFT_SPEED_PCT, lift_hw_mm(LIFT_M[target]), 0)
        return self.robot.rm_set_hand_angle(HAND_STATES_HW[target], False, 2)

    def begin(self, monitor: ArrivalMonitor, step) -> dict:
        """Expect + dispatch every device of the step (non-blocking).

        For sync steps the lift speed is duration-matched to the arm move.
        """
        parts = self.parts_for(step)
        beg = {"side": self.side, "step": step, "devices": {}}

        lift_speed = None
        if step[0] == "sync":
            # Duration-match: needs current joint + lift positions.
            try:
                ret, st = self.robot.rm_get_current_arm_state()
                cur = st["joint"] if ret == 0 else state_deg(self.side, "ready")
                arm_dur = est_arm_duration_s(
                    cur, state_deg(self.side, step[1][0]), ARM_SPEED_PCT)
                lret, lst = self.robot.rm_get_lift_state()
                pos_hw = lst.get("pos", 0) if lret == 0 else 0
                dist_phys = abs(lift_hw_mm(LIFT_M[step[1][1]]) - pos_hw) * HW_TO_PHYS
                lift_speed = matched_lift_speed_pct(arm_dur, dist_phys)
                beg["arm_dur_est_s"] = arm_dur
                beg["lift_speed_pct"] = lift_speed
            except Exception:
                lift_speed = LIFT_SPEED_PCT

        for device, target in parts:
            monitor.expect(self.handle_id, device)
        for device, target in parts:
            t0 = time.perf_counter()
            ret = self._dispatch_device(device, target, lift_speed)
            beg["devices"][device] = {"target": target, "ret": ret,
                                      "t_dispatch": t0, "t_done": None,
                                      "event": False, "verified": False,
                                      "ok": False}
        return beg

    def finish(self, monitor: ArrivalMonitor, beg: dict) -> dict:
        """Wait for every device dispatched by begin(); aggregate a record."""
        for device, d in beg["devices"].items():
            if d["ret"] != 0:
                continue
            arrived, success = monitor.wait(self.handle_id, device,
                                            self.timeout_for(device))
            d["t_done"] = time.perf_counter()
            if arrived and success:
                d["event"] = True
                d["ok"] = True
            else:
                d["verified"] = self.verify_device(device, d["target"])
                d["ok"] = d["verified"]
                if d["verified"]:
                    # Motion finished but its event was late/lost — absorb it
                    # briefly so it cannot leak into the next step's wait.
                    monitor.wait(self.handle_id, device, 0.2)

        devs = beg["devices"].values()
        rec = {
            "side": self.side, "step": beg["step"],
            "ret": next((d["ret"] for d in devs if d["ret"] != 0), 0),
            "t_dispatch": min(d["t_dispatch"] for d in devs),
            "t_done": (max(d["t_done"] for d in devs)
                       if all(d["t_done"] for d in devs) else None),
            "event": all(d["event"] for d in devs),
            "verified": sum(d["verified"] for d in devs),
            "ok": all(d["ok"] for d in devs),
            "devices": beg["devices"],
        }
        if beg["step"][0] == "sync":
            j = beg["devices"][DEV_JOINT]
            l = beg["devices"][DEV_LIFT]
            rec["sync_start_skew_s"] = l["t_dispatch"] - j["t_dispatch"]
            if j["t_done"] and l["t_done"]:
                # positive = pole finished LATE vs the arm (the bad direction)
                rec["sync_finish_skew_s"] = l["t_done"] - j["t_done"]
            rec["arm_dur_est_s"] = beg.get("arm_dur_est_s")
            rec["lift_speed_pct"] = beg.get("lift_speed_pct")
        return rec

    # ── fallback verification ──
    def verify_device(self, device: int, target) -> bool:
        """Measured-position fallback when the arrival event was missed."""
        try:
            if device == DEV_JOINT:
                ret, st = self.robot.rm_get_current_arm_state()
                if ret != 0:
                    return False
                goal = state_deg(self.side, target)
                return all(abs(a - b) <= ARM_TOL_DEG
                           for a, b in zip(st["joint"], goal))
            if device == DEV_LIFT:
                ret, st = self.robot.rm_get_lift_state()
                if ret != 0:
                    return False
                return abs(st["pos"] - lift_hw_mm(LIFT_M[target])) <= LIFT_TOL_HW_MM
            # Hand: no getter exists in the SDK and handState is absent from
            # UDP on fw 1.7.2 — the arrival event is the only signal.
            return False
        except Exception:
            return False

    def halt(self):
        try:
            self.robot.rm_set_arm_stop()
        except Exception:
            pass
        try:
            self.robot.rm_set_lift_speed(0)
        except Exception:
            pass


def connect_both():
    """Connect to both arms. Returns (left, right) ConceptArm or (None, None).

    Any SDK exception is treated as unreachable hardware so callers take
    their SKIP path instead of crashing with a traceback.
    """
    try:
        left_robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        left_handle = left_robot.rm_create_robot_arm(LEFT_IP, ROBOT_PORT, 3)
        right_robot = RoboticArm()  # mode=None skips the (already done) rm_init
        right_handle = right_robot.rm_create_robot_arm(RIGHT_IP, ROBOT_PORT, 3)
    except Exception as exc:
        print(f"  [WARN] SDK exception during connect: {exc!r}")
        return None, None
    if (left_handle is None or left_handle.id <= 0
            or right_handle is None or right_handle.id <= 0):
        return None, None
    return (ConceptArm("left", left_robot, left_handle),
            ConceptArm("right", right_robot, right_handle))


def teardown(*arms):
    for arm in arms:
        if arm is None:
            continue
        try:
            arm.robot.rm_delete_robot_arm()
        except Exception:
            pass
    try:
        RoboticArm.rm_destroy()
    except Exception:
        pass


def stop_all(*arms):
    for arm in arms:
        if arm is not None:
            arm.halt()


def run_step(arm: ConceptArm, monitor: ArrivalMonitor, step) -> dict:
    """Dispatch one step non-blocking and wait for all its arrivals."""
    return arm.finish(monitor, arm.begin(monitor, step))


def run_locked(left: ConceptArm, right: ConceptArm, monitor: ArrivalMonitor,
               sequence=CONCEPT_SEQUENCE) -> dict:
    """Parallel locked: both arms per step, barrier before the next step."""
    report = {"mode": "locked", "steps": [], "ok": True}
    for step in sequence:
        beg_l = left.begin(monitor, step)
        beg_r = right.begin(monitor, step)
        skew_ms = (min(d["t_dispatch"] for d in beg_r["devices"].values())
                   - min(d["t_dispatch"] for d in beg_l["devices"].values())) * 1000.0
        rets = [d["ret"] for b in (beg_l, beg_r) for d in b["devices"].values()]
        if any(r != 0 for r in rets):
            rec_l = left.finish(monitor, beg_l)
            rec_r = right.finish(monitor, beg_r)
            report["steps"].append({"step": step, "skew_ms": skew_ms,
                                    "left": rec_l, "right": rec_r})
            report["ok"] = False
            stop_all(left, right)
            break
        rec_l = left.finish(monitor, beg_l)
        rec_r = right.finish(monitor, beg_r)
        report["steps"].append({"step": step, "skew_ms": skew_ms,
                                "left": rec_l, "right": rec_r})
        if not (rec_l["ok"] and rec_r["ok"]):
            report["ok"] = False
            stop_all(left, right)
            break
    return report


def run_chained(leader: ConceptArm, follower: ConceptArm,
                monitor: ArrivalMonitor, sequence=CONCEPT_SEQUENCE) -> dict:
    """Chained: follower starts step k only after the leader completed step k.

    The leader advances freely, so follower step k overlaps leader step k+1
    (pipelined) — the follower performs the following task, it is not a
    synchronized copy.
    """
    n = len(sequence)
    gates = [threading.Event() for _ in range(n)]
    stop = threading.Event()
    report = {"mode": "chained",
              "leader": [None] * n, "follower": [None] * n,
              "gate_open_t": [None] * n, "ok": True}

    def leader_work():
        for k, step in enumerate(sequence):
            if stop.is_set():
                return
            rec = run_step(leader, monitor, step)
            report["leader"][k] = rec
            if not rec["ok"]:
                stop.set()
                for g in gates[k:]:
                    g.set()          # wake the follower; it will see stop
                return
            report["gate_open_t"][k] = time.perf_counter()
            gates[k].set()

    def follower_work():
        gate_timeout = ARM_TIMEOUT_S + LIFT_TIMEOUT_S
        for k, step in enumerate(sequence):
            if not gates[k].wait(gate_timeout) or stop.is_set():
                stop.set()
                return
            rec = run_step(follower, monitor, step)
            report["follower"][k] = rec
            if not rec["ok"]:
                stop.set()
                return

    threads = [threading.Thread(target=leader_work, daemon=True),
               threading.Thread(target=follower_work, daemon=True)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=n * (ARM_TIMEOUT_S + LIFT_TIMEOUT_S))
    if stop.is_set() or any(r is None or not r["ok"]
                            for r in report["leader"] + report["follower"]):
        report["ok"] = False
        stop_all(leader, follower)
    return report


def run_free(left: ConceptArm, right: ConceptArm, monitor: ArrivalMonitor,
             sequence=CONCEPT_SEQUENCE) -> dict:
    """Free execution: each arm advances on its own arrivals, no cross gates."""
    stop = threading.Event()
    report = {"mode": "free", "left": [], "right": [], "ok": True}

    def work(arm, key):
        for step in sequence:
            if stop.is_set():
                return
            rec = run_step(arm, monitor, step)
            report[key].append(rec)
            if not rec["ok"]:
                stop.set()
                return

    threads = [threading.Thread(target=work, args=(left, "left"), daemon=True),
               threading.Thread(target=work, args=(right, "right"), daemon=True)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=len(sequence) * (ARM_TIMEOUT_S + LIFT_TIMEOUT_S))
    n = len(sequence)
    if stop.is_set() or len(report["left"]) != n or len(report["right"]) != n \
            or any(not r["ok"] for r in report["left"] + report["right"]):
        report["ok"] = False
        stop_all(left, right)
    return report
