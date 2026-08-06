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
  ("combo", (sub, sub, ...))      concurrent parts on ONE arm (e.g. arm
        motion + hand motion dispatched together, all arrivals awaited)
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
HOST_IP    = os.environ.get("RM_HOST_IP", "192.168.1.239")  # lab laptop
# (the robot's main onboard host is 192.168.1.235 — butterfli_hw xacro)
UDP_PORT   = int(os.environ.get("RM_UDP_PORT", "8095"))

ARM_SPEED_PCT  = 20           # movej speed percentage, conservative
LIFT_SPEED_PCT = 50           # measured ~56.4 phys mm/s (LiftBenchmark map)

ARM_TIMEOUT_S  = 40.0
LIFT_TIMEOUT_S = 25.0
HAND_TIMEOUT_S = 10.0         # full stroke ~0.8 s + generous margin
# Duration-based hand completion (butterfli_hw acked_angle semantics:
# hold_until_duration), sized by the MEASURED stroke law rather than a
# fixed worst case: stroke_s = 0.115 latency + 373*span/SPEED_SET ms
# (bench §3.7), x1.5 margin, capped by RM_HAND_DWELL_S. A no-op send
# (span <= 10 counts) dwells only briefly.
HAND_DWELL_S = float(os.environ.get("RM_HAND_DWELL_S", "1.5"))
HAND_STROKE_K = 373.0
HAND_SPEED_SET_DEFAULT = 500.0
HAND_CMD_LATENCY_S = 0.115


def hand_dwell_s(span_counts: float) -> float:
    if span_counts <= 10:
        return min(0.15, HAND_DWELL_S)
    stroke = HAND_CMD_LATENCY_S + HAND_STROKE_K * span_counts \
        / HAND_SPEED_SET_DEFAULT / 1000.0
    return min(max(0.3, stroke * 1.5), HAND_DWELL_S)
ARM_TOL_DEG    = 2.0          # fallback arrival tolerance per joint
LIFT_TOL_HW_MM = 10           # fallback arrival tolerance, hardware mm

DEV_JOINT = 0                 # rm_event_push_data_t.device values
DEV_HAND  = 2
DEV_LIFT  = 3

# Arm-pole sync model (butterfli_hw conversions.hpp, bench-measured):
LIFT_START_LATENCY_S   = 0.38     # worst case (unknown direction)
LIFT_LATENCY_UP_S      = 0.33     # bench_sync direction-aware latencies
LIFT_LATENCY_DOWN_S    = 0.23
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

# Per-side lift gearing. V1.7.4 switched the lift to TRUE millimetres
# (1:1, travel 0-330 — confirmed physically 2026-08-06: commanding 193
# under the old 2/3 assumption left the pole mid-rail). The right arm
# keeps the V1.7.1 2/3 gearing (hw 0-200) until its upgrade — flip with
# RM_RIGHT_LIFT_GEAR=1to1 after upgrading (RM_LEFT_LIFT_GEAR=2to3 exists
# for rollback).
_GEARS = {
    "1to1": {"hw_per_m": 1000.0, "hw_to_phys": 1.0, "hw_max": 330},
    "2to3": {"hw_per_m": 2000.0 / 3.0, "hw_to_phys": 1.5, "hw_max": 200},
}


def _lift_gear(side: str, default: str) -> dict:
    v = os.environ.get(f"RM_{side.upper()}_LIFT_GEAR", default)
    v = v.strip().lower().replace(":", "to")
    if v not in _GEARS:
        raise SystemExit(f"RM_{side.upper()}_LIFT_GEAR must be 1to1 or 2to3, "
                         f"got {v!r}")
    return dict(_GEARS[v], name=v)


LIFT_GEAR = {"left": _lift_gear("left", "1to1"),
             "right": _lift_gear("right", "2to3")}

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


def lift_hw_mm(side: str, metres: float) -> int:
    """SRDF metres -> the SIDE's controller lift unit (gearing differs!)."""
    assert LIFT_MIN_M <= metres <= LIFT_MAX_M, (
        f"lift target {metres} m outside safe range [{LIFT_MIN_M}, {LIFT_MAX_M}]")
    gear = LIFT_GEAR[side]
    hw = int(round(metres * gear["hw_per_m"]))
    assert hw <= gear["hw_max"], (
        f"{side}: {hw} exceeds controller ceiling {gear['hw_max']}")
    return hw


def est_arm_duration_s(current_deg, target_deg, v_pct: int) -> float:
    """Synchronized-profile duration estimate for a movej."""
    delta = max(abs(t - c) for t, c in zip(target_deg, current_deg))
    return max(delta / (ARM_MAX_DEG_S * v_pct / 100.0), 0.05)


def matched_lift_speed_pct(arm_duration_s: float, dist_phys_mm: float,
                           ascending=None) -> int:
    """Duration-match the lift to the arm move (butterfli_hw sync contract).

    Speed such that start latency + cruise time == the arm's duration,
    quantized ROUND-UP (early finish is benign — the device waits; late is
    the failure mode; bench_sync 2026-07-25). Start latency is
    direction-aware when the direction is known (bench-measured up 0.33 s
    / down 0.23 s; 0.38 worst case otherwise).
    """
    if dist_phys_mm <= 0.0:
        return LIFT_MIN_MATCH_PCT
    latency = (LIFT_LATENCY_UP_S if ascending is True
               else LIFT_LATENCY_DOWN_S if ascending is False
               else LIFT_START_LATENCY_S)
    cruise_s = max(arm_duration_s - latency, 0.05)
    pct = math.ceil(dist_phys_mm / cruise_s / LIFT_MM_S_PER_PCT - 1e-9)
    return max(LIFT_MIN_MATCH_PCT, min(100, pct))


def parse_mode_arg(argv=None):
    """Parse --mode SIM|REAL (also --mode=..., case-insensitive).

    Returns 1 (REAL), 0 (SIM), or None when the flag is absent.
    Exits with usage on an invalid value.
    """
    args = list(sys.argv[1:] if argv is None else argv)
    for i, a in enumerate(args):
        if a == "--mode":
            val = args[i + 1] if i + 1 < len(args) else None
        elif a.startswith("--mode="):
            val = a.split("=", 1)[1]
        else:
            continue
        v = (val or "").strip().upper()
        if v in ("SIM", "SIMULATION", "0"):
            return 0
        if v in ("REAL", "1"):
            return 1
        raise SystemExit(f"usage: --mode SIM|REAL  (got {val!r})")
    return None


USAGE = """\
Usage: python3 <script>.py [--mode SIM|REAL] [--no-hands] [--no-pole]
                           [-h|--help]
  --mode SIM|REAL   engage and VERIFY the run mode before any motion
                    (restored on exit); without it the test runs as-found
  --no-hands        strip all hand steps/parts from the run
  --no-pole         skip pole pre-positioning and strip pole/sync-lift
                    steps (sync steps run as plain arm moves)
  -h, --help        show the script's documentation and exit (no motion)
C8 pole diagnostic (test_pole_only.py) only:
  --diagnose-only   read state and report; NO pole motion at all
  --clear-errors    call rm_clear_system_err before the acceptance probe

Environment overrides:
  RM_LEFT_IP / RM_RIGHT_IP / RM_ROBOT_PORT     arm endpoints
  RM_HOST_IP / RM_UDP_PORT                     UDP push target (C5)
  RM_ARM=left|right                            arm selection (C6/C7)
  RM_LEFT_LIFT_GEAR / RM_RIGHT_LIFT_GEAR       1to1 | 2to3
  RM_HAND_DWELL_S / RM_HAND_MODBUS_DEVICE / RM_KEEP_MODBUS=1
  RM_ALLOW_NO_UDP=1 / RM_EMU_TIME_SCALE
"""


def handle_cli(doc: str, argv=None, extra_flags=()):
    """-h/--help prints the script docs and exits BEFORE anything runs;
    any unknown argument is rejected (exit 2) rather than silently
    ignored — an ignored typo would otherwise move the robot.

    extra_flags: additional exact flags this script accepts (e.g. the C8
    pole diagnostic's --diagnose-only / --clear-errors)."""
    args = list(sys.argv[1:] if argv is None else argv)
    if "-h" in args or "--help" in args:
        print((doc or "").strip())
        print()
        print(USAGE)
        raise SystemExit(0)
    skip = False
    for a in args:
        if skip:
            skip = False
            continue
        if a == "--mode":
            skip = True
            continue
        if a.startswith("--mode=") or a in ("--no-hands", "--no-pole") \
                or a in extra_flags:
            continue
        print(f"unknown argument: {a!r}")
        print()
        print(USAGE)
        raise SystemExit(2)


def parse_no_hands_arg(argv=None) -> bool:
    """--no-hands strips every hand part from the run."""
    args = list(sys.argv[1:] if argv is None else argv)
    return "--no-hands" in args


def strip_hands(sequence):
    """Remove hand steps/parts from a sequence (for --no-hands runs)."""
    out = []
    for kind, target in sequence:
        if kind == "hand":
            continue
        if kind == "combo":
            subs = tuple(s for s in target if s[0] != "hand")
            if not subs:
                continue
            out.append(subs[0] if len(subs) == 1 else ("combo", subs))
        else:
            out.append((kind, target))
    return out


def parse_no_pole_arg(argv=None) -> bool:
    """--no-pole skips pole pre-positioning and strips lift parts."""
    args = list(sys.argv[1:] if argv is None else argv)
    return "--no-pole" in args


def strip_poles(sequence):
    """Remove lift steps/parts from a sequence (for --no-pole runs).

    Sync steps keep their arm half: ("sync", (arm, lift)) -> ("arm", arm),
    so the arm choreography is unchanged while the pole stays put."""
    out = []
    for kind, target in sequence:
        if kind == "lift":
            continue
        if kind == "sync":
            out.append(("arm", target[0]))
            continue
        if kind == "combo":
            subs = tuple(("arm", s[1][0]) if s[0] == "sync" else s
                         for s in target if s[0] != "lift")
            if not subs:
                continue
            out.append(subs[0] if len(subs) == 1 else ("combo", subs))
        else:
            out.append((kind, target))
    return out


def mode_label(mode) -> str:
    return {0: "SIMULATION", 1: "REAL"}.get(mode, "as-found")


def apply_run_mode(target, *arms):
    """Engage the requested run mode on every arm, VERIFIED by readback.

    target None -> no-op, returns {}. On success returns {arm: original
    mode} for restore_run_modes(). On ANY refusal returns None after
    rolling back — callers must abort before dispatching motion (a SIM
    request that silently stayed REAL would move real metal, and vice
    versa a REAL request stuck in SIM produces phantom runs).
    """
    if target is None:
        return {}
    originals = {}
    ok = True
    for arm in arms:
        if arm is None:
            continue
        try:
            ret, orig = arm.robot.rm_get_arm_run_mode()
            originals[arm] = orig if ret == 0 else None
            arm.robot.rm_set_arm_run_mode(target)
            ret, now = arm.robot.rm_get_arm_run_mode()
            if ret != 0 or now != target:
                print(f"  [FAIL] {arm.side}: could not engage "
                      f"{mode_label(target)} (ret={ret} mode={now})")
                ok = False
            else:
                print(f"  [INFO] {arm.side}: run mode set to "
                      f"{mode_label(target)} (--mode)")
        except Exception as exc:
            print(f"  [FAIL] {arm.side}: run-mode set exception {exc!r}")
            ok = False
    if not ok:
        restore_run_modes(originals)
        return None
    return originals


def restore_run_modes(originals):
    """Put every arm back to its pre-run mode (teardown path)."""
    for arm, mode in (originals or {}).items():
        if mode is None:
            continue
        try:
            arm.robot.rm_set_arm_run_mode(mode)
            print(f"  [INFO] {arm.side}: run mode restored to "
                  f"{mode_label(mode)}")
        except Exception:
            pass


def report_run_modes(*arms) -> bool:
    """Print each arm's run mode; returns True if ALL are in REAL mode.

    A SIM-mode arm executes every planned move virtually: dispatches
    succeed, events fire, but NOTHING physical moves (root cause of the
    2026-08-06 'no motion at all' locked run).
    """
    all_real = True
    for arm in arms:
        if arm is None:
            continue
        try:
            ret, mode = arm.robot.rm_get_arm_run_mode()
        except Exception:
            ret, mode = -1, None
        label = {0: "SIMULATION", 1: "REAL"}.get(mode, f"? ({mode})")
        print(f"  [INFO] {arm.side}: run mode = {label}")
        if mode != 1:
            all_real = False
            print(f"  [WARN] {arm.side} is in {label} — NO PHYSICAL MOTION "
                  "will occur; flip to REAL in the Web GUI or "
                  "rm_set_arm_run_mode(1) for a hardware run")
    return all_real


def countdown(seconds: int = 3):
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
            # [event, success, arrival timestamp]
            self._waiters[(handle_id, device)] = [threading.Event(), False,
                                                  None]

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
            waiter[2] = time.perf_counter()   # true arrival time
            waiter[0].set()

    def wait(self, handle_id: int, device: int, timeout: float):
        with self._lock:
            waiter = self._waiters.get((handle_id, device))
        if waiter is None:
            return False, False
        arrived = waiter[0].wait(timeout)
        return arrived, waiter[1]

    def last_arrival(self, handle_id: int, device: int):
        """True event-arrival timestamp (perf_counter), or None."""
        with self._lock:
            waiter = self._waiters.get((handle_id, device))
        return waiter[2] if waiter else None


class ConceptArm:
    """One arm-side wrapper: dispatch, arrival fallback verification, halt."""

    def __init__(self, side: str, robot, handle):
        self.side = side
        self.robot = robot
        self.handle_id = handle.id
        self._last_hand = None      # last commanded hand state (echo model)

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
        if kind == "combo":
            # Arbitrary concurrent parts, e.g. ("combo", (("arm", "ready"),
            # ("hand", "release"))) — all dispatched back-to-back, all
            # arrivals awaited.
            return [p for sub in target for p in self.parts_for(sub)]
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
                lift_speed or LIFT_SPEED_PCT,
                lift_hw_mm(self.side, LIFT_M[target]), 0)
        raise ValueError("hand parts use start_hand_blocking()")

    def start_hand_acked(self, target) -> dict:
        """butterfli_hw acked_angle semantics: NON-BLOCKING send +
        duration-based completion (hold_until_duration).

        Hard-won findings behind this (2026-08-06):
        - device-2 arrival events are NEVER delivered to the user event
          callback on fw V1.7.1/V1.7.4 + SDK 1.1.6 (arm/lift events fire).
        - BLOCKING rm_set_hand_angle works only in isolation (C7): with a
          concurrent arm move in flight it consumes the ARM's arrival push
          and fails instantly with -4 (arrival-device mismatch).
        butterfli_hw's proven production path is exactly this: fire
        non-blocking, wait the stroke duration, feedback is echo.
        """
        vals = HAND_STATES_HW[target]
        span = (960.0 if self._last_hand is None else
                max(abs(a - b) for a, b in zip(vals, self._last_hand)))
        dwell = hand_dwell_s(span)
        holder = {"ret": None, "t_dispatch": time.perf_counter(),
                  "t_done": None, "dwell_s": dwell}

        def work():
            holder["ret"] = self.robot.rm_set_hand_angle(vals, False, 2)
            if holder["ret"] == 0:
                self._last_hand = list(vals)
                time.sleep(dwell)
            holder["t_done"] = time.perf_counter()

        t = threading.Thread(target=work, daemon=True)
        holder["thread"] = t
        t.start()
        return holder

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
                target_hw = lift_hw_mm(self.side, LIFT_M[step[1][1]])
                dist_phys = abs(target_hw - pos_hw) \
                    * LIFT_GEAR[self.side]["hw_to_phys"]
                lift_speed = matched_lift_speed_pct(
                    arm_dur, dist_phys, ascending=target_hw > pos_hw)
                beg["arm_dur_est_s"] = arm_dur
                beg["lift_speed_pct"] = lift_speed
            except Exception:
                lift_speed = LIFT_SPEED_PCT

        for device, target in parts:
            if device != DEV_HAND:
                monitor.expect(self.handle_id, device)
        for device, target in parts:
            if device == DEV_HAND:
                holder = self.start_hand_acked(target)
                beg["devices"][device] = {"target": target, "ret": 0,
                                          "t_dispatch": holder["t_dispatch"],
                                          "t_done": None, "event": False,
                                          "acked": False, "verified": False,
                                          "ok": False, "_hand": holder}
                continue
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
            if device == DEV_HAND:
                holder = d.pop("_hand")
                holder["thread"].join(HAND_DWELL_S + 5.0)
                d["ret"] = holder["ret"] if holder["ret"] is not None else -5
                d["t_done"] = holder["t_done"] or time.perf_counter()
                # acked_angle: completion is duration-based, feedback is
                # echo (no events, no position readback on this fw).
                d["acked"] = d["ok"] = (d["ret"] == 0)
                continue
            if d["ret"] != 0:
                continue
            arrived, success = monitor.wait(self.handle_id, device,
                                            self.timeout_for(device))
            # Prefer the TRUE event-arrival time: a device that finished
            # while we were still waiting on another one keeps its real
            # completion timestamp (concurrency metrics depend on this).
            d["t_done"] = (monitor.last_arrival(self.handle_id, device)
                           or time.perf_counter())
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
                return abs(st["pos"] - lift_hw_mm(self.side, LIFT_M[target])) \
                    <= LIFT_TOL_HW_MM
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


def diagnose_lift_rejection(arm: "ConceptArm"):
    """Print everything the controller will tell us about WHY it rejects
    lift commands (ret=1 = "controller returned false: parameter error or
    arm-state error").  First seen 2026-08-06 20:38: BOTH controllers
    rejected proven-good rm_set_lift_height/rm_set_lift_speed commands.
    Every getter is optional — print what is readable, skip what is not.
    """
    r = arm.robot
    print(f"  [DIAG] {arm.side}: lift command REJECTED by the controller "
          "(set_state false) — state dump:")
    try:
        ret, power = r.rm_get_arm_power_state()
        if ret != 0:
            detail = f"unreadable ret={ret}"
        elif power == 1:
            detail = "ON"
        else:
            detail = ("OFF  <-- likely cause: e-stop pressed / arm "
                      "powered down")
        print(f"  [DIAG]   arm power: {detail}")
    except Exception as exc:
        print(f"  [DIAG]   arm power: getter unavailable ({exc!r})")
    try:
        ret, st = r.rm_get_current_arm_state()
        if ret != 0:
            detail = f"unreadable ret={ret}"
        else:
            err = (st or {}).get("err")
            codes = []
            if isinstance(err, dict):
                codes = [c for c in err.get("err", [])
                         if str(c) not in ("0", "")]
            detail = ("none" if not codes else ", ".join(map(str, codes))
                      + "  <-- likely cause: clear via --clear-errors / "
                      "Web GUI")
        print(f"  [DIAG]   controller err: {detail}")
    except Exception as exc:
        print(f"  [DIAG]   controller err: getter unavailable ({exc!r})")
    try:
        jd = r.rm_get_joint_err_flag()
        if jd.get("return_code") == 0:
            flags = jd.get("err_flag", [])
            brakes = jd.get("brake_state", [])
            bad = [(i + 1, f) for i, f in enumerate(flags) if f]
            print(f"  [DIAG]   joint err flags: "
                  + (f"{bad}  <-- joint errors latched" if bad else "clean")
                  + f"   brake_state={brakes}")
        else:
            print(f"  [DIAG]   joint err flags: unreadable "
                  f"ret={jd.get('return_code')}")
    except Exception as exc:
        print(f"  [DIAG]   joint err flags: getter unavailable ({exc!r})")
    try:
        ret, lst = r.rm_get_lift_state()
        if ret == 0:
            ef = lst.get("err_flag", 0)
            print(f"  [DIAG]   lift state: pos={lst.get('pos')} hw-mm  "
                  f"current={lst.get('current')} mA  mode={lst.get('mode')}"
                  f"  err_flag={ef}"
                  + ("  <-- LIFT DRIVER ERROR latched (stall/overcurrent?)"
                     if ef else ""))
        else:
            print(f"  [DIAG]   lift state: unreadable ret={ret}")
    except Exception as exc:
        print(f"  [DIAG]   lift state: getter unavailable ({exc!r})")
    print("  [DIAG] recovery ladder: (1) release/reset the physical e-stop,"
          " (2) clear errors (RM_ARM=%s python3 test_pole_only.py "
          "--clear-errors), (3) check the Web GUI lift panel moves the pole,"
          " (4) power-cycle the arm." % arm.side)


def home_poles_full(monitor: ArrivalMonitor, *arms) -> bool:
    """Pre-position every pole to full_length (0.29 m) before a run.

    All runs start from this deterministic state (maximum clearance).
    Dispatches all poles concurrently, waits for every arrival; on any
    failure halts all arms and returns False.
    """
    live = [a for a in arms if a is not None]
    targets = ", ".join(
        f"{a.side}={lift_hw_mm(a.side, LIFT_M['full'])} "
        f"({LIFT_GEAR[a.side]['name']})" for a in live)
    print(f"  pre-positioning pole(s) to full_length "
          f"({LIFT_M['full']} m -> {targets}) ...")
    begs = [(arm, arm.begin(monitor, ("lift", "full"))) for arm in live]
    ok = True
    for arm, beg in begs:
        rec = arm.finish(monitor, beg)
        if rec["ok"]:
            dur = rec["t_done"] - rec["t_dispatch"]
            print(f"  [INFO] {arm.side}: pole at full length "
                  f"({dur:.2f} s, event={rec['event']})")
        else:
            print(f"  [WARN] {arm.side}: pole homing FAILED "
                  f"(ret={rec['ret']}, event={rec['event']})")
            if rec["ret"] == 1:
                # Controller actively rejected the command (observed on
                # BOTH arms 2026-08-06 20:38) — dump why, if it will say.
                diagnose_lift_rejection(arm)
            ok = False
    if not ok:
        stop_all(*live)
    return ok


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
