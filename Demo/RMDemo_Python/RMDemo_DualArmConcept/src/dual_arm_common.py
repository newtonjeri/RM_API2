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
import socket
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


def host_ip_for(arm_ip: str) -> str:
    """THIS machine's address on the route to `arm_ip` — the UDP push target.

    The realtime state feed is a PUSH: `rm_set_realtime_push` hands the
    controller an IP:port and it fires frames there, deriving nothing from
    the TCP session. A WRONG address is accepted with ret=0 and pushed into
    the void, which looks exactly like "the arm never moved" — so getting
    it right matters more than the one-line API suggests.

    Rather than make the operator know their own address, ask the kernel
    which local address it would use to reach this particular arm. The UDP
    `connect()` sends no packets; it only resolves the route. Correct
    per-arm, and it survives switching interfaces or laptops.

    RM_HOST_IP still wins when set (a pinned value for odd routing setups).
    """
    pinned = os.environ.get("RM_HOST_IP")
    if pinned:
        return pinned
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((arm_ip, 1))
            local = s.getsockname()[0]
        if local and not local.startswith("127."):
            return local
    except Exception:
        pass
    return HOST_IP

ARM_SPEED_PCT  = 20           # movej speed percentage, conservative
LIFT_SPEED_PCT = 50           # measured ~56.4 phys mm/s (LiftBenchmark map)

ARM_TIMEOUT_S  = float(os.environ.get("RM_ARM_TIMEOUT_S", "40"))
# 25 s was too tight: on 2026-08-07 a left-arm sync leg took 19.7 s and the
# next one hit exactly 25.02 s and timed out, failing a step in which the ARM
# was completely healthy. The pole is far slower than the speed model
# predicts (see matched_lift_speed_pct) — until that is recalibrated, give
# lift moves real headroom.
LIFT_TIMEOUT_S = float(os.environ.get("RM_LIFT_TIMEOUT_S", "60"))
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
# ─── Pole motion model (motor-derived, 2026-08-07) ─────────────────────────
# The Web GUI Lift Control panel reports the drive constants directly:
#   max 1250 rpm, RR 0.005 m/rev, accel 5000 rpm/s, travel 0-315 mm
# so 100% = 1250/60 rev/s * 0.005 m/rev = 0.1042 m/s = 104.2 mm/s.
#
# This REPLACES the butterfli_hw constant kLiftCruiseMpsPerPct = 0.00185
# (1.85 mm/s per %). That value implied 100% = 185 mm/s = 2220 rpm, well
# above the drive's 1250 rpm ceiling — its hw->physical conversion was
# scaled by 1.5 (the assumed 2:3 gearing), which inflated every velocity
# by the same factor. 1.85 / 1.5 = 1.23, and the motor constant is 1.04.
#
# The bench's *structure* still holds and is what matters: the profile is
# ACCELERATION-LIMITED, so short strokes never reach cruise (bench §3.5 —
# every speed >= 40% delivers the same ~0.037 m/s over a 20 mm stroke).
# A single linear constant is what made the old estimate 2-7x optimistic;
# lift_travel_time_s() below models the trapezoid instead.
LIFT_MM_S_PER_PCT      = 1250.0 / 60.0 * 0.005 * 1000.0 / 100.0   # 1.0417
# butterfli_hw fitted accel 0.166 m/s^2 from the same 1.5x-inflated data;
# corrected -> 0.111 m/s^2. Validated against the 2026-08-07 C8 runs below.
LIFT_ACCEL_MM_S2       = 111.0
LIFT_MIN_MATCH_PCT     = 4        # lowered floor (TODO item 1, 2026-07-25)
ARM_MAX_DEG_S          = 180.0    # J1-J6 software speed limit (see below)

# ─── Controller limits AS CONFIGURED ON THESE ARMS ─────────────────────────
# Read from the Web GUI (Configuration -> Robotic Arm Config -> Security
# Config), Documentation/Hardware Limits screenshots. These are the machine's
# ACTUAL settings and they are NOT the API-documentation defaults — the docs
# list 0.1 m/s / 0.2 rad/s / 0.5 / 1.0, every one of which is lower than what
# is configured here. Read them from the arm rather than trusting the manual.
TCP_MAX_LINE_SPEED_MS    = 0.250   # rm_get/set_arm_max_line_speed
TCP_MAX_LINE_ACC_MS2     = 1.600   # rm_get/set_arm_max_line_acc
TCP_MAX_ANGULAR_SPEED_RS = 0.600   # rm_get/set_arm_max_angular_speed
TCP_MAX_ANGULAR_ACC_RS2  = 4.000   # rm_get/set_arm_max_angular_acc
# Joint limits: J1-J6 speed 180 deg/s (range 1-180), J7 225 deg/s
# (range 1-225); acceleration 600 deg/s^2 on every joint; J1 travel
# +/-178 deg, J7 +/-365 deg. est_arm_duration_s() uses the 180 figure, so it
# UNDER-estimates a J7-dominated move.
ARM_J7_MAX_DEG_S       = 225.0
ARM_MAX_ACC_DEG_S2     = 600.0

# ─── Sync dispatch ORDER (hardware-decided 2026-08-07) ──────────────────────
# A lift command issued while a PLANNED arm trajectory is in flight ABORTS
# that trajectory: the arm stops short and NO device-0 arrival event ever
# arrives. Proven on one arm with hands absent (C9 --no-hands froze at the
# first sync step; the same run with --no-pole passed 6/6).
#
# Our ZIGZAG01 Web-GUI program (in-house, not a vendor example) moves
# together and never does this — it issues the lift NON-BLOCKING FIRST and
# only then commands the arm, so the pole is already in flight when the arm
# move starts. That is also the ordering butterfli_hw's bench_sync proved
# (pole command first, arm follows after a measured lead). Hence the
# default here is lift-first.
#   RM_SYNC_ORDER=arm_first   restore the old (freezing) order for A/B tests
#   RM_SYNC_LEAD_MS=N         pole head start before the arm command; 0 =
#                             back-to-back, exactly like the vendor program
# ─── Sync BACKEND — how the arm half of a sync step is driven ──────────────
# Both paths are kept deliberately. C16 established (model-free, both arms)
# that a lift command and a PLANNED arm move truncate each other, so the
# 'planned' backend is expected to fail — but it stays as the reference case
# while RealMan support reviews the behaviour: if they supply a fix or a
# configuration, re-testing it is one env flip, not a rewrite.
#
#   canfd    (default) arm streamed with rm_movej_canfd while the lift runs
#            — the combination butterfli_hw bench_sync proved works
#   planned  arm driven by rm_movej — the failing case, kept for A/B and
#            for re-test when support replies
#
# A real bonus of the canfd path: the arm's duration is COMMANDED, not
# estimated, so the pole match no longer depends on est_arm_duration_s()
# (measured 24%-120% wrong).
SYNC_BACKEND = os.environ.get("RM_SYNC_BACKEND", "canfd").lower()
if SYNC_BACKEND not in ("canfd", "planned"):
    raise SystemExit(f"RM_SYNC_BACKEND must be canfd|planned, "
                     f"got {SYNC_BACKEND!r}")
# VENDOR-CONFIRMED DEFECT (RealMan support, WeChat, 2026-08-08): "there is
# a conflict between the third-generation controller's control of the
# hoist's movement and the arm's movement ... documented problem, software
# engineers working on it, no completion date. Temporary solution: when
# controlling the hoist, choose the blocking mode." Their workaround IS
# serialization — which matches C16 exactly. Until RealMan ships a fix,
# the planned backend is LOCKED; sync runs on canfd only.
if SYNC_BACKEND == "planned" \
        and os.environ.get("RM_UNLOCK_PLANNED_SYNC") != "1":
    raise SystemExit(
        "RM_SYNC_BACKEND=planned is LOCKED.\n"
        "RealMan confirmed (2026-08-08) a documented Gen-3 controller "
        "defect: concurrent hoist + arm control conflicts, no fix ETA.\n"
        "Synchronization runs on the canfd backend (the default). Set "
        "RM_UNLOCK_PLANNED_SYNC=1 only to RE-TEST after a firmware fix.")
# Streaming parameters (butterfli_hw proved 100 Hz / high-follow stable:
# canfd period mean 9.992 ms, p99 <= 10.16 under full concurrent load).
CANFD_RATE_HZ = float(os.environ.get("RM_CANFD_RATE_HZ", "100"))
CANFD_FOLLOW = os.environ.get("RM_CANFD_FOLLOW", "1") != "0"
CANFD_ARM_DURATION_S = float(os.environ.get("RM_CANFD_ARM_S", "4.0"))

SYNC_ORDER = os.environ.get("RM_SYNC_ORDER", "lift_first").lower()
if SYNC_ORDER not in ("lift_first", "arm_first"):
    raise SystemExit(f"RM_SYNC_ORDER must be lift_first|arm_first, "
                     f"got {SYNC_ORDER!r}")
SYNC_LEAD_S = max(0.0, float(os.environ.get("RM_SYNC_LEAD_MS", "0")) / 1000.0)

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

# ─── Lift heights (metres) ──────────────────────────────────────────────────
# Both poles are now configured to 315 mm of travel with 1:1 scaling
# (Newton, 2026-08-07), so the usable band widened: full_length 0.29 -> 0.30
# and minimum 0.01 -> 0.005. NOTE: this diverges from the SRDF `pole_*`
# group states these values originally came from — update the SRDF to match
# before the ROS 2 side plans against them (see PHASE_PLAN C14).
LIFT_M = {"minimum": 0.005, "quarter": 0.075, "half": 0.15, "full": 0.30}
LIFT_MIN_M   = 0.005
LIFT_MAX_M   = 0.30

# Per-side lift gearing. V1.7.4 switched the lift to TRUE millimetres
# (1:1, travel 0-330 — confirmed physically 2026-08-06: commanding 193
# under the old 2/3 assumption left the pole mid-rail). V1.7.1 is 2/3
# geared over hw 0-200.
#
# The gearing is DETECTED from the controller firmware at connect time
# (see detect_lift_gear) rather than hard-coded per side, because a wrong
# assumption is silently dangerous in both directions: 193 on a 1:1
# controller parks the pole mid-rail, and 290 on a 2:3 controller is out
# of range. The values below are only the offline/unreadable fallback.
# RM_LEFT_LIFT_GEAR / RM_RIGHT_LIFT_GEAR pin it explicitly and win over
# detection.
_GEARS = {
    # hw_max is the CONTROLLER-configured travel ceiling, not the model's
    # limit: both poles are set to 315 mm (2026-08-07). full_length 0.30 m
    # -> 300 hw-mm leaves 15 mm of headroom below it.
    "1to1": {"hw_per_m": 1000.0, "hw_to_phys": 1.0, "hw_max": 315},
    "2to3": {"hw_per_m": 2000.0 / 3.0, "hw_to_phys": 1.5, "hw_max": 200},
}


def _lift_gear(side: str, default: str) -> dict:
    v = os.environ.get(f"RM_{side.upper()}_LIFT_GEAR", default)
    v = v.strip().lower().replace(":", "to")
    if v not in _GEARS:
        raise SystemExit(f"RM_{side.upper()}_LIFT_GEAR must be 1to1 or 2to3, "
                         f"got {v!r}")
    return dict(_GEARS[v], name=v)


# Both arms now run true-mm 1:1 (right upgraded 2026-08-07). Detection at
# connect still overrides these, so a rollback needs no edit either.
LIFT_GEAR = {"left": _lift_gear("left", "1to1"),
             "right": _lift_gear("right", "1to1")}

# First controller firmware known to report the lift in TRUE millimetres.
# Left V1.7.4 = 1:1/0-330, right V1.7.1 = 2:3/0-200 (both measured); the
# exact introducing version is not documented, so this is the boundary we
# infer from. Override per side with RM_*_LIFT_GEAR if it turns out wrong.
LIFT_TRUE_MM_FROM_VERSION = (1, 7, 4)


def _version_tuple(text) -> tuple:
    """'1.7.4' / 'V1.7.4-rc' -> (1, 7, 4); () when unparseable."""
    parts = []
    for chunk in str(text or "").split("."):
        digits = "".join(c for c in chunk if c.isdigit())
        if not digits:
            break
        parts.append(int(digits))
    return tuple(parts)


def detect_lift_gear(robot) -> str:
    """Lift gearing implied by the controller firmware, or '' if unreadable."""
    try:
        ret, sw = robot.rm_get_arm_software_info()
    except Exception:
        return ""
    if ret != 0 or not isinstance(sw, dict):
        return ""
    ver = _version_tuple((sw.get("ctrl_info") or {}).get("version"))
    if not ver:
        return ""
    return "1to1" if ver >= LIFT_TRUE_MM_FROM_VERSION else "2to3"


def apply_detected_lift_gear(side: str, robot) -> None:
    """Adopt the firmware-implied gearing for this side (env pin wins).

    Called on every connect, so upgrading an arm needs no code or env
    change — and a MISMATCH between assumption and firmware is reported
    loudly instead of parking the pole in the wrong place.
    """
    if os.environ.get(f"RM_{side.upper()}_LIFT_GEAR"):
        return                                  # explicit pin: leave it alone
    detected = detect_lift_gear(robot)
    current = LIFT_GEAR[side]["name"]
    if not detected:
        print(f"  [WARN] {side}: controller version unreadable — keeping "
              f"assumed lift gearing {current} "
              f"(pin with RM_{side.upper()}_LIFT_GEAR if this is wrong)")
        return
    if detected != current:
        LIFT_GEAR[side] = dict(_GEARS[detected], name=detected)
        print(f"  [INFO] {side}: lift gearing {current} -> {detected} "
              f"(detected from controller firmware); full_length now "
              f"{lift_hw_mm(side, LIFT_M['full'])} hw-mm")

# ─── Hand states (SDK order [little,ring,middle,index,thumb_flex,thumb_rot],
#     0-1000, 1000 = open; SRDF inspire_hand states via hand_rad_to_hw) ─────
HAND_STATES_HW = {
    "release":    [993, 993, 993, 993, 981, 992],
    "half_grasp": [516, 516, 516, 516, 133, 944],
    "grasp":      [33, 33, 33, 33, 133, 944],
}

# Angle -> hardware counts, fitted from the THREE states above, for which we
# hold both forms: the SRDF `inspire_hand_*` group states give radians and
# HAND_STATES_HW gives the counts the bench measured.
#
#   fingers  0.01 rad -> 993      1.30 rad -> 33
#   thumb_1  0.01     -> 981      0.0698  -> 133
#   thumb_2  0.01     -> 992      0.454   -> 944
#
# The finger fit is linear to within one count (0.65 rad predicts 516.7 vs
# the measured 516), so a task pose named in the SRDF but absent from
# HAND_STATES_HW can be converted rather than GUESSED. That matters: the
# commode tasks use `open_tenth` (1.17 rad) and `quarter_grasp` (0.336),
# neither of which is a concept state. Substituting the nearest name would
# have driven `open_tenth` to 993 counts instead of ~130 — the hand flying
# open mid-task while gripping a cloth against the fixture.
_HAND_FIT = {           # (slope, intercept) per SDK slot
    "finger": (-744.19, 1000.44),
    "thumb_1": (-14180.60, 1122.81),
    "thumb_2": (-108.11, 993.08),
}
# SDK order is little, ring, middle, index, thumb flex, thumb rotation.
HAND_SLOT_JOINTS = ("little_1", "ring_1", "middle_1", "index_1",
                    "thumb_1", "thumb_2")


def hand_rad_to_hw(angles_by_joint: dict) -> list:
    """{joint suffix -> radians} -> the 6 hardware counts, clamped 0..1000.

    Keys may be full URDF names (`IRH_right_index_1_joint`) or the bare
    suffixes in HAND_SLOT_JOINTS. Missing joints raise rather than default:
    a silently-defaulted finger is a wrong grip, not a small error.
    """
    def _find(suffix):
        for k, v in angles_by_joint.items():
            if k == suffix or k.endswith(f"_{suffix}_joint"):
                return float(v)
        raise KeyError(f"no angle for hand joint {suffix!r} in "
                       f"{sorted(angles_by_joint)}")
    out = []
    for slot in HAND_SLOT_JOINTS:
        kind = slot if slot in ("thumb_1", "thumb_2") else "finger"
        m, b = _HAND_FIT[kind]
        out.append(int(round(max(0.0, min(1000.0, m * _find(slot) + b)))))
    return out

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


# The pole must OUTLAST the arm move: its motion has to span the arm's, so
# that neither the pole's start nor its completion lands inside the arm's
# trajectory (see the polarity note below). Target pole duration is the arm
# duration inflated by this factor.
SYNC_POLE_OUTLAST = float(os.environ.get("RM_SYNC_POLE_OUTLAST", "1.5"))


def lift_travel_time_s(dist_phys_mm: float, pct: int,
                       ascending=None) -> float:
    """Predicted pole travel time (start latency + trapezoidal profile).

    Replaces `distance / (k * pct)`, which assumed the pole was always at
    cruise. It is acceleration-limited, so a short stroke runs a TRIANGULAR
    profile and never reaches the commanded speed at all — the reason the
    old linear estimate was wildly optimistic on the strokes this concept
    actually uses.

    Checked against the 2026-08-07 C8 hardware runs (arm IDLE):
      left  140 mm @ 50%  -> predicts 3.49 s, measured 3.47 s   (+0.6%)
      left   10 mm @ 50%  -> predicts 0.83 s, measured 0.68-0.75 s
      right  15 mm @ 50%  -> predicts 1.12 s, measured 0.90-0.95 s
    Long strokes land within ~1%; short ones over-predict by 10-25%, which
    is the safe direction here (a pole predicted slower gets commanded a
    little faster, and the outlast factor absorbs it).

    ⚠ ARM-IDLE ONLY. With the arm moving, the pole ran 3-4x slower still
    (left sync leg: this model predicts ~5 s, measured 19.71 s). That
    coupling is unexplained and unmodelled — it makes the pole even later,
    so it is safe under the outlast polarity, but sync QUALITY cannot be
    trusted until it is measured (PHASE_PLAN open item).
    """
    latency = (LIFT_LATENCY_UP_S if ascending is True
               else LIFT_LATENCY_DOWN_S if ascending is False
               else LIFT_START_LATENCY_S)
    dist = max(float(dist_phys_mm), 0.0)
    v = max(int(pct), 1) * LIFT_MM_S_PER_PCT
    a = LIFT_ACCEL_MM_S2
    if dist <= v * v / a:                 # triangular: cruise never reached
        motion = 2.0 * math.sqrt(dist / a)
    else:                                 # trapezoid: ramp up + cruise + down
        motion = v / a + dist / v
    return latency + motion


def matched_lift_speed_pct(arm_duration_s: float, dist_phys_mm: float,
                           ascending=None) -> int:
    """Duration-match the lift to the arm move, biased so the pole finishes
    AFTER the arm.

    POLARITY INVERTED 2026-08-07 (hardware). butterfli_hw's contract treated
    a LATE pole as the failure mode (it blocked sequencing) and rounded the
    speed UP so the pole finished early or on time. On this controller the
    opposite is true: a pole that COMPLETES while a planned arm trajectory
    is still running raises `Position Command Step Warning` (0x4000) on every
    joint that is moving and halts the arm mid-path — decoded from the Web
    GUI on the right arm, J2/J4/J6 flagged while the stationary J1/J3/J5/J7
    stayed Normal. A late pole is merely slow; an early pole is a fault.

    So: target `arm_duration * SYNC_POLE_OUTLAST`, and quantize ROUND-DOWN
    (a lower percentage = slower = later = safer). The speed model itself is
    known to be optimistic by 2-7x on this firmware, which only makes the
    pole later still — safe in the same direction, and the reason the left
    arm survived both sync steps on 2026-08-07.

    Start latency is direction-aware when known (bench up 0.33 s / down
    0.23 s; 0.38 worst case).
    """
    if dist_phys_mm <= 0.0:
        return LIFT_MIN_MATCH_PCT
    target_s = arm_duration_s * SYNC_POLE_OUTLAST
    # Walk up from the floor and take the FIRST (= slowest) speed whose
    # predicted travel still fits the target: slowest that fits finishes
    # latest, and late is the safe direction. If even 100% cannot make the
    # target the pole is late regardless, so use 100% to minimise it.
    for pct in range(LIFT_MIN_MATCH_PCT, 101):
        if lift_travel_time_s(dist_phys_mm, pct, ascending) <= target_s:
            return pct
    return 100


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
  --clear-errors    clear LATCHED controller/joint errors before the run.
                    A sudden trajectory abort (or an e-stop) latches joint
                    errors and every motion command is then rejected with
                    ret=1 — motion tests refuse to start until they clear
  -h, --help        show the script's documentation and exit (no motion)
C8 pole diagnostic (test_pole_only.py) only:
  --diagnose-only   read state and report; NO pole motion at all

Environment overrides:
  RM_LEFT_IP / RM_RIGHT_IP / RM_ROBOT_PORT     arm endpoints
  RM_HOST_IP / RM_UDP_PORT                     UDP push target (C5)
  RM_ARM=left|right                            arm selection (C6/C7)
  RM_LEFT_LIFT_GEAR / RM_RIGHT_LIFT_GEAR       1to1 | 2to3
  RM_SYNC_BACKEND=canfd|planned                how the ARM half of a sync
                    step is driven. canfd (default) = streamed passthrough,
                    the combination bench_sync proved works. planned =
                    rm_movej — LOCKED (vendor-confirmed Gen-3 defect,
                    2026-08-08, no fix ETA); needs RM_UNLOCK_PLANNED_SYNC=1,
                    only for re-testing after a RealMan firmware fix
  RM_CANFD_RATE_HZ / RM_CANFD_ARM_S / RM_CANFD_FOLLOW   stream parameters
  RM_SYNC_ORDER=lift_first|arm_first           sync dispatch order (a lift
                    command lands on an in-flight movej under arm_first and
                    KILLS it — that order froze the arms 2026-08-06/07)
  RM_SYNC_LEAD_MS=N                            pole head start on sync steps
  RM_ARM_TIMEOUT_S / RM_LIFT_TIMEOUT_S         arrival waits (default 40 / 60)
  RM_HAND_DWELL_S / RM_HAND_MODBUS_DEVICE / RM_KEEP_MODBUS=1
  RM_ALLOW_NO_UDP=1 / RM_EMU_TIME_SCALE
"""


def handle_cli(doc: str, argv=None, extra_flags=(), value_flags=()):
    """-h/--help prints the script docs and exits BEFORE anything runs;
    any unknown argument is rejected (exit 2) rather than silently
    ignored — an ignored typo would otherwise move the robot.

    extra_flags: additional exact flags this script accepts (e.g. the C8
    pole diagnostic's --diagnose-only / --clear-errors).
    value_flags: flags that CONSUME the next argument (e.g. C15's
    --speeds / --stroke), so their value is not mistaken for a stray
    positional and rejected."""
    args = list(sys.argv[1:] if argv is None else argv)
    if "-h" in args or "--help" in args:
        print((doc or "").strip())
        print()
        print(USAGE)
        raise SystemExit(0)
    consuming = ("--mode",) + tuple(value_flags)
    skip = False
    for a in args:
        if skip:
            skip = False
            continue
        if a in consuming:
            skip = True
            continue
        if any(a.startswith(f + "=") for f in value_flags):
            continue
        if a.startswith("--mode=") \
                or a in ("--no-hands", "--no-pole", "--clear-errors") \
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


def parse_clear_errors_arg(argv=None) -> bool:
    """--clear-errors clears latched faults before the run starts."""
    args = list(sys.argv[1:] if argv is None else argv)
    return "--clear-errors" in args


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
        # Firmware decides the lift units — adopt them before any lift
        # target is computed (see apply_detected_lift_gear).
        apply_detected_lift_gear(side, robot)
        ensure_self_collision_enabled(self)

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
            # Lift FIRST by default: a lift command that lands on an
            # in-flight planned arm trajectory kills it (see SYNC_ORDER).
            if SYNC_ORDER == "arm_first":
                return [(DEV_JOINT, target[0]), (DEV_LIFT, target[1])]
            return [(DEV_LIFT, target[1]), (DEV_JOINT, target[0])]
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

    def start_arm_canfd(self, target, duration_s=None) -> dict:
        """Stream the arm to `target` with rm_movej_canfd (passthrough).

        This is the arm half of a CANFD sync step. Unlike a planned move it
        does NOT bypass-plan on the controller and it emits NO device-0
        arrival event, so completion is duration-based (the stream ends) and
        verified afterwards by joint position — the same shape as the hand's
        acked_angle path.

        The profile is a cosine ease (butterfli_hw bench_sync uses the same
        shape), which keeps commanded velocity continuous at both ends; a
        step discontinuity here is what the controller flags on the planned
        path. Duration is COMMANDED, so the pole's duration match against it
        is exact rather than estimated.
        """
        dur = float(duration_s or CANFD_ARM_DURATION_S)
        holder = {"target": target, "ret": None, "duration_s": dur,
                  "t_dispatch": time.perf_counter(), "t_done": None,
                  "sent": 0, "errors": 0}
        goal = state_deg(self.side, target)
        ret, st = self.robot.rm_get_current_arm_state()
        start = list(st["joint"]) if ret == 0 else state_deg(self.side, "ready")

        def work():
            period = 1.0 / CANFD_RATE_HZ
            t0 = time.perf_counter()
            next_t = t0
            while True:
                now = time.perf_counter()
                phase = min(1.0, (now - t0) / dur)
                # cosine ease: zero velocity at both ends
                s = (1.0 - math.cos(math.pi * phase)) / 2.0
                cmd = [a + (b - a) * s for a, b in zip(start, goal)]
                try:
                    r = self.robot.rm_movej_canfd(cmd, CANFD_FOLLOW, 0, 0, 0)
                except Exception:
                    r = -1
                holder["sent"] += 1
                if r != 0:
                    holder["errors"] += 1
                    holder["ret"] = r
                if phase >= 1.0:
                    break
                next_t += period
                time.sleep(max(0.0, next_t - time.perf_counter()))
            if holder["ret"] is None:
                holder["ret"] = 0
            holder["t_done"] = time.perf_counter()

        t = threading.Thread(target=work, daemon=True)
        holder["thread"] = t
        t.start()
        return holder

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
                # canfd: the arm duration is COMMANDED, so it is exact.
                # planned: it has to be estimated (24%-120% error measured).
                arm_dur = (CANFD_ARM_DURATION_S if SYNC_BACKEND == "canfd"
                           else est_arm_duration_s(
                               cur, state_deg(self.side, step[1][0]),
                               ARM_SPEED_PCT))
                beg["sync_backend"] = SYNC_BACKEND
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
        for index, (device, target) in enumerate(parts):
            if index and step[0] == "sync" and SYNC_LEAD_S:
                # Pole head start: it has the longer start latency (~235 ms
                # down / ~330 ms up, bench_sync), so leading it makes the
                # two devices START together rather than merely being
                # commanded together.
                time.sleep(SYNC_LEAD_S)
            if device == DEV_HAND:
                holder = self.start_hand_acked(target)
                beg["devices"][device] = {"target": target, "ret": 0,
                                          "t_dispatch": holder["t_dispatch"],
                                          "t_done": None, "event": False,
                                          "acked": False, "verified": False,
                                          "ok": False, "_hand": holder}
                continue
            if device == DEV_JOINT and step[0] == "sync" \
                    and SYNC_BACKEND == "canfd":
                # Passthrough arm half: streamed, no arrival event, so it
                # completes on stream end + position verify (see finish()).
                holder = self.start_arm_canfd(target)
                beg["devices"][device] = {"target": target, "ret": 0,
                                          "t_dispatch": holder["t_dispatch"],
                                          "t_done": None, "event": False,
                                          "acked": False, "verified": False,
                                          "ok": False, "_canfd": holder}
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
            if "_canfd" in d:
                holder = d.pop("_canfd")
                holder["thread"].join(holder["duration_s"] + 10.0)
                d["ret"] = holder["ret"] if holder["ret"] is not None else -1
                d["t_done"] = holder["t_done"] or time.perf_counter()
                # No arrival event exists for passthrough: the stream
                # finishing IS the completion, and the joint position is
                # the only real confirmation.
                d["acked"] = d["ret"] == 0
                d["verified"] = self.verify_device(device, d["target"])
                d["ok"] = d["acked"] and d["verified"]
                d["canfd_sent"] = holder["sent"]
                d["canfd_errors"] = holder["errors"]
                continue
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
                # positive = pole finished LATE vs the arm. Late is the SAFE
                # direction here: a pole completing mid-arm-trajectory faults
                # the moving joints (see matched_lift_speed_pct).
                rec["sync_finish_skew_s"] = l["t_done"] - j["t_done"]
            rec["arm_dur_est_s"] = beg.get("arm_dur_est_s")
            rec["lift_speed_pct"] = beg.get("lift_speed_pct")
            rec["sync_backend"] = beg.get("sync_backend", SYNC_BACKEND)
            rec["canfd_sent"] = beg["devices"][DEV_JOINT].get("canfd_sent")
            rec["canfd_errors"] = beg["devices"][DEV_JOINT].get("canfd_errors")
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


def ensure_self_collision_enabled(arm: "ConceptArm") -> None:
    """Turn ON the controller's self-collision safety detection.

    Per the API docs it "ensures that, when enabled, the various parts of
    the robotic arm do not collide with each other during processes such as
    trajectory planning and teaching" — whole-arm, at PLANNING time, and
    independent of anything we check offline. It is free and it covers a
    failure mode our scene primitives do not, so the tests enable it rather
    than inheriting whatever the arm was left in.

    Note this is NOT the same as the Web GUI's "Collision Protection
    Level" (0-8, rm_set_collision_state), which is a contact/torque
    threshold. Both exist; this one is the geometric check.

    RM_SELF_COLLISION=0 leaves the arm as-found (for A/B testing).
    """
    if os.environ.get("RM_SELF_COLLISION") == "0":
        print(f"  [INFO] {arm.side}: self-collision left as-found "
              "(RM_SELF_COLLISION=0)")
        return
    try:
        ret, enabled = arm.robot.rm_get_self_collision_enable()
    except Exception as exc:
        print(f"  [WARN] {arm.side}: self-collision state unreadable "
              f"({exc!r}) — cannot confirm it is on")
        return
    if ret == 0 and enabled:
        return                                   # already on, stay quiet
    try:
        sret = arm.robot.rm_set_self_collision_enable(True)
    except Exception as exc:
        sret = repr(exc)
    print(f"  [INFO] {arm.side}: self-collision detection was "
          f"{'OFF' if ret == 0 else 'UNKNOWN'} -> enabling (ret={sret})")


def error_state(arm: "ConceptArm") -> dict:
    """Every error surface the controller exposes for one arm."""
    r = arm.robot
    st = {"sys": [], "joints": [], "lift_err": None, "readable": False}
    try:
        ret, s = r.rm_get_current_arm_state()
        if ret == 0:
            st["readable"] = True
            err = (s or {}).get("err")
            if isinstance(err, dict):
                st["sys"] = [str(c) for c in err.get("err", [])
                             if str(c) not in ("0", "")]
    except Exception:
        pass
    try:
        jd = r.rm_get_joint_err_flag()
        if jd.get("return_code") == 0:
            st["readable"] = True
            st["joints"] = [(i + 1, f)
                            for i, f in enumerate(jd.get("err_flag", [])) if f]
    except Exception:
        pass
    try:
        ret, lst = r.rm_get_lift_state()
        if ret == 0:
            st["lift_err"] = lst.get("err_flag", 0)
    except Exception:
        pass
    # DISABLED joints are an error surface of their own. On 2026-08-08 the
    # left arm sat with J6/J7 "Under Voltage" and de-enabled — visible in
    # the Web GUI — while rm_get_current_arm_state reported err ['0'] and
    # the connect test declared it clean. Every motion command then failed
    # (movej to move_to_start died in 0.07 s), and teach mode was DANGEROUS:
    # pressing it releases all brakes, and unpowered wrist joints flop
    # under gravity. A joint that is not enabled must fail the gate.
    st["disabled"] = []
    try:
        ret, en = r.rm_get_joint_en_state()
        if ret == 0 and en:
            st["disabled"] = [i + 1 for i, e in enumerate(en) if not e]
    except Exception:
        pass
    return st


def error_state_clean(st: dict) -> bool:
    return (not st["sys"] and not st["joints"] and not st["lift_err"]
            and not st.get("disabled"))


def describe_error_state(st: dict) -> str:
    if not st["readable"]:
        return "unreadable"
    if error_state_clean(st):
        return "clean"
    bits = []
    if st["sys"]:
        bits.append("system " + ",".join(st["sys"]))
    if st["joints"]:
        bits.append("joints " + ",".join(f"J{i}={f}" for i, f in st["joints"]))
    if st["lift_err"]:
        bits.append(f"lift driver {st['lift_err']}")
    if st.get("disabled"):
        bits.append("DISABLED joints "
                    + ",".join(f"J{i}" for i in st["disabled"])
                    + " — do NOT use teach mode (brakes release, unpowered"
                    " joints fall); recover with recover_joints.py or the"
                    " GUI's Clear Error + Enable")
    return "; ".join(bits)


def clear_errors(*arms) -> bool:
    """Clear latched faults: system-level first, then PER JOINT.

    Hardware-proven gap (2026-08-07, both arms): after a trajectory
    truncation, `rm_clear_system_err()` returns 0 but the per-joint
    Position Command Step Warnings (16384) STAY SET — Newton had to press
    each joint's own "Clear Error" button in the Web GUI. That button is
    `rm_set_joint_clear_err(joint_num)`, so this walks the flagged joints
    and clears them individually, then re-reads to confirm.
    """
    ok = True
    for arm in [a for a in arms if a is not None]:
        try:
            ret = arm.robot.rm_clear_system_err()
        except Exception as exc:
            ret, ok = repr(exc), False
        st = error_state(arm)
        if st["joints"]:
            # the R8 gap: system clear does not touch these
            for joint_num, flag in st["joints"]:
                try:
                    jret = arm.robot.rm_set_joint_clear_err(joint_num)
                except Exception as exc:
                    jret = repr(exc)
                print(f"  [INFO] {arm.side}: rm_set_joint_clear_err(J"
                      f"{joint_num}) ret={jret}  (flag was {flag})")
            st = error_state(arm)
        print(f"  [INFO] {arm.side}: clear_errors -> "
              f"{describe_error_state(st)}")
        ok = ok and ret == 0 and error_state_clean(st)
    return ok


def preflight_error_gate(*arms, clear: bool = False):
    """Refuse to start motion on a controller with LATCHED errors.

    A sudden trajectory abort latches joint errors on the hardware (the
    lift-onto-moving-arm defect below, an e-stop, a collision stop). Every
    later motion command is then rejected with ret=1 until the errors are
    cleared, so a run started in that state fails confusingly several steps
    in. Catching it here turns that into one line before anything moves.

    Returns (ok, detail).
    """
    live = [a for a in arms if a is not None]
    if clear:
        clear_errors(*live)
    dirty = []
    for arm in live:
        st = error_state(arm)
        desc = describe_error_state(st)
        if error_state_clean(st) or not st["readable"]:
            print(f"  [INFO] {arm.side}: error state {desc}")
        else:
            print(f"  [WARN] {arm.side}: LATCHED ERRORS — {desc}")
            dirty.append(f"{arm.side}: {desc}")
    if not dirty:
        return True, "no latched errors"
    return False, ("; ".join(dirty) + " — motion will be REJECTED (ret=1); "
                   "rerun with --clear-errors (power-cycle if they persist)")


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
