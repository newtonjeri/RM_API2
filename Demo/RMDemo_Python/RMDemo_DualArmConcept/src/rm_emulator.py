"""In-process emulator of the RealMan RM_API2 Python SDK surface.

Emulates the dual RM75-6FB / Gen-3 / V1.7.1 setup (left 192.168.1.10,
right 192.168.1.103, pole lifts) closely enough to run the concept tests —
and other simple tests — without hardware:

  - connection lifecycle, per-handle demux, distinct handle ids
  - identity getters answering exactly what the real arms answered
    (RM_75, 7-DOF, 6FB, controller gen 3, ctrl/plan V1.7.1)
  - rm_movej: synchronized joint interpolation at v% of 180 deg/s,
    continuous interpolated state while moving, blocking or non-blocking
  - rm_set_lift_height: queued (the real controller queues rather than
    preempts — RMDemo_LiftBenchmark report §5.3), duration from the
    MEASURED speed map in the LiftBenchmark hw_baseline plus the measured
    0.38 s start latency
  - arrival events: process-global callback, rm_event_push_data_t-shaped
    payloads (handle_id / event_type=1 / trajectory_state / device 0|3 /
    trajectory_connect=0) fired from the motion completion thread
  - UDP realtime push (subset): periodic frames with joint_status /
    liftState / arm_current_status
  - rm_set_arm_stop halts arm AND pole (the benchmark-documented reliable
    pole halt); rm_set_lift_speed(0) halts the pole only
  - ~5 ms per-command latency (measured mean was ~8 ms)

Fault injection per arm (emu_controller(ip)):
    .reject_next_dispatch = True   next motion command returns ret 1
    .fail_next_motion     = True   next arrival reports trajectory_state False
    .drop_next_event      = True   next arrival event silently not delivered
    .command_latency_s    = float  per-command latency

Time scaling: set_time_scale(N) or env RM_EMU_TIME_SCALE — divides every
motion duration by N (events, ordering, and interpolation stay coherent).

Install as the SDK: rm_emulator.install() puts emulated Robotic_Arm modules
into sys.modules BEFORE the code under test imports them.

Cartesian chains are emulated to MEASURED-LAW fidelity (2026-08-16, all
constants from MOTION_FINDINGS §9-§10): per-move v/r queues, blend cuts
c(theta)*r*minL with sub-sample trim + measured-bulge bridges, corner-speed
retention, the ~26 mm blend floor, first-corner exemption (tangent-arc
exempt from the exemption), movec 3-point arcs with centripetal caps,
H67 angular throttling as a per-segment TIME-SCALING, stop dwells +
chain-end settle, freeze-signature warnings (RM_EMU_FREEZES=1 injects
dwells interpolated between two measured points), controller-frame pose
boundary (feedback round-trips with targets; validated T = 0.00 mm against
four REAL streams), REAL-mode aliasing noise on the position field.

MEASURED ACCURACY (motion-window durations, 2026-08-16 audit):
  * 5 runs whose constants were TUNED here: mean |err| 3.5 %, max 6.3 %
  * 15 HELD-OUT runs never used for tuning: mean 8.2 %, median 6.6 %,
    max 22.2 %
  * blend cuts land 2-4 mm short of the controller's (5.9 vs 7.9 mm at
    r=25; 11.9 vs 15.8 at r=50); vertex miss within 0.6 mm; WHICH corners
    blend, the exemption and the r=0 stops are exact.

KNOWN FAILURE MODES - do not use the emulator for these:
  * ANGULAR-CAP PROJECTIONS. The measured cap ladder is nearly flat
    (40.5/38.7/38.5 s at 0.6/0.8/1.0) while every angular-only model is
    steep; emulated durations run -12 % at cap 0.8 and -22 % at 1.0 on
    rotation-heavy tasks. The real limiter there is joint speed, which is
    deliberately not modelled. Angular-acceleration variants were tried
    and rejected (they fix the trend but cost 12 points at the operating
    cap).
  * SHORT/FAST CHAINS. 3-5 s chains run -12 % (0.45 baseline) to -17 %
    (chained approach), most likely jerk-limited profiles not modelled.
  * JOINT RATES, as ever - the redundancy scheme is unknowable offline;
    use a saved plan or controller SIM for joint loads.

Not emulated (extend as needed): force sensor data, CANFD passthrough,
online programming, fence/collision config.
"""

import itertools
import math
import os
import pathlib
import sys as _sys
import threading
import time
from types import ModuleType, SimpleNamespace

# ── RealMan's REAL offline solver (same algo family as the controller;
#    local lib v1.6.0 vs controller 1.5.5). Loaded at import time, BEFORE
#    install() replaces the Robotic_Arm modules with the emulated ones —
#    captured references stay alive afterwards.
_sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))
try:
    from Robotic_Arm.rm_robot_interface import (
        Algo as _RealAlgo,
        rm_robot_arm_model_e as _arm_model_e,
        rm_force_type_e as _force_e,
        rm_inverse_kinematics_params_t as _ik_params_t,
    )
    # The REAL ctypes structs. The offline Algo is a C library and will
    # not accept a stand-in, so the emulator must hand out the genuine
    # article for these pure-data types — and its own tool-frame store
    # reads the same fields either way.
    from Robotic_Arm.rm_ctypes_wrap import (
        rm_pose_t as _RealPose, rm_position_t as _RealPos,
        rm_euler_t as _RealEuler, rm_frame_t as _RealFrame,
    )
    # RM75-6FB carries the integrated six-axis force sensor. Using the
    # BASE model here made the emulated arm 17.2 mm short at the wrist
    # — the same F15 error the hardware capture exposed, and C14's
    # model check catches it here too.
    _ALGO = _RealAlgo(_arm_model_e.RM_MODEL_RM_75_E,
                      _force_e.RM_MODEL_RM_ISF_E)
    _ALGO.handle = None          # offline: no controller handle
except Exception as _exc:        # pragma: no cover - platform without the .so
    _ALGO = None
    print(f"[emu] WARNING: RealMan algo library unavailable ({_exc!r}) — "
          "pose feedback will be zeros and rm_movej_p will be rejected")


def _set_algo_toolframe(tool):
    """Point the shared algo library at THIS tool before any FK/IK call.

    THE ALGO LIBRARY'S CONFIGURATION IS PROCESS-GLOBAL. Any module that
    constructs its own `Algo(...)` — `orientation_cost` does, at import
    time, and `test_blend_corner.preflight_j4` imports it mid-run —
    re-initialises that global state and silently un-sets whatever
    toolframe the emulator last applied. An IK solved after that resolves
    a FLANGE target while the FK verify applies the ACTIVE tool, and every
    `rm_movej_p` under a glove frame returns ret 1 (2026-08-14, the
    chain-semantics dry-runs). So the frame is asserted at every solver
    entry point, never assumed to persist.
    """
    if _ALGO is None or not tool:
        return
    try:
        from Robotic_Arm.rm_ctypes_wrap import (
            rm_frame_t, rm_pose_t, rm_position_t, rm_euler_t)
        f = rm_frame_t()
        f.frame_name = b"emu"
        f.pose = rm_pose_t()
        f.pose.position = rm_position_t(*[float(v) for v in tool[:3]])
        f.pose.euler = rm_euler_t(*[float(v) for v in tool[3:6]])
        _ALGO.rm_algo_set_toolframe(f)
    except Exception:
        pass


def _fk_pose(joints_deg, tool=None):
    """FK via RealMan's own solver: [x,y,z, rx,ry,rz] (m / rad).

    Arm-base frame with the library's default mounting/tool config — NOT
    the butterfli world frame (per-arm mounting is controller-side config).
    """
    if _ALGO is None:
        return [0.0] * 6
    _set_algo_toolframe(tool)
    return list(_ALGO.rm_algo_forward_kinematics(list(joints_deg), 1))[:6]


# ─── Mount rotation + orientation interpolation ─────────────────────────────
#
# THE ALGO LIBRARY WORKS IN THE URDF BASE FRAME; `rm_movel` TAKES POSES IN
# THE CONTROLLER'S BASE FRAME. The arms are mounted rotated, and the
# controller reports it as install pose (0, 90, 0) — so
#     controller = Ry(+90 deg) . urdf        (x,y,z) -> (z, y, -x)
# Three independent sources agree and all are exact (URDF xacro, the
# controller's own `rm_get_install_pose`, and a Kabsch fit over 16 captures
# with <= 0.038 mm translation). `cleaning_path.movel_joint_timeline` has
# applied this since the first REAL run; `_plan_cartesian` did NOT, and the
# result was measured 2026-08-12: **3406 of 3469 samples (98.2 %) with no IK
# solution** on `toplid_left`, because every target sat 868 mm from where the
# solver looked. The emulator then interpolated across the gaps and reported
# joint rates of 3 % of limit for a stroke the hardware runs at 92 %.
#
# Orientation is interpolated by QUATERNION SLERP, not by wrapping Euler
# deltas. The waypoints carry angles near +-pi; linear Euler interpolation
# walks the long way round and injects rotations that never happen.


def _mat3(a, b, c):
    return [list(a), list(b), list(c)]


def _Ry3(t):
    c, s = math.cos(t), math.sin(t)
    return _mat3((c, 0, s), (0, 1, 0), (-s, 0, c))


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _euler_to_R(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = _mat3((1, 0, 0), (0, cx, -sx), (0, sx, cx))
    Ry = _mat3((cy, 0, sy), (0, 1, 0), (-sy, 0, cy))
    Rz = _mat3((cz, -sz, 0), (sz, cz, 0), (0, 0, 1))
    return _mm(Rz, _mm(Ry, Rx))


def _R_to_euler(R):
    sy = math.hypot(R[0][0], R[1][0])
    if sy < 1e-9:
        return (math.atan2(-R[1][2], R[1][1]), math.atan2(-R[2][0], sy), 0.0)
    return (math.atan2(R[2][1], R[2][2]), math.atan2(-R[2][0], sy),
            math.atan2(R[1][0], R[0][0]))


def _R_to_quat(R):
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (R[2][1] - R[1][2]) / s,
                (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s]
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i][i] - R[j][j] - R[k][k]) * 2
    q = [0.0] * 4
    q[0] = (R[k][j] - R[j][k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (R[j][i] + R[i][j]) / s
    q[k + 1] = (R[k][i] + R[i][k]) / s
    return q


def _quat_to_R(q):
    w, x, y, z = q
    return _mat3(
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _slerp(qa, qb, t):
    d = sum(a * b for a, b in zip(qa, qb))
    if d < 0:
        qb, d = [-v for v in qb], -d
    if d > 0.9995:
        out = [a + (b - a) * t for a, b in zip(qa, qb)]
    else:
        th = math.acos(max(-1.0, min(1.0, d)))
        s = math.sin(th)
        out = [(math.sin((1 - t) * th) / s) * a + (math.sin(t * th) / s) * b
               for a, b in zip(qa, qb)]
    n = math.sqrt(sum(v * v for v in out)) or 1.0
    return [v / n for v in out]


def _ctrl_to_algo(pose6, mount_ry_deg):
    """Controller-frame pose -> Algo (URDF) frame. Returns (xyz, R)."""
    Rm = _Ry3(math.radians(-float(mount_ry_deg)))
    pos = _mv(Rm, [float(v) for v in pose6[:3]])
    R = _mm(Rm, _euler_to_R(*[float(v) for v in pose6[3:6]]))
    return pos, R


def _algo_to_ctrl(pose6, mount_ry_deg):
    """Algo (URDF) frame pose -> controller frame, as a 6-list.

    THE BOUNDARY RULE (2026-08-16): every pose that crosses the emulated
    API — movel/movec/movej_p targets IN, current_pose / UDP waypoint OUT —
    is a CONTROLLER-frame pose, exactly as on the real arm. Before this,
    movel targets were converted but feedback was raw algo frame, so a
    recorded stream disagreed with its own commanded poses by the 90 deg
    mount rotation — an inconsistency the real controller does not have,
    exposed the first time the emulated stream was analysed like a real one.
    """
    Rm = _Ry3(math.radians(float(mount_ry_deg)))
    pos = _mv(Rm, [float(v) for v in pose6[:3]])
    R = _mm(Rm, _euler_to_R(*[float(v) for v in pose6[3:6]]))
    return list(pos) + list(_R_to_euler(R))


def _ik_min_motion(seed_deg, pose6, span_deg=60.0, step_deg=5.0):
    """IK choosing the ARM ANGLE that moves the joints least.

    `rm_algo_inverse_kinematics` leaves the RM75's one redundant DOF free
    and picks whatever its own scheme lands on. MEASURED 2026-08-12 on
    `toplid_left`: that walks J5 through 1127 deg of travel where the saved
    plan uses 113 — a factor of TEN — while J4 (which the redundancy cannot
    reach) matches the plan to 1 %. Free-redundancy IK is therefore useless
    for predicting joint RATES, which is the whole point of the exercise.

    This variant searches the arm angle around the seed's own value and
    keeps the solution with the smallest max joint step, i.e. it resolves
    the redundancy by minimum motion instead of by solver preference.
    Returns (ret, joints_deg).
    """
    if _ALGO is None:
        return 1, None
    r0, a0 = _ALGO.rm_algo_calculate_arm_angle_from_config_rm75(
        list(seed_deg))
    if r0 != 0:
        return _ik_seeded(seed_deg, pose6)
    best, best_cost = None, None
    n = int(span_deg / step_deg)
    params = _ik_params_t(q_in=list(seed_deg), q_pose=list(pose6), flag=1)
    for k in range(-n, n + 1):
        ret, q = _ALGO.rm_algo_inverse_kinematics_rm75_for_arm_angle(
            params, float(a0 + k * step_deg))
        if ret != 0 or q is None:
            continue
        q = list(q)[:7]
        cost = max(abs(a - b) for a, b in zip(q, seed_deg))
        if best_cost is None or cost < best_cost:
            best, best_cost = q, cost
    if best is None:
        return _ik_seeded(seed_deg, pose6)
    return 0, best


def _ik_seeded(seed_deg, pose6):
    """Seeded IK via RealMan's own solver (the controller's scheme).

    Returns (ret, joints_deg): ret 0 = solved; nonzero = no solution.
    """
    if _ALGO is None:
        return 1, None
    params = _ik_params_t(q_in=list(seed_deg), q_pose=list(pose6), flag=1)
    ret, q = _ALGO.rm_algo_inverse_kinematics(params)
    return ret, (list(q)[:7] if ret == 0 else None)

# ─── Timing model (measured values where available) ─────────────────────────
ARM_MAX_DEG_S = 180.0          # synchronized-profile joint speed at v=100

# JOINT POSITION LIMITS, READ FROM THE ARMS 2026-08-11 (census/{left,right}
# .json, `rm_get_joint_drive_{max,min}_pos`; symmetric about zero on both).
#
# CORRECTED 2026-08-12. The single list that stood here —
#   [177.6, 130.0, 177.6, 135.0, 177.6, 128.0, 360.0]
# — was TIGHTER THAN THE MACHINE on every joint, and wrong by 2x on J5
# (177.6 against a real 360). It rejected a `toplid_left` stroke that the
# hardware runs, at J5 = 177.6 deg. An emulator stricter than the arm
# refuses work the arm accepts, which is the F25/H3 failure inverted and
# just as expensive.
#
# The arms also DIFFER: J4 is +-169 on the left and +-140 on the right
# (H37). One shared list cannot express that, so the limits are per side.
JOINT_LIMIT_DEG = {
    "left":  [183.0, 135.0, 183.0, 169.0, 360.0, 133.0, 365.0],
    "right": [183.0, 135.0, 183.0, 140.0, 360.0, 133.0, 365.0],
}
# Kept for callers that predate the per-side split; the TIGHTER of the two,
# so a check against it can only be conservative.
RM75_LIMIT_DEG = [min(JOINT_LIMIT_DEG["left"][j], JOINT_LIMIT_DEG["right"][j])
                  for j in range(7)]
# Latched-fault codes after an abrupt trajectory abort. The BEHAVIOUR is
# hardware-observed (joint errors latch; motion stays rejected until
# rm_clear_system_err); these specific numbers are placeholders — replace
# them if the real codes are ever captured from a frozen arm.
ABORT_JOINT_ERR = 0x0010
ABORT_SYS_ERR = 5001
# The truncation is BIDIRECTIONAL (C16, model-free, both arms): a PLANNED
# motion command to one device also truncates an in-flight motion on the
# other. When the victim is the pole it simply stops — measured consistently
# ~25 mm into a 100 mm move, at every commanded speed, with NO error raised
# and no arrival event. (A minority of high-speed cells survived on hardware;
# that is not modelled — the emulator reproduces the dominant behaviour.)
LIFT_TRUNCATE_MM = 25.0
CMD_LATENCY_S = 0.005          # per-command TCP round trip (measured ~8 ms)
LIFT_START_LATENCY_S = 0.38    # measured (butterfli_hw kLiftStartLatencyS)
# Pole motion model — the drive constants the Web GUI reports (1250 rpm,
# RR 0.005 m/rev, 0-315 mm), same as dual_arm_common. Duplicated rather
# than imported: the emulator installs itself as the SDK *before*
# dual_arm_common imports it, so importing back would be circular.
LIFT_MM_S_PER_PCT = 1250.0 / 60.0 * 0.005 * 1000.0 / 100.0   # 1.0417 mm/s/%
LIFT_ACCEL_MM_S2 = 111.0
# Per-arm lift gearing (mirrors dual_arm_common: left 1:1 on V1.7.4,
# right 2/3 on V1.7.1; same env overrides RM_LEFT/RIGHT_LIFT_GEAR).
def _gear(side, default):
    # hw_max mirrors the controller-configured travel: both poles are set
    # to 315 mm with 1:1 scaling (2026-08-07).
    v = os.environ.get(f"RM_{side.upper()}_LIFT_GEAR", default)
    v = v.strip().lower().replace(":", "to")
    return {"1to1": (1.0, 315), "2to3": (1.5, 200)}.get(v, (1.5, 200))

MIN_MOTION_S = 0.05
DEV_JOINT, DEV_HAND, DEV_LIFT = 0, 2, 3
HAND_STROKE_K = 373.0          # measured stroke law (bench §3.7)
HAND_CMD_LATENCY_S = 0.115     # measured hand command latency

_time_scale = float(os.environ.get("RM_EMU_TIME_SCALE", "1.0"))

# Emulated arm addresses follow the same env vars as the tests, so an
# address change stays consistent across emulated and real runs.
EMU_LEFT_IP = os.environ.get("RM_LEFT_IP", "192.168.1.10")
EMU_RIGHT_IP = os.environ.get("RM_RIGHT_IP", "192.168.1.103")

# IPs that count as "this host" for the UDP push. A push configured to any
# other target is accepted (ret 0, like the real controller, which cannot
# know your host's address) but frames are NEVER delivered — reproducing
# the real-hardware wrong-IP trap. Override with emu_set_host_ips() or the
# RM_HOST_IP env var.
EMU_HOST_IPS = {os.environ.get("RM_HOST_IP", "192.168.1.239")}


def _emu_local_ips():
    """Addresses this machine would actually use to reach the arms.

    The tests no longer hard-code a push target: dual_arm_common.host_ip_for
    asks the kernel which local address routes to the arm. The emulator must
    accept the same answers, or the trap below would fire on a CORRECT
    address whenever the suite runs off the lab laptop. Anything that is
    genuinely not this host is still rejected — which is the point.
    """
    import socket
    found = set()
    for peer in (EMU_LEFT_IP, EMU_RIGHT_IP, "192.168.1.1"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((peer, 1))
                ip = s.getsockname()[0]
            if ip and not ip.startswith("127."):
                found.add(ip)
        except Exception:
            pass
    return found


EMU_HOST_IPS |= _emu_local_ips()


def emu_set_host_ips(*ips):
    global EMU_HOST_IPS
    EMU_HOST_IPS = set(ips)


def set_time_scale(scale: float):
    global _time_scale
    _time_scale = max(float(scale), 1e-6)


def _scaled(seconds: float) -> float:
    return seconds / _time_scale


def _lift_travel_s(dist_mm: float, pct: int) -> float:
    """Trapezoidal (or triangular, for short strokes) pole travel time.

    The profile is ACCELERATION-LIMITED, so a short stroke never reaches
    the commanded speed — modelling that is what makes emulated pole
    durations line up with the hardware (checked: 140 mm @ 50% -> 3.11 s
    of motion, measured 3.47 s incl. ~0.33 s start latency).
    """
    import math
    dist = max(float(dist_mm), 0.0)
    v = max(1, min(100, int(pct))) * LIFT_MM_S_PER_PCT
    a = LIFT_ACCEL_MM_S2
    if dist <= v * v / a:
        return 2.0 * math.sqrt(dist / a)
    return v / a + dist / v


# ─── Process-global SDK state (mirrors the real SDK's globals) ──────────────
_event_cb = None
_state_cb = None
_thread_mode = None          # set by RoboticArm(mode); 0 = single-thread
_registry_lock = threading.Lock()
_by_ip = {}
_by_handle = {}
_handle_ids = itertools.count(1)
_powered_off = set()


def emu_power_off(ip: str):
    _powered_off.add(ip)


def emu_power_on(ip: str):
    _powered_off.discard(ip)


def emu_controller(ip: str):
    """Access an arm's controller for state inspection / fault injection."""
    if ip not in _default_specs():
        raise KeyError(f"no emulated arm at {ip}")
    return _get_or_create(ip)


# ── Measured chain-motion laws (MOTION_FINDINGS §9–§10, SIM+REAL) ──────────
# Every constant here is a MEASUREMENT, not a guess. Sources in brackets.

FK_VERIFY_TOL_M = 0.010    # movej_p accept gate; the arm accepts ~4.5 mm
                           # best-effort misses (C6 hardware, 2026-08-06)
BLEND_FLOOR_M = 0.026      # segments under ~26 mm never blend [§9.3f: 24.6 mm
                           # declined, 27.5+ mm blended on hardware]
CORNER_DWELL_S = 0.30      # full-stop dwell at an unblended corner [§9.3b:
                           # 170-290 ms at tips + re-accel latency; calibrated
                           # against the 001/002 chain durations]
CHAIN_SETTLE_S = 0.28      # end-of-chain settle [§10.4: 230-290 ms measured]
TANGENT_DEG = 15.0         # junction under this angle needs no blend at all
                           # [§10.1: tangent arc entry at ~cruise, exemption moot]
FREEZE_R_PCT = 25          # r >= this near short segments risks the planner
FREEZE_SEG_M = 0.090       # chain-refill freeze [§10.3: sites adjacent to
                           # 37–88 mm segments; deterministic, arm goes IDLE]
# Injected freeze dwell per signature-matched junction (RM_EMU_FREEZES=1).
# EMPIRICAL, TWO POINTS, MECHANISM UNKNOWN (roadmap gap 3): measured burden
# over the no-freeze prediction at cap 0.6 on top_left (18 matched sites) was
# 17.6 s at r=25 (0.98 s/site) and 5.4 s at r=50 (0.30 s/site). Consistency
# check: r=25 gives 0.92-0.99 s/site across caps 0.6/0.8/1.0. Larger blends
# evidently give the planner more slack; until the mechanism is characterized
# this is an interpolation between two measurements, not a model.
FREEZE_DWELL_R25_S = 0.98
FREEZE_DWELL_R50_S = 0.30


def _freeze_dwell(r_pct):
    if r_pct <= 25:
        return FREEZE_DWELL_R25_S
    if r_pct >= 50:
        return FREEZE_DWELL_R50_S
    f = (r_pct - 25) / 25.0
    return FREEZE_DWELL_R25_S + f * (FREEZE_DWELL_R50_S - FREEZE_DWELL_R25_S)

# Corner speed retention (fraction of cruise) by (turn angle deg, r%):
# measured minima / commanded cruise [§9.3b, §9.3d, §10 ladder runs].
_RETENTION = {              # rows: angle; cols: r 10 / 25 / 50
    45.0:  (0.30, 0.45, 0.70),
    90.0:  (0.24, 0.36, 0.53),
    135.0: (0.12, 0.20, 0.35),
    175.0: (0.06, 0.10, 0.21),
}


def _corner_model(theta_deg, r_pct, min_l_m):
    """(cut_m, v_ratio, stop) for a chain junction — the measured law.

    cut_m    total path removed (entry+exit) around the vertex
    v_ratio  corner speed as a fraction of cruise (0 when stopping)
    stop     True → full stop + CORNER_DWELL_S
    """
    if theta_deg < TANGENT_DEG:
        return 0.0, 1.0, False           # effectively straight-through
    if theta_deg >= 179.5:
        return 0.0, 0.0, True            # exact retrace: stop, zero cut [§9.2]
    if r_pct <= 0 or min_l_m < BLEND_FLOOR_M:
        return 0.0, 0.0, True            # no blend: dead stop [§2 / §9.3f]
    r = r_pct / 100.0
    # cut coefficient: ~0 below 60° [§9.2], 0.70 at 90°-class [§9.3d exact].
    # Sharp corners/reversals: contract A.2 (re-measured 2026-08-19 over 1454
    # corners) — c is NOT flat: c(10)=1.70, c(25)=1.57, c(50)=1.33. The old
    # flat 1.4 UNDERSTATED the cut at the mandated r=10 (was here until
    # 2026-08-19; clamped to [10,50], the measured r domain).
    if theta_deg < 60.0:
        c = 0.0
    elif theta_deg < 120.0:
        c = 0.70
    else:
        rp = min(max(float(r_pct), 10.0), 50.0)
        if rp <= 25.0:
            c = 1.70 + (rp - 10.0) * (1.57 - 1.70) / 15.0
        else:
            c = 1.57 + (rp - 25.0) * (1.33 - 1.57) / 25.0
    cut = c * r * min_l_m
    # bilinear-ish interpolation of the retention table
    angs = sorted(_RETENTION)
    a = min(max(theta_deg, angs[0]), angs[-1])
    hi = next(x for x in angs if x >= a)
    lo = max([x for x in angs if x <= a])
    f = 0.0 if hi == lo else (a - lo) / (hi - lo)
    rs = (10.0, 25.0, 50.0)
    rr = min(max(float(r_pct), rs[0]), rs[-1])
    j = 1 if rr <= 25.0 else 2
    g = (rr - rs[j - 1]) / (rs[j] - rs[j - 1])
    ret_lo = _RETENTION[lo][j - 1] + g * (_RETENTION[lo][j] - _RETENTION[lo][j - 1])
    ret_hi = _RETENTION[hi][j - 1] + g * (_RETENTION[hi][j] - _RETENTION[hi][j - 1])
    return cut, ret_lo + f * (ret_hi - ret_lo), False


def _arc_points(pa, pv, pb, step_m):
    """Sample the circular arc through three 3-D points, pa -> pb via pv.

    Returns positions EXCLUDING pa, INCLUDING pb. Falls back to the chord
    if the points are (near-)collinear. Verified against the REAL 006 runs:
    hardware traced this construction to 0.4 mm median [§10.4].
    """
    import numpy as _np
    a, v, b = (_np.asarray(x, float) for x in (pa, pv, pb))
    e1 = v - a
    e2 = b - a
    n = _np.cross(e1, e2)
    if _np.linalg.norm(n) < 1e-9 * max(_np.linalg.norm(e1), 1e-9):
        d = _np.linalg.norm(b - a)
        k = max(1, int(math.ceil(d / step_m)))
        return [list(a + (b - a) * (i / k)) for i in range(1, k + 1)]
    n = n / _np.linalg.norm(n)
    u = e1 / _np.linalg.norm(e1)
    w = _np.cross(n, u)
    to2 = lambda p: _np.array([_np.dot(p - a, u), _np.dot(p - a, w)])
    A, V, B = to2(a), to2(v), to2(b)
    d = 2 * (A[0] * (V[1] - B[1]) + V[0] * (B[1] - A[1]) + B[0] * (A[1] - V[1]))
    ux = ((A @ A) * (V[1] - B[1]) + (V @ V) * (B[1] - A[1])
          + (B @ B) * (A[1] - V[1])) / d
    uy = ((A @ A) * (B[0] - V[0]) + (V @ V) * (A[0] - B[0])
          + (B @ B) * (V[0] - A[0])) / d
    C = _np.array([ux, uy])
    R = float(_np.linalg.norm(A - C))
    a0 = math.atan2(*(A - C)[::-1])
    a1 = math.atan2(*(B - C)[::-1])
    avv = math.atan2(*(V - C)[::-1])
    sweep = (a1 - a0) % (2 * math.pi)
    if not ((avv - a0) % (2 * math.pi)) <= sweep:
        sweep = sweep - 2 * math.pi
    k = max(2, int(math.ceil(abs(sweep) * R / step_m)))
    out = []
    for i in range(1, k + 1):
        th = a0 + sweep * (i / k)
        p2 = C + R * _np.array([math.cos(th), math.sin(th)])
        out.append(list(a + u * p2[0] + w * p2[1]))
    return out


class _Motion:
    """One in-flight motion: linear interpolation + completion timer."""

    def __init__(self, start, target, duration_s, on_done):
        self.start = list(start) if isinstance(start, (list, tuple)) else start
        self.target = list(target) if isinstance(target, (list, tuple)) else target
        self.t0 = time.perf_counter()
        self.duration = max(duration_s, 1e-4)
        self.done = threading.Event()
        self.will_fail = False
        self.direction = 0
        self.timer = threading.Timer(self.duration, on_done, args=(self,))
        self.timer.daemon = True
        self.timer.start()

    def progress(self) -> float:
        return min(1.0, (time.perf_counter() - self.t0) / self.duration)

    def current(self):
        a = self.progress()
        if isinstance(self.start, list):
            return [s + (t - s) * a for s, t in zip(self.start, self.target)]
        return self.start + (self.target - self.start) * a

    def cancel(self):
        self.timer.cancel()
        self.done.set()


class _PathMotion(_Motion):
    """A motion through MANY joint waypoints, not just start -> target.

    `_Motion` interpolates one straight line in joint space, which is what
    `movej` does. A Cartesian `movel` chain is a joint-space CURVE, and
    the whole point of emulating it is that the curve is where the joint
    rates live — a straight line between its endpoints would report a
    fraction of the true rate and hide exactly what we are looking for.

    Walks the waypoint list at uniform time, so d(position)/dt over the
    recorded stream reproduces the same profile a SIM run gives.
    """

    def __init__(self, waypoints, duration_s, on_done):
        self.waypoints = [list(w) for w in waypoints]
        super().__init__(self.waypoints[0], self.waypoints[-1],
                         duration_s, on_done)


class _TimedPathMotion(_PathMotion):
    """A path motion with a MEASURED-LAW schedule, not uniform time.

    `times` gives each waypoint its own timestamp (corner dips, blend
    shortcuts, stop dwells, freezes — the whole §9/§10 timing model), so
    d(position)/dt over the emulated stream reproduces the speed profile
    the recorder measures on SIM and REAL. `gaps` are [t0, t1] windows in
    which the controller reports NOT-moving: stop dwells, and the brief
    MOVE_L exits at blends that analyse_run's [BLEND ACTIVE] detector
    keys on.
    """

    def __init__(self, waypoints, times, gaps, on_done):
        self.times = [float(t) for t in times]
        self.gaps = [(float(a), float(b)) for a, b in (gaps or [])]
        super().__init__(waypoints, self.times[-1] if self.times else 1e-3,
                         on_done)

    def current(self):
        tr = min(time.perf_counter() - self.t0,
                 self.times[-1]) if self.times else 0.0
        import bisect
        i = bisect.bisect_right(self.times, tr)
        if i <= 0:
            return list(self.waypoints[0])
        if i >= len(self.waypoints):
            return list(self.waypoints[-1])
        t0, t1 = self.times[i - 1], self.times[i]
        f = 0.0 if t1 <= t0 else (tr - t0) / (t1 - t0)
        a, b = self.waypoints[i - 1], self.waypoints[i]
        return [x + (y - x) * f for x, y in zip(a, b)]

    def status_moving(self) -> bool:
        tr = time.perf_counter() - self.t0
        for a, b in self.gaps:
            if a <= tr <= b:
                return False
        return not self.done.is_set()

    def current(self):
        a = self.progress()
        n = len(self.waypoints) - 1
        if n <= 0:
            return list(self.waypoints[0])
        x = a * n
        i = min(n - 1, int(x))
        f = x - i
        lo, hi = self.waypoints[i], self.waypoints[i + 1]
        return [lo[k] + (hi[k] - lo[k]) * f for k in range(len(lo))]


class EmuController:
    """State + motion engine for one emulated RM75-6FB arm with pole lift."""

    def __init__(self, ip: str, joints_deg, lift_hw_mm: float):
        self.ip = ip
        self.handle_id = None
        self.joints_deg = list(joints_deg)
        self.lift_hw = float(lift_hw_mm)
        self.hand_hw = [993, 993, 993, 993, 981, 992]   # SRDF 'release'
        self.modbus_mode = False        # end-port RS485 in modbus mode
        self.hand_speed_set = 500
        self.hand_force_set = 500
        self._hand_motion = None
        self.run_mode = 1                     # 1 = REAL, 0 = SIMULATION
        # (was 2/3 from a V1.7.1-era read; the V1.7.4 capability probe
        #  reports 4 on BOTH arms — see self.caps below)
        # (still unaligned on the fleet — see PHASE_PLAN R4)
        side = "left" if ip == EMU_LEFT_IP else "right"
        # Per-side joint POSITION limits: J4 is +-169 left, +-140 right (H37).
        self.joint_limit_deg = list(JOINT_LIMIT_DEG[side])
        # Both arms run true-mm 1:1 since the right's 2026-08-07 upgrade;
        # RM_*_LIFT_GEAR=2to3 still models a pre-upgrade controller.
        self.lift_hw_to_phys, self.lift_hw_max = _gear(side, "1to1")
        self._lock = threading.RLock()
        self._arm_motion = None
        self._lift_motion = None
        self._lift_queue = []
        self._push_thread = None
        self._push_stop = threading.Event()
        # fault injection
        self.reject_next_dispatch = False
        self.fail_next_motion = False
        self.drop_next_event = False
        self.command_latency_s = CMD_LATENCY_S
        # Latched lift-rejection state (observed on REAL hardware
        # 2026-08-06 20:38: both controllers rejected every proven-good
        # lift command with set_state false).  RM_EMU_LIFT_LOCKED=
        # left[,right] starts the side latched; rm_clear_system_err
        # unlatches (the modelled recovery path).
        locked = {s.strip() for s in
                  os.environ.get("RM_EMU_LIFT_LOCKED", "").lower().split(",")
                  if s.strip()}
        self.lift_locked = side in locked
        self.sys_err_code = 4103 if self.lift_locked else 0
        # Latched JOINT errors from an abrupt trajectory abort (see
        # set_lift_height). While latched, every motion command is
        # rejected with ret 1 until rm_clear_system_err — the recovery
        # step the hardware demands after a sudden stop.
        self.joint_err_flags = [0] * 7
        self.motion_locked = False
        # Tool frames as the controllers hold them: Arm_Tip always exists,
        # the left arm additionally ships with the hand-taught 'Hand'.
        # A tool frame carries a PAYLOAD and its CENTROID (x/y/z, mm from
        # the flange). The centroid used to be hard-coded 0.0 here and
        # never stored, so the emulator could not reproduce the 2026-08-10
        # defect at all: six frames written with a centroid of 128 METRES,
        # which made the controller's torque model call every movel an
        # "arm collision" (0x100D). F25 again — an emulator that cannot
        # hold the state cannot catch the bug.
        self.tool_frames = {"Arm_Tip": {"pose": [0.0] * 6, "payload": 0.0,
                                        "com": [0.0, 0.0, 0.0]}}
        if side == "left":
            self.tool_frames["Hand"] = {
                "pose": [-0.035, 0.01, 0.259999, 0.0, 0.0, 0.0],
                "payload": 0.706,
                # METRES, because that is what the hardware getter returns
                # (arm_census 2026-08-11: `Hand` reads -0.012/0.044/0.128
                # where the GUI shows -12/44/128 mm). Holding it in mm here
                # made the emulator disagree with the arm about units —
                # the same class of fidelity gap as F25.
                "com": [-0.012, 0.044, 0.128]}
        # THE GLOVE FRAMES, because the hardware has them and every cleaning
        # task selects one. Without them `rm_change_tool_frame("L_glove_2")`
        # returns 1 here and 0 on the arm, so the emulator refuses a run the
        # hardware would accept — and any test that guards on the tool frame
        # becomes unrunnable offline. Same principle as the comment above:
        # an emulator that cannot hold the state cannot exercise the code
        # that depends on it. Offsets come from `orientation_cost.TOOL_OFFSETS`,
        # which is the frame table verified against recorded poses to 29 um.
        try:
            from orientation_cost import TOOL_OFFSETS
            pre = "L_" if side == "left" else "R_"
            for nm, off in TOOL_OFFSETS.items():
                if not nm.startswith(pre):
                    continue
                self.tool_frames[nm] = {
                    "pose": [off[0], off[1], off[2], 0.0, 0.0, 0.0],
                    "payload": 0.706 if side == "left" else 0.711,
                    "com": [0.0, 0.0, 0.0]}
        except Exception:                                   # noqa: BLE001
            pass                    # never let frame seeding break the emulator
        self.active_tool = "Hand" if side == "left" else "Arm_Tip"
        # F10: read from both arms 2026-08-07.
        self.limits = {"line_speed": 0.250, "line_acc": 1.600,
                       "angular_speed": 0.600, "angular_acc": 4.000}
        # MEASURED off the left arm 2026-08-11, not assumed. The old value
        # here was [180]*6 + [225] — wrong for J3..J6, which are 225 not
        # 180, so every offline utilisation figure computed against the
        # emulator overstated the load on four joints by 25 %.
        # rm_get_joint_max_speed AND rm_get_joint_drive_max_speed both
        # return this, so plan and drive limits are identical on this arm.
        self.joint_max_speed = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0,
                                225.0]                       # deg/s
        # 600 deg/s^2 = 100 RPM/s, against a documented ceiling of
        # 500 RPM/s = 3000 deg/s^2 — the arm ships at 20 % of what it is
        # allowed, so joint acceleration IS a lever (5x). An earlier note
        # claiming it was already at its ceiling read the default from the
        # docs instead of the arm.
        self.joint_max_acc = [600.0] * 7                     # deg/s^2
        # As read from both arms 2026-08-08: every assist OFF,
        # dynamics collision stage 4.
        self.caps = {"avoid_singularity": 0, "endeff_collision": False,
                     "collision_stage": 4, "static_collision": 0,
                     "collision_remove": False}
        # per-joint drive enable; inject faults with
        # emu_controller(ip).joint_en[5] = 0  (J6 dead)
        self.joint_en = [1] * 7
        # per-joint bus voltage / current for the realtime push. Fault
        # injection: emu_controller(ip).joint_voltage[5] = 42.0 (J6 sag).
        self.joint_voltage = [48.0] * 7
        self.joint_current_idle_ma = 350.0
        # Web GUI screenshots (2026-05-07) show the arms shipped with this
        # OFF, so default to off and let the tests turn it on.
        self.self_collision = os.environ.get("RM_EMU_SELF_COLLISION") == "1"

    # ── event delivery ──
    def _emit(self, device: int, ok: bool = True):
        with self._lock:
            if self.drop_next_event:
                self.drop_next_event = False
                return
            hid = self.handle_id
        cb = _event_cb
        if _thread_mode == 0:
            return               # single-thread mode: no event delivery
        if cb is not None and hid is not None:
            cb(SimpleNamespace(handle_id=hid, event_type=1,
                               trajectory_state=ok, device=device,
                               trajectory_connect=0, program_id=0))

    def _consume_fail_flag(self) -> bool:
        # caller holds self._lock
        if self.fail_next_motion:
            self.fail_next_motion = False
            return True
        return False

    def _active_tool_pose(self):
        f = self.tool_frames.get(self.active_tool)
        return f.get("pose") if f else None

    def movel_chain(self, poses, v: int, block: int) -> int:
        """Execute a queued movel chain as ONE trajectory, ONE event.

        CARTESIAN GEOMETRY IS MODELLED (added 2026-08-12). Until then this
        method slept `0.25 * n_segments * 20/v` and LEFT THE JOINTS ALONE,
        so an emulated cleaning stroke exercised the dispatch and nothing
        about the motion. That made the emulator blind to the one thing
        that has actually stopped the arm: joint-speed saturation. It is
        also the F25/H3 failure mode in the other direction — an emulator
        that models less than the controller certifies runs the hardware
        then refuses.

        What it now does, using RealMan's OWN offline solver (the same
        `_ALGO` the pose getters use, so FK/IK agree with the controller's
        scheme):

          1. seeds from the current joints and the current tool pose
          2. walks the queued poses as a Cartesian polyline, sampling
             every CART_STEP_M
          3. solves seeded IK at each sample; an IK failure or a solution
             past a joint POSITION limit returns ret 1, which is what the
             controller answers
          4. times each segment on a trapezoidal profile from the arm's
             OWN `line_speed` / `line_acc` limits, with `line_speed`
             scaled by v% (vendor-confirmed: for `movel` the baseline is
             the TCP speed constraint — see `_plan_cartesian`) — so the
             stop-at-every-waypoint behaviour that dominates the measured
             stroke (H43: 42 % of time in dips) appears here too
          5. drives the joints through the solutions

        DELIBERATELY NOT MODELLED: joint speed limits are NOT enforced.
        Hardware demonstrably exceeds them without clamp or fault (H41),
        and the emulator must not be stricter than the machine. The
        emulator therefore COMPLETES a stroke that saturates a joint —
        exactly as SIM does (`20260811T184017` completed with J4 at 100 %
        while the REAL run at the same settings stopped silently). The
        saturation is there to be measured, not to be caught here.
        """
        with self._lock:
            if self.motion_locked:
                return 1
            if self.reject_next_dispatch:
                self.reject_next_dispatch = False
                return 1
        time.sleep(_scaled(self.command_latency_s))
        with self._lock:
            self._truncate_lift_locked()
            fail = self._consume_fail_flag()
            seed = self.current_joints_locked()
            tool = self._active_tool_pose()
            ls = float(self.limits.get("line_speed", 0.25))
            la = float(self.limits.get("line_acc", 1.6))

        # Chain targets are poses OF THE ACTIVE TOOL — assert the frame on
        # the shared solver before planning (see _set_algo_toolframe).
        _set_algo_toolframe(tool)
        # Normalize: per-move dicts from the queue (v/r/via honored, §9.3d),
        # or bare pose lists from direct callers (legacy: uniform v, r=0).
        moves = [m if isinstance(m, dict)
                 else {"pose": m, "v": v, "r": 0, "via": None}
                 for m in poses]
        ang = float(self.limits.get("angular_speed", 0.6))
        ang_a = float(self.limits.get("angular_acc", 4.0))
        path, tpath, gaps, _w = self._plan_chain(seed, tool, moves,
                                                 ls, la, ang, ang_a)
        dur = tpath[-1] if tpath else 0.0
        if path is None:
            return 1                      # IK failure / limit — controller ret 1

        with self._lock:
            if self._arm_motion and not self._arm_motion.done.is_set():
                self._arm_motion.cancel()
            motion = _TimedPathMotion(path,
                                      [_scaled(x) for x in tpath],
                                      [(_scaled(a), _scaled(b))
                                       for a, b in gaps],
                                      self._arm_done)
            motion.will_fail = fail
            self._arm_motion = motion
        if block:
            motion.done.wait(_scaled(dur) + 5.0)
        return 0

    # Cartesian sampling step. 2 mm over a ~5.8 m stroke is ~2900 IK
    # solves — about a second offline, and fine enough that the joint
    # trajectory resolves the corners that produce the dips.
    CART_STEP_M = 0.002
    # Mounting angle about +Y, degrees — the same value `rm_get_install_pose`
    # reports. `rm_movel` poses are in the controller frame; the Algo library
    # solves in the URDF frame; this is what separates them.
    mount_ry_deg = 90.0
    # Largest plausible joint move for one CART_STEP_M sample. The saved
    # plans step at most ~8 deg between adjacent waypoints at full
    # resolution, so 10 deg over 2 mm is generous; anything past it is a
    # solver branch flip.
    MAX_JOINT_STEP_DEG = 10.0

    def _plan_cartesian(self, seed, tool, poses, v, line_speed, line_acc):
        """(joint waypoints, duration) for a Cartesian polyline, or (None, 0).

        The profile is per SEGMENT, not per chain: blending does not take
        effect on this hardware (H35 — ~1 deceleration per waypoint, full
        stops at 30-35 degree corners), so each segment accelerates from
        rest and returns to rest.
        """
        if _ALGO is None:
            return None, 0.0
        # `_fk_pose` answers in the ALGO frame; `poses` arrive in the
        # CONTROLLER frame. Convert the latter, and carry orientation as a
        # quaternion so it can be SLERPed rather than Euler-interpolated.
        s_pos = list(_fk_pose(seed, tool))
        pts = [(s_pos[:3], _R_to_quat(_euler_to_R(*s_pos[3:6])))]
        for p in poses:
            pos, R = _ctrl_to_algo(p, self.mount_ry_deg)
            pts.append((pos, _R_to_quat(R)))
        # WHAT `v` SCALES — RealMan, 2026-08-12, consistent across both of
        # their replies (SPEED_INVESTIGATION.md §1):
        #
        #     movej:  v_interface x realtime_adjust x JOINT speed limit
        #     movel:  v_interface x realtime_adjust x TCP speed constraint
        #
        # The baseline DIFFERS BY COMMAND. For movel it is the TCP speed
        # constraint, i.e. `line_speed` — which is what this code models.
        #
        # `realtime_adjust` is the pendant's global override. It is NOT
        # modelled here: it is unreadable from the API (H58) and was measured
        # at 100 % for every run we have (H59).
        #
        # ACCELERATION IS LEFT UNSCALED: the vendor describes v as a *speed*
        # ratio coefficient and says nothing about acceleration, and line_acc
        # is a separately-named constraint. This is a judgement call, and it
        # is unobservable today because every dispatch in this project is
        # v=100 — where scaled and unscaled acceleration coincide.
        vpct = max(1, min(100, int(v))) / 100.0
        vmax = max(1e-3, line_speed * vpct)
        amax = max(1e-3, line_acc)

        out, total = [list(seed)], 0.0
        q = list(seed)
        unsolved = 0
        total_samples = 0
        for (pa, qa), (pb, qb) in zip(pts, pts[1:]):
            d = math.dist(pa, pb)
            n = max(1, int(math.ceil(d / self.CART_STEP_M)))
            seg_seed = list(q)
            for k in range(1, n + 1):
                f = k / n
                pos = [pa[i] + (pb[i] - pa[i]) * f for i in range(3)]
                rx, ry, rz = _R_to_euler(_quat_to_R(_slerp(qa, qb, f)))
                target = pos + [rx, ry, rz]
                total_samples += 1
                ret, q_new = _ik_seeded(q, target)
                if ret != 0 or q_new is None:
                    # Retry from the segment's own start before giving up:
                    # seeded IK can walk itself into a branch it cannot
                    # continue from.
                    ret, q_new = _ik_seeded(seg_seed, target)
                if ret != 0 or q_new is None:
                    # MEASURED 2026-08-12: all 28 commanded endpoints of
                    # `toplid_left` solve, yet 4 of 27 segments contain an
                    # INTERPOLATED point the offline solver cannot reach —
                    # while the controller executes those segments. So an
                    # unsolvable intermediate is a limitation of the
                    # offline library, not a statement about the arm.
                    # Carry the last good solution and continue: aborting
                    # here would make the emulator refuse strokes the
                    # hardware runs, which is the F25 error inverted.
                    unsolved += 1
                    continue
                if any(abs(x) > lim + 1e-6
                       for x, lim in zip(q_new, self.joint_limit_deg)):
                    return None, 0.0
                # BRANCH-FLIP GUARD. A CART_STEP_M move cannot need tens of
                # degrees on a joint; when it appears, the solver has jumped
                # to another IK branch. Accepting it would inject a joint
                # rate that never happens on the arm — and joint rate is the
                # whole reason this trajectory is being built.
                if max(abs(x - y) for x, y in zip(q_new, q)) > \
                        self.MAX_JOINT_STEP_DEG:
                    unsolved += 1
                    continue
                q = q_new
                out.append(list(q))
            # trapezoid if the segment is long enough to reach vmax,
            # triangle otherwise
            if d >= vmax * vmax / amax:
                seg_t = d / vmax + vmax / amax
            else:
                seg_t = 2.0 * math.sqrt(max(d, 0.0) / amax)
            total += seg_t
        if unsolved:
            self.last_unsolved = (unsolved, total_samples)
            if unsolved > 0.05 * max(1, total_samples):
                print("[emu] WARNING: %d of %d Cartesian samples had no IK "
                      "solution (%.1f %%) — the emulated joint trajectory is "
                      "interpolated across them and its rates are an "
                      "UNDER-estimate"
                      % (unsolved, total_samples,
                         100.0 * unsolved / total_samples))
        else:
            self.last_unsolved = (0, total_samples)
        return out, max(total, MIN_MOTION_S)

    def _plan_chain(self, seed, tool, moves, line_speed, line_acc, ang_cap,
                    ang_acc=4.0):
        """Timeline for a per-move chain under the MEASURED laws (§9–§10).

        moves: [{'pose': ctrl-frame 6-list, 'v': 1..100, 'r': 0..100,
                 'via': ctrl-frame 6-list or None}]   (via → movec arc)

        Returns (joint_waypoints, times_s, gaps, warns) or (None, ...).
        Models: per-move v and r [§9.3d]; angular throttling H67; blend
        cuts c(θ)·r·minL with corner-speed retention [§9.2/9.3b]; the
        blend floor [§9.3f]; first-corner exemption unless tangent
        [§9.2, §10.1]; movec arcs [§10.1]; stop dwells; freeze-signature
        warnings, with optional injected dwells via RM_EMU_FREEZES=1
        [§10.3 — sites are deterministic on hardware].
        """
        if _ALGO is None:
            return None, None, None, None
        step = self.CART_STEP_M
        s_pos = list(_fk_pose(seed, tool))
        prev_pos = list(s_pos[:3])
        prev_q = _R_to_quat(_euler_to_R(*s_pos[3:6]))
        segs = []
        for mv in moves:
            pos_b, Rb = _ctrl_to_algo(mv["pose"], self.mount_ry_deg)
            qb = _R_to_quat(Rb)
            if mv.get("via") is not None:
                pos_v, _Rv = _ctrl_to_algo(mv["via"], self.mount_ry_deg)
                pts = _arc_points(prev_pos, pos_v, pos_b, step)
            else:
                d = math.dist(prev_pos, pos_b)
                k = max(1, int(math.ceil(d / step)))
                pts = [[prev_pos[i] + (pos_b[i] - prev_pos[i]) * (j / k)
                        for i in range(3)] for j in range(1, k + 1)]
            # cumulative arclength + slerped orientation per sample
            dists, acc = [], 0.0
            last = prev_pos
            for p in pts:
                acc += math.dist(last, p)
                dists.append(acc)
                last = p
            length = max(acc, 1e-9)
            samples = [(p, _slerp(prev_q, qb, dl / length))
                       for p, dl in zip(pts, dists)]
            rot = 2.0 * math.acos(min(1.0, abs(
                sum(a * b for a, b in zip(prev_q, qb)))))
            v_cmd = line_speed * max(1, min(100, int(mv.get("v", 100)))) / 100.0
            # ORIENTATION TIME under BOTH angular limits (2026-08-16). The
            # velocity-only form (v_eff = cap*L/rot, orientation_cost's
            # screen) over-credits a raised cap: it ignores angular
            # ACCELERATION, whose ramp cost rot/cap + cap/alpha GROWS with
            # the cap. That is why the measured cap ladder gained only ~5 %
            # where the velocity-only model predicted ~27 % (§10.2).
            if rot < 1e-9:
                t_rot = 0.0
            elif rot <= ang_cap * ang_cap / ang_acc:
                t_rot = 2.0 * math.sqrt(rot / ang_acc)        # triangular
            else:
                t_rot = rot / ang_cap + ang_cap / ang_acc     # trapezoid
            v_eff = v_cmd
            if mv.get("via") is not None and len(pts) >= 3:
                # CENTRIPETAL CAP on arcs: the controller runs a circle no
                # faster than ~0.70*sqrt(a*R) — REAL 006 measured
                # 0.15-0.18 m/s on R=22.5 mm at line_acc 1.6.
                import numpy as _np
                mid = pts[len(pts) // 2]
                chord = _np.array(pos_b) - _np.array(prev_pos)
                mvec = _np.array(mid) - (_np.array(prev_pos) + chord / 2)
                h = float(_np.linalg.norm(mvec))
                c2 = float(_np.linalg.norm(chord)) / 2
                if h > 1e-6:
                    R_arc = (h * h + c2 * c2) / (2 * h)
                    v_eff = min(v_eff, 0.70 * math.sqrt(line_acc * R_arc))
            segs.append({"samples": samples, "len": length, "rot": rot,
                         "v": max(v_eff, 1e-3), "r": int(mv.get("r", 0)),
                         "t_rot": t_rot, "arc": mv.get("via") is not None})
            prev_pos, prev_q = list(pos_b), qb
        # junctions: cut geometry + entry/exit speeds
        warns = []
        inject = os.environ.get("RM_EMU_FREEZES", "") == "1"
        n = len(segs)
        vin = [0.0] * n
        vout = [0.0] * n
        dwell_after = [0.0] * n
        blend_at = [False] * n
        for i in range(n - 1):
            a, b = segs[i], segs[i + 1]
            ta = [x - y for x, y in zip(a["samples"][-1][0],
                                        (a["samples"][-2][0]
                                         if len(a["samples"]) > 1 else prev_pos))]
            tb = [x - y for x, y in zip(b["samples"][0][0], a["samples"][-1][0])]
            if len(b["samples"]) > 1:
                tb = [x - y for x, y in zip(b["samples"][1][0],
                                            b["samples"][0][0])]
            na = math.sqrt(sum(x * x for x in ta)) or 1e-9
            nb = math.sqrt(sum(x * x for x in tb)) or 1e-9
            cosv = max(-1.0, min(1.0, sum(x * y for x, y in zip(ta, tb))
                                 / (na * nb)))
            theta = math.degrees(math.acos(cosv))
            min_l = min(a["len"], b["len"])
            cut, vr, stop = _corner_model(theta, a["r"], min_l)
            if i == 0 and theta >= TANGENT_DEG:
                cut, vr, stop = 0.0, 0.0, True   # first-corner exemption §9.2
            if (a["r"] >= FREEZE_R_PCT and min_l < FREEZE_SEG_M
                    and 20.0 < theta < 179.5):
                warns.append(i)
                if inject:
                    dwell_after[i] += _freeze_dwell(a["r"])
            if stop:
                dwell_after[i] += CORNER_DWELL_S
                vj = 0.0
            else:
                vj = vr * min(a["v"], b["v"])
                if cut > 1e-6:
                    # sub-sample trim: whole-sample removal alone leaves up
                    # to one CART_STEP per side uncut (measured as cuts at
                    # ~half the law on the emulated stream). The exit side
                    # measures from the VERTEX — its first sample already
                    # sits one step past it.
                    vertex_pos = list(a["samples"][-1][0])
                    half = cut / 2.0
                    while len(a["samples"]) > 2 and half > 0:
                        p1, p0 = a["samples"][-1][0], a["samples"][-2][0]
                        d = math.dist(p0, p1)
                        if d > half:
                            f_ = half / d
                            q1, q0 = a["samples"][-1], a["samples"][-2]
                            a["samples"][-1] = (
                                [x + (y - x) * (1 - f_)
                                 for x, y in zip(q0[0], q1[0])], q1[1])
                            a["len"] -= half
                            half = 0
                            break
                        a["samples"].pop()
                        a["len"] -= d
                        half -= d
                    half = cut / 2.0
                    prev = vertex_pos
                    while len(b["samples"]) > 2 and half > 0:
                        p0 = b["samples"][0][0]
                        d = math.dist(prev, p0)
                        if d > half:
                            f_ = half / d
                            q0 = b["samples"][0]
                            b["samples"][0] = (
                                [x + (y - x) * f_
                                 for x, y in zip(prev, q0[0])], q0[1])
                            b["len"] -= half
                            half = 0
                            break
                        prev = list(p0)
                        b["samples"].pop(0)
                        b["len"] -= d
                        half -= d
                    # BRIDGE: the controller's blend curve passes FARTHER
                    # from the vertex than a straight chord (measured vmiss
                    # 4.3 mm at r25 / 6.6 at r50 on 90 deg corners, §9.3d) —
                    # so the trace departs the legs gradually rather than at
                    # a hard trim point. Quadratic Bezier with its apex at
                    # 1.1x(cut/2). NOTE the realized stream still UNDER-cuts
                    # the controller by ~2 mm/side; the cut LAW in
                    # _corner_model is exact, the softening is in the bridge.
                    # Sweeping this factor 0.9-1.5 moved the result by less
                    # than analyse_coverage's own 2 mm resolution, so it is
                    # not tuned further.
                    A2 = a["samples"][-1]
                    B2 = b["samples"][0]
                    mid = [(x + y) / 2 for x, y in zip(A2[0], B2[0])]
                    dvec = [m_ - v_ for m_, v_ in zip(mid, vertex_pos)]
                    dn = math.sqrt(sum(x * x for x in dvec)) or 1e-9
                    miss = 1.1 * (cut / 2.0)
                    apex = [v_ + dv / dn * miss
                            for v_, dv in zip(vertex_pos, dvec)]
                    ctrlp = [2 * ap - m_ for ap, m_ in zip(apex, mid)]
                    blen = math.dist(A2[0], B2[0])
                    nb = max(2, int(math.ceil(blen / step)))
                    added = 0.0
                    lastp = A2[0]
                    for kk in range(1, nb):
                        tt_ = kk / nb
                        bp = [(1 - tt_) ** 2 * x0 + 2 * (1 - tt_) * tt_ * xc
                              + tt_ ** 2 * x1
                              for x0, xc, x1 in zip(A2[0], ctrlp, B2[0])]
                        a["samples"].append((bp, _slerp(A2[1], B2[1], tt_)))
                        added += math.dist(lastp, bp)
                        lastp = bp
                    a["len"] += added
                    blend_at[i] = True
            vout[i] = vj
            vin[i + 1] = vj
        if warns:
            print("[emu] WARNING: %d junction(s) match the FREEZE signature "
                  "(r>=%d near <%.0f mm segments, §10.3)%s"
                  % (len(warns), FREEZE_R_PCT, 1000 * FREEZE_SEG_M,
                     " — dwells injected (RM_EMU_FREEZES=1)" if inject
                     else " — hardware/SIM will freeze there; "
                          "set RM_EMU_FREEZES=1 to model it"))
        # timing: v(s) = min(veff, sqrt(vin²+2as), sqrt(vout²+2a(S−s)))
        amax = max(line_acc, 1e-3)
        all_samples, times, gaps = [], [], []
        t = 0.0
        last_pos = list(s_pos[:3])
        for i, sg in enumerate(segs):
            S = max(sg["len"], 1e-9)
            s_acc = 0.0
            seg_dt, seg_pts = [], []
            for (p, qq) in sg["samples"]:
                d = math.dist(last_pos, p)
                s_acc = min(s_acc + d, S)
                v_here = min(sg["v"],
                             math.sqrt(vin[i] ** 2 + 2 * amax * s_acc),
                             math.sqrt(max(vout[i], 0.0) ** 2
                                       + 2 * amax * max(S - s_acc, 0.0)))
                # analytic floor: an interval that starts/ends at rest still
                # takes only sqrt(2d/a) — dividing by a near-zero endpoint
                # speed over-counted stop-adjacent samples by ~25 %
                seg_dt.append(d / max(v_here,
                                      math.sqrt(0.5 * amax * max(d, 1e-9))))
                seg_pts.append((p, qq))
                last_pos = p
            # H67 IS A TIME-SCALING, NOT A SPEED CAP — the only form that
            # does not double-count ramps: integrate the segment on its
            # LINEAR limits, then stretch it uniformly if the orientation
            # cannot be delivered within that time. Folding t_rot into a
            # speed cap instead made cap-0.6 runs +16 % long (2026-08-16).
            t_lin = sum(seg_dt)
            # ANGULAR RAMP IS PAID ONLY WHERE THE CHAIN STOPS. Orientation
            # carries across a blended junction exactly as position does, so
            # charging cap/alpha on every segment made cap-0.6 runs +18 %
            # long. Half a ramp per stopped end.
            # H67, VELOCITY FORM — deliberately identical to
            # orientation_cost.segment_report so the screen and the emulator
            # cannot disagree. Angular-ACCELERATION variants were tried
            # (2026-08-16) and rejected: they fix the cap-ladder trend but
            # cost 12 points of accuracy at the operating cap. NOTE the
            # measured cap ladder is nearly FLAT (40.5/38.7/38.5 s) while
            # every angular-only model is steep — the real limiter at raised
            # caps is joint speed, which this emulator does not model. DO NOT
            # use emulated durations to project angular-cap benefits.
            rot_i = sg.get("rot", 0.0)
            t_rot = (rot_i / ang_cap) if rot_i > 1e-9 else 0.0
            if t_rot > t_lin > 0:
                seg_dt = [x * (t_rot / t_lin) for x in seg_dt]
            for dt_, (p, qq) in zip(seg_dt, seg_pts):
                t += dt_
                all_samples.append((p, qq))
                times.append(t)
            if blend_at[i]:
                gaps.append((max(0.0, t - 0.06), t + 0.06))
            if dwell_after[i] > 0:
                gaps.append((t, t + dwell_after[i]))
                t += dwell_after[i]
                all_samples.append(all_samples[-1])
                times.append(t)
        # end-of-chain settle (measured 230-290 ms, §10.4)
        gaps.append((t, t + CHAIN_SETTLE_S))
        t += CHAIN_SETTLE_S
        all_samples.append(all_samples[-1])
        times.append(t)
        # IK walk (same guards as _plan_cartesian; unsolved carries last q,
        # with the per-stretch re-seed retry the old planner had — without
        # it one branch-flip rejection cascades: every later target is
        # >MAX_JOINT_STEP from the carried q and the walk freezes)
        out, tout = [list(seed)], [0.0]
        q = list(seed)
        stretch_seed = list(seed)
        unsolved = 0
        for (p, qq), tt in zip(all_samples, times):
            rx, ry, rz = _R_to_euler(_quat_to_R(qq))
            target = list(p) + [rx, ry, rz]
            ret, q_new = _ik_seeded(q, target)
            ok = (ret == 0 and q_new is not None and
                  max(abs(x - y) for x, y in zip(q_new, q))
                  <= self.MAX_JOINT_STEP_DEG)
            if not ok:
                ret, q_new = _ik_seeded(stretch_seed, target)
                ok = ret == 0 and q_new is not None
            if not ok:
                # last resort: min-motion arm-angle search — the seeded
                # solver derails near redundancy folds; this variant was
                # written for exactly that (see _ik_min_motion docstring)
                ret, q_new = _ik_min_motion(q, target)
                ok = ret == 0 and q_new is not None
            if ok:
                # FK-VERIFY: the offline solver returns ret 0 with a
                # best-effort solution for UNREACHABLE poses (same trap
                # movej_p guards against) — accepting one poisons the walk.
                fk = _fk_pose(q_new, None)
                ok = math.dist(fk[:3], target[:3]) <= 0.003
            if not ok:
                unsolved += 1
                q_new = q
            elif any(abs(x) > lim + 1e-6
                     for x, lim in zip(q_new, self.joint_limit_deg)):
                return None, None, None, None
            else:
                stretch_seed = list(q_new)
            q = list(q_new)
            out.append(list(q))
            tout.append(tt)
        if unsolved > 0.05 * max(1, len(all_samples)):
            print("[emu] WARNING: %d of %d samples had no IK solution — "
                  "joint rates are an UNDER-estimate (offline-solver "
                  "limitation, not the arm)" % (unsolved, len(all_samples)))
        self.last_plan = {"samples": all_samples, "times": times,
                          "gaps": gaps, "warn_junctions": warns}
        return out, tout, gaps, warns

    # ── arm motion ──
    def movej(self, target_deg, v: int, block: int) -> int:
        if not (1 <= int(v) <= 100):
            return 1                       # real controller: parameter error
        if any(abs(q) > lim + 1e-6
               for q, lim in zip(target_deg, self.joint_limit_deg)):
            return 1                       # target beyond RM75 joint limits
        with self._lock:
            if self.motion_locked:
                return 1                   # latched joint errors: clear first
            if self.reject_next_dispatch:
                self.reject_next_dispatch = False
                return 1
        time.sleep(_scaled(self.command_latency_s))
        v = int(v)
        with self._lock:
            self._truncate_lift_locked()       # planned move preempts the pole
            if self._arm_motion and not self._arm_motion.done.is_set():
                self.joints_deg = self._arm_motion.current()
                self._arm_motion.cancel()      # retarget from current pose
            delta = max(abs(t - c)
                        for t, c in zip(target_deg, self.joints_deg))
            dur = max(delta / (ARM_MAX_DEG_S * v / 100.0), MIN_MOTION_S)
            motion = _Motion(self.joints_deg, list(target_deg),
                             _scaled(dur), self._arm_done)
            motion.will_fail = self._consume_fail_flag()
            self._arm_motion = motion
        if block:
            if not motion.done.wait(_scaled(dur) + 5.0):
                return -5                  # single-thread-style timeout
            if motion.will_fail:
                return 1                   # blocking mode reports the failure
        return 0

    def _arm_done(self, fired):
        with self._lock:
            m = self._arm_motion
            if m is not fired or m.done.is_set():
                return                     # stale timer (retargeted/stopped)
            if m.will_fail:
                # A failed trajectory stops short of the target.
                self.joints_deg = [s + (t - s) * 0.6
                                   for s, t in zip(m.start, m.target)]
            else:
                self.joints_deg = list(m.target)
            m.done.set()
        self._emit(DEV_JOINT, ok=not m.will_fail)

    # ── lift motion (queued, like the real controller) ──
    def set_lift_height(self, speed_pct: int, hw_mm: int, block: int) -> int:
        with self._lock:
            if self.lift_locked:
                return 1                   # latched rejection, no motion
            if self.motion_locked:
                return 1                   # latched joint errors: clear first
            if self.reject_next_dispatch:
                self.reject_next_dispatch = False
                return 1
            # HARDWARE-OBSERVED (2026-08-07, C9 --no-hands, left arm): a
            # lift command that lands while a PLANNED arm trajectory is in
            # flight ABORTS that trajectory — the arm stops where it is and
            # NO device-0 arrival event is ever delivered (the dispatch
            # itself still returns 0). The reverse does not happen: an arm
            # command issued while the pole is moving leaves the pole
            # running, which is why our ZIGZAG01 Web-GUI program commands
            # the lift first and then the arm.
            m = self._arm_motion
            if m is not None and not m.done.is_set():
                self.joints_deg = m.current()      # frozen mid-trajectory
                m.cancel()                         # cancelled => no event
                self._arm_motion = None
                # The abrupt stop is a HARDWARE FAULT, not a clean cancel:
                # it latches joint errors, and every later motion command
                # is rejected until rm_clear_system_err (Newton, hardware,
                # 2026-08-07). The codes below model the BEHAVIOUR — the
                # exact controller codes are not captured here.
                self.joint_err_flags = [ABORT_JOINT_ERR] * 7
                self.sys_err_code = ABORT_SYS_ERR
                self.motion_locked = True
        if not (0 <= int(hw_mm) <= 2600):
            return 1                       # documented controller range
        time.sleep(_scaled(self.command_latency_s))
        done = threading.Event()
        with self._lock:
            self._lift_queue.append((speed_pct, float(hw_mm), done))
            if self._lift_motion is None:
                self._start_next_lift()
        if block:
            if not done.wait(_scaled(LIFT_TIMEOUT_GUESS_S) + 5.0):
                return -5
        return 0

    def _start_next_lift(self):
        # caller holds self._lock
        if not self._lift_queue:
            self._lift_motion = None
            return
        speed_pct, target, done = self._lift_queue.pop(0)
        dist_phys = abs(target - self.lift_hw) * self.lift_hw_to_phys
        dur = LIFT_START_LATENCY_S + _lift_travel_s(dist_phys, speed_pct)
        motion = _Motion(self.lift_hw, target, _scaled(max(dur, MIN_MOTION_S)),
                         self._lift_done)
        motion.caller_done = done
        motion.will_fail = self._consume_fail_flag()
        motion.direction = 1 if target >= self.lift_hw else -1
        self._lift_motion = motion

    def _lift_done(self, fired):
        with self._lock:
            m = self._lift_motion
            if m is not fired or m.done.is_set():
                return                     # stale timer (halted/replaced)
            if m.will_fail:
                self.lift_hw = m.start + (m.target - m.start) * 0.6
            else:
                self.lift_hw = m.target
            m.done.set()
            m.caller_done.set()
            self._lift_motion = None
        self._emit(DEV_LIFT, ok=not m.will_fail)
        with self._lock:
            # A command issued during _emit may have started already.
            if self._lift_motion is None:
                self._start_next_lift()

    # ── pose-target joint-planned move (rm_movej_p, REAL seeded IK) ──
    def movej_p(self, pose6, v: int, block: int) -> int:
        if not (1 <= int(v) <= 100) or len(pose6) != 6:
            return 1
        with self._lock:
            if self.motion_locked:
                return 1               # latched joint errors: clear first
            seed = self.current_joints_locked()
        # The request is a pose OF THE ACTIVE TOOL — assert that frame on
        # the shared solver before IK, or the target is solved for the
        # flange while the verify below applies the tool (see
        # _set_algo_toolframe on why the frame cannot be assumed set).
        _set_algo_toolframe(self._active_tool_pose())
        # target arrives in the CONTROLLER frame, like every API pose
        # (_algo_to_ctrl boundary rule); solve in the algo frame
        _p, _R = _ctrl_to_algo(pose6, self.mount_ry_deg)
        pose6 = list(_p) + list(_R_to_euler(_R))
        ret, target_deg = _ik_seeded(seed, pose6)
        if ret != 0:
            return 1                # IK failure — the controller's ret 1
        if any(abs(q) > lim + 1e-6
               for q, lim in zip(target_deg, self.joint_limit_deg)):
            return 1                # solution beyond joint limits
        # The offline algo lib can return ret 0 with a best-effort solution
        # for UNREACHABLE poses (observed: 2 m target, ret 0), so FK-verify
        # before moving. TOLERANCE IS 10 mm, NOT 2 mm (corrected 2026-08-16):
        # the real controller ACCEPTS a best-effort a few mm short — C6 on
        # hardware commanded +0.200 m and landed +0.196 m without complaint,
        # and the same target's solver miss here is 4.49 mm. At 2 mm the
        # emulator refused a move the arm performs, which is the F25/H3
        # failure inverted (an emulator stricter than the machine refuses
        # work the machine accepts). 10 mm still catches the gross
        # best-effort garbage this check exists for.
        import math as _m
        fk = _fk_pose(target_deg, self._active_tool_pose())
        if _m.dist(fk[:3], list(pose6[:3])) > FK_VERIFY_TOL_M \
                or any(abs((a - b + _m.pi) % (2 * _m.pi) - _m.pi) > 0.01
                       for a, b in zip(fk[3:6], pose6[3:6])):
            return 1                # solver could not actually reach the pose
        # From here it IS a joint-space planned move, like the real one.
        return self.movej(target_deg, v, block)

    def current_pose(self):
        """TCP pose = FK(current joints), in the CONTROLLER frame.

        The real arm reports poses in the same frame it accepts them
        (_algo_to_ctrl boundary rule) — feedback must round-trip with
        movel/movec/movej_p targets.
        """
        return _algo_to_ctrl(
            _fk_pose(self.current_joints(), self._active_tool_pose()),
            self.mount_ry_deg)

    def current_joints_locked(self):
        # caller holds self._lock
        if self._arm_motion and not self._arm_motion.done.is_set():
            return self._arm_motion.current()
        return list(self.joints_deg)

    # ── hand motion (Inspire RH56, protocol path rm_set_hand_angle) ──
    def set_hand_angle(self, values, block: bool, timeout_s: int,
                       _modbus: bool = False) -> int:
        if len(values) != 6 or any(not (-1 <= int(v) <= 1000) for v in values):
            return 1
        if self.modbus_mode and not _modbus:
            # fw 1.7.x exclusivity (bench §3.8): while the end port is in
            # modbus mode the hand PROTOCOL path is dead — blocking calls
            # return -5, non-blocking sends are silently swallowed (ret 0,
            # no motion, no arrival event) — the exact 2026-08-06 C6 failure.
            return -5 if block else 0
        with self._lock:
            if self.reject_next_dispatch:
                self.reject_next_dispatch = False
                return 1
        time.sleep(_scaled(self.command_latency_s))
        with self._lock:
            if self._hand_motion and not self._hand_motion.done.is_set():
                self.hand_hw = [int(round(x))
                                for x in self._hand_motion.current()]
                self._hand_motion.cancel()
            target = [self.hand_hw[i] if int(v) == -1 else int(v)
                      for i, v in enumerate(values)]
            span = max(abs(t - c) for t, c in zip(target, self.hand_hw))
            # stroke_ms = k * span / SPEED_SET (bench §3.7) + command latency
            dur = HAND_CMD_LATENCY_S + max(
                HAND_STROKE_K * span / max(self.hand_speed_set, 1) / 1000.0,
                MIN_MOTION_S)
            motion = _Motion([float(x) for x in self.hand_hw],
                             [float(t) for t in target],
                             _scaled(dur), self._hand_done)
            motion.will_fail = self._consume_fail_flag()
            # OBSERVED (2026-08-06): device-2 arrivals never reach the
            # user event callback on fw 1.7.1/1.7.4 + SDK 1.1.6 — only
            # the SDK-internal blocking wait sees them. Emulate: hand
            # motions NEVER emit user-callback events.
            motion.emit_event = False
            self._hand_motion = motion
        if block:
            # OBSERVED (2026-08-06, C6 ret=-4 in 0.00 s): a blocking hand
            # call with an arm/lift motion in flight consumes THAT device's
            # arrival push and fails instantly with -4. The hand keeps
            # moving physically.
            with self._lock:
                other_busy = (
                    (self._arm_motion and not self._arm_motion.done.is_set())
                    or self._lift_motion is not None)
            if other_busy:
                return -4
            if not motion.done.wait(_scaled(max(timeout_s, 1)) + 5.0):
                return -5
            if motion.will_fail:
                return 1
        return 0

    def _hand_done(self, fired):
        with self._lock:
            m = self._hand_motion
            if m is not fired or m.done.is_set():
                return
            if m.will_fail:
                self.hand_hw = [int(round(s + (t - s) * 0.6))
                                for s, t in zip(m.start, m.target)]
            else:
                self.hand_hw = [int(round(t)) for t in m.target]
            m.done.set()
        if getattr(m, "emit_event", True):
            self._emit(DEV_HAND, ok=not m.will_fail)

    def current_hand(self):
        with self._lock:
            m = self._hand_motion
            if m and not m.done.is_set():
                return [int(round(x)) for x in m.current()]
            return list(self.hand_hw)

    # ── stops ──
    def movej_canfd(self, target_deg) -> int:
        """Streamed setpoint: state follows the command, no event, and no
        interaction with an in-flight lift move (hardware-consistent)."""
        with self._lock:
            if self.motion_locked:
                return 1
            if any(abs(q) > lim + 1e-6
                   for q, lim in zip(target_deg, self.joint_limit_deg)):
                return 1
            # a passthrough setpoint supersedes any planned motion cleanly
            if self._arm_motion and not self._arm_motion.done.is_set():
                self._arm_motion.cancel()
                self._arm_motion = None
            self.joints_deg = list(target_deg)
        return 0

    def stop_arm(self):
        """rm_set_arm_stop: halts arm AND pole, no arrival events."""
        with self._lock:
            if self._arm_motion and not self._arm_motion.done.is_set():
                self.joints_deg = self._arm_motion.current()
                self._arm_motion.cancel()
            self._halt_lift_locked()
        return 0

    def stop_lift(self):
        with self._lock:
            if self.lift_locked:
                return 1                   # rejects the stop command too
            self._halt_lift_locked()
        return 0

    def _truncate_lift_locked(self):
        """A planned arm command stops an in-flight pole move (C16).

        The pole gets ~LIFT_TRUNCATE_MM further and then stops, silently:
        no error, no arrival event. If that distance is enough to reach the
        target the move simply completes normally — which is why short pole
        legs survive a concurrent arm move and long ones do not.
        """
        m = self._lift_motion
        if m is None or m.done.is_set():
            return
        here = m.current()
        remaining = abs(m.target - here)
        if remaining <= LIFT_TRUNCATE_MM / max(self.lift_hw_to_phys, 1e-6):
            return                              # close enough: it will finish
        m.cancel()                              # cancelled => NO event
        step = LIFT_TRUNCATE_MM / max(self.lift_hw_to_phys, 1e-6)
        self.lift_hw = here + (step if m.target > here else -step)
        self._lift_motion = None
        for _, _, done in self._lift_queue:
            done.set()
        self._lift_queue.clear()

    def _halt_lift_locked(self):
        if self._lift_motion and not self._lift_motion.done.is_set():
            self.lift_hw = self._lift_motion.current()
            self._lift_motion.cancel()
            self._lift_motion.caller_done.set()
        self._lift_motion = None
        for _, _, done in self._lift_queue:
            done.set()
        self._lift_queue.clear()

    # ── state ──
    def arm_moving(self) -> bool:
        m = self._arm_motion
        return bool(m and not m.done.is_set())

    def current_joints(self):
        with self._lock:
            if self.arm_moving():
                return self._arm_motion.current()
            return list(self.joints_deg)

    def current_lift_hw(self) -> float:
        with self._lock:
            m = self._lift_motion
            if m and not m.done.is_set():
                return m.current()
            return self.lift_hw

    # ── UDP push (subset) ──
    def _push_field_enabled(self, name: str) -> bool:
        cfg = getattr(self, "push_config", None) or {}
        cc = cfg.get("custom_config")
        return cc is not None and getattr(cc, name, -1) == 1

    def start_push(self, cycle: int):
        self.stop_push()
        stop_evt = threading.Event()
        self._push_stop = stop_evt
        period = _scaled(max(cycle, 1) * 0.005)

        def loop():
            while not stop_evt.wait(period):
                cb = _state_cb
                if cb is None:
                    continue
                moving = self.arm_moving()
                # BLEND/DWELL CHOREOGRAPHY (§10.3, analyse_run's detector):
                # a timed chain reports NOT-moving inside its gap windows —
                # the brief MOVE_L exits at blends and the stop/freeze
                # dwells — while joints keep their positions.
                m = self._arm_motion
                if moving and isinstance(m, _TimedPathMotion):
                    moving = m.status_moving()
                lift_now = int(round(self.current_lift_hw())) \
                    if self._push_field_enabled("lift_state") else 0
                speed = [0.0] * 7    # populated only when enabled (bench)
                pose_now = list(self.current_pose())
                if self.run_mode == 1:
                    # REAL mode: the position field ALIASES — error grows
                    # with TCP speed (measured med 0.9 mm at 0.10 m/s →
                    # 1.5 mm at 0.35, p99 5–7 mm; §9 jitter calibration).
                    prev = getattr(self, "_push_prev", None)
                    now = time.perf_counter()
                    v_est = 0.0
                    if prev is not None and now > prev[0]:
                        v_est = math.dist(pose_now[:3], prev[1]) \
                            / (now - prev[0])
                    self._push_prev = (now, list(pose_now[:3]))
                    import random as _rnd
                    sig = 0.0009 + 0.006 * min(v_est, 2.0)
                    pose_now[0] += _rnd.gauss(0.0, sig)
                    pose_now[1] += _rnd.gauss(0.0, sig)
                    pose_now[2] += _rnd.gauss(0.0, sig)
                cb(SimpleNamespace(
                    errCode=0, arm_ip=self.ip, arm_port=8080,
                    joint_status=SimpleNamespace(
                        joint_position=self.current_joints(),
                        joint_speed=speed,
                        # power telemetry, as the real push carries (V, mA)
                        joint_voltage=list(self.joint_voltage),
                        joint_current=[self.joint_current_idle_ma
                                       + (2200.0 if moving else 0.0)] * 7,
                        joint_temperature=[35.0] * 7,
                        joint_en_flag=list(self.joint_en)),
                    force_sensor=SimpleNamespace(
                        force=[0.0] * 6, zero_force=[0.0] * 6, coordinate=0),
                    # the REAL push carries the controller's own TCP
                    # pose here, position AND euler — a recorder reading
                    # wp.euler must not fault on the emulated frame
                    waypoint=SimpleNamespace(
                        position=SimpleNamespace(*(), **dict(zip(
                            ("x", "y", "z"), pose_now[:3]))),
                        euler=SimpleNamespace(*(), **dict(zip(
                            ("rx", "ry", "rz"), pose_now[3:6])))),
                    liftState=SimpleNamespace(
                        height=lift_now, pos=float(lift_now),
                        current=0,
                        err_flag=int(bool(self.lift_locked)),
                        en_flag=1),
                    handState=SimpleNamespace(     # absent on fw 1.7.2
                        hand_pos=[0] * 6, hand_angle=[0] * 6,
                        hand_force=[0] * 6, hand_state=[0] * 6, hand_err=0),
                    arm_current_status=2 if moving else 0,
                    err=SimpleNamespace(err_len=0, err=[])))

        self._push_thread = threading.Thread(target=loop, daemon=True)
        self._push_thread.start()

    def stop_push(self):
        self._push_stop.set()
        self._push_thread = None


LIFT_TIMEOUT_GUESS_S = 30.0

# Left arm parked at its SRDF zero (J7 = 180 deg), right at zero, lifts half.
def _default_specs():
    return {
        EMU_LEFT_IP: ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 180.0], 100.0),
        EMU_RIGHT_IP: ([0.0] * 7, 100.0),
    }


def _get_or_create(ip: str):
    with _registry_lock:
        ctrl = _by_ip.get(ip)
        if ctrl is None:
            joints, lift = _default_specs()[ip]
            ctrl = EmuController(ip, joints, lift)
            _by_ip[ip] = ctrl
        return ctrl


class rm_robot_handle:
    def __init__(self, hid: int):
        self.id = hid


class RoboticArm:
    """Drop-in emulated replacement for Robotic_Arm.rm_robot_interface.RoboticArm."""

    def __init__(self, mode=None):
        global _thread_mode
        if mode is not None:
            _thread_mode = int(mode)
            if _thread_mode == 0:
                print("[emu] WARNING: single-thread mode — arrival events "
                      "will NOT be delivered (matches the real SDK)")
            print("current c api version:  emu-1.1.6")
        self._ctrl = None
        self.arm_dof = 7

    # ── lifecycle ──
    def rm_create_robot_arm(self, ip, port, level=3, log_func=None):
        if ip in _powered_off or ip not in _default_specs():
            print("[rm_create_robot_arm] socket connect err! (emulated)")
            self.arm_dof = 0                   # mirrors the real wrapper
            self.robot_controller_version = 4
            return rm_robot_handle(-1)
        ctrl = _get_or_create(ip)
        with _registry_lock:
            hid = next(_handle_ids)
            ctrl.handle_id = hid
            _by_handle[hid] = ctrl
        self._ctrl = ctrl
        self.arm_dof = 7
        self.robot_controller_version = 3
        return rm_robot_handle(hid)

    def rm_delete_robot_arm(self):
        # NOTE: the push is NOT stopped — push config is controller state
        # and survives disconnects on real hardware. rm_destroy() stops it.
        self._ctrl = None
        return 0

    @staticmethod
    def rm_destroy():
        with _registry_lock:
            ctrls = list(_by_ip.values())
        for c in ctrls:
            c.stop_arm()
            c.stop_push()
        return 0

    # ── identity / state getters (answers mirror the real arms) ──
    def rm_get_robot_info(self):
        return 0, {"arm_dof": 7, "arm_model": "RM_75", "force_type": "6FB",
                   "robot_controller_version": 3}

    def rm_get_arm_software_info(self):
        # The reported firmware is DERIVED from the emulated lift gearing so
        # the two can never disagree: the real fleet's V1.7.4 arms report
        # the lift in true mm (1:1) and V1.7.1 arms are 2:3 geared, and
        # dual_arm_common detects the gearing from exactly this field.
        true_mm = self._ctrl.lift_hw_to_phys == 1.0
        ver = "V1.7.4-emu" if true_mm else "V1.7.1-emu"
        algo = "1.5.9-emu" if true_mm else "1.5.5-emu"
        return 0, {"product_version": "RM75-6FB",
                   "algorithm_info": {"version": algo},
                   "ctrl_info": {"version": ver,
                                 "build_time": "2025/06/16 16:43:55"},
                   "dynamic_info": {"model_version": "2"},
                   "plan_info": {"version": ver,
                                 "build_time": "2025/06/16 16:44:12"}}

    def rm_get_arm_run_mode(self):
        return 0, self._ctrl.run_mode

    def rm_set_arm_run_mode(self, mode):
        self._ctrl.run_mode = int(mode)
        return 0

    def rm_get_current_arm_state(self):
        time.sleep(_scaled(self._ctrl.command_latency_s))
        joints = self._ctrl.current_joints()
        # Real V1.7.1 pads a clean arm as err_len=1 with code '0'
        # (observed on both arms, 2026-08-06).
        code = self._ctrl.sys_err_code
        return 0, {"joint": joints,
                   "pose": [round(c, 6) for c in self._ctrl.current_pose()],
                   "err": {"err_len": 1, "err": [str(code)]}}

    def rm_get_arm_power_state(self):
        return 0, 1

    # ── self-collision safety detection ──
    # Modelled as plain state: the tests enable it on connect and the
    # emulator must be able to report both states so the "was OFF ->
    # enabling" path is exercised offline. RM_EMU_SELF_COLLISION=1 starts
    # it already on.
    def rm_get_self_collision_enable(self):
        return 0, self._ctrl.self_collision

    def rm_set_self_collision_enable(self, enable):
        self._ctrl.self_collision = bool(enable)
        return 0

    def rm_get_joint_err_flag(self):
        return {"return_code": 0,
                "err_flag": list(self._ctrl.joint_err_flags),
                "brake_state": [0] * 7}

    def rm_clear_system_err(self):
        # Hardware-faithful (2026-08-07, both arms): returns 0 and clears
        # SYSTEM-level state, but does NOT touch per-joint error flags —
        # the R8 gap. Joint 16384 warnings need rm_set_joint_clear_err.
        ctrl = self._ctrl
        with ctrl._lock:
            ctrl.sys_err_code = 0
            ctrl.lift_locked = False       # modelled recovery path
            if not any(ctrl.joint_err_flags):
                ctrl.motion_locked = False
        return 0

    def rm_set_joint_clear_err(self, joint_num):
        # The Web GUI's per-joint "Clear Error" button.
        ctrl = self._ctrl
        if not 1 <= int(joint_num) <= 7:
            return 1
        with ctrl._lock:
            ctrl.joint_err_flags[int(joint_num) - 1] = 0
            if not any(ctrl.joint_err_flags) and ctrl.sys_err_code == 0:
                ctrl.motion_locked = False
        return 0

    def rm_get_joint_degree(self):
        return 0, self._ctrl.current_joints()

    def rm_get_lift_state(self):
        time.sleep(_scaled(self._ctrl.command_latency_s))
        ctrl = self._ctrl
        m = ctrl._lift_motion
        if m is None or m.done.is_set():
            mode = 0
        else:
            mode = 2 if m.direction >= 0 else 4   # pos-motion, +/- direction
        return 0, {"pos": int(round(ctrl.current_lift_hw())),
                   "err_flag": 1 if ctrl.lift_locked else 0,
                   "mode": mode, "current": 0}

    # ── motion ──
    def rm_movej(self, joint, v, r, connect, block):
        # r (blend) and connect (chaining) are accepted but not emulated:
        # every move executes immediately with an exact stop.
        if len(joint) != 7:
            return 1
        return self._ctrl.movej(list(joint), v, block)

    # ── Cartesian moves + tool frames (Phase 2 cleaning paths) ──
    # `trajectory_connect` IS emulated here, unlike movej: the cleaning
    # path is one chain of 27-43 segments and its whole completion model
    # rests on the chain semantics — connect=1 queues and NEVER completes,
    # the closing connect=0 executes and yields exactly ONE arrival event.
    # A dispatcher that waits on a connect=1 waits forever, so the
    # emulator has to be able to reproduce that.
    MAX_QUEUE = int(os.environ.get("RM_EMU_MAX_QUEUE", "50"))

    def rm_movel(self, pose, v, r, connect, block):
        # The real SDK indexes `pose[:3]` / `pose[3:]` to build rm_pose_t,
        # so a struct argument raises TypeError there. Reproduce that
        # instead of quietly accepting either shape — an emulator more
        # permissive than the API certifies calls the hardware rejects
        # (2026-08-08: this exact bug reached the arms).
        try:
            _ = [float(x) for x in pose[:6]]
        except TypeError:
            raise TypeError(f"'{type(pose).__name__}' object is not "
                            "subscriptable")
        if len(pose) < 6:
            return 1
        if not (1 <= int(v) <= 100) or not (0 <= int(r) <= 100):
            return 1
        if self._ctrl.motion_locked:
            return 1
        q = getattr(self, "_movel_queue", None)
        if q is None:
            q = self._movel_queue = []
        if int(connect) == 1:
            if len(q) >= self.MAX_QUEUE:
                return 1            # controller refuses a deeper queue
            # PER-MOVE v AND r ARE HONORED by the controller (§9.3d) —
            # the queue must carry them, not just the pose. Before
            # 2026-08-16 it stored poses only and executed the whole
            # chain with the CLOSING move's v: provably wrong.
            q.append({"pose": pose, "v": int(v), "r": int(r), "via": None})
            return 0                # queued: no motion, no event
        # closing segment: the whole chain executes as ONE trajectory
        chain = list(q) + [{"pose": pose, "v": int(v), "r": int(r),
                            "via": None}]
        self._movel_queue = []
        return self._ctrl.movel_chain(chain, v, block)

    def rm_movec(self, pose_via, pose_to, v, r, loop, connect, block):
        # FULL ARC EMULATION (2026-08-16): the move is queued as ONE entry
        # carrying its via, and _plan_chain samples the true 3-point circle
        # (_arc_points) with tangent-junction speeds — the construction the
        # REAL 006 runs traced to 0.4 mm median with no junction stops
        # (§10.1/§10.4). `loop` is accepted but not modelled (never used
        # mid-chain in any measured run).
        for p in (pose_via, pose_to):
            try:
                _ = [float(x) for x in p[:6]]
            except TypeError:
                raise TypeError(f"'{type(p).__name__}' object is not "
                                "subscriptable")
            if len(p) < 6:
                return 1
        if not (1 <= int(v) <= 100) or not (0 <= int(r) <= 100) \
                or int(loop) < 0:
            return 1
        if self._ctrl.motion_locked:
            return 1
        q = getattr(self, "_movel_queue", None)
        if q is None:
            q = self._movel_queue = []
        move = {"pose": pose_to, "v": int(v), "r": int(r),
                "via": list(pose_via)}
        if int(connect) == 1:
            if len(q) >= self.MAX_QUEUE:
                return 1
            q.append(move)
            return 0
        chain = list(q) + [move]
        self._movel_queue = []
        return self._ctrl.movel_chain(chain, v, block)

    def rm_change_tool_frame(self, name):
        name = name.decode() if isinstance(name, bytes) else str(name)
        if name not in self._ctrl.tool_frames:
            return 1
        self._ctrl.active_tool = name
        return 0

    def rm_get_current_tool_frame(self):
        name = self._ctrl.active_tool
        f = self._ctrl.tool_frames.get(name, {})
        com = f.get("com", [0.0, 0.0, 0.0])
        return 0, {"name": name, "pose": f.get("pose", [0.0] * 6),
                   "payload": f.get("payload", 0.0),
                   "x": com[0], "y": com[1], "z": com[2]}

    def rm_get_total_tool_frame(self):
        return {"return_code": 0,
                "tool_names": list(self._ctrl.tool_frames),
                "len": len(self._ctrl.tool_frames)}

    def rm_get_given_tool_frame(self, name):
        name = name.decode() if isinstance(name, bytes) else str(name)
        f = self._ctrl.tool_frames.get(name)
        if f is None:
            return 1, {}
        com = f.get("com", [0.0, 0.0, 0.0])
        return 0, {"name": name, "pose": f["pose"],
                   "payload": f.get("payload", 0.0),
                   "x": com[0], "y": com[1], "z": com[2]}

    def rm_set_manual_tool_frame(self, frame):
        name = frame.frame_name.decode() if isinstance(
            frame.frame_name, bytes) else str(frame.frame_name)
        if not name or len(name) > 11:
            return 1
        if name in self._ctrl.tool_frames:
            return 1                # CREATE only — the real ret=1 (F17)
        if len(self._ctrl.tool_frames) >= 10:
            return 1                # rm_frame_name_t*10
        p, e = frame.pose.position, frame.pose.euler
        self._ctrl.tool_frames[name] = {
            "pose": [p.x, p.y, p.z, e.rx, e.ry, e.rz],
            "payload": float(frame.payload),
            "com": [float(frame.x), float(frame.y), float(frame.z)]}
        return 0

    def rm_update_tool_frame(self, frame):
        name = frame.frame_name.decode() if isinstance(
            frame.frame_name, bytes) else str(frame.frame_name)
        if name not in self._ctrl.tool_frames:
            return 1
        p, e = frame.pose.position, frame.pose.euler
        self._ctrl.tool_frames[name] = {
            "pose": [p.x, p.y, p.z, e.rx, e.ry, e.rz],
            "payload": float(frame.payload),
            "com": [float(frame.x), float(frame.y), float(frame.z)]}
        return 0

    def rm_delete_tool_frame(self, name):
        name = name.decode() if isinstance(name, bytes) else str(name)
        if name == self._ctrl.active_tool or name not in \
                self._ctrl.tool_frames:
            return 1
        del self._ctrl.tool_frames[name]
        return 0

    def rm_set_lift_height(self, speed, height, block):
        if not (1 <= int(speed) <= 100):
            return 1
        return self._ctrl.set_lift_height(speed, height, block)

    def rm_set_lift_speed(self, speed):
        speed = int(speed)
        if speed == 0:
            return self._ctrl.stop_lift()
        if not (-100 <= speed <= 100):
            return 1
        # Open-loop jog: run toward the travel end at |speed|%.
        target = self._ctrl.lift_hw_max if speed > 0 else 0
        return self._ctrl.set_lift_height(abs(speed), target, 0)

    # ── configured speed/acceleration envelope (F10, both arms) ──
    # v% in movej/movel is a percentage OF THESE, so a runner that reports
    # "cleaning at 100%" needs them to say what that is in m/s.
    def rm_get_arm_max_line_speed(self):
        return 0, self._ctrl.limits["line_speed"]

    def rm_get_arm_max_line_acc(self):
        return 0, self._ctrl.limits["line_acc"]

    def rm_get_arm_max_angular_speed(self):
        return 0, self._ctrl.limits["angular_speed"]

    def rm_get_arm_max_angular_acc(self):
        return 0, self._ctrl.limits["angular_acc"]

    def rm_get_joint_max_speed(self):
        return 0, list(self._ctrl.joint_max_speed)

    def rm_get_joint_max_acc(self):
        return 0, list(self._ctrl.joint_max_acc)

    def rm_set_arm_max_line_speed(self, speed):
        self._ctrl.limits["line_speed"] = float(speed)
        return 0

    def rm_set_arm_max_line_acc(self, acc):
        self._ctrl.limits["line_acc"] = float(acc)
        return 0

    def rm_set_arm_max_angular_speed(self, speed):
        self._ctrl.limits["angular_speed"] = float(speed)
        return 0

    def rm_set_arm_max_angular_acc(self, acc):
        self._ctrl.limits["angular_acc"] = float(acc)
        return 0

    def rm_set_joint_max_speed(self, joint_num, speed):
        if not (1 <= int(joint_num) <= 7):
            return 1
        self._ctrl.joint_max_speed[int(joint_num) - 1] = float(speed)
        return 0

    def rm_set_joint_max_acc(self, joint_num, acc):
        if not (1 <= int(joint_num) <= 7):
            return 1
        self._ctrl.joint_max_acc[int(joint_num) - 1] = float(acc)
        return 0

    def rm_get_joint_en_state(self):
        return 0, [bool(x) for x in self._ctrl.joint_en]

    def rm_set_joint_en_state(self, joint_num, en_state):
        if not (1 <= int(joint_num) <= 7):
            return 1
        self._ctrl.joint_en[int(joint_num) - 1] = 1 if en_state else 0
        return 0

    # ── safety/assist capabilities (V1.7.4). The arms ship with these
    #    OFF; Phase 2 turns singularity avoidance on for Cartesian paths.
    def rm_get_avoid_singularity_mode(self):
        return 0, int(self._ctrl.caps["avoid_singularity"])

    def rm_set_avoid_singularity_mode(self, mode):
        if int(mode) not in (0, 1):
            return 1
        self._ctrl.caps["avoid_singularity"] = int(mode)
        return 0

    def rm_get_self_endeffector_collision_enable(self):
        return 0, bool(self._ctrl.caps["endeff_collision"])

    def rm_set_self_endeffector_collision_enable(self, set_enable):
        self._ctrl.caps["endeff_collision"] = bool(set_enable)
        return 0

    def rm_get_collision_stage(self):
        return 0, int(self._ctrl.caps["collision_stage"])

    def rm_set_collision_state(self, stage):
        if not (0 <= int(stage) <= 8):
            return 1
        self._ctrl.caps["collision_stage"] = int(stage)
        return 0

    def rm_get_collision_detection(self):
        return 0, int(self._ctrl.caps["static_collision"])

    def rm_set_collision_detection(self, mode):
        if int(mode) not in (0, 1):
            return 1
        self._ctrl.caps["static_collision"] = int(mode)
        return 0

    def rm_get_collision_remove_enable(self):
        return 0, bool(self._ctrl.caps["collision_remove"])

    def rm_set_collision_remove_enable(self, set_enable):
        self._ctrl.caps["collision_remove"] = bool(set_enable)
        return 0

    def rm_get_install_pose(self):
        """Mounting angle, as ONE dict (not the (ret, dict) tuple
        most getters use) — mirroring the real SDK signature."""
        return {"return_code": 0, "x": 0.0, "y": 90.0, "z": 0.0}

    def rm_set_arm_stop(self):
        return self._ctrl.stop_arm()

    def rm_movej_canfd(self, joint, follow, expand=0, trajectory_mode=0,
                       radio=0):
        """Passthrough: the commanded angles become the state immediately.

        Two fidelity points that matter for the sync tests:
          * NO arrival event is emitted — passthrough bypasses planning, so
            there is no trajectory to "arrive". Completion is the caller's
            stream ending.
          * It does NOT engage the planned-move truncation path, which is
            exactly why lift + canfd works on hardware while lift + movej
            does not (butterfli_hw bench_sync vs C16).
        """
        if len(joint) != 7:
            return 1
        return self._ctrl.movej_canfd(list(joint))

    def rm_movej_p(self, pose, v, r, connect, block):
        # r/connect accepted but not emulated (immediate exact-stop move).
        return self._ctrl.movej_p(list(pose), v, block)

    # Signature matches the REAL SDK exactly — block and timeout are
    # POSITIONAL there. Giving them defaults here let a caller that
    # omitted `timeout` pass under emulation and then fail on
    # hardware with a TypeError (2026-08-08, stage_runner).
    def rm_set_hand_angle(self, hand_angle, block, timeout):
        return self._ctrl.set_hand_angle(hand_angle, block, timeout)

    # ── end-port modbus RTU (the butterfli_hw ALL-MODBUS hand path) ──
    # Register map (butterfli_hw conversions.hpp): ANGLE_SET 1486,
    # FORCE_SET 1498, SPEED_SET 1522, ANGLE_ACT 1546, FORCE_ACT 1582.
    def rm_set_modbus_mode(self, port, baudrate, timeout):
        if port != 1:
            return 1
        self._ctrl.modbus_mode = True
        return 0

    def rm_close_modbus_mode(self, port):
        if port != 1:
            return 1
        self._ctrl.modbus_mode = False
        return 0

    @staticmethod
    def _decode_hi_lo(data):
        return [((data[2 * i] & 0xFF) << 8) | (data[2 * i + 1] & 0xFF)
                for i in range(len(data) // 2)]

    def rm_write_registers(self, write_params, data):
        ctrl = self._ctrl
        if not ctrl.modbus_mode or write_params.port != 1:
            return 1
        vals = self._decode_hi_lo(list(data))
        if write_params.address == 1486 and len(vals) == 6:   # ANGLE_SET
            return ctrl.set_hand_angle(vals, False, 2, _modbus=True)
        if write_params.address == 1522 and vals:             # SPEED_SET
            ctrl.hand_speed_set = max(1, min(1000, vals[0]))
            return 0
        if write_params.address == 1498 and vals:             # FORCE_SET
            ctrl.hand_force_set = max(1, min(1000, vals[0]))
            return 0
        return 1

    def rm_write_single_register(self, write_params, data):
        ctrl = self._ctrl
        if not ctrl.modbus_mode or write_params.port != 1:
            return 1
        base, off = write_params.address, None
        if 1486 <= base <= 1496 and (base - 1486) % 2 == 0:
            ch = (base - 1486) // 2
            target = list(ctrl.current_hand())
            target[ch] = int(data)
            return ctrl.set_hand_angle(target, False, 2, _modbus=True)
        if base == 1522:
            ctrl.hand_speed_set = max(1, min(1000, int(data)))
            return 0
        return 1

    def rm_read_multiple_holding_registers(self, read_params):
        ctrl = self._ctrl
        if not ctrl.modbus_mode or read_params.port != 1:
            return 1, []
        n = read_params.num
        if read_params.address == 1546:                       # ANGLE_ACT
            vals = ctrl.current_hand()[:n]
        elif read_params.address == 1582:                     # FORCE_ACT
            vals = [0] * n
        else:
            return 1, []
        out = []
        for v in vals:
            out += [(int(v) >> 8) & 0xFF, int(v) & 0xFF]
        return 0, out

    def rm_set_hand_speed(self, speed):
        if not (1 <= int(speed) <= 1000):
            return 1
        self._ctrl.hand_speed_set = int(speed)
        return 0

    def rm_set_hand_force(self, force):
        if not (1 <= int(force) <= 1000):
            return 1
        self._ctrl.hand_force_set = int(force)
        return 0

    # ── callbacks / push ──
    def rm_get_arm_event_call_back(self, event_callback):
        global _event_cb
        _event_cb = event_callback

    def rm_realtime_arm_state_call_back(self, realtime_callback):
        global _state_cb
        _state_cb = realtime_callback

    def rm_set_realtime_push(self, config):
        cycle = getattr(config, "cycle", 1)
        enable = getattr(config, "enable", True)
        ip = getattr(config, "ip", "192.168.1.235")
        port = getattr(config, "port", 8089)
        if cycle is None or int(cycle) < 1 \
                or not (0 < int(port) < 65536) or not ip:
            return -4                       # illegal push configuration
        cycle = int(cycle)
        self._ctrl.push_config = {
            "cycle": cycle, "enable": enable,
            "port": getattr(config, "port", 8089),
            "force_coordinate": getattr(config, "force_coordinate", -1),
            "ip": getattr(config, "ip", "192.168.1.235"),
            "custom_config": getattr(config, "custom_config", None),
        }
        if enable:
            if ip not in EMU_HOST_IPS:
                # The real controller accepts ANY target IP (ret 0) and
                # pushes into the void if it is not your host. Reproduce
                # that trap, but say so clearly.
                print(f"[emu] UDP push target {ip}:{port} is NOT this host "
                      f"(emulated host IPs: {sorted(EMU_HOST_IPS)}) — "
                      "config accepted (ret 0) but NO frames will be "
                      "delivered, matching real hardware")
                self._ctrl.stop_push()
            else:
                self._ctrl.start_push(cycle)
        else:
            self._ctrl.stop_push()
        return 0

    # ── version/capability getters (answers mirror the real V1.7.1 arms) ──
    def rm_get_joint_software_version(self):
        return 0, {"version": [54544] * 6 + [58640]}

    def rm_get_sn(self):
        return -2, ""                  # not supported on this firmware

    def rm_algo_version(self):
        return "1.6.0-emu"

    def rm_get_electronic_fence_enable(self):
        return 0, {"enable_state": False, "in_out_side": 0,
                   "effective_region": 0}

    def rm_get_torque_data(self):
        return -2, [], 0               # no joint torque sensors on RM75-6FB

    def rm_get_realtime_push(self):
        cfg = getattr(self._ctrl, "push_config", None) or {
            "cycle": 1, "enable": False, "port": 8089,
            "force_coordinate": -1, "ip": "192.168.1.235",
            "custom_config": None,
        }
        return 0, dict(cfg)


# ─── sys.modules installation ───────────────────────────────────────────────
class _KwargsStruct:
    cycle = 1
    enable = True
    port = 8089
    ip = "192.168.1.235"
    force_coordinate = -1
    custom_config = None
    lift_state = -1
    expand_state = -1
    joint_speed = -1
    hand_state = -1
    arm_current_status = -1

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


def rm_get_arm_event_call_back(event_callback):
    """Module-level registrar, mirroring rm_ctypes_wrap."""
    global _event_cb
    _event_cb = event_callback


def rm_realtime_arm_state_call_back(realtime_callback):
    global _state_cb
    _state_cb = realtime_callback


def install():
    """Install the emulator as the Robotic_Arm package in sys.modules.

    Must run BEFORE the code under test imports Robotic_Arm.
    """
    import sys as _sys
    pkg = ModuleType("Robotic_Arm")
    pkg.__path__ = []
    ri = ModuleType("Robotic_Arm.rm_robot_interface")
    cw = ModuleType("Robotic_Arm.rm_ctypes_wrap")

    import enum

    class _ThreadMode(enum.IntEnum):        # callable: rm_thread_mode_e(2)
        RM_SINGLE_MODE_E = 0
        RM_DUAL_MODE_E = 1
        RM_TRIPLE_MODE_E = 2

    # ── Cartesian / frame structs, mirroring the ctypes layout the tests
    #    populate field-by-field (position/euler, frame_name/pose/payload).
    class _XYZ:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = float(x), float(y), float(z)

    class _RPY:
        def __init__(self, rx=0.0, ry=0.0, rz=0.0):
            self.rx, self.ry, self.rz = float(rx), float(ry), float(rz)

    class _Pose:
        def __init__(self, position=None, euler=None):
            self.position = position or _XYZ()
            self.euler = euler or _RPY()

    class _Frame:
        def __init__(self):
            self.frame_name = b""
            self.pose = _Pose()
            self.payload = 0.0
            self.x = self.y = self.z = 0.0

    shared = {
        "RoboticArm": RoboticArm,
        "rm_thread_mode_e": _ThreadMode,
        "rm_robot_handle": rm_robot_handle,
        "rm_event_callback_ptr": (lambda f: f),
        "rm_realtime_arm_state_callback_ptr": (lambda f: f),
        "rm_event_push_data_t": SimpleNamespace,
        "rm_realtime_arm_joint_state_t": SimpleNamespace,
        "rm_realtime_push_config_t": _KwargsStruct,
        "rm_peripheral_read_write_params_t": _KwargsStruct,
        "rm_udp_custom_config_t": _KwargsStruct,
        # Cartesian types — the cleaning path builds rm_pose_t for movel,
        # and rm_frame_t for the C14 tool-frame writer.
        "rm_position_t": _RealPos if _ALGO else _XYZ,
        "rm_euler_t": _RealEuler if _ALGO else _RPY,
        "rm_pose_t": _RealPose if _ALGO else _Pose,
        "rm_frame_t": _RealFrame if _ALGO else _Frame,
        "rm_get_arm_event_call_back": rm_get_arm_event_call_back,
        "rm_api_version": (lambda: "emu-1.1.6"),
        "rm_realtime_arm_state_call_back": rm_realtime_arm_state_call_back,
        # The OFFLINE solver is a local library, not the controller —
        # captured before install() swapped the modules out. Exposing
        # the real one keeps FK/IK checks meaningful under emulation.
        "Algo": _RealAlgo,
        "rm_robot_arm_model_e": _arm_model_e,
        "rm_force_type_e": _force_e,
        "rm_inverse_kinematics_params_t": _ik_params_t,
    }
    for name, val in shared.items():        # real SDK star-imports the wrap
        setattr(ri, name, val)
        setattr(cw, name, val)

    _sys.modules["Robotic_Arm"] = pkg
    _sys.modules["Robotic_Arm.rm_robot_interface"] = ri
    _sys.modules["Robotic_Arm.rm_ctypes_wrap"] = cw
    return ri, cw
