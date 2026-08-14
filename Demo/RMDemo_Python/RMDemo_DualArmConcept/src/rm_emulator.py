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

Not emulated (extend as needed): Cartesian moves/pose state, force sensor
data, hands, CANFD passthrough, online programming, fence/collision config.
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
        path, dur = self._plan_cartesian(seed, tool, poses, v, ls, la)
        if path is None:
            return 1                      # IK failure / limit — controller ret 1

        with self._lock:
            if self._arm_motion and not self._arm_motion.done.is_set():
                self._arm_motion.cancel()
            motion = _PathMotion(path, _scaled(dur), self._arm_done)
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
        ret, target_deg = _ik_seeded(seed, pose6)
        if ret != 0:
            return 1                # IK failure — the controller's ret 1
        if any(abs(q) > lim + 1e-6
               for q, lim in zip(target_deg, self.joint_limit_deg)):
            return 1                # solution beyond joint limits
        # The offline algo lib can return ret 0 with a best-effort solution
        # for UNREACHABLE poses (observed: 2 m target, ret 0). The real
        # controller refuses those, so FK-verify the solution against the
        # request before moving (2 mm / ~0.6 deg tolerance).
        import math as _m
        fk = _fk_pose(target_deg, self._active_tool_pose())
        if _m.dist(fk[:3], list(pose6[:3])) > 0.002 \
                or any(abs((a - b + _m.pi) % (2 * _m.pi) - _m.pi) > 0.01
                       for a, b in zip(fk[3:6], pose6[3:6])):
            return 1                # solver could not actually reach the pose
        # From here it IS a joint-space planned move, like the real one.
        return self.movej(target_deg, v, block)

    def current_pose(self):
        """TCP pose = FK(current joints) via RealMan's solver."""
        return _fk_pose(self.current_joints(),
                        self._active_tool_pose())

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
                lift_now = int(round(self.current_lift_hw())) \
                    if self._push_field_enabled("lift_state") else 0
                speed = [0.0] * 7    # populated only when enabled (bench)
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
                            ("x", "y", "z"),
                            self.current_pose()[:3]))),
                        euler=SimpleNamespace(*(), **dict(zip(
                            ("rx", "ry", "rz"),
                            self.current_pose()[3:6])))),
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
            q.append(pose)
            return 0                # queued: no motion, no event
        # closing segment: the whole chain executes as ONE trajectory
        chain = list(q) + [pose]
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
