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
import os
import threading
import time
from types import ModuleType, SimpleNamespace

# ─── Timing model (measured values where available) ─────────────────────────
ARM_MAX_DEG_S = 180.0          # synchronized-profile joint speed at v=100
RM75_LIMIT_DEG = [177.6, 130.0, 177.6, 135.0, 177.6, 128.0, 360.0]
CMD_LATENCY_S = 0.005          # per-command TCP round trip (measured ~8 ms)
LIFT_START_LATENCY_S = 0.38    # measured (butterfli_hw kLiftStartLatencyS)
LIFT_SPEED_MAP = [             # speed% -> physical mm/s (LiftBenchmark)
    (10, 17.3), (20, 32.07), (30, 42.73),
    (50, 56.44), (70, 63.25), (100, 66.89),
]
HW_TO_PHYS = 1.5               # hw mm -> physical mm (2/3 scale inverse)

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
EMU_HOST_IPS = {os.environ.get("RM_HOST_IP", "192.168.1.235")}


def emu_set_host_ips(*ips):
    global EMU_HOST_IPS
    EMU_HOST_IPS = set(ips)


def set_time_scale(scale: float):
    global _time_scale
    _time_scale = max(float(scale), 1e-6)


def _scaled(seconds: float) -> float:
    return seconds / _time_scale


def _lift_speed_mm_s(pct: int) -> float:
    pct = max(1, min(100, int(pct)))
    pts = LIFT_SPEED_MAP
    if pct <= pts[0][0]:
        return pts[0][1] * pct / pts[0][0]
    for (p0, v0), (p1, v1) in zip(pts, pts[1:]):
        if pct <= p1:
            return v0 + (v1 - v0) * (pct - p0) / (p1 - p0)
    return pts[-1][1]


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


class EmuController:
    """State + motion engine for one emulated RM75-6FB arm with pole lift."""

    def __init__(self, ip: str, joints_deg, lift_hw_mm: float):
        self.ip = ip
        self.handle_id = None
        self.joints_deg = list(joints_deg)
        self.lift_hw = float(lift_hw_mm)
        self.hand_hw = [993, 993, 993, 993, 981, 992]   # SRDF 'release'
        self.hand_speed_set = 500
        self.hand_force_set = 500
        self._hand_motion = None
        self.run_mode = 1                     # 1 = REAL, 0 = SIMULATION
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

    # ── arm motion ──
    def movej(self, target_deg, v: int, block: int) -> int:
        if not (1 <= int(v) <= 100):
            return 1                       # real controller: parameter error
        if any(abs(q) > lim + 1e-6
               for q, lim in zip(target_deg, RM75_LIMIT_DEG)):
            return 1                       # target beyond RM75 joint limits
        with self._lock:
            if self.reject_next_dispatch:
                self.reject_next_dispatch = False
                return 1
        time.sleep(_scaled(self.command_latency_s))
        v = int(v)
        with self._lock:
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
            if self.reject_next_dispatch:
                self.reject_next_dispatch = False
                return 1
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
        dist_phys = abs(target - self.lift_hw) * HW_TO_PHYS
        dur = LIFT_START_LATENCY_S + dist_phys / _lift_speed_mm_s(speed_pct)
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

    # ── hand motion (Inspire RH56, protocol path rm_set_hand_angle) ──
    def set_hand_angle(self, values, block: bool, timeout_s: int) -> int:
        if len(values) != 6 or any(not (-1 <= int(v) <= 1000) for v in values):
            return 1
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
            self._hand_motion = motion
        if block:
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
        self._emit(DEV_HAND, ok=not m.will_fail)

    def current_hand(self):
        with self._lock:
            m = self._hand_motion
            if m and not m.done.is_set():
                return [int(round(x)) for x in m.current()]
            return list(self.hand_hw)

    # ── stops ──
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
            self._halt_lift_locked()
        return 0

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
                        joint_speed=speed),
                    force_sensor=SimpleNamespace(
                        force=[0.0] * 6, zero_force=[0.0] * 6, coordinate=0),
                    waypoint=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0, z=0.0)),
                    liftState=SimpleNamespace(height=lift_now, pos=lift_now),
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
        return 0, {"product_version": "RM75-6FB",
                   "algorithm_info": {"version": "1.5.5-emu"},
                   "ctrl_info": {"version": "V1.7.1-emu",
                                 "build_time": "2025/06/16 16:43:55"},
                   "dynamic_info": {"model_version": "2"},
                   "plan_info": {"version": "V1.7.1-emu",
                                 "build_time": "2025/06/16 16:44:12"}}

    def rm_get_arm_run_mode(self):
        return 0, self._ctrl.run_mode

    def rm_set_arm_run_mode(self, mode):
        self._ctrl.run_mode = int(mode)
        return 0

    def rm_get_current_arm_state(self):
        time.sleep(_scaled(self._ctrl.command_latency_s))
        joints = self._ctrl.current_joints()
        return 0, {"joint": joints,
                   "pose": [0.0] * 6,
                   "err": {"err_len": 0, "err": []}}

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
                   "err_flag": 0, "mode": mode, "current": 0}

    # ── motion ──
    def rm_movej(self, joint, v, r, connect, block):
        # r (blend) and connect (chaining) are accepted but not emulated:
        # every move executes immediately with an exact stop.
        if len(joint) != 7:
            return 1
        return self._ctrl.movej(list(joint), v, block)

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
        target = 200 if speed > 0 else 0
        return self._ctrl.set_lift_height(abs(speed), target, 0)

    def rm_set_arm_stop(self):
        return self._ctrl.stop_arm()

    def rm_set_hand_angle(self, hand_angle, block=True, timeout=10):
        return self._ctrl.set_hand_angle(hand_angle, block, timeout)

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

    shared = {
        "RoboticArm": RoboticArm,
        "rm_thread_mode_e": _ThreadMode,
        "rm_robot_handle": rm_robot_handle,
        "rm_event_callback_ptr": (lambda f: f),
        "rm_realtime_arm_state_callback_ptr": (lambda f: f),
        "rm_event_push_data_t": SimpleNamespace,
        "rm_realtime_arm_joint_state_t": SimpleNamespace,
        "rm_realtime_push_config_t": _KwargsStruct,
        "rm_udp_custom_config_t": _KwargsStruct,
        "rm_get_arm_event_call_back": rm_get_arm_event_call_back,
        "rm_realtime_arm_state_call_back": rm_realtime_arm_state_call_back,
    }
    for name, val in shared.items():        # real SDK star-imports the wrap
        setattr(ri, name, val)
        setattr(cw, name, val)

    _sys.modules["Robotic_Arm"] = pkg
    _sys.modules["Robotic_Arm.rm_robot_interface"] = ri
    _sys.modules["Robotic_Arm.rm_ctypes_wrap"] = cw
    return ri, cw
