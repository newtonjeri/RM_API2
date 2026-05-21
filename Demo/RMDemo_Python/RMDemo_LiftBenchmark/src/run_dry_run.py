#!/usr/bin/env python3
"""
run_dry_run.py
--------------
Hardware-absent logic and math verification for the RMDemo_LiftBenchmark
test suite.  No real robot connection is made; a lightweight MockRobot is
injected via monkeypatching before each test module is imported.

Sections
--------
  A  Scale-factor math (PHYS_TO_HW / HW_TO_PHYS round-trips)
  B  safe_height() boundary conditions
  C  hw_height() conversions
  D  phys_height() conversions
  E  Trajectory helpers (pos_m_to_mm, vel_m_s_to_pct)
  F  Full test-module smoke tests (mock RoboticArm, no hardware)

Exit codes:
    0  – all checks passed
    1  – one or more checks failed
"""

import sys
import math
import time
import types
import threading
import importlib
import pathlib
import unittest.mock as mock

# ─── Common constants (duplicated here so this script is standalone) ─────────
POLE_MIN_MM    = 10
POLE_MAX_MM    = 290
POLE_TRAVEL_MM = 300
PHYS_TO_HW     = 2.0 / 3.0
HW_TO_PHYS     = 3.0 / 2.0

_pass = 0
_fail = 0


def ok(name: str, detail: str = ""):
    global _pass
    _pass += 1
    suf = f"  ({detail})" if detail else ""
    print(f"  [PASS] {name}{suf}")


def fail(name: str, detail: str = ""):
    global _fail
    _fail += 1
    suf = f"  ({detail})" if detail else ""
    print(f"  [FAIL] {name}{suf}")


def section(title: str):
    print(f"\n{'─'*64}\n  {title}\n{'─'*64}")


# ═══════════════════════════════════════════════════════════════════════════════
# A  Scale-factor math
# ═══════════════════════════════════════════════════════════════════════════════
section("A  Scale-factor math (PHYS_TO_HW × HW_TO_PHYS round-trip)")

# Known exact conversions
cases_phys_to_hw = [
    (0,   0),
    (150, 100),   # mid-point
    (300, 200),   # full travel
    (10,  round(10  * PHYS_TO_HW)),   # min limit
    (290, round(290 * PHYS_TO_HW)),   # max limit
]

for phys, expected_hw in cases_phys_to_hw:
    hw = int(round(phys * PHYS_TO_HW))
    if hw == expected_hw:
        ok(f"phys {phys} mm → hw {hw} mm")
    else:
        fail(f"phys {phys} mm → expected hw {expected_hw}, got {hw}")

# Round-trip: phys → hw → phys  (within ±1 mm due to rounding)
for phys in [10, 50, 100, 150, 200, 250, 290]:
    hw        = int(round(phys * PHYS_TO_HW))
    recovered = hw * HW_TO_PHYS
    if abs(recovered - phys) <= 1.0:
        ok(f"round-trip {phys} mm  → hw={hw} → phys={recovered:.1f} mm")
    else:
        fail(f"round-trip {phys} mm  off by {abs(recovered-phys):.2f} mm")


# ═══════════════════════════════════════════════════════════════════════════════
# B  safe_height() boundary conditions
# ═══════════════════════════════════════════════════════════════════════════════
section("B  safe_height() boundary conditions")

# Local safe_height for the dry-run (same logic as in each test file)
def _safe_height(h: int) -> int:
    assert POLE_MIN_MM <= h <= POLE_MAX_MM, (
        f"Height {h} mm violates pole limits [{POLE_MIN_MM}, {POLE_MAX_MM}]"
    )
    return h

for h in [POLE_MIN_MM, POLE_MAX_MM, 150]:
    try:
        _safe_height(h)
        ok(f"safe_height({h}) accepted")
    except AssertionError as e:
        fail(f"safe_height({h}) rejected unexpectedly: {e}")

for h in [POLE_MIN_MM - 1, POLE_MAX_MM + 1, -1, 0, 300, 500]:
    try:
        _safe_height(h)
        fail(f"safe_height({h}) should have raised AssertionError")
    except AssertionError:
        ok(f"safe_height({h}) correctly rejected")


# ═══════════════════════════════════════════════════════════════════════════════
# C  hw_height() conversions
# ═══════════════════════════════════════════════════════════════════════════════
section("C  hw_height() conversion (physical mm → hardware units)")

def _hw_height(phys_mm: int) -> int:
    _safe_height(phys_mm)
    return int(round(phys_mm * PHYS_TO_HW))

cases = [
    (10,  int(round(10  * PHYS_TO_HW))),
    (150, 100),
    (290, int(round(290 * PHYS_TO_HW))),
]
for phys, expected in cases:
    hw = _hw_height(phys)
    if hw == expected:
        ok(f"hw_height({phys}) = {hw}")
    else:
        fail(f"hw_height({phys}): expected {expected}, got {hw}")

# hw_height should stay within [0, 200] for all valid physical inputs
all_hw_in_range = all(0 <= _hw_height(h) <= 200
                      for h in range(POLE_MIN_MM, POLE_MAX_MM + 1))
if all_hw_in_range:
    ok("hw_height(h) ∈ [0, 200] for all h ∈ [POLE_MIN_MM, POLE_MAX_MM]")
else:
    fail("hw_height produced a value outside [0, 200] for some h")


# ═══════════════════════════════════════════════════════════════════════════════
# D  phys_height() conversions
# ═══════════════════════════════════════════════════════════════════════════════
section("D  phys_height() conversion (hardware units → physical mm)")

def _phys_height(hw_mm) -> float:
    return hw_mm * HW_TO_PHYS

cases_phys = [
    (0,   0.0),
    (100, 150.0),
    (200, 300.0),
    # After rounding in hw_height: 10mm → hw=7, phys_height(7)=10.5 (±0.5 mm ok)
    (int(round(10  * PHYS_TO_HW)), int(round(10  * PHYS_TO_HW)) * HW_TO_PHYS),
    (int(round(290 * PHYS_TO_HW)), int(round(290 * PHYS_TO_HW)) * HW_TO_PHYS),
]
for hw, expected in cases_phys:
    phys = _phys_height(hw)
    if abs(phys - expected) < 1e-9:
        ok(f"phys_height({hw}) = {phys:.1f} mm")
    else:
        fail(f"phys_height({hw}): expected {expected:.3f}, got {phys:.3f}")


# ═══════════════════════════════════════════════════════════════════════════════
# E  Trajectory helpers
# ═══════════════════════════════════════════════════════════════════════════════
section("E  Trajectory unit-conversion helpers")

def pos_m_to_mm(pos_m: float) -> int:
    return int(round(pos_m * 1000.0))

def vel_m_s_to_pct(vel_m_s: float) -> int:
    mm_s = vel_m_s * 1000.0
    pct  = int(round(mm_s))
    return max(1, min(100, pct))

traj_cases = [
    ("pos_m_to_mm(0.075)", pos_m_to_mm(0.075), 75),
    ("pos_m_to_mm(0.150)", pos_m_to_mm(0.150), 150),
    ("pos_m_to_mm(0.295)", pos_m_to_mm(0.295), 295),
    ("vel_m_s_to_pct(0.10)", vel_m_s_to_pct(0.10),  100),
    ("vel_m_s_to_pct(0.05)", vel_m_s_to_pct(0.05),   50),
    ("vel_m_s_to_pct(0.005)", vel_m_s_to_pct(0.005),   5),
    ("vel_m_s_to_pct(0.0)",  vel_m_s_to_pct(0.0),    1),   # clamped to min=1
    ("vel_m_s_to_pct(0.2)",  vel_m_s_to_pct(0.2),  100),   # clamped to max=100
]
for label, got, expected in traj_cases:
    if got == expected:
        ok(f"{label} = {got}")
    else:
        fail(f"{label}: expected {expected}, got {got}")

# All trajectory waypoint heights must survive hw_height (clamped to safe range)
# Use representative values from TRAJECTORY data (75–295 mm physical)
sample_heights = [75, 100, 150, 200, 250, 280, 295]
for h in sample_heights:
    clamped = max(POLE_MIN_MM, min(POLE_MAX_MM, h))
    hw = _hw_height(clamped)
    if 0 <= hw <= 200:
        ok(f"trajectory height {h} mm → hw={hw} (clamped to [{POLE_MIN_MM},{POLE_MAX_MM}])")
    else:
        fail(f"trajectory height {h} mm → hw={hw} out of [0, 200]")


# ═══════════════════════════════════════════════════════════════════════════════
# F  Mock-robot smoke tests
# ═══════════════════════════════════════════════════════════════════════════════
section("F  Smoke tests with MockRobot (no hardware)")

# ── Build a minimal mock of RoboticArm ─────────────────────────────────────

class _MockHandle:
    id = 1


class _MockRoboticArm:
    """Minimal mock that mimics RoboticArm for smoke-test purposes."""

    _FAKE_POS_HW = int(round(150 * PHYS_TO_HW))  # 100 in hw units → 150 mm phys

    def __init__(self, *args, **kwargs):
        self._handle = _MockHandle()

    def rm_create_robot_arm(self, ip, port, level):
        return self._handle

    def rm_delete_robot_arm(self):
        return 0

    @staticmethod
    def rm_destroy():
        return 0

    def rm_get_robot_info(self):
        return (0, {"arm_dof": 7, "arm_model": "RM75", "algorithm_model": "RM75"})

    def rm_get_lift_state(self):
        return (0, {
            "pos":      self._FAKE_POS_HW,
            "current":  120,
            "err_flag": 0,
            "mode":     0,
        })

    def rm_set_lift_height(self, speed: int, height_hw: int, block):
        # Simulate blocking with proportional sleep (capped at 0.01 s for speed)
        if block:
            time.sleep(0.005)
        return 0

    def rm_set_realtime_push(self, cfg):
        return 0

    def rm_set_arm_stop(self):
        return 0


def _make_rm_module():
    """Build a minimal fake Robotic_Arm module tree that matches real imports."""
    ra_mod = types.ModuleType("Robotic_Arm")
    ri_mod = types.ModuleType("Robotic_Arm.rm_robot_interface")
    ri_mod.RoboticArm = _MockRoboticArm

    cw_mod = types.ModuleType("Robotic_Arm.rm_ctypes_wrap")

    class _Enum:
        def __init__(self, v=0): self.value = v
        def __int__(self): return self.value

    class rm_thread_mode_e(_Enum): pass
    class rm_realtime_push_config_t:
        def __init__(self, **kw): pass
    class rm_udp_custom_config_t:
        def __init__(self, **kw): pass
    class rm_event_push_data_t:
        event_type = 0; device = 0
        trajectory_state = False; trajectory_connect = 0; program_id = 0
    class rm_event_callback_ptr:
        def __init__(self, fn): self._fn = fn
        def __call__(self, *a): return self._fn(*a)
    class rm_get_arm_event_call_back:
        def __init__(self, cb): pass
    class rm_realtime_arm_state_callback_ptr:
        def __init__(self, fn): self._fn = fn
        def __call__(self, *a): return self._fn(*a)
    class rm_realtime_arm_state_call_back:
        def __init__(self, cb): pass
    class rm_realtime_arm_joint_state_t: pass

    for name, obj in [
        ("rm_thread_mode_e", rm_thread_mode_e),
        ("rm_realtime_push_config_t", rm_realtime_push_config_t),
        ("rm_udp_custom_config_t", rm_udp_custom_config_t),
        ("rm_event_push_data_t", rm_event_push_data_t),
        ("rm_event_callback_ptr", rm_event_callback_ptr),
        ("rm_get_arm_event_call_back", rm_get_arm_event_call_back),
        ("rm_realtime_arm_state_callback_ptr", rm_realtime_arm_state_callback_ptr),
        ("rm_realtime_arm_state_call_back", rm_realtime_arm_state_call_back),
        ("rm_realtime_arm_joint_state_t", rm_realtime_arm_joint_state_t),
    ]:
        setattr(cw_mod, name, obj)

    ra_mod.rm_robot_interface = ri_mod
    ra_mod.rm_ctypes_wrap     = cw_mod
    sys.modules["Robotic_Arm"]                       = ra_mod
    sys.modules["Robotic_Arm.rm_robot_interface"]    = ri_mod
    sys.modules["Robotic_Arm.rm_ctypes_wrap"]        = cw_mod
    return ra_mod


_make_rm_module()

# ── Verify MockRobot API directly ──────────────────────────────────────────
print("\n  F.0  Direct MockRobot API checks")
bot = _MockRoboticArm()
h   = bot.rm_create_robot_arm("192.168.1.10", 8080, 3)
if h is not None and h.id == 1:
    ok("MockRobot: rm_create_robot_arm returns handle.id=1")
else:
    fail("MockRobot: rm_create_robot_arm returned bad handle")

ret, info = bot.rm_get_robot_info()
if ret == 0 and info.get("arm_dof") == 7:
    ok("MockRobot: rm_get_robot_info arm_dof=7")
else:
    fail(f"MockRobot: rm_get_robot_info ret={ret} info={info}")

ret, state = bot.rm_get_lift_state()
if ret == 0 and "pos" in state and state["err_flag"] == 0:
    pos_hw   = state["pos"]
    pos_phys = pos_hw * HW_TO_PHYS
    ok(f"MockRobot: rm_get_lift_state pos_hw={pos_hw} → phys={pos_phys:.1f} mm")
else:
    fail(f"MockRobot: rm_get_lift_state ret={ret}")

for ht in [_hw_height(80), _hw_height(150), _hw_height(220)]:
    r = bot.rm_set_lift_height(50, ht, False)
    if r == 0:
        ok(f"MockRobot: rm_set_lift_height(50, hw={ht}, block=False) = 0")
    else:
        fail(f"MockRobot: rm_set_lift_height(50, hw={ht}) returned {r}")

# ── Import and smoke-run each test module ──────────────────────────────────
_SRC = str(pathlib.Path(__file__).resolve().parent)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

TEST_MODULES = [
    "test_lift_connect",
    "test_lift_nonblocking_rate",
    "test_lift_trajectory_rate",
    "test_lift_udp_feedback",
    "test_lift_blocking_vs_nonblocking",
    "test_lift_speed_param",
]

for mod_name in TEST_MODULES:
    print(f"\n  F.{TEST_MODULES.index(mod_name)+1}  Smoke-import: {mod_name}")
    try:
        # Remove cached version so re-import works after mock injection
        if mod_name in sys.modules:
            del sys.modules[mod_name]
        mod = importlib.import_module(mod_name)
        # Basic sanity: scale-factor constants present
        if (hasattr(mod, "PHYS_TO_HW")
                and hasattr(mod, "HW_TO_PHYS")
                and hasattr(mod, "hw_height")
                and hasattr(mod, "phys_height")):
            ok(f"{mod_name}: has PHYS_TO_HW, HW_TO_PHYS, hw_height, phys_height")
        else:
            fail(f"{mod_name}: missing scale-factor symbols")

        # Spot-check hw_height inside the module matches expected values
        if hasattr(mod, "hw_height"):
            hw150 = mod.hw_height(150)
            if hw150 == 100:
                ok(f"{mod_name}.hw_height(150) = 100 ✓")
            else:
                fail(f"{mod_name}.hw_height(150): expected 100, got {hw150}")

            ph100 = mod.phys_height(100)
            if ph100 == 150.0:
                ok(f"{mod_name}.phys_height(100) = 150.0 ✓")
            else:
                fail(f"{mod_name}.phys_height(100): expected 150.0, got {ph100}")

    except Exception as exc:
        fail(f"{mod_name}: import/check raised {type(exc).__name__}: {exc}")


# ═══════════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 64)
total = _pass + _fail
print(f"  Dry-run complete:  {_pass}/{total} passed  "
      f"({_fail} failed)")
print("=" * 64)

sys.exit(0 if _fail == 0 else 1)
