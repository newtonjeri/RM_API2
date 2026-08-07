"""Run the full dual-arm concept suite against the in-process emulator.

No hardware, no network: rm_emulator installs itself as the Robotic_Arm SDK
before the test modules import it, then every test runs UNMODIFIED — same
dispatch code, same arrival-event demux, same mode runners — against
emulated arms with realistic RM75/lift timing.

    RM_EMU_TIME_SCALE=10 python3 run_emulated_suite.py   (default scale 10)

Set RM_EMU_TIME_SCALE=1 for real-time motion durations.
"""

import os
import sys

import os
os.environ.setdefault("RM_HAND_DWELL_S", "0.1")
# our canfd stream is real wall time; the emulator only scales
# its own motion durations, so shrink the stream for the suite
os.environ.setdefault("RM_CANFD_ARM_S", "0.2")
import rm_emulator

_SCALE = float(os.environ.get("RM_EMU_TIME_SCALE", "10"))
rm_emulator.set_time_scale(_SCALE)
rm_emulator.install()                      # BEFORE any test import

from log_utils import setup_log
setup_log(__file__)

import test_dual_connect
import test_sim_motion_visibility
import test_hand_only
import test_dual_locked
import test_dual_chained
import test_dual_free
import test_single_arm_planned
import test_single_arm_locked
import test_pole_only

SUITE = [
    ("C1 connect", test_dual_connect),
    ("C8 pole diag", test_pole_only),
    ("C5 sim probe", test_sim_motion_visibility),
    ("C7 hand only", test_hand_only),
    ("C2 locked", test_dual_locked),
    ("C9 single-locked", test_single_arm_locked),
    ("C3 chained", test_dual_chained),
    ("C4 free", test_dual_free),
    ("C6 single-arm", test_single_arm_planned),
]


def _flag_drill() -> int:
    """Arm-only C2 run: --no-hands --no-pole must strip hand and lift
    parts cleanly (sync steps run as plain arm moves) and pass."""
    argv0 = list(sys.argv)
    sys.argv = [argv0[0], "--no-hands", "--no-pole"]
    try:
        return test_dual_locked.main()
    finally:
        sys.argv = argv0


def _sync_order_drill() -> int:
    """PLANNED backend: the dispatch order decides WHICH device is truncated.

    C16 established the truncation is bidirectional, so under the planned
    backend BOTH orders fail — what differs is the victim, and only one
    victim damages the controller:

      arm_first  -> the ARM is truncated -> joint errors LATCH (recovery
                    needed before anything else can run)
      lift_first -> the POLE is truncated -> the step fails but the
                    controller stays CLEAN

    That asymmetry is why lift_first is the shipped order: if the planned
    path must be used at all, the pole is the safe thing to sacrifice."""
    import dual_arm_common as dac
    import test_single_arm_locked
    argv0 = list(sys.argv)
    saved_order, saved_timeout = dac.SYNC_ORDER, dac.ARM_TIMEOUT_S
    saved_backend = dac.SYNC_BACKEND
    dac.ARM_TIMEOUT_S = 2.0            # stand-in for the real 40 s wait
    dac.SYNC_BACKEND = "planned"       # this drill is about the planned path
    ctrl = rm_emulator.emu_controller(dac.LEFT_IP)
    try:
        sys.argv = [argv0[0], "--no-hands", "--clear-errors"]
        dac.SYNC_ORDER = "arm_first"
        print("\n-- arm_first: the ARM is the victim -> faults must LATCH --")
        code_arm = test_single_arm_locked.main()
        latched = any(ctrl.joint_err_flags) and ctrl.motion_locked
        print(f"  joint errors latched: {latched} ({ctrl.joint_err_flags})")

        dac.SYNC_ORDER = "lift_first"
        print("\n-- lift_first: the POLE is the victim -> no faults --")
        code_lift = test_single_arm_locked.main()
        clean = not any(ctrl.joint_err_flags) and not ctrl.motion_locked
        print(f"  controller clean afterwards: {clean}")
    finally:
        sys.argv = argv0
        dac.SYNC_ORDER, dac.ARM_TIMEOUT_S = saved_order, saved_timeout
        dac.SYNC_BACKEND = saved_backend
    ok = code_arm != 0 and latched and code_lift != 0 and clean
    print(f"\n  drill verdict: arm_first exit {code_arm} + latched {latched} "
          f"(want nonzero/True), lift_first exit {code_lift} + clean {clean} "
          f"(want nonzero/True) -> {'OK' if ok else 'FAIL'}")
    return 0 if ok else 1


def _sync_backend_drill() -> int:
    """Both sync backends must behave as documented, in one process.

    planned : the arm half is rm_movej -> the controller truncation the
              emulator models -> the sync step FAILS (kept as the reference
              case while RealMan support reviews it).
    canfd   : the arm half is streamed -> no truncation -> it COMPLETES.
    Same test, same sequence; the backend is the only variable."""
    import dual_arm_common as dac
    import test_single_arm_locked
    argv0, saved = list(sys.argv), dac.SYNC_BACKEND
    saved_timeout = dac.ARM_TIMEOUT_S
    dac.ARM_TIMEOUT_S = 2.0
    sys.argv = [argv0[0], "--no-hands", "--clear-errors"]
    try:
        dac.SYNC_BACKEND = "planned"
        print("\n-- backend=planned: sync must FAIL (known controller limit) --")
        code_planned = test_single_arm_locked.main()
        dac.SYNC_BACKEND = "canfd"
        print("\n-- backend=canfd: sync must COMPLETE (bench_sync recipe) --")
        code_canfd = test_single_arm_locked.main()
    finally:
        sys.argv = argv0
        dac.SYNC_BACKEND, dac.ARM_TIMEOUT_S = saved, saved_timeout
    ok = code_planned != 0 and code_canfd == 0
    print(f"\n  drill verdict: planned exit {code_planned} (want nonzero), "
          f"canfd exit {code_canfd} (want 0) -> {'OK' if ok else 'FAIL'}")
    return 0 if ok else 1


def _locked_pole_drill() -> int:
    """Reproduce the 2026-08-06 20:38 lift-rejection state on the emulated
    LEFT arm, expect C8 to FAIL with the diagnosis, then expect a
    --clear-errors run to recover and go green."""
    import dual_arm_common as dac
    ctrl = rm_emulator.emu_controller(dac.LEFT_IP)
    ctrl.lift_locked = True
    ctrl.sys_err_code = 4103
    print("\n-- drill A: pole locked, C8 must FAIL with the diagnosis --")
    code_locked = test_pole_only.main()
    print("\n-- drill B: C8 --clear-errors must recover and pass --")
    argv0 = list(sys.argv)
    sys.argv = [argv0[0], "--clear-errors"]
    try:
        code_recover = test_pole_only.main()
    finally:
        sys.argv = argv0
    ok = code_locked != 0 and code_recover == 0
    print(f"\n  drill verdict: locked exit {code_locked} (want nonzero), "
          f"recover exit {code_recover} (want 0) -> "
          f"{'OK' if ok else 'FAIL'}")
    return 0 if ok else 1


def main() -> int:
    import dual_arm_common as _dac
    _dac.handle_cli(__doc__)
    print("=" * 68)
    print(f"EMULATED dual-arm concept suite  (time scale x{_SCALE:g})")
    print("Arms are emulated in-process — nothing physical moves.")
    print("=" * 68)

    codes = {}
    for name, mod in SUITE:
        print(f"\n{'#' * 68}\n# {name}  ({mod.__name__})\n{'#' * 68}")
        codes[name] = mod.main()

    print(f"\n{'#' * 68}\n# C2 arm-only drill  (--no-hands --no-pole)\n"
          f"{'#' * 68}")
    codes["C2 arm-only drill"] = _flag_drill()

    print(f"\n{'#' * 68}\n# C9 sync-order drill  (planned: which device gets truncated)\n"
          f"{'#' * 68}")
    codes["C9 sync-order drill"] = _sync_order_drill()

    print(f"\n{'#' * 68}\n# C9 sync-BACKEND drill  (planned vs canfd)\n"
          f"{'#' * 68}")
    codes["C9 backend drill"] = _sync_backend_drill()

    print(f"\n{'#' * 68}\n# C8 locked-pole drill  (fault injection)\n"
          f"{'#' * 68}")
    codes["C8 locked drill"] = _locked_pole_drill()

    print("\n" + "=" * 68)
    print("Emulated suite summary")
    for name, code in codes.items():
        print(f"  {name:12s} exit {code}  ({'OK' if code == 0 else 'FAIL'})")
    print("=" * 68)
    return max(codes.values())


if __name__ == "__main__":
    sys.exit(main())
