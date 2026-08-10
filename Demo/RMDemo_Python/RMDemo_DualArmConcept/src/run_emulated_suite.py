"""Run the full dual-arm concept suite against the in-process emulator.

No hardware, no network: rm_emulator installs itself as the Robotic_Arm SDK
before the test modules import it, then every test runs UNMODIFIED — same
dispatch code, same arrival-event demux, same mode runners — against
emulated arms with realistic RM75/lift timing.

    RM_EMU_TIME_SCALE=10 python3 run_emulated_suite.py   (default scale 10)

Set RM_EMU_TIME_SCALE=1 for real-time motion durations.
"""

import os
import pathlib
import sys
import tempfile

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
import test_rehearsal_validate
import test_frame_alignment
import stage_runner

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

# ── Phase 2: the frames must exist before any cleaning task can select
#    them, exactly as on hardware (C14 then the task). Order matters.
PHASE2_TASKS = ("hinge_area_right", "hinge_area_left", "toplid_right",
                "toplid_left")


def _caps_drill() -> int:
    """Capabilities must ROUND-TRIP against the emulated SDK, not just a mock.

    The dry run exercises controller_caps against a hand-written mock, so a
    stale emulator can disagree with the real arms and nothing notices. It
    did: a V1.7.1-era block of hard-coded getters sat AFTER the stateful
    ones and silently overrode them — avoid_singularity always read 0, so a
    setter would appear to work while the readback said OFF forever.
    """
    import controller_caps
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    bad = []
    for ip in (rm_emulator.EMU_LEFT_IP, rm_emulator.EMU_RIGHT_IP):
        r = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        h = r.rm_create_robot_arm(ip, 8080, 3)
        if h is None or h.id <= 0:
            bad.append(f"{ip}: no handle"); continue
        found = controller_caps.read(r)
        # Every capability must be READABLE, and the stage must match the
        # real probe. NOT asserted: the on/off values as found — earlier
        # suite entries legitimately enable self-collision, so pinning the
        # initial state would couple this drill to suite ordering. The
        # property that matters is the round trip.
        for key in ("avoid_singularity", "self_collision",
                    "endeff_collision", "collision_stage",
                    "static_collision", "collision_remove"):
            if isinstance(found.get(key), str):
                bad.append(f"{ip}: {key} unreadable -> {found.get(key)!r}")
        if not isinstance(found.get("collision_stage"), str) and \
                int(found["collision_stage"]) != 4:
            bad.append(f"{ip}: collision_stage={found['collision_stage']} "
                       "want 4 (the V1.7.4 probe read 4 on both arms)")
        # flip BOTH away from whatever they are, then restore to as-found
        want_sing = 0 if int(found["avoid_singularity"]) else 1
        want_self = not bool(found["self_collision"])
        prev = controller_caps.apply(r, avoid_singularity=want_sing,
                                     self_collision=want_self)
        after = controller_caps.read(r)
        if int(after["avoid_singularity"]) != want_sing or \
                bool(after["self_collision"]) != want_self:
            bad.append(f"{ip}: apply() did not take -> {after}")
        controller_caps.restore(r, prev)
        back = controller_caps.read(r)
        if int(back["avoid_singularity"]) != int(found["avoid_singularity"]) \
                or bool(back["self_collision"]) != bool(found["self_collision"]):
            bad.append(f"{ip}: restore() did not return to as-found "
                       f"{found} -> {back}")
        r.rm_delete_robot_arm()
    if bad:
        print("  capability drill FAILURES:")
        for b in bad:
            print(f"    {b}")
        return 1
    print("  capabilities read/apply/restore round-trip on both arms -> OK")
    return 0


def _c11_rehearsal() -> int:
    """C11 under emulation, writing to a SCRATCH path.

    The default save path is `src/rehearsal_<side>.json`, which holds the
    real hardware capture — an emulated run overwrote it once. Emulated
    output never lands where recorded hardware data lives.
    """
    argv0 = list(sys.argv)
    sys.argv = [argv0[0], "--save",
                str(pathlib.Path(tempfile.gettempdir()) / "emu_rehearsal.json")]
    try:
        return test_rehearsal_validate.main()
    finally:
        sys.argv = argv0


def _c14_frames() -> int:
    """C14 on both arms: write the glove/ik frames and verify the readback."""
    rc = 0
    for side in ("right", "left"):
        os.environ["RM_ARM"] = side
        test_frame_alignment.ARM_SIDE = side
        argv0 = list(sys.argv)
        sys.argv = [argv0[0], "--create-frames"]
        try:
            rc |= test_frame_alignment.main()
        finally:
            sys.argv = argv0
    return rc


def _phase2_task(task):
    """One cleaning task end to end: stages serialized, chained movel."""
    def run() -> int:
        argv0 = list(sys.argv)
        sys.argv = [argv0[0], "--task", task]
        try:
            return stage_runner.main()
        finally:
            sys.argv = argv0
    return run


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

    print(f"\n{'#' * 68}\n# C11 rehearsal  (scratch capture)\n{'#' * 68}")
    codes["C11 rehearsal"] = _c11_rehearsal()

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

    # ── Phase 2, in the order hardware requires: frames, then tasks ──
    print(f"\n{'#' * 68}\n# capability drill  (read/apply/restore)\n"
          f"{'#' * 68}")
    codes["capability drill"] = _caps_drill()

    print(f"\n{'#' * 68}\n# C14 glove frames  (both arms)\n{'#' * 68}")
    codes["C14 frames"] = _c14_frames()
    for task in PHASE2_TASKS:
        print(f"\n{'#' * 68}\n# P2 {task}  (serialized stages + chained "
              f"movel)\n{'#' * 68}")
        codes[f"P2 {task}"] = _phase2_task(task)()

    print("\n" + "=" * 68)
    print("Emulated suite summary")
    for name, code in codes.items():
        print(f"  {name:22s} exit {code}  ({'OK' if code == 0 else 'FAIL'})")
    print("=" * 68)
    return max(codes.values())


if __name__ == "__main__":
    sys.exit(main())
