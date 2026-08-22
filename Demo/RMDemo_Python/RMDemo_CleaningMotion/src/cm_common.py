"""Bridge to RMDemo_DualArmConcept — THIS FOLDER FORKS NOTHING.

Everything the cleaning-motion tests need already exists next door and is
hardware-calibrated: the arrival-event semantics, the acc >= 3 x speed
rejection rule, the 15.3 mm Arm_Tip -> ConnectorLink offset, the Ry(+90)
mount, the error-code tables, the recorder. Copying any of it here would
start drifting the day the next measurement lands, and the drift would be
silent — two files that disagree about a calibration both look correct.

So this module does one thing: it puts `../../RMDemo_DualArmConcept/src` on
`sys.path` and re-exports the pieces, so every other file here can say

    from cm_common import handle_cli, ConceptArm, speed_limits

and get the ONE implementation. If a behaviour needs to change, it changes
next door and both trees get it.

WHAT IS GENUINELY NEW HERE, and therefore lives in this folder:
  * `cm_loader`  — reading a cleaning-motion file into a MotionProgram
  * `cm_frames`  — source frame -> arm base with the POLE PINNED AT MINIMUM
  * `cm_speed`   — speed selection from CLI arguments
  * `run_cleaning_motion` — the driver

THIS IS AN ARM-ONLY TEST BED (Newton, 2026-08-22). No pole motion, no hand
motion, ever. The pole is not merely "not commanded" — it is ASSUMED to be
at its minimum and every pose is resolved against that assumption, because
the pole carries the arm base and so its height is baked into the frame
transform (see `cm_frames`). `assert_arm_only()` refuses the flags that
would break that assumption rather than letting a run produce numbers that
silently belong to a different geometry.
"""

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent                                  # RMDemo_CleaningMotion/
CONCEPT = ROOT.parent / "RMDemo_DualArmConcept"
CONCEPT_SRC = CONCEPT / "src"

if not CONCEPT_SRC.is_dir():
    raise SystemExit(
        "cannot find RMDemo_DualArmConcept/src next to this folder.\n"
        "  expected: %s\n"
        "This test bed deliberately reuses that tree rather than copying it; "
        "without it there is no arrival monitor, no speed-limit rule and no "
        "emulator, and nothing here can run." % CONCEPT_SRC)

# Ahead of anything else on the path: these names (`speed_limits`,
# `error_codes`, ...) are generic enough to collide with something installed.
if str(CONCEPT_SRC) not in sys.path:
    sys.path.insert(0, str(CONCEPT_SRC))

# ── re-exports: the ONE implementation of each, imported not copied ──────
from dual_arm_common import (                                    # noqa: E402
    handle_cli, parse_mode_arg, mode_label,
    apply_run_mode, restore_run_modes, report_run_modes,
    ArrivalMonitor, ConceptArm, preflight_error_gate, countdown, teardown,
    state_deg, host_ip_for,
    DEV_JOINT, LEFT_IP, RIGHT_IP, ROBOT_PORT, UDP_PORT,
    LIFT_MIN_M, LIFT_M, ARM_SPEED_PCT,
)
from run_recorder import RunRecorder                             # noqa: E402,F401
import speed_limits                                              # noqa: E402,F401
import error_codes                                               # noqa: E402,F401
import log_utils                                                 # noqa: E402,F401

# ── the arm-only invariants ─────────────────────────────────────────────
POLE_M = LIFT_MIN_M
"""Pole height every motion in this folder is resolved against [m].

Taken from `dual_arm_common.LIFT_MIN_M` rather than written as 0.005 here,
so that if the minimum is ever re-measured this tree follows automatically.
"""

# THE ENTRY SEQUENCE IS NOT PARAMETERISED HERE, ON PURPOSE.
#
# rest -> prestart -> approach, its transit speed and its 50 mm world-+Z hop
# all belong to `test_blend_corner.goto_start_sequence`, which this bed
# calls directly. Re-declaring TRANSIT_V / PRESTART_LIFT_M here would create
# two numbers for one behaviour: the first draft of this file did exactly
# that, said 20 %, and the emulator printed "all at 50 %" — the constant was
# never read and quietly disagreed with the run. Entries here are the same
# entries as next door BECAUSE they are the same code, which is also what
# makes a recording from this bed comparable with one from that one.


def assert_arm_only(argv=None):
    """Refuse the flags that would break the arm-only / minimum-pole premise.

    `--no-pole` and `--no-hands` are MEANINGLESS here, not merely redundant:
    this bed never commands either device, so accepting the flags would
    imply there was something to disable and invite the reading that
    without them the pole DOES move. And a flag that asks to move the pole
    invalidates every pose in the program, because the transform that
    produced them assumed minimum.
    """
    args = list(sys.argv[1:] if argv is None else argv)
    for bad, why in (
        ("--no-pole", "the pole is never commanded here; it is ASSUMED at "
                      "minimum and baked into the frame transform"),
        ("--no-hands", "the hand is never commanded here"),
        ("--pole", "the pole height is not selectable — every pose is "
                   "resolved at the minimum (%.3f m)" % POLE_M),
    ):
        if bad in args:
            raise SystemExit(
                "%s is not accepted by this test bed: %s.\n"
                "  This is an ARM-ONLY bed (Newton, 2026-08-22). If you need "
                "a different pole height, the motion must be re-resolved at "
                "that height — see cm_frames.to_arm_base()." % (bad, why))


def concept_path(*parts) -> pathlib.Path:
    """A path inside RMDemo_DualArmConcept (paths/, task_configs/, ...)."""
    return CONCEPT.joinpath(*parts)


def runs_dir() -> pathlib.Path:
    d = ROOT / "runs"
    d.mkdir(exist_ok=True)
    return d


def motions_dir() -> pathlib.Path:
    return ROOT / "motions"
