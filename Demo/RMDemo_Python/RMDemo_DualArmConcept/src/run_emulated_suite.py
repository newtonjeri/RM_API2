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

import rm_emulator

_SCALE = float(os.environ.get("RM_EMU_TIME_SCALE", "10"))
rm_emulator.set_time_scale(_SCALE)
rm_emulator.install()                      # BEFORE any test import

from log_utils import setup_log
setup_log(__file__)

import test_dual_connect
import test_sim_motion_visibility
import test_dual_locked
import test_dual_chained
import test_dual_free
import test_single_arm_planned

SUITE = [
    ("C1 connect", test_dual_connect),
    ("C5 sim probe", test_sim_motion_visibility),
    ("C2 locked", test_dual_locked),
    ("C3 chained", test_dual_chained),
    ("C4 free", test_dual_free),
    ("C6 single-arm", test_single_arm_planned),
]


def main() -> int:
    print("=" * 68)
    print(f"EMULATED dual-arm concept suite  (time scale x{_SCALE:g})")
    print("Arms are emulated in-process — nothing physical moves.")
    print("=" * 68)

    codes = {}
    for name, mod in SUITE:
        print(f"\n{'#' * 68}\n# {name}  ({mod.__name__})\n{'#' * 68}")
        codes[name] = mod.main()

    print("\n" + "=" * 68)
    print("Emulated suite summary")
    for name, code in codes.items():
        print(f"  {name:12s} exit {code}  ({'OK' if code == 0 else 'FAIL'})")
    print("=" * 68)
    return max(codes.values())


if __name__ == "__main__":
    sys.exit(main())
