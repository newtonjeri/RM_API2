#!/usr/bin/env python3
"""Do the two tools produce the SAME motion? Prove it, don't assume it.

`RMDemo_CleaningMotion` and `RMDemo_PointSequence` read formats that are
deliberately NOT interchangeable — one states a start pose plus DELTAS, the
other states ABSOLUTE arm-world poses — but when both describe the same
motion they must dispatch the same TCP path (Newton, 2026-08-22). That
equivalence is the check: if the delta resolver has the quaternion order,
the composition order, the frame transform or the traversal wrong, the two
stop agreeing and this says so.

It cannot be faked by the resolver being self-consistent, because the two
programs share no code on this path: the cleaning config goes through
`cm_config` -> `cm_frames` -> `cm_arm`, and the points file goes through
`point_sequence`'s own loader and dispatcher.

Both folders hold the SAME config format, pasted by hand into each. This
runs both programs over their own copy and diffs every dispatched call: the
movej_p target, and every movel's pose and its (v, r, connect). So it checks
two things at once — that the two resolvers agree, and that the two copies
are actually the same motion.

USAGE
    python3 verify_equivalence.py
    python3 verify_equivalence.py --config ../motions/x.yaml --points ../../RMDemo_PointSequence/points/y.yaml
"""

import json
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
BASE = HERE.parent.parent                      # .../RMDemo_Python
DEFAULT_CONFIG = HERE.parent / "motions" / "example_cleaning_config.yaml"
POINTS = BASE / "RMDemo_PointSequence" / "points" / "example_points.yaml"
PS_SRC = BASE / "RMDemo_PointSequence" / "src" / "point_sequence.py"

# Speeds must be matched for the comparison to mean anything: this bed has a
# separate `transit_v` for the movej_p while point_sequence applies one `v`
# to every move, so an unmatched run differs on the entry SPEED while the
# PATH is identical. That is a real difference in the tools, not a defect,
# and it is removed here rather than hidden.
V = "100"
BLEND = "10"
CONNECT = "1"


# Each program runs in its OWN PROCESS. They cannot share one: point_sequence
# needs a stubbed SDK in `sys.modules`, while run_cleaning_motion needs the
# emulator to install the real module tree there, and whichever loads second
# finds the other's modules already in place. Two processes make the
# isolation structural instead of a loading-order rule to remember.

_PS_HARNESS = """
import json, runpy, sys, types
calls = []
mod = types.ModuleType("Robotic_Arm.rm_robot_interface")
class _H: id = 7
class RoboticArm:
    def __init__(self, *a): pass
    def rm_create_robot_arm(self, *a): return _H()
    def rm_get_robot_info(self): return 0, {"arm_model": "RM_75"}
    def rm_delete_robot_arm(self): return 0
    def rm_movej(self, j, v, r, c, b): calls.append(["movej", None, v, r, c]); return 0
    def rm_movej_p(self, p, v, r, c, b):
        calls.append(["movej_p", [round(float(x), 9) for x in p], v, r, c]); return 0
    def rm_movel(self, p, v, r, c, b):
        calls.append(["movel", [round(float(x), 9) for x in p], v, r, c]); return 0
mod.RoboticArm = RoboticArm
mod.rm_thread_mode_e = lambda m: m
mod.rm_api_version = lambda: "stub"
pkg = types.ModuleType("Robotic_Arm"); pkg.rm_robot_interface = mod
sys.modules["Robotic_Arm"] = pkg
sys.modules["Robotic_Arm.rm_robot_interface"] = mod
SCRIPT, OUT, POINTS, V, BLEND, CONNECT = sys.argv[1:7]
sys.path.insert(0, __import__("os").path.dirname(SCRIPT))
sys.argv = ["point_sequence.py", "--points", POINTS,
            "--v", V, "--blend", BLEND, "--connect", CONNECT]
try: runpy.run_path(SCRIPT, run_name="__main__")
except SystemExit: pass
open(OUT, "w").write(json.dumps(calls))
"""

_CM_HARNESS = """
import json, runpy, sys, os
CONCEPT, SCRIPT, OUT, CONFIG, V, BLEND, CONNECT = sys.argv[1:8]
sys.path.insert(0, CONCEPT)
import rm_emulator; rm_emulator.install()
from Robotic_Arm.rm_robot_interface import RoboticArm
calls = []
for name in ("rm_movej_p", "rm_movel"):
    original = getattr(RoboticArm, name)
    def wrap(tag, fn):
        def inner(self, p, v, r, c, b, *a, **k):
            calls.append([tag, [round(float(x), 9) for x in p], v, r, c])
            return fn(self, p, v, r, c, b, *a, **k)
        return inner
    setattr(RoboticArm, name, wrap(name[3:], original))
sys.path.insert(0, os.path.dirname(SCRIPT))
sys.argv = ["run_cleaning_motion.py", "--motion", CONFIG, "--mode", "REAL",
            "--blend", BLEND, "--connect", CONNECT, "--cleaning-v", V,
            "--transit-v", V, "--no-record"]
try: runpy.run_path(SCRIPT, run_name="__main__")
except SystemExit: pass
open(OUT, "w").write(json.dumps(calls))
"""


def _run(harness, args, label):
    with tempfile.NamedTemporaryFile("r", suffix=".json", delete=False) as fh:
        out = fh.name
    # OUT is threaded in as the argument after the script path, which is
    # why each harness reads its argv in that order.
    n = 2 if harness is _CM_HARNESS else 1
    cmd = ([sys.executable, "-c", harness] + [str(a) for a in args[:n]]
           + [out] + [str(a) for a in args[n:]])
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=str(HERE))
    try:
        calls = json.loads(pathlib.Path(out).read_text())
    except Exception:
        print((p.stdout or "") + (p.stderr or ""))
        raise SystemExit("%s produced no capture — see the output above."
                         % label)
    return [(c[0], tuple(c[1]) if c[1] else None, c[2], c[3], c[4])
            for c in calls]


def run_point_sequence(points=POINTS):
    return _run(_PS_HARNESS, [PS_SRC, points, V, BLEND, CONNECT],
                "point_sequence")


def run_cleaning_motion(config_path):
    return _run(_CM_HARNESS,
                [BASE / "RMDemo_DualArmConcept" / "src",
                 HERE / "run_cleaning_motion.py", config_path,
                 V, BLEND, CONNECT],
                "run_cleaning_motion")


def compare(tag, A, B):
    if len(A) != len(B):
        print("  %-9s COUNT DIFFERS: %d vs %d" % (tag, len(A), len(B)))
        return False
    worst, bad = 0.0, 0
    for a, b in zip(A, B):
        worst = max(worst, max(abs(x - y) for x, y in zip(a[1], b[1])))
        if a[2:] != b[2:]:
            bad += 1
            if bad <= 3:
                print("  %-9s param mismatch: %s vs %s" % (tag, a[2:], b[2:]))
    print("  %-9s %d moves, max |pose difference| %.3e, params differing %d"
          % (tag, len(A), worst, bad))
    return worst < 1e-9 and bad == 0


def main():
    argv = sys.argv[1:]
    if "-h" in argv or "--help" in argv:
        print(__doc__.strip())
        return 0
    config = pathlib.Path(
        argv[argv.index("--config") + 1] if "--config" in argv
        else DEFAULT_CONFIG)
    points = pathlib.Path(
        argv[argv.index("--points") + 1] if "--points" in argv else POINTS)
    for f in (config, points):
        if not f.is_file():
            raise SystemExit("no such file: %s" % f)

    b = run_point_sequence(points)
    a = run_cleaning_motion(config)

    print("\n" + "=" * 70)
    print("  run_cleaning_motion  vs  point_sequence")
    print("  config : %s" % config.name)
    print("  points : %s" % points.name)
    print("-" * 70)
    ok = compare("movej_p", [c for c in a if c[0] == "movej_p"],
                 [c for c in b if c[0] == "movej_p"])
    ok = compare("movel", [c for c in a if c[0] == "movel"],
                 [c for c in b if c[0] == "movel"]) and ok
    print("=" * 70)
    if ok:
        print("SAME MOTION — identical TCP path and identical per-move "
              "(v, r, connect).")
        print("Both tools would trace the same path on the controller.")
    else:
        print("THEY DIFFER — the two tools would NOT trace the same path.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
