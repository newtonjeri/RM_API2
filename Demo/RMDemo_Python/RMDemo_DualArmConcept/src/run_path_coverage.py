"""C12b — How faithfully does a SPARSE chain reproduce the cleaning path?

C12 answers "is the controller's motion collision-free?". That is not the
same question as "does it still clean the surface", and the hinge data
showed the two can disagree loudly: a 20-target chain was collision-safe
while departing the planned path by 58 mm.

This tool answers the coverage question, for both candidate commands:

  movej chain   the controller interpolates JOINT-linearly between targets,
                so the tool traces a CURVE through Cartesian space. Modelled
                exactly: FK of the joint-linear interpolation.
  movel chain   the controller moves the TOOL in a straight Cartesian line
                between targets. Modelled exactly too, and without IK: the
                tool path IS the straight segment between the two tool
                positions (that is what movel guarantees).

Reference is MoveIt's own dense path — the surface-following trajectory the
task actually wants. For each candidate we report how far the dense path
strays from the commanded chain, i.e. how much of the intended wipe is not
being followed.

The output picks the target count each task needs, per command, for a given
tolerance. That number is the input to the Phase-2 task builder.

Usage:
  python3 run_path_coverage.py [--task NAME] [--tol MM]
  python3 run_path_coverage.py --all
"""

import sys

import numpy as np

from dual_arm_common import handle_cli
from segment_verifier import (
    SegmentVerifier, arm_stages, load_plan, resolve_plan, stage_maps,
    subsample)

USAGE = ("Usage: python3 run_path_coverage.py [--task NAME] [--tol MM] "
         "[-h|--help]\n"
         "       python3 run_path_coverage.py --all\n"
         "  --task NAME   one of: " + ", ".join(
             ("hinge_area_left", "hinge_area_right",
              "toplid_left", "toplid_right")) + "\n"
         "                (one task per run; --all sweeps every task)\n"
         "  --tol MM      chain-vs-plan tolerance (default 5)\n"
         "  --all         analyse every bundled task\n"
         "  -h, --help    show this documentation and exit")

# All FOUR bundled tasks. hinge_area_left was missing here, so `--all`
# silently swept 3 of 4 and reported a complete-looking table.
TASKS = ("hinge_area_left", "hinge_area_right", "toplid_left",
         "toplid_right")
COUNTS = (10, 20, 40, 80, 160, 320)
DENSE_STRIDE = 10          # sample the reference every Nth waypoint


def _arg(flag, default):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def _side(plan):
    for st in plan["sub_trajectories"]:
        for j in st["joint_names"]:
            if j.startswith("R_joint"):
                return "right"
            if j.startswith("L_joint"):
                return "left"
    raise SystemExit("no arm joints in plan")


def _seg_dists(points, a, b):
    """Distance from each point to segment a->b (vectorized)."""
    ab = b - a
    den = float(ab @ ab)
    if den <= 1e-15:
        return np.linalg.norm(points - a, axis=1)
    t = np.clip((points - a) @ ab / den, 0.0, 1.0)[:, None]
    return np.linalg.norm(points - (a + t * ab), axis=1)


def polyline_dist(points, poly):
    """Min distance from each point to a polyline (as an array of nodes)."""
    best = np.full(len(points), np.inf)
    for a, b in zip(poly, poly[1:]):
        best = np.minimum(best, _seg_dists(points, a, b))
    return best


def analyse_stage(v, maps, joints, counts, link):
    """Deviation of the DENSE path from each candidate chain, per count."""
    def tool(jm):
        full = dict(v.home)
        full.update(jm)
        return v.model.link_world_transforms(full)[link][:3, 3]

    dense = np.array([tool(m) for m in maps[::DENSE_STRIDE]])
    rows = []
    for n in counts:
        if n >= len(maps):
            continue
        tg = subsample(maps, n)
        tg_tool = np.array([tool(m) for m in tg])

        # movel: the tool path IS the straight segments between targets
        movel = polyline_dist(dense, tg_tool)

        # movej: FK of joint-linear interpolation, densely sampled
        chain = []
        for a, b in zip(tg, tg[1:]):
            for i in range(20):
                f = i / 19.0
                chain.append(tool({k: a[k] * (1 - f) + b[k] * f
                                   for k in joints}))
        chain.append(tg_tool[-1])
        movej = polyline_dist(dense, np.array(chain))

        rows.append((n, float(movej.max()) * 1000, float(movej.mean()) * 1000,
                     float(movel.max()) * 1000, float(movel.mean()) * 1000))
    return rows


def main() -> int:
    # Shared parser: docs on -h/--help, unknown argument rejected (exit 2).
    # `--stage` was in the old usage line but never read — every stage is
    # always analysed — so it is gone from USAGE rather than accepted and
    # ignored, which would have looked like a filter that did nothing.
    handle_cli(__doc__, extra_flags=("--all",), allow_common=False,
               usage=USAGE, value_flags=("--task", "--tol"))
    tol = float(_arg("--tol", 5.0))
    tasks = TASKS if "--all" in sys.argv else (
        [_arg("--task", TASKS[0])])

    print("=" * 78)
    print("C12b  Sparse-chain COVERAGE of the cleaning path "
          f"(tolerance {tol:.0f} mm)")
    print("=" * 78)

    verdicts = []
    for task in tasks:
        path = resolve_plan(f"{task}_ruckig_pro_only.json")
        plan = load_plan(path)
        side = _side(plan)
        pref = "R_" if side == "right" else "L_"
        joints = [f"{pref}joint{i}" for i in range(1, 8)]
        link = f"{pref}ConnectorLink"
        v = SegmentVerifier(fixture="commode_c", side=side, quiet=True)

        for st in arm_stages(plan, prefix=f"{pref}joint"):
            if st["num_waypoints"] < 200:      # transits are not paths
                continue
            maps = stage_maps(st)
            print(f"\n  {task}  [{side}]  stage '{st['stage_name']}'  "
                  f"{st['num_waypoints']} waypoints")
            print(f"    {'targets':>8s} | {'movej max':>10s} {'mean':>7s} | "
                  f"{'movel max':>10s} {'mean':>7s}   (mm)")
            best = {}
            for n, jmax, jmean, lmax, lmean in analyse_stage(
                    v, maps, joints, COUNTS, link):
                flag_j = "OK" if jmax <= tol else ""
                flag_l = "OK" if lmax <= tol else ""
                print(f"    {n:8d} | {jmax:10.1f} {jmean:7.1f} {flag_j:2s}| "
                      f"{lmax:10.1f} {lmean:7.1f} {flag_l}")
                if jmax <= tol:
                    best.setdefault("movej", n)
                if lmax <= tol:
                    best.setdefault("movel", n)
            verdicts.append((task, st["stage_name"], best))

    print("\n" + "=" * 78)
    print(f"  TARGETS NEEDED to stay within {tol:.0f} mm of the planned path")
    print("  " + "-" * 74)
    for task, stage, best in verdicts:
        j = best.get("movej", f">{COUNTS[-1]}")
        m = best.get("movel", f">{COUNTS[-1]}")
        print(f"  {task:20s} {stage:16s} movej: {str(j):>5s}   "
              f"movel: {str(m):>5s}")
    print("  " + "-" * 74)
    print("  Read this carefully: the MAX deviations for movej and movel are"
          "\n  nearly identical at every count. At coarse spacing the error "
          "is\n  dominated by the surface CURVING between targets — both "
          "commands\n  cut that corner the same way. Target DENSITY is the "
          "lever, not the\n  command.")
    print("  movel wins consistently on MEAN deviation (~2x), because along"
          "\n  near-flat stretches a straight Cartesian segment already IS "
          "the\n  path while a joint-linear move bows away from it. That is "
          "a real\n  advantage for wipe quality — it is just not what sets "
          "the count.")
    return 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
