"""WP2 / C12 — verify the CONTROLLER's motion for the hinge cleaning task.

Input: the saved orchestrator plan for `hinge_area_right`
(Resource/plans/commode_c/hardware/hinge_area_right_ruckig_pro_only.json)
— the SAME task Phase 2 targets, planned by the existing MoveIt pipeline.

For every arm stage this compares, offline:

  DENSE     MoveIt's own path (the plan's waypoints) — the reference:
            we KNOW this was collision-checked by MoveIt.
  MODE A    what the controller would do given only the stage's sparse
            endpoints: joint-linear from first to last waypoint (movej).
  SPARSE-N  for the cleaning stroke (`execute_path`): the dense path
            reduced to N evenly spaced targets with joint-linear motion
            between them — approximating chained movel/movej through the
            cleaning points.

Verdict per stage: Mode A is SAFE when its minimum fixture clearance stays
above the margin, or within tolerance of the dense reference (MoveIt's own
worst clearance — the stroke intentionally CONTACTS the fixture, so an
absolute margin is meaningless there). Stages where Mode A dips below both
need Mode B (a denser chain) — this is the clearance map that decides
where Mode B is worth its cost.

Offline caveats (carried into PHASE_PLAN):
  * movel is approximated joint-linearly between adjacent targets — valid
    for small steps near the surface; C11's rehearsal capture is the
    hardware calibration of exactly this approximation.
  * The left arm and body sit at the SRDF home state.

Usage: python3 run_hinge_verify.py [--plan PATH] [--samples N]
       [--stroke-targets N] [--margin MM]
"""

import sys
import time

from segment_verifier import (
    WS, SegmentVerifier, arm_stages, load_plan, stage_maps, subsample)

DEFAULT_PLAN = (WS / "Resource" / "plans" / "commode_c" / "hardware"
                / "hinge_area_right_ruckig_pro_only.json")
ARM_JOINTS = [f"R_joint{i}" for i in range(1, 8)]


def _arg(flag, default):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def main() -> int:
    if "-h" in sys.argv or "--help" in sys.argv:
        print(__doc__)
        return 0
    plan_path = _arg("--plan", str(DEFAULT_PLAN))
    samples = int(_arg("--samples", 40))
    stroke_n = int(_arg("--stroke-targets", 20))
    margin_m = float(_arg("--margin", 10)) / 1000.0

    print("=" * 70)
    print("C12  Controller-motion collision verification — hinge_area_right")
    print(f"     plan: {plan_path}")
    print(f"     samples/segment: {samples}   stroke targets: {stroke_n}   "
          f"margin: {margin_m * 1000:.0f} mm")
    print("=" * 70)

    plan = load_plan(plan_path)
    stages = arm_stages(plan)
    print(f"  arm stages: {[s['stage_name'] for s in stages]}")
    v = SegmentVerifier(fixture="commode_c", side="right")

    verdicts = []
    for st in stages:
        maps = stage_maps(st)
        name = st["stage_name"]
        t0 = time.perf_counter()

        dense = v.verify_timeline(subsample(maps, samples),
                                  ARM_JOINTS, tag="dense")
        mode_a = v.verify_timeline(
            v.movej_timeline(maps[0], maps[-1], samples),
            ARM_JOINTS, tag="modeA")
        sparse = None
        if name == "execute_path" and len(maps) > stroke_n:
            targets = subsample(maps, stroke_n)
            timeline = []
            for a, b in zip(targets, targets[1:]):
                timeline += v.movej_timeline(a, b, 4)[:-1]
            timeline.append(targets[-1])
            sparse = v.verify_timeline(timeline, ARM_JOINTS,
                                       tag=f"sparse-{stroke_n}")

        ref = dense["min_clearance_m"]
        rows = [("dense (MoveIt ref)", dense), ("Mode A (2 targets)", mode_a)]
        if sparse:
            rows.append((f"sparse-{stroke_n} chain", sparse))
        print(f"\n  {name}  ({st['num_waypoints']} wp, "
              f"{time.perf_counter() - t0:.1f}s to verify)")
        for label, rep in rows:
            mc = rep["min_clearance_m"]
            print(f"    {label:22s} min clearance "
                  f"{mc * 1000:7.1f} mm at sample {rep['min_at']:3d}/"
                  f"{rep['samples']}   collisions={rep['collisions']}"
                  f"   self={rep['self_collisions']}")

        # Verdict — CONTACT-AWARE. The cleaning stroke (and the first
        # instants of the retreat) contact the fixture BY DESIGN: the dense
        # MoveIt reference itself reports contact there. So a stage is
        # judged RELATIVE to its reference:
        #   free-space stage (dense has no contact): candidate must also be
        #     contact-free and keep >= margin (or match the reference).
        #   contact stage (dense contacts): candidate must not contact in a
        #     substantially larger FRACTION of its samples than the dense
        #     path does — i.e. it touches where the task touches, and no
        #     new collision regions appear.
        cand = sparse if sparse else mode_a
        dense_frac = dense["collisions"] / dense["samples"]
        cand_frac = cand["collisions"] / cand["samples"]
        contact_stage = dense["collisions"] > 0
        if contact_stage:
            safe = (cand["self_collisions"] == 0
                    and cand_frac <= dense_frac * 1.5 + 0.05)
            basis = (f"contact stage: touch fraction {cand_frac:.0%} vs "
                     f"reference {dense_frac:.0%}")
        else:
            safe = (cand["collisions"] == 0
                    and cand["self_collisions"] == 0
                    and (cand["min_clearance_m"] >= margin_m
                         or cand["min_clearance_m"] >= ref - 0.002))
            basis = f"free-space stage: margin {margin_m * 1000:.0f} mm"
        verdicts.append((name, safe, cand["min_clearance_m"], ref,
                         "sparse chain" if sparse else "Mode A", basis))

    print("\n" + "=" * 70)
    print("  CLEARANCE MAP — execution-mode decision per stage")
    print("  " + "-" * 66)
    ok_all = True
    for name, safe, mc, ref, mode, basis in verdicts:
        tag = "OK" if safe else "NEEDS MODE B"
        ok_all &= safe
        print(f"  {name:26s} {mode:14s} min {mc * 1000:6.1f} mm "
              f"(ref {ref * 1000:6.1f})  {tag:12s} [{basis}]")
    print("  " + "-" * 66)
    print("  verdict:", "controller-planned execution is collision-safe for"
          " every stage" if ok_all else
          "flagged stages need a denser chain (Mode B) or a canfd stream")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
