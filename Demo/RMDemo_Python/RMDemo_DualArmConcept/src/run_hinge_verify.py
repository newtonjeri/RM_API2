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
    BUNDLED_PLANS, SegmentVerifier, arm_stages, load_plan, plan_state_upto,
    resolve_plan, stage_maps, subsample)
from task_config import TaskConfig

# The config names MoveIt collision objects; the offline scene names its
# meshes. One explicit bridge, so a rename in either system fails loudly
# here rather than quietly allowing contact it should not.
CONFIG_OBJECT_TO_MESHES = {
    "commode_lid": ("lid_closed", "lid_open"),
    "commode_seat": ("seat_closed", "seat_open"),
    "commode_body": ("body_static",),
}


def allowed_mesh_keys(cfg, surface):
    """Scene mesh keys the task's contact_surface permits."""
    keys = set()
    for obj in cfg.contact_objects(surface):
        mapped = CONFIG_OBJECT_TO_MESHES.get(obj)
        if mapped is None:
            print(f"    [WARN] contact object {obj!r} has no scene mesh "
                  "mapping — treated as NOT allowed")
            continue
        keys.update(mapped)
    return keys


def contact_verdict(v, timeline, arm_joints, cfg, surface):
    """Does contact stay inside what the config declares?

    Two independent conditions, both from contact_links.yaml:
      * only objects in contact_surface_objects[surface] may be touched
      * only links in contact_link_groups[ik_frame] may do the touching

    NOTE (Newton, 2026-08-08): the link groups are currently the FULL
    glove-covered hand set for every ik_frame, so today the second
    condition rarely bites and the OBJECT condition is doing the work.
    The groups are to be narrowed; this reads them from the file, so it
    sharpens automatically when they are — no code change needed.
    """
    allowed_objs = allowed_mesh_keys(cfg, surface)
    allowed_links = set(cfg.contact_links())
    bad_obj, bad_link, touched = {}, {}, set()
    for jm in timeline:
        for key, links in v.contact_report(jm).items():
            touched.add(key)
            if key not in allowed_objs:
                bad_obj.setdefault(key, set()).update(links)
                continue
            stray = links - allowed_links
            if stray:
                bad_link.setdefault(key, set()).update(stray)
    return {
        "surface": surface,
        "allowed_objects": sorted(allowed_objs),
        "allowed_link_count": len(allowed_links),
        "touched": sorted(touched),
        "undeclared_objects": {k: sorted(x) for k, x in bad_obj.items()},
        "stray_links": {k: sorted(x) for k, x in bad_link.items()},
        "ok": not bad_obj and not bad_link,
    }

# Workspace copy if present, else the copy bundled in this repo (../plans).
DEFAULT_TASK = "hinge_area_right"


def plan_for(task):
    return resolve_plan(f"{task}_ruckig_pro_only.json")


def side_of(plan):
    """Which arm does this plan drive? Read it from the plan, never assume.

    toplid_left is a LEFT-arm task while hinge_area_right is a right-arm
    one, and the fixture geometry, home state and collision link filter all
    depend on getting this right.
    """
    for st in plan["sub_trajectories"]:
        for j in st["joint_names"]:
            if j.startswith("R_joint"):
                return "right"
            if j.startswith("L_joint"):
                return "left"
    raise SystemExit("no arm joints in this plan")


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
    task = _arg("--task", DEFAULT_TASK)
    plan_path = _arg("--plan", str(plan_for(task)))
    samples = int(_arg("--samples", 40))
    stroke_n = int(_arg("--stroke-targets", 20))
    margin_m = float(_arg("--margin", 10)) / 1000.0

    print("=" * 70)
    print(f"C12  Controller-motion collision verification — {task}")
    print(f"     plan: {plan_path}")
    print("     source: " + ("this repo's plans/ (the contract)"
                             if str(BUNDLED_PLANS) in str(plan_path)
                             else "EXPLICIT --plan override"))
    print(f"     samples/segment: {samples}   stroke targets: {stroke_n}   "
          f"margin: {margin_m * 1000:.0f} mm")
    print("=" * 70)

    plan = load_plan(plan_path)
    cfg = TaskConfig.load(task)
    surface, contact_stages = cfg.contact_window()
    side = _arg("--side", side_of(plan))
    pref = "R_" if side == "right" else "L_"
    arm_joints = [f"{pref}joint{i}" for i in range(1, 8)]
    stages = arm_stages(plan, prefix=f"{pref}joint")
    print(f"  side: {side}   arm stages: "
          f"{[s['stage_name'] for s in stages]}")
    if surface:
        print(f"  contact window: allow_collision(contact_surface="
              f"{surface!r}) over {sorted(contact_stages)}")
        print(f"    declared objects: {sorted(cfg.contact_objects(surface))}"
              f" -> meshes {sorted(allowed_mesh_keys(cfg, surface))}")
        print(f"    declared links:   {len(cfg.contact_links())} "
              f"(currently the full hand set for every ik_frame)")
    else:
        print("  contact window: none — every stage must stay clear")
    v = SegmentVerifier(fixture="commode_c", side=side,
                        articulation=cfg.params.get(
                            "commode_fixture_type"))

    verdicts = []
    for st in stages:
        # State the task has ESTABLISHED by the time this stage runs — the
        # pole above all, which carries the arm base. Merged UNDER every
        # sample so the arm is verified where the task actually puts it.
        base = plan_state_upto(plan, plan["sub_trajectories"].index(st))
        maps = [dict(base, **m) for m in stage_maps(st)]
        name = st["stage_name"]
        pole = next((f"{k}={v:.3f}" for k, v in base.items()
                     if "sliding_plate" in k), "pole=SRDF home")
        t0 = time.perf_counter()

        dense = v.verify_timeline(subsample(maps, samples),
                                  arm_joints, tag="dense")
        mode_a = v.verify_timeline(
            [dict(base, **m)
             for m in v.movej_timeline(maps[0], maps[-1], samples)],
            arm_joints, tag="modeA")
        sparse = None
        # by SIZE, not by stage name — see build_targets in C11
        if len(maps) > stroke_n:
            targets = subsample(maps, stroke_n)
            timeline = []
            for a, b in zip(targets, targets[1:]):
                timeline += v.movej_timeline(a, b, 4)[:-1]
            timeline.append(targets[-1])
            sparse = v.verify_timeline(timeline, arm_joints,
                                       tag=f"sparse-{stroke_n}")

        ref = dense["min_clearance_m"]
        rows = [("dense (MoveIt ref)", dense), ("Mode A (2 targets)", mode_a)]
        if sparse:
            rows.append((f"sparse-{stroke_n} chain", sparse))
        print(f"\n  {name}  ({st['num_waypoints']} wp, "
              f"{time.perf_counter() - t0:.1f}s to verify)   [{pole}]")
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
        # A stage inside the allow_collision bracket is judged by the
        # CONFIG CONTRACT — declared links may touch declared objects,
        # nothing else may touch anything. Touch fraction was only ever a
        # proxy; this is the rule the task itself states.
        contact_stage = name in contact_stages
        if contact_stage:
            timeline = (subsample(maps, samples) if sparse is None
                        else [dict(base, **m) for m in
                              subsample(maps, samples)])
            cv = contact_verdict(v, timeline, arm_joints, cfg, surface)
            safe = cv["ok"] and cand["self_collisions"] == 0
            if cv["undeclared_objects"]:
                basis = ("contact stage: touched UNDECLARED "
                         f"{sorted(cv['undeclared_objects'])}")
            elif cv["stray_links"]:
                stray = sorted({x for v_ in cv["stray_links"].values()
                                for x in v_})
                basis = f"contact stage: links outside the group {stray[:4]}"
            else:
                basis = (f"contact stage: only {cv['surface']!r} objects "
                         f"{cv['touched']} touched, all by declared links")
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
