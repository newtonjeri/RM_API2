"""C11 / WP4 — Rehearsal validator: does the arm go where C12 predicted?

C12 (`run_hinge_verify.py`) declares the controller's motion between sparse
targets collision-free using a MODEL of that motion: joint-linear
interpolation, plus an offline `rm_algo` that is v1.6.0 while the
controllers run 1.5.9.  Nothing has ever checked that model against the
machine.  This test does, and turns the answer into a number the verifier
can carry forever:

    SIM-execute the real sparse targets   (no physical motion, F2/C5)
      -> capture the ACTUAL joints over the UDP realtime push
        -> FCL-sweep the capture against the commode scene
          -> compare capture vs prediction  ==>  RESIDUAL = the FCL margin

Why simulation is enough: C5 established that sim mode runs the real
planner and streams the interpolated states over UDP, so the captured
trajectory is the controller's own geometry — which is exactly what C12
models.  Sim does not execute lift or hand (F3), so non-arm stages are
skipped; this gate is about arm geometry.

Two halves, deliberately separable — the arms are only reachable from the
lab machine, the analysis is not:

    capture   (hardware, SIM)  writes rehearsal_<side>.json
    analyse   (--replay FILE)  runs anywhere, no arm, no SDK connection

so the lab session is a short capture and the verdict can be produced on
any machine afterwards.

The residual is reported three ways:
  * joint-space deviation   max distance from the captured path to the
                            predicted polyline (deg, 7-dim)
  * Cartesian deviation     the same worst sample at the tool point (mm)
  * CLEARANCE deviation     predicted min clearance MINUS captured min
                            clearance — positive means the predictor was
                            OPTIMISTIC by that much, and that number is
                            the margin every future C12 verdict must add.

Usage:
    RM_ARM=right python3 test_rehearsal_validate.py            # SIM capture
    python3 test_rehearsal_validate.py --replay rehearsal_right.json
Options: --plan PATH  --targets N  --samples N  --speed PCT
         --save PATH  --replay PATH  --mode SIM|REAL
"""

import json
import math
import os
import pathlib
import sys
import threading
import time

from dual_arm_common import (
    handle_cli, parse_mode_arg, countdown, host_ip_for,
    ARM_TIMEOUT_S, DEV_JOINT, LEFT_IP, RIGHT_IP, ROBOT_PORT,
    UDP_PORT, ArrivalMonitor,
)
from segment_verifier import (
    BUNDLED_PLANS, SegmentVerifier, arm_stages, load_plan, resolve_plan, stage_maps,
    subsample)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import (
    rm_thread_mode_e, rm_realtime_arm_state_callback_ptr,
    rm_realtime_push_config_t, rm_udp_custom_config_t,
)

ARM_SIDE = os.environ.get("RM_ARM", "right").lower()
ARM_IP = RIGHT_IP if ARM_SIDE == "right" else LEFT_IP
# Asked of the kernel, not of the operator — see host_ip_for().
HOST_IP = host_ip_for(ARM_IP)
PREF = "R_" if ARM_SIDE == "right" else "L_"
ARM_JOINTS = [f"{PREF}joint{i}" for i in range(1, 8)]
CONNECTOR = f"{PREF}ConnectorLink"

# Workspace copy if present, else the copy bundled in this repo — the
# capture half must work on a machine with no ROS workspace.
DEFAULT_PLAN = resolve_plan("hinge_area_right_ruckig_pro_only.json")
DEFAULT_SAVE = pathlib.Path(__file__).resolve().parent / \
    f"rehearsal_{ARM_SIDE}.json"

STROKE_TARGETS = 20          # must match run_hinge_verify's --stroke-targets
REHEARSAL_SPEED_PCT = 20     # geometry is what matters; slow is fine
UDP_WATCHDOG_S = 2.0
STAGE_TOL_DEG = 1.0          # captured endpoint vs commanded target

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 7

_udp_lock = threading.Lock()
_udp_samples = []            # (t, [7 joint deg])


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _arg(flag, default):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def _on_state(data):
    try:
        q = [float(data.joint_status.joint_position[i]) for i in range(7)]
    except Exception:
        return
    with _udp_lock:
        _udp_samples.append((time.perf_counter(), q))


_state_cb = rm_realtime_arm_state_callback_ptr(_on_state)   # keep alive


# ── targets: EXACTLY what C12 verified and what a dispatcher would send ──
def build_targets(plan_path, stroke_n):
    """Sparse targets per arm stage — Mode A endpoints, stroke subsampled.

    Kept byte-identical in construction to run_hinge_verify.py so the
    residual measured here applies to the clearance map produced there.
    """
    plan = load_plan(plan_path)
    stages = []
    for st in arm_stages(plan, prefix=f"{PREF}joint"):
        maps = stage_maps(st)
        if st["stage_name"] == "execute_path" and len(maps) > stroke_n:
            targets = subsample(maps, stroke_n)
        else:
            targets = [maps[0], maps[-1]]
        stages.append({
            "name": st["stage_name"],
            "num_waypoints": st["num_waypoints"],
            "targets": [[math.degrees(m[j]) for j in ARM_JOINTS]
                        for m in targets],
        })
    return stages


def predicted_path(stages, per_segment=25):
    """The C12 model: joint-linear between consecutive targets (deg).

    Returns (path, stage_index_per_point) so every captured sample can be
    attributed to the stage it belongs to WITHOUT relying on timestamps —
    which makes the analysis work identically on a live capture and on a
    replayed file.
    """
    path, owner = [], []
    for k, st in enumerate(stages):
        tg = st["targets"]
        for a, b in zip(tg, tg[1:]):
            for i in range(per_segment):
                f = i / (per_segment - 1)
                path.append([ai * (1 - f) + bi * f for ai, bi in zip(a, b)])
                owner.append(k)
    return path, owner


# ── residual geometry ──
def attribute(captured, predicted, owner):
    """For each captured sample: distance to the predicted polyline, the
    projected point, and which STAGE that nearest segment belongs to.

    Stage attribution is geometric, not temporal, so it works identically
    on a live capture and on a replayed file — and it is what lets the
    samples be balanced PER STAGE afterwards. Without that, the cleaning
    stroke (2002 of 2033 waypoints, and by far the slowest stage in wall
    time) swamps the transits and their clearance numbers come from one
    or two samples.

    Vectorized point-to-segment over all segments at once: every captured
    frame costs one array op instead of ~550 interpreted iterations, so
    the whole capture can be attributed rather than a subsample of it.

    Returns a list of (dist_deg, captured_q, projected_q, stage_index).
    """
    import numpy as np
    C = np.asarray(captured, dtype=float)
    P = np.asarray(predicted, dtype=float)
    A, B = P[:-1], P[1:]
    AB = B - A
    den = np.einsum("ij,ij->i", AB, AB)
    den[den == 0.0] = 1.0            # zero-length stage-bridging segments
    own = np.asarray(owner[:-1])
    out = []
    for c in C:
        t = np.clip(np.einsum("ij,ij->i", c - A, AB) / den, 0.0, 1.0)
        proj = A + t[:, None] * AB
        d = np.linalg.norm(c - proj, axis=1)
        k = int(np.argmin(d))
        out.append((float(d[k]), c.tolist(), proj[k].tolist(), int(own[k])))
    return out


def _maps(path_deg):
    return [dict(zip(ARM_JOINTS, [math.radians(v) for v in q]))
            for q in path_deg]


def tool_gap_mm(verifier, qa_deg, qb_deg):
    """Distance between the tool points of two configurations (mm)."""
    import numpy as np
    pts = []
    for q in (qa_deg, qb_deg):
        jm = dict(verifier.home)
        jm.update(dict(zip(ARM_JOINTS, [math.radians(v) for v in q])))
        tw = verifier.model.link_world_transforms(jm)
        pts.append(tw[CONNECTOR][:3, 3])
    return float(np.linalg.norm(pts[0] - pts[1])) * 1000.0


# ── the hardware half ──
def capture(mode, plan_path, stroke_n, speed_pct, save_path):
    stages = build_targets(plan_path, stroke_n)
    n_targets = sum(len(s["targets"]) for s in stages)
    print(f"  plan stages: {[s['name'] for s in stages]}")
    print(f"  sparse targets: {n_targets} "
          f"(stroke subsampled to {stroke_n})")

    robot = None
    original_mode = None
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ARM_IP, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ARM_IP}")
            _results["SKIP"] += N_CHECKS
            return None

        # ── R1: run mode — SIMULATION unless REAL was demanded ──
        ret, original_mode = robot.rm_get_arm_run_mode()
        want = 1 if mode == 1 else 0
        robot.rm_set_arm_run_mode(want)
        ret, got = robot.rm_get_arm_run_mode()
        if ret != 0 or got != want:
            result("FAIL", "run mode engaged and verified",
                   f"ret={ret} mode={got} — no motion dispatched")
            return None
        if want == 1:
            print("\n  " + "!" * 60)
            print("  REAL MODE — the arm will execute the whole cleaning "
                  "path.\n  This is NOT a rehearsal. Ctrl-C now to abort.")
            print("  " + "!" * 60)
            countdown(3)
            result("PASS", "run mode engaged and verified", "REAL (explicit)")
        else:
            result("PASS", "run mode engaged and verified",
                   "SIMULATION — no physical motion")

        # ── R2: the capture channel ──
        monitor = ArrivalMonitor()
        monitor.register(robot)
        robot.rm_realtime_arm_state_call_back(_state_cb)
        robot.rm_set_realtime_push(rm_realtime_push_config_t(
            cycle=1, enable=True, port=UDP_PORT, ip=HOST_IP,
            custom_config=rm_udp_custom_config_t(joint_speed=1)))
        deadline = time.perf_counter() + UDP_WATCHDOG_S
        while time.perf_counter() < deadline:
            with _udp_lock:
                if _udp_samples:
                    break
            time.sleep(0.05)
        with _udp_lock:
            n0 = len(_udp_samples)
        if n0 == 0:
            result("FAIL", "UDP capture channel live",
                   f"0 frames from {HOST_IP}:{UDP_PORT}")
            print(f"  [FATAL] Without the capture there is no rehearsal. "
                  f"The push target {HOST_IP} was resolved from the route "
                  f"to {ARM_IP}; if that is not this machine's address on "
                  "the arm LAN, pin it with RM_HOST_IP, and check "
                  f"UDP {UDP_PORT} is free and not firewalled.")
            return None
        result("PASS", "UDP capture channel live", f"{n0} frames in {UDP_WATCHDOG_S:.0f}s")

        # ── staging move: the plan starts from ITS first waypoint ──
        first = stages[0]["targets"][0]
        print(f"\n  staging -> stage '{stages[0]['name']}' first waypoint "
              "(excluded from the residual)")
        monitor.expect(handle.id, DEV_JOINT)
        if robot.rm_movej(first, speed_pct, 0, 0, 0) != 0:
            result("FAIL", "staging move accepted")
            return None
        monitor.wait(handle.id, DEV_JOINT, ARM_TIMEOUT_S)

        # ── R3: dispatch every sparse target, one at a time ──
        with _udp_lock:
            _udp_samples.clear()
        t0 = time.perf_counter()
        marks, missed = [], 0
        for st in stages:
            for k, target in enumerate(st["targets"]):
                monitor.expect(handle.id, DEV_JOINT)
                td = time.perf_counter() - t0
                if robot.rm_movej(target, speed_pct, 0, 0, 0) != 0:
                    missed += 1
                    print(f"    [WARN] {st['name']}[{k}] rejected")
                    continue
                arrived, ok = monitor.wait(handle.id, DEV_JOINT,
                                           ARM_TIMEOUT_S)
                marks.append({"stage": st["name"], "i": k, "t": td,
                              "arrived": bool(arrived), "ok": bool(ok)})
                if not arrived:
                    missed += 1
                    print(f"    [WARN] {st['name']}[{k}] no arrival event")
            print(f"    {st['name']:22s} {len(st['targets']):3d} targets done")
        time.sleep(0.3)

        if missed == 0:
            result("PASS", "every sparse target executed", f"{n_targets}/{n_targets}")
        else:
            result("FAIL", "every sparse target executed",
                   f"{n_targets - missed}/{n_targets} — {missed} missed")

        # ── R4: did the arm finish where it was told? ──
        with _udp_lock:
            samples = list(_udp_samples)
        last_target = stages[-1]["targets"][-1]
        if samples:
            err = max(abs(a - b) for a, b in zip(samples[-1][1], last_target))
            if err <= STAGE_TOL_DEG:
                result("PASS", "capture ends at the commanded target",
                       f"max joint err {err:.3f} deg")
            else:
                result("FAIL", "capture ends at the commanded target",
                       f"max joint err {err:.3f} deg > {STAGE_TOL_DEG}")
        else:
            result("FAIL", "capture ends at the commanded target",
                   "no samples captured")

        rec = {
            "side": ARM_SIDE, "plan": str(plan_path), "mode":
                "REAL" if want else "SIM",
            "speed_pct": speed_pct, "stroke_targets": stroke_n,
            "joint_names": ARM_JOINTS, "stages": stages, "marks": marks,
            "samples": [{"t": round(t - t0, 4), "q": [round(v, 4) for v in q]}
                        for t, q in samples],
        }
        pathlib.Path(save_path).write_text(json.dumps(rec))
        print(f"\n  capture saved: {save_path}  "
              f"({len(samples)} frames, {samples[-1][0] - t0:.1f} s)"
              if samples else f"\n  capture saved: {save_path}")
        return rec
    finally:
        try:
            if robot is not None:
                robot.rm_set_realtime_push(rm_realtime_push_config_t(
                    cycle=1, enable=False, port=UDP_PORT, ip=HOST_IP))
            if robot is not None and original_mode is not None:
                robot.rm_set_arm_run_mode(original_mode)
                print(f"  [INFO] run mode restored: "
                      f"{'SIMULATION' if original_mode == 0 else 'REAL'}")
        except Exception as exc:
            print(f"  [WARN] restore failed: {exc!r}")
        if robot is not None:
            try:
                robot.rm_delete_robot_arm()
            except Exception:
                pass
            try:
                RoboticArm.rm_destroy()
            except Exception:
                pass


# ── the analysis half (no hardware) ──
def analyse(rec, samples_n):
    stages = rec["stages"]
    cap_all = [s["q"] for s in rec["samples"]]
    if len(cap_all) < 4:
        result("SKIP", "captured trajectory is self-collision free",
               f"only {len(cap_all)} frames")
        result("SKIP", "fixture contact matches the prediction", "no capture")
        result("SKIP", "predictor residual measured", "no capture")
        return
    predicted, owner = predicted_path(stages)
    # attribute EVERY frame, then balance the FCL sweep per stage
    dev = attribute(cap_all, predicted, owner)
    per_stage = max(12, samples_n // max(1, len(stages)))
    print(f"\n  analysing {len(cap_all)} captured frames vs "
          f"{len(predicted)} predicted; sweeping <={per_stage}/stage")

    v = SegmentVerifier(fixture="commode_c", side=rec.get("side", ARM_SIDE))

    rows, self_hits = [], 0
    for k, st in enumerate(stages):
        cap_q = [d[1] for d in dev if d[3] == k]
        pred_q = [p for i, p in enumerate(predicted) if owner[i] == k]
        if not cap_q or not pred_q:
            print(f"    [WARN] stage '{st['name']}' attracted no captured "
                  "samples — excluded from the residual")
            continue
        cap_rep = v.verify_timeline(_maps(subsample(cap_q, per_stage)),
                                    ARM_JOINTS, tag=f"cap:{st['name']}")
        pred_rep = v.verify_timeline(_maps(subsample(pred_q, per_stage)),
                                     ARM_JOINTS, tag=f"pred:{st['name']}")
        worst = max((d for d in dev if d[3] == k), key=lambda d: d[0])
        self_hits += cap_rep["self_collisions"]
        rows.append({
            "name": st["name"],
            "dev_deg": worst[0],
            "tool_mm": tool_gap_mm(v, worst[1], worst[2]),
            "cap_min": cap_rep["min_clearance_m"] * 1000.0,
            "pred_min": pred_rep["min_clearance_m"] * 1000.0,
            "cap_frac": cap_rep["collisions"] / cap_rep["samples"],
            "pred_frac": pred_rep["collisions"] / pred_rep["samples"],
        })

    # ── R5: self-collision in what the machine actually did ──
    if self_hits == 0:
        result("PASS", "captured trajectory is self-collision free")
    else:
        result("FAIL", "captured trajectory is self-collision free",
               f"{self_hits} samples flagged by rm_algo")

    # ── R6: does it touch WHERE, and only where, the prediction said? ──
    bad = [r["name"] for r in rows
           if r["cap_frac"] > r["pred_frac"] * 1.5 + 0.05]
    if not bad:
        result("PASS", "fixture contact matches the prediction",
               f"all {len(rows)} stages within tolerance")
    else:
        result("FAIL", "fixture contact matches the prediction",
               f"NEW contact in: {', '.join(bad)}")

    # ── R7: the residual — the number this whole gate exists to produce ──
    # Clearance optimism is only meaningful where there IS clearance: on a
    # CONTACT stage both sides saturate at 0.0 mm and the difference says
    # nothing. Those stages are judged by touch fraction (R6) and by the
    # tool deviation instead.
    print("\n  " + "─" * 72)
    print("  PER-STAGE RESIDUAL — controller reality vs the C12 model")
    print("  " + "─" * 72)
    print(f"  {'stage':22s} {'dev deg':>8s} {'tool mm':>8s} "
          f"{'pred mm':>8s} {'cap mm':>8s} {'optimism':>9s}  contact")
    optimism_free, tool_worst = 0.0, 0.0
    for r in rows:
        contact = r["pred_frac"] > 0 or r["cap_frac"] > 0
        opt = r["pred_min"] - r["cap_min"]
        tool_worst = max(tool_worst, r["tool_mm"])
        if contact:
            opt_s, tag = "     n/a", (f"touch {r['cap_frac']:.0%}"
                                      f"/{r['pred_frac']:.0%}")
        else:
            optimism_free = max(optimism_free, opt)
            opt_s, tag = f"{opt:8.2f}", "free"
        print(f"  {r['name']:22s} {r['dev_deg']:8.3f} {r['tool_mm']:8.2f} "
              f"{r['pred_min']:8.2f} {r['cap_min']:8.2f} {opt_s:>9s}  {tag}")
    print("  " + "─" * 72)
    # round before ceil: two identical clearances differ in the 8th decimal
    margin_mm = max(round(optimism_free, 2), 0.0)
    print(f"    clearance optimism (free-space stages) {optimism_free:8.2f} mm"
          "   <== the measured margin")
    print(f"    worst tool-point deviation             {tool_worst:8.2f} mm"
          "   (conservative bound)")
    print("  " + "─" * 72)
    print(f"    ==> FCL MARGIN for C12: {math.ceil(margin_mm) + 5} mm "
          f"(measured {margin_mm:.1f} + 5 mm safety)")
    print("    Re-verify with:  python3 run_hinge_verify.py "
          f"--margin {math.ceil(margin_mm) + 5}")
    if tool_worst > 10.0:
        print("    NOTE: a stage deviating > 10 mm at the tool is poorly "
              "modelled by\n          joint-linear motion — a Mode B "
              "candidate (denser chain).")
    print("  " + "─" * 72)
    result("PASS", "predictor residual measured",
           f"margin {margin_mm:.1f} mm, worst tool dev {tool_worst:.1f} mm")


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=(),
               value_flags=("--plan", "--targets", "--samples", "--speed",
                            "--save", "--replay"))
    plan_path = _arg("--plan", str(DEFAULT_PLAN))
    stroke_n = int(_arg("--targets", STROKE_TARGETS))
    samples_n = int(_arg("--samples", 150))
    speed_pct = int(_arg("--speed", REHEARSAL_SPEED_PCT))
    save_path = _arg("--save", str(DEFAULT_SAVE))
    replay = _arg("--replay", None)
    mode = parse_mode_arg()

    print("=" * 70)
    print("C11  Rehearsal validation — capture the controller, calibrate C12")
    print(f"     side={ARM_SIDE}  plan={plan_path}")
    print("     plan source: " + ("BUNDLED copy in this repo"
                                  if str(BUNDLED_PLANS) in str(plan_path)
                                  else "ROS workspace"))
    if replay:
        print(f"     REPLAY {replay} — analysis only, no hardware")
    else:
        print(f"     arm={ARM_IP}  UDP -> {HOST_IP}:{UDP_PORT}  "
              f"v={speed_pct}%  mode="
              f"{'REAL' if mode == 1 else 'SIMULATION'}")
    print("=" * 70)

    try:
        if replay:
            rec = json.loads(pathlib.Path(replay).read_text())
            result("SKIP", "run mode engaged and verified", "replay")
            result("SKIP", "UDP capture channel live", "replay")
            result("SKIP", "every sparse target executed", "replay")
            result("SKIP", "capture ends at the commanded target", "replay")
        else:
            rec = capture(mode, plan_path, stroke_n, speed_pct, save_path)
        if rec is not None:
            analyse(rec, samples_n)
    finally:
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
