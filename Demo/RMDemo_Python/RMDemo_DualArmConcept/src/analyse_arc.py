"""Arc-tracing analysis for chain_semantics_006-style runs.

For every movec in the recorded chain (run.json commanded.arc_vias), build
the IDEAL circle through (start, via, end) — three points define it — and
measure the traced samples inside that arc's spatial window:

    radial error   |dist(sample, center) - R|   (SIM position is exact)
    junction speed minimum windowed speed near the arc's entry and exit
                   (tangent geometry: a dip here is chain behaviour)
    arc coverage   fraction of the arc's angular span visited

Usage:  python3 analyse_arc.py ../runs/<run_dir>
"""
import csv
import json
import math
import sys

import numpy as np

SPEED_WINDOW = 7          # 70 ms windowed speed — the aliasing rule


def circle3(a, b, c):
    """Center and radius of the circle through three planar points."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None, None
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
          + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
          + (cx * cx + cy * cy) * (bx - ax)) / d
    ctr = np.array([ux, uy])
    return ctr, float(np.linalg.norm(np.asarray(a) - ctr))


def main():
    rundir = sys.argv[1].rstrip("/")
    d = json.load(open(rundir + "/run.json"))
    cmd = d["commanded"]
    arcs = cmd.get("arc_vias")
    if not arcs or all(a is None for a in arcs):
        raise SystemExit("no arc moves recorded in this run's commanded block")
    poses = [np.array(p[:3]) for p in cmd["poses"]]
    names = cmd["waypoint_names"]
    rows = list(csv.DictReader(open(rundir + "/stream.csv")))
    T = np.array([[float(r["tcp_x"]), float(r["tcp_y"]), float(r["tcp_z"])]
                  for r in rows])
    t = np.array([float(r["t_mono"]) for r in rows])
    t = t - t[0]
    sp = np.full(len(T), np.nan)
    for i in range(SPEED_WINDOW, len(T)):
        sp[i] = 1000 * np.linalg.norm(T[i] - T[i - SPEED_WINDOW]) \
            / (t[i] - t[i - SPEED_WINDOW])
    print(f"{d['run_id']}  ({d.get('mode')})")
    for i, via in enumerate(arcs):
        if via is None:
            continue
        A, B = poses[i], poses[i + 1]
        V = np.array(via[:3])
        # plane: these paths are z-planar; work in xy and report z drift
        ctr, R = circle3(A[:2], V[:2], B[:2])
        if ctr is None:
            print(f"  move {i + 1}: degenerate (collinear) — skipped")
            continue
        # samples belonging to this arc: inside the annulus AND inside the
        # arc's ANGULAR interval (start->end through the via, small margin).
        # Without the angular gate, stroke samples near the arc ends sit in
        # the annulus and contaminate the radial tail — the synthetic
        # self-test read p95 2.25 mm against 0.3 mm injected noise.
        dc = np.linalg.norm(T[:, :2] - ctr, axis=1)
        m = np.abs(dc - R) < 0.5 * R
        a0 = math.atan2(A[1] - ctr[1], A[0] - ctr[0])
        a1 = math.atan2(B[1] - ctr[1], B[0] - ctr[0])
        av = math.atan2(V[1] - ctr[1], V[0] - ctr[0])
        ang_all = np.arctan2(T[:, 1] - ctr[1], T[:, 0] - ctr[0])

        def between(x, lo, hi):
            # is angle x on the shorter sweep lo->hi that contains `through`?
            return (x - lo) % (2 * math.pi) <= (hi - lo) % (2 * math.pi)
        if between(av, a0, a1):
            lo, hi = a0, a1
        else:
            lo, hi = a1, a0
        MARGIN = math.radians(8)
        m &= np.array([between(x, lo - MARGIN, hi + MARGIN)
                       for x in ang_all])
        idx = np.where(m)[0]
        if len(idx) < 5:
            print(f"  move {i + 1} ({names[i]}->{names[i + 1]}): "
                  f"only {len(idx)} samples near the arc — did it execute?")
            continue
        seg = np.split(idx, np.where(np.diff(idx) > 5)[0] + 1)
        idx = max(seg, key=len)                    # densest contiguous visit
        rad_err = np.abs(dc[idx] - R)
        zdrift = np.abs(T[idx, 2] - A[2])
        # angular span visited
        ang = np.arctan2(T[idx, 1] - ctr[1], T[idx, 0] - ctr[0])
        a0 = math.atan2(A[1] - ctr[1], A[0] - ctr[0])
        a1 = math.atan2(B[1] - ctr[1], B[0] - ctr[0])
        span = abs((a1 - a0 + math.pi) % (2 * math.pi) - math.pi)
        visited = np.ptp(np.unwrap(np.sort(ang)))
        vmin_entry = np.nanmin(sp[max(0, idx[0] - 10):idx[0] + 10]) \
            if idx[0] > 10 else float("nan")
        vmin_exit = np.nanmin(sp[idx[-1] - 10:idx[-1] + 10])
        print(f"  move {i + 1} ({names[i]}->{names[i + 1]}) R={1000 * R:.1f}mm"
              f" n={len(idx)}")
        print(f"     radial error  med {1000 * np.median(rad_err):.2f}  "
              f"p95 {1000 * np.percentile(rad_err, 95):.2f}  "
              f"max {1000 * rad_err.max():.2f} mm   z-drift max "
              f"{1000 * zdrift.max():.2f} mm")
        print(f"     arc span visited {math.degrees(visited):.0f} of "
              f"{math.degrees(span):.0f} deg   junction v_min "
              f"entry {vmin_entry:.0f} / exit {vmin_exit:.0f} mm/s")


if __name__ == "__main__":
    main()
