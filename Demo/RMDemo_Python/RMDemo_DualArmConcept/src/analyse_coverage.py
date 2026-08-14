"""Sequence-aware coverage analysis of blend runs.

The point-set test failed because toplid/hinge revisit waypoints: distance to
the nearest polyline segment is small everywhere in the criss-crossed region.
Fix: match the trace to the commanded path MONOTONICALLY in sequence
(DTW-style DP), then ask, for every arc position s of the commanded path,
"was the tool near C(s) at the sequence-correct time?"  Coverage gaps found
this way cannot be faked by a different pass through the same region.
"""
import json, csv, sys
import numpy as np

STEP = 0.002          # commanded-path discretization (m)
WIN = 0.030           # +/- window (m) around s when probing coverage distance
SMOOTH_S = 0.07       # REAL position channel: box-smooth >= 70 ms (aliasing rule)

def load_run(rundir):
    d = json.load(open(f"{rundir}/run.json"))
    rows = list(csv.DictReader(open(f"{rundir}/stream.csv")))
    T = np.array([[float(r["tcp_x"]), float(r["tcp_y"]), float(r["tcp_z"])] for r in rows])
    t = np.array([float(r["t_mono"]) for r in rows])
    if not d.get("sim"):
        dt = np.median(np.diff(t)) if len(t) > 1 else 0.01
        w = max(1, int(round(SMOOTH_S / dt)))
        if w % 2 == 0:
            w += 1
        k = np.ones(w) / w
        pad = w // 2
        Tp = np.pad(T, ((pad, pad), (0, 0)), mode="edge")
        T = np.stack([np.convolve(Tp[:, i], k, mode="valid") for i in range(3)], axis=1)
    P = np.array([p[:3] for p in d["commanded"]["poses"]])
    return d, T, t, P

def discretize(P):
    """Polyline -> points every STEP. Returns C (N,3), s (N,), vertex arc-positions,
    per-vertex geometry (turn angle deg, L_in, L_out). Near-zero segments kept as
    zero-advance vertices (no discretization points, geometry marked None)."""
    segs = np.diff(P, axis=0)
    L = np.linalg.norm(segs, axis=1)
    s_v = np.concatenate([[0.0], np.cumsum(L)])       # arc position of each waypoint
    pts, s_list = [], []
    for i in range(len(L)):
        if L[i] < 1e-6:
            continue
        n = max(1, int(np.ceil(L[i] / STEP)))
        f = np.arange(n) / n
        pts.append(P[i] + f[:, None] * segs[i])
        s_list.append(s_v[i] + f * L[i])
    pts.append(P[-1][None, :]); s_list.append(np.array([s_v[-1]]))
    C = np.concatenate(pts); s = np.concatenate(s_list)
    # per interior vertex: turn angle (0 = straight, 180 = reversal)
    geom = []
    for k in range(1, len(P) - 1):
        li, lo = L[k - 1], L[k]
        if li < 1e-6 or lo < 1e-6:
            geom.append((k, s_v[k], None, li, lo))
            continue
        u_in = segs[k - 1] / li
        u_out = segs[k] / lo
        cosv = np.clip(np.dot(u_in, u_out), -1, 1)
        turn = np.degrees(np.arccos(cosv))            # 0 = straight-through
        geom.append((k, s_v[k], turn, li, lo))
    return C, s, s_v, geom

def monotone_match(T, C):
    """DP: s index per trace sample, monotone nondecreasing, free endpoints.
    Returns idx (len T) into C, and matched distance per sample."""
    S = len(C)
    nT = len(T)
    Dprev = np.linalg.norm(C - T[0], axis=1)
    ptr = np.zeros((nT, S), dtype=np.int32)
    aridx = np.arange(S, dtype=np.int32)
    for ti in range(1, nT):
        pm = np.minimum.accumulate(Dprev)
        newmin = Dprev <= pm
        arg = np.maximum.accumulate(np.where(newmin, aridx, 0))
        ptr[ti] = arg
        Dprev = np.linalg.norm(C - T[ti], axis=1) + pm
    end = int(np.argmin(Dprev))
    idx = np.empty(nT, dtype=np.int32)
    idx[-1] = end
    for ti in range(nT - 1, 0, -1):
        idx[ti - 1] = ptr[ti][idx[ti]]
    dmatch = np.linalg.norm(T - C[idx], axis=1)
    return idx, dmatch

def coverage_profile(T, C, s, idx):
    """g(s): for every discrete path point, min distance to any trace sample whose
    matched arc position is within +/- WIN. Sequence-consistent by construction."""
    s_t = s[idx]                                     # monotone
    g = np.full(len(C), np.inf)
    lo = np.searchsorted(s_t, s - WIN, side="left")
    hi = np.searchsorted(s_t, s + WIN, side="right")
    for i in range(len(C)):
        if hi[i] > lo[i]:
            seg = T[lo[i]:hi[i]] - C[i]
            g[i] = np.sqrt((seg * seg).sum(axis=1).min())
    return g, s_t

def analyse(rundir, tol):
    d, T, t, P = load_run(rundir)
    C, s, s_v, geom = discretize(P)
    idx, dmatch = monotone_match(T, C)
    g, s_t = coverage_profile(T, C, s, idx)
    total = s[-1]
    covered = (g <= tol)
    cov_len = covered.sum() * STEP
    reach = s_t[-1]                                   # how far along the path we got
    corners = []
    for (k, sv, turn, li, lo_) in geom:
        j = np.searchsorted(s, sv)
        j = min(j, len(s) - 1)
        # entry cut: distance back from vertex to last covered point at s<=sv
        back = np.where(covered[: j + 1])[0]
        entry = sv - s[back[-1]] if len(back) else sv
        fwd = np.where(covered[j:])[0]
        exit_ = (s[j + fwd[0]] - sv) if len(fwd) else (total - sv)
        # sequence-consistent closest approach to the vertex itself
        vd = g[j]
        corners.append(dict(k=k, s=sv, turn=turn, L_in=li, L_out=lo_,
                            entry=entry, exit=exit_, vertex_miss=vd))
    names = d["commanded"].get("waypoint_names")
    return dict(run=rundir.rstrip("/").split("/")[-1], task=d.get("path_file"),
                side=d.get("side"), sim=bool(d.get("sim")),
                r=d["speeds"].get("blend_pct"), v=d.get("line_speed_cap_m_s"),
                total=total, cov_len=cov_len, cov_frac=cov_len / total,
                reach=reach, short=total - reach,
                med_dmatch=float(np.median(dmatch)), p95_dmatch=float(np.percentile(dmatch, 95)),
                corners=corners, names=names, tol=tol)

if __name__ == "__main__":
    rundir = sys.argv[1]
    tol = float(sys.argv[2]) if len(sys.argv) > 2 else 0.002
    r = analyse(rundir, tol)
    print(f"{r['run']}  {r['task']}  {'SIM' if r['sim'] else 'REAL'}  r={r['r']} v={r['v']}")
    print(f"  path {r['total']*1000:.0f} mm | covered {r['cov_frac']*100:.1f}% @tol {tol*1000:.0f}mm"
          f" | reached {r['reach']*1000:.0f} ({r['short']*1000:.0f} short)"
          f" | match residual med {r['med_dmatch']*1000:.2f} p95 {r['p95_dmatch']*1000:.2f} mm")
    print(f"  {'k':>3} {'name':<10} {'turn':>6} {'L_in':>6} {'L_out':>6} {'entry':>7} {'exit':>7} {'vmiss':>6}")
    for c in r["corners"]:
        nm = r["names"][c["k"]] if r["names"] else ""
        tv = f"{c['turn']:6.1f}" if c["turn"] is not None else "  n/a "
        print(f"  {c['k']:>3} {nm:<10} {tv} {c['L_in']*1000:6.0f} {c['L_out']*1000:6.0f}"
              f" {c['entry']*1000:7.1f} {c['exit']*1000:7.1f} {c['vertex_miss']*1000:6.1f}")
