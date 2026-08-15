#!/usr/bin/env python3
"""
bowl_audit.py -- reproduces every number in BOWL_PATH_FINDINGS.md.

Runs the SHIPPED alix_taskgen pipeline (surface_type_detector -> plan_path ->
coverage) against the real commode_c labeled meshes, measures the two defects
it exposes, and scores the surface_spacing.py prototype against it.

    python3 bowl_audit.py --models /path/to/commode_c/models \
                          --taskgen /path/to/alix_taskgen/src

Requires numpy, scipy, trimesh, shapely.

NOTE ON THE PRESS SIGN. alix_taskgen.pose_generator resolves the global press
direction by a majority vote anchored on the arm's real start pose, which is not
available offline. This script therefore builds the contact frames directly
(press = -outward normal, tool X = path tangent) and calls the project's own
compute_coverage with them. Without that, every concave region scores exactly
0.0% -- the documented signature of an unanchored press sign, not under-coverage.
"""
import argparse, sys, warnings, numpy as np
warnings.filterwarnings("ignore")

ap = argparse.ArgumentParser()
ap.add_argument("--models", default="/mnt/user-data/uploads/alix/ros/orchestration/"
                "alix_tasks/config/commode_cleaning/commode_c/models")
ap.add_argument("--taskgen", default="pkg")
ap.add_argument("--samples", type=int, default=6000)
A = ap.parse_args()
sys.path.insert(0, A.taskgen)

import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from alix_taskgen.surface_type_detector import detect_surface_type
from alix_taskgen.path_planners import plan_path
from alix_taskgen import coverage as COV, constants as C
from surface_spacing import plan_surface_rows, enforce_min_segment, choose_field

BOWL = ["bowl_inside_front", "bowl_inside_back", "bowl_inside_ring",
        "bowl_inside_rim", "top", "sides"]
CAP = 0.60          # rad/s -- factory TCP angular cap
START = [0., 0., 0., 0., 0., 0., 1.]

def load_obj(path):
    V, groups, cur = [], {}, None
    for line in open(path):
        if line.startswith("v "):   V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("o "): cur = line[2:].strip().split(".")[0]; groups.setdefault(cur, [])
        elif line.startswith("f "):
            i = [int(t.split("/")[0]) for t in line.split()[1:]]
            i = [k-1 if k > 0 else len(V)+k for k in i]
            for k in range(1, len(i)-1): groups[cur].append((i[0], i[k], i[k+1]))
    return np.array(V), {k: np.array(v) for k, v in groups.items() if len(v)}

V, G = load_obj(f"{A.models}/body_regions.obj")
AXIS = np.vstack([V[np.unique(G[k])] for k in BOWL[:4]])[:, :2].mean(0)

def inward(c):
    t = np.zeros_like(c); t[..., 0] = AXIS[0]-c[..., 0]; t[..., 1] = AXIS[1]-c[..., 1]
    return t/np.maximum(np.linalg.norm(t, axis=-1, keepdims=True), 1e-9)

def mesh(k):
    F = G[k]; vv = np.unique(F); rm = {v: i for i, v in enumerate(vv)}
    m = trimesh.Trimesh(vertices=V[vv], faces=np.vectorize(rm.get)(F), process=False)
    bad = (m.face_normals*inward(m.triangles_center)).sum(1) < 0
    f = m.faces.copy(); f[bad] = f[bad][:, [0, 2, 1]]
    return trimesh.Trimesh(vertices=m.vertices, faces=f, process=False)

def score(m, pts):
    if len(pts) < 2: return None
    cp = {}
    for i, p in enumerate(pts):
        pos = np.array(p["position"]); n = np.array(p["normal"]); t = np.array(p["tangent"])
        if n @ inward(pos) < 0: n = -n
        pr = -n; w = t-(t@pr)*pr; nw = np.linalg.norm(w)
        w = (w/nw) if nw > 1e-6 else np.cross(pr, [0, 0, 1.]); w /= np.linalg.norm(w)
        R = np.column_stack([w, np.cross(pr, w), pr])
        cp[f"p{i}"] = {"translation": list(pos),
                       "rotation": list(Rotation.from_matrix(R).as_euler("XYZ", degrees=True))}
    seq = [[f"p{i}", f"p{i+1}"] for i in range(len(pts)-1)]
    r = COV.compute_coverage(m, START, {"cleaning_points": cp, "cleaning_sequence": seq},
                             contact_axis="z", samples=A.samples)
    P = np.array([q["position"] for q in pts]); d = np.linalg.norm(np.diff(P, axis=0), axis=1)*1000
    N = np.array([q["normal"] for q in pts]); N = np.where((N*inward(P)).sum(1, keepdims=True) < 0, -N, N)
    ang = np.arccos(np.clip((N[:-1]*N[1:]).sum(1), -1, 1)); ok = d > 0.1
    r.update(_n=len(pts), _len=d.sum()/1000, _short=100*(d < 30).mean(),
             _wv50=np.percentile(ang[ok]/(d[ok]/1000), 50),
             _wv90=np.percentile(ang[ok]/(d[ok]/1000), 90))
    return r

def principal(m, radius=0.030):
    Vv, N = m.vertices, m.vertex_normals; tree = cKDTree(Vv); Z = np.array([0., 0., 1.])
    kperp, kpar = [], []
    for i in range(len(Vv)):
        idx = tree.query_ball_point(Vv[i], radius)
        if len(idx) < 7: idx = list(tree.query(Vv[i], 10)[1])
        q = Vv[idx]-Vv[i]; nr = N[i]
        a = np.array([1., 0, 0]) if abs(nr[0]) < 0.9 else np.array([0, 1., 0])
        e1 = np.cross(nr, a); e1 /= np.linalg.norm(e1); e2 = np.cross(nr, e1)
        u, v, w = q@e1, q@e2, q@nr
        try: c = np.linalg.lstsq(np.c_[u*u, u*v, v*v, u, v, np.ones_like(u)], w, rcond=None)[0]
        except Exception: continue
        a2, b2, c2, d, e = c[:5]; den = np.sqrt(1+d*d+e*e)
        try: S = np.linalg.solve(np.array([[1+d*d, d*e], [d*e, 1+e*e]]),
                                 np.array([[2*a2/den, b2/den], [b2/den, 2*c2/den]]))
        except Exception: continue
        t_ = np.cross(nr, Z)
        if np.linalg.norm(t_) < 1e-6: continue
        t_ /= np.linalg.norm(t_); b_ = np.cross(nr, t_)
        for d_, out in ((b_, kperp), (t_, kpar)):
            cc = np.array([d_@e1, d_@e2]); cc /= np.linalg.norm(cc)+1e-15
            out.append(abs(cc@(S@cc)))
    return np.array(kperp), np.array(kpar)

print("="*94)
print("A. REAL CURVATURE  (local quadric fit, 30 mm neighbourhood, resolved to the stroke)")
print("="*94)
print("%-20s %7s %8s %8s %8s %8s %9s" % ("region", "area m2", "kpar p90", "kperp p90",
                                          "kperp p95", "swath@.6", "R_fillet"))
KP = {}
for k in BOWL:
    m = mesh(k); kp, ka = principal(m)
    p90 = float(np.percentile(kp, 90)); KP[k] = p90
    sw = 2000*min(0.040, np.sqrt(2*(1-0.60)*0.020/max(p90, 1e-6)))
    print("%-20s %7.4f %8.1f %8.1f %8.1f %7.0fmm %7.0fmm" % (
        k, m.area, np.percentile(ka, 90), p90, np.percentile(kp, 95), sw, 1000/p90))

print("\n"+"="*94)
print("B. PROJECTION METERING  (true on-surface row pitch = nominal / |n_face . n_PCAplane|)")
print("="*94)
print("%-20s %-17s %9s %9s %13s" % ("region", "surface_type", "infl p50", "infl p90", "true pitch@50"))
for k in BOWL:
    m = mesh(k); nrm = m.face_normals; Ar = m.area_faces
    Vc = m.vertices-m.vertices.mean(0); _, _, vh = np.linalg.svd(Vc, full_matrices=False)
    infl = 1./np.maximum(np.abs(nrm@vh[2]), 1e-6)
    o = np.argsort(infl); cw = np.cumsum((Ar/Ar.sum())[o])
    i50 = infl[o][np.searchsorted(cw, .5)]; i90 = infl[o][np.searchsorted(cw, .9)]
    print("%-20s %-17s %9.2f %9.2f %10.0f mm" % (k, detect_surface_type(m), i50, i90, 50*i50))

print("\n"+"="*94)
print("C. SHIPPED vs PROTOTYPE   (project's own compute_coverage, %d samples)" % A.samples)
print("="*94)
print("%-20s %-9s %5s %7s %8s %8s %7s %9s" % ("region", "planner", "n", "cov%",
                                               "redund", "path m", "<30mm", "w/v p90"))
tot = [0., 0.]
for k in BOWL[:5]:
    m = mesh(k); st = detect_surface_type(m)
    a = score(m, plan_path(m, st, C.AUTO_SPACING_M, along_step_m=C.AUTO_ALONG_STEP_M))
    b = score(m, enforce_min_segment(plan_surface_rows(m)))
    for tag, r in (("shipped", a), ("prototype", b)):
        if r is None: print("%-20s %-9s   (empty)" % (k, tag)); continue
        print("%-20s %-9s %5d %7.1f %8.2f %8.2f %6.0f%% %9.1f" % (
            k, tag, r["_n"], r["coverage_pct"], r["redundancy"], r["_len"], r["_short"], r["_wv90"]))
    if a: tot[0] += a["_len"]
    if b: tot[1] += b["_len"]
print("%-20s %-9s %25s %8.2f" % ("TOTAL", "shipped", "", tot[0]))
print("%-20s %-9s %25s %8.2f   (%+.0f%%)" % ("TOTAL", "prototype", "", tot[1],
                                             100*(tot[1]/tot[0]-1)))

print("\n"+"="*94)
print("D. ANGULAR-CAP CEILING  (cap %.2f rad/s; v_max = cap / (dpress/ds))" % CAP)
print("="*94)
print("%-20s %10s %10s %11s %11s" % ("region", "w/v p50", "w/v p90", "v@p50 mm/s", "v@p90 mm/s"))
for k in BOWL[:5]:
    m = mesh(k)
    r = score(m, plan_path(m, detect_surface_type(m), C.AUTO_SPACING_M, along_step_m=C.AUTO_ALONG_STEP_M))
    print("%-20s %10.1f %10.1f %11.0f %11.0f" % (k, r["_wv50"], r["_wv90"],
                                                  1000*CAP/r["_wv50"], 1000*CAP/r["_wv90"]))
