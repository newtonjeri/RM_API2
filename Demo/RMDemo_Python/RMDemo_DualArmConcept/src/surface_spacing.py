"""
On-surface, curvature-adaptive row spacing for concave fixture interiors.

Two defects in the shipped boustrophedon planner, both measured on commode_c:

  1. PROJECTION METERING. Rows are stepped in the region's PCA best-fit plane,
     so the TRUE on-surface pitch is nominal / |n_face . n_plane|. Measured
     median inflation: bowl_inside_ring 5.4x, sides 9.4x, bowl_inside_rim 2.6x.
     One nominal number therefore means 50 mm on a flat patch and 269 mm on the
     bowl ring.

  2. RIGID-PAD SWATH. Spacing is derived from the glove's 80 mm edge, but a flat
     80 mm pad cannot conform to the bowl. With sagitta sag = |k|l^2/2 against
     the pad's compliance depth t, pressure uniformity eta = 1 - sag/t, so the
     conformable half-width is l <= sqrt(2(1-eta)t/|k|). At the ring/rim
     (|k_perp| p90 = 60-67 1/m, t = 20 mm) the usable swath is 31-33 mm at
     eta = 0.6, not 80 mm.

Fix: meter rows along the surface, and set the pitch from the local across-stroke
curvature. Level sets of the fixture-axis coordinate are the strokes (they are
the low-curvature direction on a bowl: |k_par| p90 = 10-14 vs |k_perp| 60-67).
"""
import numpy as np, trimesh
from scipy.spatial import cKDTree

PAD_ACROSS = 0.080     # glove edge perpendicular to the stroke (glove_frames.yaml)
PAD_ALONG  = 0.035
COMPLIANCE = 0.020     # glove thickness_m, frames 1/2
ETA_MIN    = 0.60      # floor on contact-pressure uniformity across the footprint
OVERLAP    = 0.20      # commanded swath overlap

def conformable_swath(k, t=COMPLIANCE, eta=ETA_MIN, cap=PAD_ACROSS):
    """Full conformable swath (m) at across-stroke curvature k (1/m)."""
    k = np.maximum(np.abs(k), 1e-6)
    return np.minimum(cap, 2.0*np.sqrt(2.0*(1.0-eta)*t/k))

def _principal_curvatures(mesh, radius=0.030):
    """Per-vertex shape operator by local quadric fit. Returns (k_perp, k_par)
    resolved into across-stroke (meridional) and along-stroke (level-set)."""
    Vv, N = mesh.vertices, mesh.vertex_normals
    tree, Z = cKDTree(Vv), np.array([0.,0.,1.])
    kperp = np.zeros(len(Vv)); kpar = np.zeros(len(Vv))
    for i in range(len(Vv)):
        idx = tree.query_ball_point(Vv[i], radius)
        if len(idx) < 7: idx = list(tree.query(Vv[i], 10)[1])
        q = Vv[idx]-Vv[i]; nr = N[i]
        a = np.array([1.,0,0]) if abs(nr[0]) < 0.9 else np.array([0,1.,0])
        e1 = np.cross(nr,a); e1 /= np.linalg.norm(e1); e2 = np.cross(nr,e1)
        u,v,w = q@e1, q@e2, q@nr
        A = np.c_[u*u,u*v,v*v,u,v,np.ones_like(u)]
        try: c = np.linalg.lstsq(A,w,rcond=None)[0]
        except Exception: continue
        a2,b2,c2,d,e = c[:5]; den = np.sqrt(1+d*d+e*e)
        I  = np.array([[1+d*d,d*e],[d*e,1+e*e]])
        II = np.array([[2*a2/den,b2/den],[b2/den,2*c2/den]])
        try: S = np.linalg.solve(I,II)
        except Exception: continue
        t_ = np.cross(nr,Z)                       # along stroke (level set)
        if np.linalg.norm(t_) < 1e-6: continue
        t_ /= np.linalg.norm(t_); b_ = np.cross(nr,t_)   # across stroke
        for d_,out in ((b_,kperp),(t_,kpar)):
            cc = np.array([d_@e1,d_@e2]); cc /= np.linalg.norm(cc)+1e-15
            out[i] = abs(cc@(S@cc))
    return kperp, kpar

def choose_field(mesh, axis_xy=None):
    """Strokes must run along the LOW-curvature direction, so the level-set
    field must run along the HIGH-curvature one. On a bowl wall that is depth
    (horizontal contours); on a flat rim or shelf it is radius (annular
    contours). Decide from the surface's own orientation: area-weighted
    |n . zhat| near 1 = shelf -> radial field; near 0 = wall -> depth field."""
    n = mesh.face_normals; A = mesh.area_faces
    vert = float((np.abs(n[:, 2]) * A).sum() / A.sum())
    return ("radial" if vert > 0.5 else "axial"), vert


def plan_surface_rows(mesh, axis=2, overlap=OVERLAP, eta=ETA_MIN,
                      along_step=None, adaptive=True, curv_radius=0.030,
                      field=None, axis_xy=None):
    """Contour rows metered ON THE SURFACE, pitch set by local curvature.

    field : "axial" (level sets of height -> horizontal contours, bowl walls),
            "radial" (level sets of distance from the fixture axis -> annular
            contours, rims and shelves), or None to choose automatically.

    Returns [{position, normal, tangent}] in the planner's own dict format.
    """
    Vv = mesh.vertices; N = mesh.vertex_normals
    kperp,_ = _principal_curvatures(mesh, curv_radius) if adaptive else (np.zeros(len(Vv)),None)
    tree = cKDTree(Vv)
    if field is None: field, _ = choose_field(mesh)
    if field == "radial":
        if axis_xy is None: axis_xy = Vv[:, :2].mean(0)
        return _radial_rows(mesh, Vv, N, kperp, tree, axis_xy, overlap, eta,
                            along_step, adaptive)
    lo, hi = Vv[:,axis].min(), Vv[:,axis].max()
    e = np.zeros(3); e[axis] = 1.0
    # |grad_S f| = sin(angle between normal and the axis) -> converts an axis
    # step into the true on-surface step.
    def local(c):
        d,i = tree.query(c, 8)
        w = 1.0/np.maximum(d,1e-6); w /= w.sum()
        n = (N[i]*w[:,None]).sum(0); n /= np.linalg.norm(n)+1e-15
        return n, float((kperp[i]*w).sum())
    levels, f = [], hi - 1e-4
    guard = 0
    while f > lo and guard < 2000:
        guard += 1; levels.append(f)
        sec = mesh.section(plane_origin=e*f, plane_normal=e)
        if sec is None: f -= 0.005; continue
        pts = np.asarray(sec.vertices)
        if not len(pts): f -= 0.005; continue
        ns, ks = zip(*[local(p) for p in pts[::max(1,len(pts)//24)]])
        grad = np.mean([max(np.sqrt(max(0.0,1.0-(n@e)**2)),0.15) for n in ns])
        pitch = conformable_swath(np.mean(ks), eta=eta)*(1.0-overlap) if adaptive \
                else PAD_ACROSS*(1.0-overlap)
        f -= max(0.004, pitch*grad)          # axis step giving `pitch` ON the surface
    out, flip = [], False
    for f in levels:
        sec = mesh.section(plane_origin=e*f, plane_normal=e)
        if sec is None: continue
        polys = []
        try:
            for ent in sec.entities:
                idx = np.asarray(ent.points)
                if len(idx) >= 2: polys.append(np.asarray(sec.vertices)[idx])
        except Exception:
            polys = [np.asarray(d) for d in getattr(sec, "discrete", [])]
        for P in polys:
            if len(P) < 2: continue
            L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]
            if L[-1] <= 1e-6: continue
            step = along_step or min(PAD_ALONG*2.0, L[-1]/3.0)
            s = np.arange(0, L[-1], step)
            if len(s) < 2: s = np.array([0.0, L[-1]*0.5])
            Q = np.c_[[np.interp(s,L,P[:,j]) for j in range(3)]].T
            if flip: Q = Q[::-1]
            flip = not flip
            T = np.gradient(Q,axis=0); T /= np.maximum(np.linalg.norm(T,axis=1,keepdims=True),1e-9)
            for j,p in enumerate(Q):
                _,i = tree.query(p); n = N[i]
                out.append({"position":list(map(float,p)),
                            "normal":list(map(float,n)),
                            "tangent":list(map(float,T[j]))})
    return out


def _radial_rows(mesh, Vv, N, kperp, tree, axis_xy, overlap, eta, along_step, adaptive):
    """Annular contours: level sets of distance from the fixture axis. Used for
    rims, shelves and seat rings, where depth contours degenerate."""
    r = np.linalg.norm(Vv[:, :2] - axis_xy, axis=1)
    lo, hi = float(r.min()), float(r.max())
    out, flip, lev, f = [], False, [], hi - 1e-4
    guard = 0
    while f > lo and guard < 2000:
        guard += 1; lev.append(f)
        sel = np.abs(r - f) < 0.010
        if sel.sum() < 3: f -= 0.005; continue
        idx = np.where(sel)[0]
        n = N[idx]
        # |grad_S r| = component of the radial direction lying in the tangent plane
        rad = np.zeros((len(idx), 3)); rad[:, :2] = Vv[idx][:, :2] - axis_xy
        rad /= np.maximum(np.linalg.norm(rad, axis=1, keepdims=True), 1e-9)
        grad = max(float(np.mean(np.sqrt(np.maximum(0.0, 1.0 - (n*rad).sum(1)**2)))), 0.15)
        k = float(np.mean(kperp[idx])) if adaptive else 0.0
        pitch = conformable_swath(k, eta=eta)*(1.0-overlap) if adaptive \
                else PAD_ACROSS*(1.0-overlap)
        f -= max(0.004, pitch*grad)
    for f in lev:
        sel = np.where(np.abs(r - f) < 0.012)[0]
        if len(sel) < 3: continue
        P = Vv[sel]
        th = np.arctan2(P[:, 1]-axis_xy[1], P[:, 0]-axis_xy[0])
        o = np.argsort(th); P = P[o]
        L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
        if L[-1] <= 1e-6: continue
        step = along_step or min(PAD_ALONG*2.0, L[-1]/3.0)
        s_ = np.arange(0, L[-1], step)
        if len(s_) < 2: s_ = np.array([0.0, L[-1]*0.5])
        Q = np.c_[[np.interp(s_, L, P[:, j]) for j in range(3)]].T
        if flip: Q = Q[::-1]
        flip = not flip
        T = np.gradient(Q, axis=0); T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
        for j, p in enumerate(Q):
            _, i = tree.query(p)
            out.append({"position": list(map(float, p)),
                        "normal": list(map(float, N[i])),
                        "tangent": list(map(float, T[j]))})
    return out


BLEND_FLOOR_M = 0.030
def enforce_min_segment(pts, min_m=BLEND_FLOOR_M):
    """Drop waypoints closer than the controller's blend floor.

    Measured on this controller (MOTION_FINDINGS 9.3f/9.3g): segments below
    ~25-30 mm fall below the blend floor and the planner substitutes a FULL
    STOP -- 29 stops in 40 corners on one task -- and the threshold is not a
    clean function of length, so sub-30 mm hops must be treated as unsafe
    rather than merely slow. Keeps the last point of every run.
    """
    if len(pts) < 3: return pts
    out = [pts[0]]
    for p in pts[1:]:
        d = np.linalg.norm(np.asarray(p["position"]) - np.asarray(out[-1]["position"]))
        if d >= min_m: out.append(p)
    if len(out) > 1:
        last = np.asarray(pts[-1]["position"])
        if np.linalg.norm(last - np.asarray(out[-1]["position"])) > 1e-9: out[-1] = pts[-1]
    return out
