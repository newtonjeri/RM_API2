"""Re-run every check in ROBOT_MODEL.md. Offline, no arm.

    python3 verify_robot_model.py

A document full of numbers is a claim; this is the claim under test. It
re-derives the kinematics from the SDK, checks them against the vendor's
published figures and against the URDF, and — the one that needed hardware
to settle — identifies WHICH RM75 variant our arms are, from a recording.

Every constant the vendor publishes is written out here as an expectation.
If RealMan revise the model, or the SDK ships a new algorithm library, this
fails and names the field rather than letting ROBOT_MODEL.md drift.
"""

import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help                          # noqa: E402
import orientation_cost as oc                             # noqa: E402

# ── what the vendor publishes, transcribed once ────────────────────────────
# RealMan RM75 Ontology Parameters + rm_models URDF. Quoted, not computed:
# these are the independent side of every comparison below.
VENDOR_D = [0.2405, 0.0, 0.2560, 0.0, 0.2100, 0.0, 0.1612]   # 6FB
VENDOR_ALPHA = [-90.0, 90.0, -90.0, 90.0, -90.0, 90.0, 0.0]
VENDOR_RANGE = [178.0, 130.0, 178.0, 135.0, 178.0, 128.0, 360.0]
VENDOR_SPEED = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]
VENDOR_REACH_MM = 627.0                 # 6FB working radius
VENDOR_MASS_KG = 7.8                    # self-weight
# The four singularity types RealMan document, with their example
# configurations. These are the independent side of the singularity check:
# if a future algorithm library stops flagging one of them, this fails.
VENDOR_SINGULAR = {
    "type1 q2=0,q6=0":     [0, 0, 0, 90, 0, 0, 0],
    "type2 q4=0 (elbow)":  [0, 60, 0, 0, 0, 90, 0],
    "type3 q2=0,q3=+-90":  [0, 0, 90, 90, 0, 90, 0],
    "type4 q6=0,q5=+-90":  [0, 90, 90, 90, 90, 0, 0],
}
BENIGN = [0, -30, 0, 60, 0, 30, 0]
SING_THRESHOLD = 0.01                   # SDK default, minimum singular value
ANALYTIC_THRESHOLDS = (10.0, 10.0, 0.05)   # limit_qe deg, limit_qw deg, limit_d m
# butterfli.urdf right-arm wrist: joint6->joint7 axis + fixed camera holder
LOCAL_RIGHT_WRIST = 0.114 + 0.05
URDF_MASS = [1.862, 1.574, 1.217, 1.110, 0.685, 0.619, 0.602, 0.144]
URDF_JOINT_ORIGIN = [0.2405, 0.256, 0.210, 0.1612]   # the non-zero ones
D7_VARIANTS = {"RM75-B": 0.1440, "RM75-6FB": 0.1612, "RM75-B-V": 0.1668,
               "RM75-6F": 0.1725, "RM75-6FB-V": 0.1840}

fails = []


def check(name, ok, detail=""):
    print("  %-52s %s%s" % (name, "ok" if ok else "FAIL",
                            "   " + detail if detail else ""))
    if not ok:
        fails.append(name)
    return ok


def dh_now():
    d = oc._algo.rm_algo_get_dh()
    return ([float(x) for x in d["d"]], [float(x) for x in d["a"]],
            [float(x) for x in d["alpha"]], [float(x) for x in d["offset"]])


def fk_from_dh(q, d, a, alpha, offset):
    """Standard DH: T_i = Rz(t+off) Tz(d) Tx(a) Rx(alpha)."""
    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
                for i in range(4)]
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for i in range(7):
        th = math.radians(q[i] + offset[i])
        al = math.radians(alpha[i])
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        T = mm(T, [[ct, -st * ca, st * sa, a[i] * ct],
                   [st, ct * ca, -ct * sa, a[i] * st],
                   [0, sa, ca, d[i]],
                   [0, 0, 0, 1]])
    return T


def tcp_world(q, tool):
    """Recorded-frame TCP from joint angles: FK -> tool -> mount rotation."""
    f = oc.fk(q)
    R = oc._Rmat(*f[3:6])
    t = oc.TOOL_OFFSETS[tool]
    tip = [f[i] + sum(R[i][k] * t[k] for k in range(3)) for i in range(3)]
    Rm = oc._Ry(math.radians(oc.MOUNT_RY_DEG))
    return [sum(Rm[i][k] * tip[k] for k in range(3)) for i in range(3)]


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    print("Algorithm library: %s\n" % oc._algo.rm_algo_version())

    # ── 1. the SDK's own numbers against the vendor's ──────────────────────
    print("1. SDK vs vendor")
    d, a, alpha, offset = dh_now()
    check("DH d matches the published MDH d column",
          all(abs(d[i] - VENDOR_D[i]) < 1e-6 for i in range(7)),
          str([round(v, 4) for v in d]))
    check("DH alpha matches", all(abs(alpha[i] - VENDOR_ALPHA[i]) < 1e-6
                                  for i in range(7)))
    check("DH a is all zero (pure S-R-S, no link offsets)",
          all(abs(v) < 1e-9 for v in a))
    check("DH offset is all zero (joint angle == model angle)",
          all(abs(v) < 1e-9 for v in offset))
    mx = oc._algo.rm_algo_get_joint_max_limit()
    mn = oc._algo.rm_algo_get_joint_min_limit()
    check("joint ranges match the vendor page",
          all(abs(mx[i] - VENDOR_RANGE[i]) < 0.05
              and abs(mn[i] + VENDOR_RANGE[i]) < 0.05 for i in range(7)))
    check("joint ranges are symmetric about zero",
          all(abs(mx[i] + mn[i]) < 1e-6 for i in range(7)))
    sp = oc._algo.rm_algo_get_joint_max_speed()
    check("joint max speeds match (180/180/225x5)",
          all(abs(sp[i] - VENDOR_SPEED[i]) < 1e-6 for i in range(7)))

    # ── 2. the DH actually generates the SDK's kinematics ──────────────────
    print("\n2. The DH table reproduces the SDK's forward kinematics")
    worst, q_worst = 0.0, None
    # A fixed sweep, not random: a self-test that tests something different
    # every run cannot fail reproducibly.
    step = 37.0
    for n in range(200):
        q = [((n * step + 53 * j) % 240.0) - 120.0 for j in range(7)]
        q = [max(mn[j] + 1, min(mx[j] - 1, q[j])) for j in range(7)]
        T = fk_from_dh(q, d, a, alpha, offset)
        ref = oc.fk(q)
        e = math.dist([T[0][3], T[1][3], T[2][3]], ref[:3])
        if e > worst:
            worst, q_worst = e, q
    check("standard-DH FK == rm_algo_forward_kinematics (200 poses)",
          worst < 1e-5, "worst %.6f mm" % (1000 * worst))

    # ── 3. reach and mass, the two independent published cross-checks ──────
    print("\n3. Derived figures against published specifications")
    reach = 1000 * (sum(d) - d[0])
    check("reach from DH == published 6FB working radius",
          abs(reach - VENDOR_REACH_MM) < 1.0,
          "%.1f vs %.0f mm" % (reach, VENDOR_REACH_MM))
    check("URDF link masses sum to the published self-weight",
          abs(sum(URDF_MASS) - VENDOR_MASS_KG) < 0.05,
          "%.3f vs %.1f kg" % (sum(URDF_MASS), VENDOR_MASS_KG))
    nz = [v for v in d if v > 1e-9]
    check("URDF joint origins == the DH d column",
          all(abs(nz[i] - URDF_JOINT_ORIGIN[i]) < 1e-6
              for i in range(len(nz))))

    # ── 4. which variant do we actually have ──────────────────────────────
    # The one question no document can answer: five RM75 variants differ ONLY
    # in d7, by up to 40 mm, and picking wrong displaces every Cartesian
    # target along the tool axis. Settled by reconstructing a recorded TCP
    # from the recorded joint angles and seeing which d7 reproduces it.
    print("\n4. Which variant are our arms? (from a recording)")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # BOTH arms, because "our arms are 6FB" is a claim about two machines and
    # the right one carries a camera that the -V URDF folds into link 7. One
    # arm passing says nothing about the other.
    for side, pat, tools in (
            ("right", "*_right_right", ("R_glove_2", "R_glove_4", "R_glove_3")),
            ("left", "*_left_left", ("L_glove_2", "L_glove_4", "L_glove_3"))):
        runs = sorted(glob.glob(os.path.join(root, "runs", pat)))
        if not runs:
            print("  [SKIP] no %s-arm run under runs/ to test against" % side)
            continue
        rd = runs[-1]
        rows = list(csv.DictReader(open(os.path.join(rd, "stream.csv"))))
        rows = [r for r in rows if "tcp_x" in r and "position1" in r]
        best = None
        from Robotic_Arm.rm_ctypes_wrap import rm_dh_t
        try:
            for name, d7 in sorted(D7_VARIANTS.items(), key=lambda kv: kv[1]):
                dd = list(d)
                dd[6] = d7
                oc._algo.rm_algo_set_dh(rm_dh_t(
                    d=dd + [0.0], a=a + [0.0],
                    alpha=alpha + [0.0], offset=offset + [0.0]))
                for tool in tools:
                    errs = []
                    for r in rows[::200]:
                        q = [float(r["position%d" % j]) for j in range(1, 8)]
                        p = (float(r["tcp_x"]), float(r["tcp_y"]),
                             float(r["tcp_z"]))
                        errs.append(1000 * math.dist(tcp_world(q, tool), p))
                    errs.sort()
                    med = errs[len(errs) // 2]
                    if best is None or med < best[0]:
                        best = (med, name, tool, d7)
        finally:
            # ALWAYS put it back. rm_algo_set_dh mutates library state for the
            # whole process; leaving it on a trial value would silently
            # corrupt every FK and IK afterwards.
            oc._algo.rm_algo_set_dh(rm_dh_t(
                d=d + [0.0], a=a + [0.0],
                alpha=alpha + [0.0], offset=offset + [0.0]))
        d2, a2, al2, of2 = dh_now()
        check("DH restored after the variant sweep",
              all(abs(d2[i] - d[i]) < 1e-9 for i in range(7)))
        print("  %s arm, best fit on %s:" % (side, os.path.basename(rd)))
        print("     %s with tool %s -> median %.2f mm"
              % (best[1], best[2], best[0]))
        check("our %s arm is RM75-6FB (d7 = 0.1612)" % side,
              best[1] == "RM75-6FB" and best[0] < 1.0,
              "%.2f mm" % best[0])

    # ── 4b. singularities ─────────────────────────────────────────────────
    print("\n4b. Singularities")
    check("analytic thresholds are the documented 10 deg / 10 deg / 0.05 m",
          all(abs(x - y) < 1e-6 for x, y in
              zip(oc._algo.rm_algo_kin_singularity_thresholds_init()
                  or oc._algo.rm_algo_kin_get_singularity_thresholds(),
                  ANALYTIC_THRESHOLDS)))
    hit = 0
    for name, q in VENDOR_SINGULAR.items():
        v = oc._algo.rm_algo_universal_singularity_analyse(q, SING_THRESHOLD)
        hit += (v == -1)
    check("all 4 documented singular configurations are flagged",
          hit == 4, "%d/4" % hit)
    check("a benign pose is NOT flagged",
          oc._algo.rm_algo_universal_singularity_analyse(
              BENIGN, SING_THRESHOLD) == 0)
    # The 6-DOF-only analytic call must not be used on this arm — confirm it
    # actually refuses 7 joints rather than silently truncating, because a
    # silent truncation would return a verdict about a different robot.
    try:
        oc._algo.rm_algo_kin_robot_singularity_analyse([0.0] * 7)
        check("the 6-DOF analytic call rejects 7 joints", False,
              "it ACCEPTED them — a verdict from it would be meaningless")
    except (IndexError, ValueError):
        check("the 6-DOF analytic call rejects 7 joints (as documented)", True)
    # Our own continuous measure must agree with the SDK's verdict, or the
    # sigma_min numbers in ROBOT_MODEL.md 3.2 are not comparable to its
    # threshold. Per-RADIAN: oc.jacobian is per-degree.
    try:
        import numpy as np

        def sigma_min(q):
            J = np.array(oc.jacobian(q)) * (180.0 / math.pi)
            return float(np.linalg.svd(J, compute_uv=False)[-1])
        agree = 0
        cases = list(VENDOR_SINGULAR.items()) + [
            ("benign", BENIGN), ("q4=5", [0, 60, 0, 5, 0, 90, 0]),
            ("q4=20", [0, 60, 0, 20, 0, 90, 0]),
            # A pose the arm has actually held: the blend path's start, read
            # off the pendant. A synthetic sweep that never includes a real
            # configuration is not evidence about this cell.
            ("our blend-path start",
             [-3.025, -69.557, 7.452, 134.344, 20.974, 2.191, 161.567])]
        for name, q in cases:
            mine = sigma_min(q) < SING_THRESHOLD
            sdk = oc._algo.rm_algo_universal_singularity_analyse(
                q, SING_THRESHOLD) == -1
            agree += (mine == sdk)
        check("per-radian sigma_min agrees with the SDK verdict",
              agree == len(cases), "%d/%d" % (agree, len(cases)))
    except ImportError:
        print("  [SKIP] sigma_min cross-check needs numpy")

    # ── 4c. our workspace URDF ────────────────────────────────────────────
    print("\n4c. butterfli.urdf right arm (ROBOT_MODEL.md section 6)")
    off = LOCAL_RIGHT_WRIST - VENDOR_D[6]
    check("its joint6->flange is 2.8 mm LONG vs the measured d7",
          abs(off - 0.0028) < 0.0002,
          "%.4f vs %.4f m (%+.1f mm)"
          % (LOCAL_RIGHT_WRIST, VENDOR_D[6], 1000 * off))

    # ── 5. the tool+mount transform ───────────────────────────────────────
    print("\n5. Tool and mount transform")
    err = oc.selfcheck("L_glove_4")
    if err is None:
        print("  [SKIP] no reference run/plan present for the self-check")
    else:
        check("tool+mount reproduces a recorded commanded pose",
              err < 0.001, "%.1f um" % (1e6 * err))

    print("\n%s  (%d failure%s)"
          % ("ROBOT_MODEL.md VERIFIED" if not fails else "VERIFICATION FAILED",
             len(fails), "" if len(fails) == 1 else "s"))
    for f in fails:
        print("   - %s" % f)
    return 1 if fails else 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    raise SystemExit(main())
