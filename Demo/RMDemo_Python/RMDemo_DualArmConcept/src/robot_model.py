"""The RM75-6FB model, in code. The counterpart to ROBOT_MODEL.md.

Import this rather than re-deriving kinematics. Everything here is checked by
`verify_robot_model.py`, and every constant carries the source it came from.

    from robot_model import sigma_min, gravity_torque, joint_frames

WHY A MODULE AND NOT A COPY IN EACH SCRIPT. Two months of this work has been
frame and model errors — the tool offset omitted from IK, the Euler wrap on
the right arm's rz, the mount rotation, interpolating between flange
endpoints instead of in TCP space. Each cost a hardware session. They were
all the same failure: a kinematic detail re-derived in a new file by someone
in a hurry. The DH here is verified to 0.00008 mm against the SDK's own
forward kinematics; use it and that class of bug cannot recur.

WHAT IS NOT HERE, and why:

  * A predictor for all seven joints along a `movel`. It cannot exist
    offline. Measured 2026-08-13: the ARM ANGLE swings 80-153 deg
    peak-to-peak within a single run, so the controller re-resolves the
    redundancy as it goes and the configuration is not a function of the
    path. Only J4 is redundancy-invariant, which is why the screens in
    `orientation_cost` report J4 and nothing else.
  * Torque in amps. No torque constant is published and the SDK has no
    torque channel, so `gravity_torque` returns N*m and the current channel
    stays a RELATIVE indicator until RealMan supply K_t.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import orientation_cost as oc                            # noqa: E402

# ── the arm, as ROBOT_MODEL.md records it ─────────────────────────────────
# Link masses and centres of mass: RealMan rm_models URDF (RM75-6FB), link_1
# through link_7. They sum with base_link to 7.813 kg against a published
# self-weight of 7.8 kg, which is what makes them credible.
LINK_MASS = [1.574, 1.217, 1.110, 0.685, 0.619, 0.602, 0.144]
LINK_COM = [(0.000241, -0.013273, -0.009950),
            (-0.000357, -0.106789, 0.005329),
            (0.000003, -0.013980, -0.011324),
            (-0.000005, -0.084658, 0.004747),
            (0.000078, -0.012937, -0.008781),
            (-0.000014, -0.078524, 0.002819),
            (0.001094, -0.000077, -0.010119)]
JOINT_EFFORT_NM = [60.0, 60.0, 30.0, 30.0, 10.0, 10.0, 10.0]   # URDF
JOINT_SPEED_LIMIT = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]
G = 9.81

# The SDK's default threshold on the Jacobian's minimum singular value.
# `sigma_min` below returns the SDK's OWN value by bisecting this analyser,
# so the two agree by construction rather than by a scaling assumption.
SINGULARITY_THRESHOLD = 0.01
# Where we start looking, at 3x the SDK's own threshold. Nothing we have run
# comes close: the lowest across every recording is toplid_right at 0.0285,
# i.e. 2.85x.
#
# IT IS NOT A PREDICTOR OF JOINT LOAD, and an earlier version of this file
# claimed it was. Measured with the SDK's own value, the correlation between
# sigma_min and maximum joint utilisation is -0.40 and -0.47 on the two
# test_motion runs but +0.49 on toplid_right, whose peak joint demand occurs
# at sigma_min 0.107 against a run median of 0.076 — a WELL-conditioned pose.
# The sign is path-dependent, so this must not be used to gate anything.
# Report it; do not infer from it.
SINGULARITY_WATCH = 0.03


def dh():
    """(d, a, alpha_rad, offset_rad) straight from the algorithm library.

    Read every call rather than cached: `rm_algo_set_dh` mutates library
    state process-wide, so a cached copy can silently disagree with what the
    SDK's own FK and IK are using.
    """
    p = oc._algo.rm_algo_get_dh()
    return ([float(x) for x in p["d"]], [float(x) for x in p["a"]],
            [math.radians(float(x)) for x in p["alpha"]],
            [math.radians(float(x)) for x in p["offset"]])


def _mul(X, Y):
    return [[sum(X[i][k] * Y[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _link(th, d, a, al):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]]


def joint_frames(q):
    """([(origin, z_axis)] per joint, flange transform) in the BASE frame.

    Joint j's axis is the z of the frame BEFORE link j is applied — that is
    the axis it rotates about. Getting this wrong is not academic: an earlier
    attempt derived the frames by truncating the chain and reading the flange
    pose, which ranked J3 as the most gravity-loaded joint when the arm
    plainly draws most on J4, and the whole analysis had to be withheld.
    """
    d, a, al, off = dh()
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    out = []
    for i in range(7):
        out.append(([T[r][3] for r in range(3)], [T[r][2] for r in range(3)]))
        T = _mul(T, _link(math.radians(q[i]) + off[i], d[i], a[i], al[i]))
    return out, T


def link_com_positions(q):
    """Each link's centre of mass in the BASE frame."""
    d, a, al, off = dh()
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    out = []
    for i in range(7):
        T = _mul(T, _link(math.radians(q[i]) + off[i], d[i], a[i], al[i]))
        c = LINK_COM[i]
        out.append([T[r][3] + sum(T[r][k] * c[k] for k in range(3))
                    for r in range(3)])
    return out


def gravity_vector_base():
    """Gravity in the BASE frame.

    Gravity is -Z in the WORLD frame the poses are commanded in; the arm is
    mounted Ry(+90) from it, so undo that. Stated explicitly because a sign
    error here is invisible in the output and inverts every conclusion — the
    check that catches it is whether the model ranks the joints the way the
    current channel does.
    """
    Rm = oc._Ry(-math.radians(oc.MOUNT_RY_DEG))
    return [sum(Rm[i][k] * v for k, v in enumerate((0.0, 0.0, -G)))
            for i in range(3)]


def gravity_torque(q, payload_kg=0.0, payload_com=(0.0, 0.0, 0.0),
                   tool=None):
    """Static gravity torque per joint, N*m, from the arm links + payload.

    `payload_com` is in the TOOL frame if `tool` is given, else the flange
    frame. Read the payload with `payload_audit.py`; it is CONTROLLER state
    and is not recorded in run.json.

    STATIC ONLY — no inertial term, no friction. At our speeds the arm
    reaches 0.85 of J4's rate limit, so the dynamic term is NOT negligible
    and this under-predicts during acceleration. It is the gravity share,
    not the whole torque.
    """
    fr, T = joint_frames(q)
    coms = link_com_positions(q)
    masses = list(LINK_MASS)
    if payload_kg > 0:
        off = list(payload_com)
        if tool is not None and tool in oc.TOOL_OFFSETS:
            t = oc.TOOL_OFFSETS[tool]
            off = [off[i] + t[i] for i in range(3)]
        coms.append([T[r][3] + sum(T[r][k] * off[k] for k in range(3))
                     for r in range(3)])
        masses.append(payload_kg)
    g = gravity_vector_base()
    tau = []
    for j in range(7):
        org, ax = fr[j]
        s = 0.0
        for i in range(j, len(masses)):
            F = [masses[i] * gv for gv in g]
            r = [coms[i][k] - org[k] for k in range(3)]
            cr = (r[1] * F[2] - r[2] * F[1],
                  r[2] * F[0] - r[0] * F[2],
                  r[0] * F[1] - r[1] * F[0])
            s += sum(cr[k] * ax[k] for k in range(3))
        tau.append(s)
    return tau


def sigma_min(q, lo=1e-4, hi=1.0, tol=1e-5):
    """The SDK's OWN minimum singular value, recovered by bisection.

    `rm_algo_universal_singularity_analyse(q, limit)` answers only "is this
    below `limit`?". Bisecting `limit` until the verdict flips returns the
    number the controller is actually using — exactly, with no assumption
    about how it normalises the Jacobian.

    WHY NOT JUST TAKE THE SVD OURSELVES. It was tried and it is WRONG. A
    6x7 Jacobian mixes metres (translation rows) with radians (rotation
    rows), so its singular values are not scale-invariant and they shrink
    whenever the arm is simply RETRACTED — a reach effect, not a rank
    deficiency. On `20260811T222451` sample 1766 the hand-rolled value read
    0.0034, apparently singular, while the SDK called the same pose fine and
    the arm was stationary. A validation over 8 configurations had missed it
    because they were all either deep in a singularity or far from one; the
    two measures only diverge in between, which is precisely the region
    worth reporting.

    Returns a value in [lo, hi]; `hi` means "no singularity found up to 1.0",
    `lo` means "at or below the smallest threshold tested". None if the SDK
    call fails.
    """
    f = oc._algo.rm_algo_universal_singularity_analyse
    if f(q, hi) != -1:          # not singular even at the loosest threshold
        return hi
    if f(q, lo) == -1:          # singular even at the tightest
        return lo
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if f(q, mid) == -1:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def singularity_note(s):
    """One line about a sigma_min value, or '' if it is unremarkable."""
    if s is None:
        return ""
    if s < SINGULARITY_THRESHOLD:
        return ("SINGULAR by the SDK's own default threshold (%.2f) — joint "
                "rates are unbounded here" % SINGULARITY_THRESHOLD)
    if s < SINGULARITY_WATCH:
        return ("only %.1fx the SDK's singular threshold — the lowest across "
                "all our recordings is toplid_right at 0.0285 (2.9x)"
                % (s / SINGULARITY_THRESHOLD))
    return ""
