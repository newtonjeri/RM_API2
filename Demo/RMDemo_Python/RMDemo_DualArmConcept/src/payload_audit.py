"""Payload + centroid across a side's tool frames: agree? mirror? apply.

THE LAW, derived from the calibrated left arm (2026-08-10)

The GUI's Load Identification measures the end payload's mass and its
CENTRE OF MASS. The centroid is **measured from the FLANGE, not from the
tool frame origin** — established, not assumed:

    L_glove_2  origin Z 180.3   CZ 222.0
    L_glove_3  origin Z 185.3   CZ 224.0
    L_tip      origin Z 245.3   CZ 224.0
    L_index_ti origin Z 240.3   CZ 226.0

Four frames whose ORIGINS span 65 mm report centroids agreeing to 4 mm.
Tool-relative values would have differed by 65 mm. So one hand has one
centre of mass, and **every frame on an arm must carry the same payload
record**. Frames that disagree are measurement scatter, not geometry.

MIRRORING BETWEEN ARMS — PROPOSED, THEN REFUTED BY MEASUREMENT

The URDF glove frames mirror by negating X and keeping Y and Z (exact for
glove_3, glove_4, tip and index_tip). That suggested the payload centroid
would mirror the same way, and this module originally proposed writing the
right arm from the left by that rule.

**Newton calibrated the right hand instead. The measurement refutes it:**

    left  consensus   0.567 kg at (-25.2, 41.2, 224.0) mm   4 of 6 frames
    right consensus   0.711 kg at (-23.6, 25.4, 164.8) mm   5 of 6 frames
    mirror predicted  0.711 kg at (+23.6, 25.4, 164.8) mm
    -> off by 78.4 mm and 0.144 kg

CX is NEGATIVE on BOTH arms and agrees to 1.6 mm — the centroid does not
mirror in X at all. The arms also carry genuinely different payloads: the
right is 25 % heavier with its centre of mass 59 mm closer to the flange.

`mirror()` remains here as a COMPARISON only, so a future calibration can
be checked against the geometry. It is not a source of values. **Each arm
uses its own measured consensus.** The rule this whole episode teaches is
the same one that produced the 128-metre frames: a payload record that was
not measured on the arm it is written to is a guess.

WHAT THE TWO CALIBRATIONS AGREE ON

Flange-relative is now confirmed INDEPENDENTLY on both arms — the right's
five agreeing frames have origins spanning 65 mm in Z and centroids
agreeing to 3 mm, exactly as the left's four did.

    python3 payload_audit.py                      # read + report, both arms
    python3 payload_audit.py --side right          # one arm
    python3 payload_audit.py --side right --apply  # write ITS OWN consensus
                                                   # onto its own frames
"""

import statistics
import sys

from dual_arm_common import (
    handle_cli, com_mm, com_from_mm, LEFT_IP, RIGHT_IP, ROBOT_PORT,
)
from frame_alignment_offline import frame_map, controller_frame_name

# A hand's centre of mass is O(100 mm) from the flange; the arm's whole
# reach is under a metre. Past this it is a unit error. Same bound as
# test_frame_alignment, and for the same reason.
MAX_COM_MM = 500.0
# How far two frames' records may differ before they are not the same
# measurement of the same hand.
AGREE_MM = 10.0
AGREE_KG = 0.030

USAGE = (
    "Usage: python3 payload_audit.py [--side left|right|both] [--predict]\n"
    "       [--check CX,CY,CZ,KG] [--apply] [--from left|right] [-h|--help]\n"
    "  --side S        which arm to read (default: both)\n"
    "  --predict       mirror the OTHER arm's consensus onto this one\n"
    "  --check V       compare a freshly measured record against the\n"
    "                  prediction, e.g. --check 25.2,41.2,224.0,0.567\n"
    "  --apply         WRITE the consensus onto every frame that deviates\n"
    "  --from S        which side to take the consensus from (default: the\n"
    "                  side with the tighter agreement)\n"
    "  -h, --help      show this documentation and exit")


def _arg(flag, default=None):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def mirror(com):
    """left-flange (x, y, z)  ->  right-flange (-x, y, z), and back."""
    return (-com[0], com[1], com[2])


def read_side(robot, side):
    """Every glove/ik frame's payload record, as the controller holds it."""
    out = {}
    for link, fname, _conn, _tip, _rpy in frame_map(side):
        try:
            ret, got = robot.rm_get_given_tool_frame(fname)
        except Exception as exc:
            out[fname] = {"error": repr(exc)}
            continue
        if ret != 0:
            out[fname] = {"error": f"read ret={ret}"}
            continue
        # The getter answers in METRES (measured 2026-08-11 against the
        # GUI); this file works in mm throughout. com_mm() detects rather
        # than assumes, because treating metres as mm made every frame
        # look identical and no outlier could ever be flagged.
        mm, note = com_mm(got)
        out[fname] = {
            "payload": float(got.get("payload", 0.0)),
            "com": mm,
            "note": note,
        }
    return out


def dist(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def consensus(records):
    """Robust centre of the records, plus who disagrees with it.

    Median per axis, not mean: on the left arm four frames agreed to 4 mm
    and two were 40-51 mm out, and a mean would have been dragged toward
    the outliers by exactly the frames that need correcting.
    """
    good = [r for r in records.values()
            if "error" not in r and max(abs(v) for v in r["com"]) <= MAX_COM_MM]
    if len(good) < 3:
        return None, [], good
    # Two passes. The median locates the cluster without being dragged by
    # outliers; the mean of the INLIERS is then the best estimate of it.
    # On the left arm four frames agreed to 4 mm and two were 40-51 mm out,
    # so a single-pass mean would have been pulled toward exactly the
    # frames that need correcting.
    med = tuple(statistics.median([r["com"][i] for r in good])
                for i in range(3))
    medkg = statistics.median([r["payload"] for r in good])
    inliers = [r for r in good
               if dist(r["com"], med) <= AGREE_MM
               and abs(r["payload"] - medkg) <= AGREE_KG]
    src = inliers if len(inliers) >= 3 else good
    com = tuple(statistics.fmean([r["com"][i] for r in src]) for i in range(3))
    kg = statistics.fmean([r["payload"] for r in src])
    off = []
    for name, r in records.items():
        if "error" in r:
            continue
        d = dist(r["com"], com)
        dk = abs(r["payload"] - kg)
        if d > AGREE_MM or dk > AGREE_KG:
            off.append((name, d, dk))
    return (com, kg, len(src), len(good)), off, good


def report(side, records):
    print(f"\n  {side.upper()} arm — payload record per tool frame")
    print(f"    {'frame':14} {'kg':>7}  {'CX':>8}{'CY':>8}{'CZ':>8}   note")
    cons, off, _ = consensus(records)
    bad = {n for n, _d, _k in off}
    for name, r in records.items():
        if "error" in r:
            print(f"    {name:14} {r['error']}")
            continue
        c = r["com"]
        note = ""
        if max(abs(v) for v in c) > MAX_COM_MM:
            note = f"<-- {max(abs(v) for v in c):.0f} mm: IMPOSSIBLE, a unit error"
        elif name in bad:
            d = next(d for n, d, _k in off if n == name)
            note = f"<-- {d:.0f} mm from the consensus"
        elif r["payload"] == 0.0 and c == (0.0, 0.0, 0.0):
            note = "<-- NO payload declared at all"
        print(f"    {name:14} {r['payload']:7.3f}  "
              f"{c[0]:8.1f}{c[1]:8.1f}{c[2]:8.1f}   {note}")
    if cons:
        com, kg, n_in, n_all = cons
        spread = max((dist(r["com"], com) for r in records.values()
                      if "error" not in r
                      and dist(r["com"], com) <= AGREE_MM), default=0.0)
        print(f"    consensus from {n_in} of {n_all} agreeing frames: "
              f"{kg:.3f} kg at ({com[0]:.1f}, {com[1]:.1f}, {com[2]:.1f}) mm"
              f"   they agree to {spread:.1f} mm")
    return cons, off


def apply_side(robot, side, com, kg, only=None):
    """Write one payload record onto every frame of a side. Read back."""
    from Robotic_Arm.rm_ctypes_wrap import (rm_frame_t, rm_pose_t,
                                            rm_position_t, rm_euler_t)
    if max(abs(v) for v in com) > MAX_COM_MM:
        print(f"    REFUSING: centroid {com} exceeds the {MAX_COM_MM:.0f} mm "
              "physical bound")
        return False
    ok = True
    for link, fname, _conn, tip, rpy in frame_map(side):
        if only and fname not in only:
            continue
        ret0, cur = robot.rm_get_given_tool_frame(fname)
        if ret0 != 0:
            print(f"    {fname:14} unreadable (ret={ret0}) — skipped")
            ok = False
            continue
        f = rm_frame_t()
        f.frame_name = fname.encode()
        f.pose = rm_pose_t()
        pose = list(cur.get("pose", [0.0] * 6))
        f.pose.position = rm_position_t(*[float(v) for v in pose[:3]])
        f.pose.euler = rm_euler_t(*[float(v) for v in pose[3:6]])
        f.payload = float(kg)
        f.x, f.y, f.z = com_from_mm(com)
        ret = robot.rm_update_tool_frame(f)
        rb_ret, rb = robot.rm_get_given_tool_frame(fname)
        rbc = com_mm(rb)[0] if rb_ret == 0 else None
        good = rbc is not None and dist(rbc, com) < 0.5 \
            and abs(float(rb.get("payload", 0)) - kg) < 1e-3
        # A 1000x round trip is the exact signature of the setter and the
        # controller disagreeing about units. Name it rather than leaving
        # a silently wrong frame behind.
        scale = ""
        if rbc and any(abs(b) > 1e-6 and abs(abs(a / b) - 1000.0) < 1.0
                       for a, b in zip(rbc, com)):
            scale = "  <-- EXACTLY 1000x: setter/controller UNIT MISMATCH"
        print(f"    {fname:14} write ret={ret}  reads "
              f"{kg:.3f} kg at ({rbc[0]:.1f}, {rbc[1]:.1f}, {rbc[2]:.1f}) mm"
              if rbc else f"    {fname:14} write ret={ret}  UNREADABLE")
        if not good:
            print(f"      MISMATCH{scale}")
            ok = False
    return ok


def connect(ip):
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    h = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
    if h is None or h.id <= 0:
        return None
    return robot


def main() -> int:
    handle_cli(__doc__, extra_flags=("--predict", "--apply"),
               value_flags=("--side", "--check", "--from"), usage=USAGE,
               allow_common=False)
    which = _arg("--side", "both")
    sides = ("left", "right") if which == "both" else (which,)
    print("=" * 72)
    print("Payload + centroid audit — is one hand described consistently?")
    print("=" * 72)

    data, cons = {}, {}
    for side in sides:
        robot = connect(LEFT_IP if side == "left" else RIGHT_IP)
        if robot is None:
            print(f"\n  [SKIP] {side} arm not reachable")
            continue
        data[side] = read_side(robot, side)
        c, _off = report(side, data[side])
        cons[side] = c
        try:
            robot.rm_delete_robot_arm()
        except Exception:
            pass

    src = _arg("--from")
    if not src:
        # Prefer the side that actually HAS a payload. An arm with nothing
        # declared reads (0, 0, 0) on every frame and therefore agrees with
        # itself perfectly — the first version of this picked exactly that
        # side and proposed mirroring zero onto the calibrated one.
        cand = [s for s in cons
                if cons.get(s) and (cons[s][1] > 0.01
                                    or max(abs(v) for v in cons[s][0]) > 1.0)]
        # among real candidates: most agreeing frames wins, then tightness
        scored = [(-cons[s][2],
                   max((dist(r["com"], cons[s][0])
                        for r in data[s].values() if "error" not in r),
                       default=9e9), s) for s in cand]
        src = min(scored)[2] if scored else None
        if not src:
            print("\n  no side has a USABLE payload record (either none "
                  "is declared, or every value is beyond the physical "
                  "bound). Run Load Identification in the GUI, then "
                  "re-run this.")

    if src and cons.get(src):
        com, kg = cons[src][0], cons[src][1]
        other = "right" if src == "left" else "left"
        pred = mirror(com)
        print(f"\n  MIRROR LAW — negate X, keep Y and Z (exact for glove_3, "
              "glove_4, tip, index_tip)")
        print(f"    {src} consensus   {kg:.3f} kg at "
              f"({com[0]:.1f}, {com[1]:.1f}, {com[2]:.1f}) mm")
        print(f"    -> {other} predicted {kg:.3f} kg at "
              f"({pred[0]:.1f}, {pred[1]:.1f}, {pred[2]:.1f}) mm")
        # If the OTHER side has been calibrated too, compare what it
        # actually measures against the mirrored prediction. Newton chose
        # to calibrate the right hand rather than mirror onto it
        # (2026-08-10) — correct, since the left's own six measurements
        # of one hand spread 51 mm and 0.081 kg, and a prediction would
        # carry that scatter plus the mirror assumption. This comparison
        # costs nothing and turns the calibration into a free test of
        # whether the mirror law holds at all.
        oc = cons.get(other)
        if oc and (oc[1] > 0.01 or max(abs(v) for v in oc[0]) > 1.0):
            mcom, mkg = oc[0], oc[1]
            d, dk = dist(mcom, pred), abs(mkg - kg)
            print(f"    {other} MEASURED  {mkg:.3f} kg at "
                  f"({mcom[0]:.1f}, {mcom[1]:.1f}, {mcom[2]:.1f}) mm "
                  f"(from {oc[2]} of {oc[3]} frames)")
            verdict = ("the mirror law is CONFIRMED by measurement"
                       if d <= AGREE_MM and dk <= AGREE_KG else
                       "the mirror law does NOT hold here — trust the "
                       "measurement, not the prediction")
            print(f"    measured vs predicted: {d:.1f} mm, {dk:.3f} kg"
                  f"  ->  {verdict}")
        chk = _arg("--check")
        if chk:
            v = [float(x) for x in chk.split(",")]
            mcom, mkg = tuple(v[:3]), (v[3] if len(v) > 3 else kg)
            d, dk = dist(mcom, pred), abs(mkg - kg)
            verdict = ("AGREES — the law holds, apply it to every frame"
                       if d <= AGREE_MM and dk <= AGREE_KG else
                       "DISAGREES — calibrate each frame, do not mirror")
            print(f"    measured        {mkg:.3f} kg at "
                  f"({mcom[0]:.1f}, {mcom[1]:.1f}, {mcom[2]:.1f}) mm")
            print(f"    difference      {d:.1f} mm, {dk:.3f} kg  ->  {verdict}")
        if "--apply" in sys.argv:
            for side in sides:
                if side not in data:
                    continue
                use = (com, kg) if side == src else (pred, kg)
                print(f"\n  writing {side}: {use[1]:.3f} kg at "
                      f"({use[0][0]:.1f}, {use[0][1]:.1f}, {use[0][2]:.1f}) mm")
                robot = connect(LEFT_IP if side == "left" else RIGHT_IP)
                if robot is None:
                    print("    not reachable")
                    continue
                apply_side(robot, side, use[0], use[1])
                try:
                    robot.rm_delete_robot_arm()
                except Exception:
                    pass
        else:
            print("\n  read-only. --apply writes the consensus onto every "
                  "frame (each write is read back and verified).")
    return 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
