"""Read one recorded run and say everything the stream can support.

    python3 analyse_run.py ../runs/<run_dir>
    python3 analyse_run.py ../runs/<run_dir> --plot
    python3 analyse_run.py ../runs/A ../runs/B ../runs/C        # compare
    python3 analyse_run.py ../runs/*_v350_left --plot --quiet
    python3 analyse_run.py ../runs/SIM ../runs/REAL --rates derived

`--rates auto | derived | reported` picks the joint-rate estimator.
`auto` (default) reads the reported `speed{n}` channel and falls back to
d(position)/dt when that channel is dead, which it is in SIM. **Force
`derived` whenever comparing SIM against REAL** — the two estimators differ
by ~20 % (H78), so an auto/auto comparison prices the estimator, not the
mode.

⚠ **DO NOT RUN AN H63 SAFETY VERDICT OFF `derived` ON A REAL RUN.** The two
estimators do not merely differ, they differ WITH A SIGN: derived runs HIGH.
Measured on REAL `20260815T201708`, reported vs derived as % of limit —
J1 84.6/90.3, J2 34.4/35.4, J3 52.0/55.5, J4 65.1/64.2, J5 18.3/18.3,
J6 60.6/62.8, J7 42.4/44.4 (rm-api2 session, 2026-08-17). Up to **5.7 points
high**, and H63 keys on dwell at **>=98 %** of a limit — so a derived verdict
false-positives exactly in the band where the rule decides something.

**AND THE DIVERGENCE IS LARGEST ON J1** — the joint most likely to be near a
limit in the first place. J1 is the measured binding joint on `top_left` at
0.25 m/s (84.6-86.3 % reported, flat across angular caps), so on that family
the estimator choice moves the number in the band where H63 decides something.

*(An earlier version of this note cited `hinge_area` at "99.6 % with 30 ms
dwell" as the example. That was withdrawn 2026-08-17: the run is
`blend_r25_v250_right`, a blend-characterisation run at r=25 — a radius §0
forbids on dense geometry — and its same-configuration twin reads J1 at 71 %.
Five of the six runs on that path carry zero dwell. The estimator point stands
on `top_left`; the hinge_area example did not survive re-measurement.)*

The rule that follows:
  * REAL run, safety/dwell verdict  -> `reported`. It is the controller's own
    number, and it is what H63 was calibrated on.
  * SIM run                         -> `derived`. The reported channel is dead;
    there is nothing else.
  * SIM vs REAL comparison          -> `derived` on both, and read the result
    as a mode comparison, not as an absolute utilisation.

OFFLINE AND READ-ONLY. It opens `stream.csv` and `run.json` and nothing
else — no arm, no controller, no emulator. Any run ever recorded can be
re-read with it, including ones taken before this file existed.

WHY IT EXISTS. Every finding in SPEED_INVESTIGATION and half of PHASE_PLAN's
F-numbers came out of the same handful of columns, each time by a one-off
script that was then thrown away. Three of those one-offs were WRONG in ways
that took a hardware session to notice — speed differenced over single
samples, corners located by cumulative arc, a path-followed check that
accepted a run which stopped 45 mm short. The corrections live here now, in
one place, so the next question does not start by re-deriving them.

WHAT IT REPORTS, and which objective each part serves:

  A PROVENANCE      what was commanded and what the controller was
                    configured with. PHASE_PLAN F32: the controller holds
                    state no repository records, and it fails remotely from
                    its cause. A run whose limits are not recorded is not
                    evidence.
  B STREAM HEALTH   rate, gaps, and the two artifacts that have produced
                    false findings: the position channel is not synchronous
                    with the 100 Hz push, and summed arc runs 7-10 % long.
  C MOTION          time-to-first-motion (F26/F27's signature was 4.56 s
                    idle then 0.05 s of motion), path traced vs commanded,
                    end-pose error.
  D TCP SPEED       achieved vs the cap in force, dips, time spent slow.
  E JOINTS          travel, travel per metre, peak/p95 rate against the
                    controller's own limits, and DWELL near the limit —
                    H63 found dwell at >=98 % separated the outcomes
                    (0 ms completed / 110 ms stalled / 330 ms violent).
  F JACOBIAN        how much of the TCP motion each joint actually
                    produces, and how much cancels. Needs the SDK's FK;
                    skipped cleanly if it is not importable.
  G ELECTRICAL      R10, the top open risk in PHASE_PLAN and the one thing
                    SIM cannot answer. Per-joint current peaks, the
                    voltage channel's usable resolution, and whether a
                    current spike coincides with a speed dip — the plan's
                    own discriminator is WHICH joint spikes (J2/J4 =
                    supply margin, J6/J7 = wrist demand).
  H FAULTS          joint error flags, enables, and `arm_status`
                    transitions including 9 = RM_STOP_E.

Numbers this file will not print: anything the stream cannot support. A
column that is a constant placeholder is reported as such rather than
plotted as a flat line and read as a measurement.
"""

import csv
import json
import math
import os
import pathlib
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help                          # noqa: E402

# Joint speed limits are read from the run's own `limits_in_force` when it is
# there — the controller is the authority and it can be reconfigured. These
# are only the fallback for older recordings that predate the field.
FALLBACK_JOINT_LIMIT = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]

ARM_STATUS = {0: "IDLE", 1: "MOVE_L", 2: "MOVE_J", 3: "MOVE_C",
              4: "MOVE_S", 9: "STOP (RM_STOP_E)", 10: "SLOW_STOP"}

# Differencing consecutive samples is not safe on this stream: the position
# field is not synchronous with the push, so 24-36 % of moving intervals
# advance under 40 % of the median step and single-interval speed reaches
# 162 % of a cap it never physically exceeded. 7 samples was the measured
# optimum for corner work; the same window is used here so every number in
# this report and in `test_blend_corner` comes from the same estimator.
SPEED_WINDOW = 7


def load(run_dir):
    d = pathlib.Path(run_dir)
    meta = {}
    if (d / "run.json").exists():
        try:
            meta = json.load(open(d / "run.json"))
        except (ValueError, OSError):
            meta = {}
    rows = list(csv.DictReader(open(d / "stream.csv")))
    return d, meta, rows


def col(rows, name, cast=float):
    """One column, or None if the stream does not carry it."""
    if not rows or name not in rows[0]:
        return None
    out = []
    for r in rows:
        try:
            out.append(cast(r[name]))
        except (TypeError, ValueError):
            out.append(None)
    return out


def joints(rows, prefix):
    got = [col(rows, "%s%d" % (prefix, j)) for j in range(1, 8)]
    return got if all(g is not None for g in got) else None


def pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    return sorted_vals[min(len(sorted_vals) - 1, int(p * len(sorted_vals)))]


def windowed_speed(t, p, k=SPEED_WINDOW):
    """|dp|/dt over a k-sample window. See SPEED_WINDOW."""
    half, out = k // 2, []
    for i in range(len(p)):
        a, b = max(0, i - half), min(len(p) - 1, i + half)
        dt = t[b] - t[a]
        out.append(math.dist(p[b], p[a]) / dt if dt > 1e-6 else 0.0)
    return out


def derive_joint_rates(t, qs, k=SPEED_WINDOW):
    """|dq|/dt per joint, over the same k-sample window as `windowed_speed`
    so every rate in this report comes from one estimator."""
    half, out = k // 2, []
    for j in range(7):
        c = []
        for i in range(len(t)):
            a, b = max(0, i - half), min(len(t) - 1, i + half)
            dt = t[b] - t[a]
            c.append(abs(qs[j][b] - qs[j][a]) / dt if dt > 1e-6 else 0.0)
        out.append(c)
    return out


def resolve_joint_rates(t, qs, qds):
    """(rates, source) — fall back to d(position)/dt when `speed{n}` is dead.

    SIM populates `speed{n}` with 0.1-0.5 deg/s while the arm moves hundreds
    of degrees, so an `any(nonzero)` test does NOT detect it: one cap-ladder
    run carried 16 nonzero samples out of 27237 and the old guard passed it
    straight through. The report then read every joint at 0 % of its limit
    with an H63 dwell of 0 ms — the most reassuring possible answer, on the
    mode we use precisely BECAUSE it is a free pre-flight check
    (MODE_CHARACTERIZATION 1: SIM's position channel is faithful, so
    differentiate it — that is the way through).

    The test is therefore whether the reported peak is CONSISTENT WITH THE
    POSITION TRAVEL, not whether it is nonzero. The two estimators differ by
    ~20 % where both are live (H78), so the source is reported with the
    numbers rather than left to be guessed.
    """
    if not qs:
        return qds, "reported speed{n}"
    derived = derive_joint_rates(t, qs)
    der_pk = max((max(c) for c in derived), default=0.0)
    if not qds:
        return derived, "d(position)/dt"
    rep_pk = max((max(abs(v) for v in c) for c in qds), default=0.0)
    if der_pk > 1.0 and rep_pk < 0.05 * der_pk:
        return derived, "d(position)/dt — REPORTED CHANNEL DEAD (SIM)"
    return qds, "reported speed{n}"


def constant_channel(vals):
    """(is_placeholder, distinct_count). A channel with one value carries no
    information; F30 found `joint_voltage` pinned at 22.00 V across 2085
    samples and the R10 premise had to be revisited because of it."""
    seen = {v for v in vals if v is not None}
    return len(seen) <= 1, len(seen)


# ── A. provenance ──────────────────────────────────────────────────────────
def section_provenance(d, meta):
    print("A. PROVENANCE — what was asked for, and how the arm was configured")
    print("   run            %s" % d.name)
    if not meta:
        print("   [WARN] no run.json. Everything below is the stream alone: "
              "no commanded path, no limits, no mode. PHASE_PLAN F32 exists "
              "because unrecorded controller state is how findings get "
              "attributed to the wrong cause.")
        return
    cmd = meta.get("commanded") or {}
    lim = meta.get("limits_in_force") or {}
    lad = meta.get("ladder") or {}
    print("   mode           %s%s" % (meta.get("mode"),
          "   (nothing physical moved)" if meta.get("sim") else ""))
    print("   side / path    %s / %s" % (meta.get("side"),
                                         meta.get("path_file")))
    print("   tool frame     %s" % (cmd.get("tool_frame") or "not recorded"))
    print("   waypoints      %s over %s segments"
          % (cmd.get("num_waypoints"), cmd.get("segments")))
    sp = meta.get("speeds") or {}
    print("   blend / connect r=%s %%   connect=%s"
          % (sp.get("blend_pct"), sp.get("connect")))
    if lad:
        print("   ladder rung    %s of %s   (all rungs %s)"
              % (lad.get("rung"), lad.get("of"), lad.get("all_rungs")))
        if lad.get("predicted_j4_pct") is not None:
            print("   offline screen predicted J4 at %.0f %% of its limit "
                  "— section E has what it actually did"
                  % lad["predicted_j4_pct"])
    # THE STAGES, and whether the one that matters is present. This is not
    # decoration: a toplid run was quoted as reaching 100 % of a joint limit
    # when its recording stops at `open_tenth` and never contains a cleaning
    # stroke at all — the figures came from the movej approach, which uses
    # full joint rate by design. Any per-task number must say which stage it
    # came from, and a missing stage must be loud.
    stages = [x.get("stage_name") for x in (meta.get("stages") or [])]
    if stages:
        print("   stages         %s" % " -> ".join(stages))
        if "execute_path" not in stages:
            print("   [WARN] there is NO execute_path stage — this run never "
                  "reached the cleaning stroke. Every figure below describes "
                  "the approach, which is movej and legitimately uses the "
                  "full joint rate. Do not quote it as task performance.")
        elif stages[-1] not in ("hand_release", "move_to_rest"):
            print("   [WARN] the run ends at '%s', not at a rest stage — it "
                  "did not finish." % stages[-1])
    if lim:
        print("   limits in force line_speed %.3f  line_acc %.3f  "
              "angular %.3f / %.3f"
              % (lim.get("line_speed", -1), lim.get("line_acc", -1),
                 lim.get("angular_speed", -1), lim.get("angular_acc", -1)))
        ls = lim.get("line_speed")
        if ls and ls > 0.25 + 1e-9:
            print("                   ^ above the 0.250 m/s factory default; "
                  "these RATCHET, so this run left the arm faster than it "
                  "found it unless reset_limits ran afterwards")
    print()


# ── B. stream health ───────────────────────────────────────────────────────
def section_health(meta, rows, t, p):
    print("B. STREAM HEALTH — is this recording usable at all")
    n = len(rows)
    dur = (t[-1] - t[0]) if n > 1 else 0.0
    rate = (n - 1) / dur if dur > 1e-9 else 0.0
    print("   %d samples over %.2f s = %.1f Hz" % (n, dur, rate))
    if meta.get("frames_dropped"):
        print("   [WARN] %s frames dropped" % meta["frames_dropped"])
    if meta.get("first_parse_error"):
        print("   [WARN] first parse error: %s" % meta["first_parse_error"])
    dts = sorted(1000 * (t[i] - t[i - 1]) for i in range(1, n))
    if dts:
        print("   dt ms          median %.1f  p5 %.1f  p95 %.1f  max %.1f"
              % (pct(dts, 0.5), pct(dts, 0.05), pct(dts, 0.95), dts[-1]))
    # The two artifacts that have produced false findings.
    steps = [1000 * math.dist(p[i], p[i - 1]) for i in range(1, n)]
    moving = sorted(s for s in steps if s > 1e-6)
    if moving:
        med = pct(moving, 0.5)
        tiny = sum(1 for s in moving if s < 0.4 * med)
        print("   position channel: %.0f %% of moving intervals advance < 40 %% "
              "of the median step (%.2f mm)"
              % (100.0 * tiny / len(moving), med))
        if tiny > 0.10 * len(moving):
            print("                     ^ EXPECTED on this controller — the "
                  "position field is not synchronous with the push. Speeds "
                  "here are differenced over %d samples for that reason; "
                  "single-sample differencing reads up to 162 %% of a cap "
                  "the arm never exceeded." % SPEED_WINDOW)
    return dur, rate


# ── C. motion ──────────────────────────────────────────────────────────────
def section_motion(meta, t, p, spd):
    print("\nC. MOTION — did it go where it was sent, and when did it start")
    cmd = (meta.get("commanded") or {}).get("poses")
    thresh = 0.005
    first = next((i for i, v in enumerate(spd) if v > thresh), None)
    if first is None:
        print("   [WARN] the tool never exceeded %.0f mm/s — nothing moved"
              % (1000 * thresh))
    else:
        lat = t[first] - t[0]
        print("   first motion   %.2f s after recording started" % lat)
        if lat > 1.0:
            print("                  ^ F26/F27 recorded a FIXED ~4.5 s "
                  "pre-motion latency before a 0x100D abort, on a single "
                  "segment as well as a 43-segment chain. A long idle head "
                  "is a signature worth checking against those runs.")
    arc = sum(math.dist(p[i], p[i - 1]) for i in range(1, len(p)))
    print("   traced         %.3f m" % arc)
    if cmd:
        c = sum(math.dist(cmd[i][:3], cmd[i + 1][:3])
                for i in range(len(cmd) - 1))
        print("   commanded      %.3f m   (traced/commanded %.0f %%)"
              % (c, 100 * arc / max(c, 1e-9)))
        print("                  ^ summed arc runs 7-10 % LONG on this "
              "stream (noise, not travel), so 100-110 %% is normal and a "
              "shortfall can hide inside it — which is why the end pose is "
              "checked separately:")
        same_frame = math.dist(p[0], cmd[0][:3]) <= 0.010
        e0 = 1000 * math.dist(p[0], cmd[0][:3])
        e1 = 1000 * math.dist(p[-1], cmd[-1][:3])
        print("   start error    %.1f mm      end error %.1f mm" % (e0, e1))
        if not same_frame:
            print("                  ^ the run did not begin at the path's "
                  "first waypoint. Either it started from the wrong pose, or "
                  "the stream is in a different frame than the path — the "
                  "end error above means nothing in the second case.")
        elif e1 > 10:
            print("                  ^ ENDED %.0f mm SHORT of its final "
                  "waypoint. This is what an aborted trajectory looks like; "
                  "run 20260813T205319 did exactly this after the controller "
                  "refused it, while still measuring 101 %% of commanded arc."
                  % e1)
    print()


# ── D. TCP speed ───────────────────────────────────────────────────────────
def section_tcp(meta, spd):
    print("D. TCP SPEED")
    cap = meta.get("line_speed_cap_m_s")
    mv = sorted(v for v in spd if v > 0.005)
    if not mv:
        print("   nothing moving to measure\n")
        return
    typ, p95 = pct(mv, 0.5), pct(mv, 0.95)
    print("   typical %.0f mm/s   p95 %.0f mm/s   peak %.0f mm/s"
          % (1000 * typ, 1000 * p95, 1000 * mv[-1]))
    if cap:
        print("   cap in force %.0f mm/s -> typical is %.0f %% of it, "
              "p95 %.0f %%" % (1000 * cap, 100 * typ / cap, 100 * p95 / cap))
        print("                  ^ the MEDIAN sitting well under the cap is "
              "expected, not a fault: it is bound by ACCELERATION (H57 — at "
              "fixed line_speed 0.45 the median rose 62.6 -> 87.6 % as "
              "line_acc went 1.6 -> 3.6 while p95 stayed pinned at the cap).")
        if p95 < 0.9 * cap:
            print("   [WARN] p95 only %.0f %% of cap. At a 100 %% pendant "
                  "override p95 lands AT or slightly above it (105-110 %%, "
                  "H59). This run looks DERATED — check the real-time speed "
                  "slider before trusting any timing from it."
                  % (100 * p95 / cap))
    half = 0.5 * typ
    dips, i = 0, 1
    while i < len(spd):
        if spd[i] < half:
            while i < len(spd) and spd[i] < half:
                i += 1
            dips += 1
        i += 1
    below = sum(1 for v in spd if 0.005 < v < half)
    print("   %d decelerations below half cruise; %.0f %% of moving samples "
          "spent under it" % (dips, 100.0 * below / max(len(mv), 1)))
    print()


# ── E. joints ──────────────────────────────────────────────────────────────
def section_joints(meta, rows, t, p, qs, qds, qd_src="reported speed{n}"):
    print("E. JOINTS — travel, rate against the limit, and DWELL near it")
    print("   rate estimator: %s" % qd_src)
    lim = ((meta.get("limits_in_force") or {}).get("joint_speed")
           or FALLBACK_JOINT_LIMIT)
    arc = sum(math.dist(p[i], p[i - 1]) for i in range(1, len(p)))
    travel = [sum(abs(qs[j][i] - qs[j][i - 1]) for i in range(1, len(t)))
              for j in range(7)]
    print("   %-5s %10s %10s %10s %9s %8s %9s"
          % ("joint", "travel deg", "deg per m", "peak deg/s", "% limit",
             "p95", "limit"))
    worst = (0.0, 0)
    for j in range(7):
        sp = sorted(abs(v) for v in qds[j]) if qds else []
        peak = sp[-1] if sp else 0.0
        f = peak / lim[j] if lim[j] else 0.0
        if f > worst[0]:
            worst = (f, j + 1)
        print("   J%-4d %10.1f %10.1f %10.0f %8.0f%% %8.0f %9.0f"
              % (j + 1, travel[j], travel[j] / max(arc, 1e-9), peak,
                 100 * f, pct(sp, 0.95) if sp else 0, lim[j]))
    if not qds or not any(any(v for v in q) for q in qds):
        print("   [NOTE] the joint SPEED channel is all zeros. SIMULATION "
              "populates positions only — this run cannot tell you anything "
              "about joint rates, and must not be read as showing low ones.")
        print()
        return None
    print("   worst: J%d at %.0f %% of its limit" % (worst[1], 100 * worst[0]))
    # DWELL. H63: peak alone did not separate the outcomes — time spent near
    # the limit did. Six completed runs had 0 ms at >=98 %; the run that
    # stopped had 110 ms and never exceeded 100 %; the violent one had 330 ms.
    util = [max(abs(qds[j][i]) / lim[j] for j in range(7) if lim[j])
            for i in range(len(t))]
    dt = (t[-1] - t[0]) / max(len(t) - 1, 1)
    print("   dwell (max over joints of |rate|/limit at each sample):")
    for thr in (0.95, 0.98, 1.00):
        ms = 1000 * dt * sum(1 for u in util if u >= thr)
        print("     >= %3.0f %%   %6.0f ms%s"
              % (100 * thr, ms,
                 "   <-- H63: >=98 %% for 110 ms preceded a silent stall, "
                 "330 ms preceded the violent run" if thr == 0.98 and ms > 0
                 else ""))
    print()
    return worst


# ── F. jacobian ────────────────────────────────────────────────────────────
def section_jacobian(meta, p, qs, step=4):
    print("F. WHERE THE TCP MOTION COMES FROM")
    tool = (meta.get("commanded") or {}).get("tool_frame")
    if not tool:
        print("   [SKIP] no tool frame recorded, so the TCP cannot be "
              "reconstructed from the joints.\n")
        return
    try:
        import orientation_cost as oc
    except Exception as exc:                                # noqa: BLE001
        print("   [SKIP] needs the SDK's forward kinematics (%s)\n" % exc)
        return
    if tool not in oc.TOOL_OFFSETS:
        print("   [SKIP] unknown tool frame %r\n" % tool)
        return

    def tcp(q):
        f = oc.fk(q)
        R = oc._Rmat(*f[3:6])
        tt = oc.TOOL_OFFSETS[tool]
        tip = [f[i] + sum(R[i][k] * tt[k] for k in range(3)) for i in range(3)]
        Rm = oc._Ry(math.radians(oc.MOUNT_RY_DEG))
        return [sum(Rm[i][k] * tip[k] for k in range(3)) for i in range(3)]

    n = len(p)
    q0 = [qs[j][0] for j in range(7)]
    err = 1000 * math.dist(tcp(q0), p[0])
    if err > 1.0:
        print("   [SKIP] the FK model reproduces the recorded TCP only to "
              "%.1f mm, so the split below would be fiction. Wrong tool "
              "frame, or a different mount." % err)
        print()
        return
    print("   FK+tool model matches the recorded TCP to %.2f mm — the split "
          "below is sound" % err)
    contrib, total = [0.0] * 7, 0.0
    eps = 1e-4
    for i in range(step, n, step):
        q = [qs[j][i] for j in range(7)]
        dq = [qs[j][i] - qs[j][i - step] for j in range(7)]
        for j in range(7):
            a, b = list(q), list(q)
            a[j] -= eps
            b[j] += eps
            pa, pb = tcp(a), tcp(b)
            cj = math.dist(pa, pb) / (2 * eps) * abs(dq[j])
            contrib[j] += cj
            total += cj
    arc = sum(math.dist(p[i], p[i - 1]) for i in range(1, n))
    print("   %-5s %14s %8s" % ("joint", "TCP metres", "share"))
    for j in range(7):
        print("   J%-4d %14.3f %7.0f%%"
              % (j + 1, contrib[j], 100 * contrib[j] / max(total, 1e-9)))
    print("   the joints produce %.2f m of TCP motion to deliver a %.2f m "
          "path = %.1fx" % (total, arc, total / max(arc, 1e-9)))
    print("                  ^ the excess CANCELS: this is a 7-DOF arm on a "
          "6-DOF task with no null-space objective, so nothing penalises "
          "internal motion that goes nowhere. A joint with large travel and "
          "a small share is churning.")
    print()


# ── G. electrical (R10) ────────────────────────────────────────────────────
def section_electrical(rows, t, qds, spd):
    print("G. ELECTRICAL — R10, the top open risk in PHASE_PLAN")
    cur = joints(rows, "current")
    vol = joints(rows, "voltage")
    tmp = joints(rows, "temperature")
    if cur:
        print("   %-5s %12s %12s" % ("joint", "peak mA", "median mA"))
        pk = []
        for j in range(7):
            a = sorted(abs(v) for v in cur[j])
            pk.append(a[-1])
            print("   J%-4d %12.0f %12.0f" % (j + 1, a[-1], pct(a, 0.5)))
        hi = max(range(7), key=lambda j: pk[j])
        print("   highest draw: J%d at %.2f A" % (hi + 1, pk[hi] / 1000.0))
        # A HARD CURRENT CHECK, because no other channel provides one. The
        # aborted run 20260813T205319 drew 26.35 A on J4 — more than the
        # 16.7 A of the run we call "violent" — at the instant the controller
        # stopped it, and reported nothing on any error flag, enable or
        # status field. Current is the only column that saw it.
        if pk[hi] / 1000.0 >= 10.0:
            print("   " + "!" * 68)
            print("   [WARN] J%d peaked at %.1f A. For scale: a clean run of "
                  "this path draws 4-6 A, and the 0.80 m/s run that reversed "
                  "four joints in 80 ms drew 16.7 A." % (hi + 1,
                                                         pk[hi] / 1000.0))
            print("          A spike this size with the joint at REST is an "
                  "abort — the drive stopping hard, not the path.")
            print("   " + "!" * 68)
        print("                  ^ PHASE_PLAN's R10 discriminator is WHICH "
              "joint spikes: J2/J4 points at supply margin, J6/J7 at wrist "
              "demand. For reference the 0.80 m/s run that reversed four "
              "joints in 80 ms drew 16.7 A on J4.")
        # Does a current spike coincide with a speed dip? That pairing is the
        # thing the plan's ladder is trying to catch.
        if spd and len(spd) == len(cur[0]):
            mv = sorted(v for v in spd if v > 0.005)
            if mv:
                half = 0.5 * pct(mv, 0.5)
                idx = [i for i, v in enumerate(spd) if 0.005 < v < half]
                if idx:
                    at_dip = max(max(abs(cur[j][i]) for i in idx)
                                 for j in range(7))
                    print("   peak current DURING a speed dip: %.2f A "
                          "(%.0f %% of the run's peak)"
                          % (at_dip / 1000.0, 100 * at_dip / max(pk[hi], 1)))
    if vol:
        flat, nd = constant_channel(vol[0])
        lo = min(min(v for v in vol[j]) for j in range(7))
        hi_ = max(max(v for v in vol[j]) for j in range(7))
        if flat:
            print("   voltage        CONSTANT at %.2f V — a placeholder, not "
                  "a measurement (F30 found this across 2085 samples). No "
                  "recording in this state can show an undervoltage sag."
                  % lo)
        else:
            print("   voltage        %.0f-%.0f V, %d distinct values -> ~%.0f V "
                  "resolution" % (lo, hi_, nd, max(hi_ - lo, 1) / max(nd - 1, 1)))
            print("                  ^ this channel is LIVE, which F30 "
                  "recorded it as not being. Coarse, but it can now show a "
                  "sag — worth pairing with the current column above for the "
                  "R10 ladder.")
    if tmp:
        rng = [(min(tmp[j]), max(tmp[j])) for j in range(7)]
        print("   temperature    %s C per joint"
              % " ".join("%.0f" % a if a == b else "%.0f-%.0f" % (a, b)
                         for a, b in rng))
        if all(a == b for a, b in rng):
            print("                  ^ constant within this run: 1 C "
                  "resolution and a short run. Compare ACROSS runs in a "
                  "session for drift, not within one.")
    print()


# ── J. conditioning / singularity ──────────────────────────────────────────
def section_conditioning(meta, t, qs):
    """How close did this run come to a singularity, and where?

    A singular configuration is one where the Jacobian loses rank, so a
    modest TCP speed demands unbounded joint rates. RM75 has four documented
    types and one of them is q4 = 0 — a STRAIGHT ELBOW — which our
    near-extension paths live close to.

    The value is the SDK's OWN, recovered by bisecting the threshold its
    analyser accepts — not a hand-rolled SVD. That distinction cost a
    correction: an SVD of the 6x7 Jacobian shrinks when the arm is merely
    RETRACTED, and reported a pose as singular that the SDK, and the
    stationary arm, called fine.

    REPORT IT, DO NOT INFER FROM IT. It is tempting to read low sigma_min as
    "joints will be working hard", and on two of our runs the correlation
    with peak joint utilisation is indeed -0.40 and -0.47 — but on
    toplid_right it is +0.49, with peak demand occurring at a BETTER
    conditioned pose than the run's median. The sign is path-dependent, so
    nothing is gated on this.

    It is SPEED-INDEPENDENT — purely configurational — so it can be measured
    once on the slowest rung of a ladder and carried to every rung above.
    """
    print("J. CONDITIONING — how close to a singularity")
    try:
        import robot_model as rm
    except Exception as exc:                                # noqa: BLE001
        print("   [SKIP] %s\n" % exc)
        return None
    vals = []
    step = max(1, len(t) // 400)
    for i in range(0, len(t), step):
        v = rm.sigma_min([qs[j][i] for j in range(7)])
        if v is None:
            print("   [SKIP] needs numpy for the singular-value "
                  "decomposition\n")
            return None
        vals.append((v, i))
    vals.sort()
    lo, med = vals[0], vals[len(vals) // 2]
    print("   sigma_min   min %.4f at t=%.2fs   p05 %.4f   median %.4f"
          % (lo[0], t[lo[1]], vals[len(vals) // 20][0], med[0]))
    print("   the SDK calls a pose singular below %.2f (its default)"
          % rm.SINGULARITY_THRESHOLD)
    note = rm.singularity_note(lo[0])
    if note:
        print("   [WARN] %s" % note)
    n_below = sum(1 for v, _ in vals if v < rm.SINGULARITY_WATCH)
    if n_below:
        print("   %.0f %% of samples sit below the %.2f watch level"
              % (100.0 * n_below / len(vals), rm.SINGULARITY_WATCH))
    print()
    return lo[0]


# ── I. payload gravity residual (F30) ──────────────────────────────────────
def section_payload(meta, rows, t, qs, payload):
    """Gravity torque per joint, and how it compares with measured current.

    F30's hypothesis: RealMan's collision detection compares MEASURED joint
    current against its MODEL's prediction, and a payload declared at
    centroid (0,0,0) puts the mass at the FLANGE, understating the moment
    arm — so predicted current sits below what J4 actually draws and the
    controller calls the difference an external collision.

    This section was REFUSED in an earlier version, correctly: the joint
    frames were derived by truncating the FK chain and reading the flange
    pose, which ranked J3 as the most gravity-loaded joint when the arm
    plainly draws most on J4. `robot_model.joint_frames` now derives them
    from the verified DH, and the ranking is physically sensible.

    STILL NOT AN ABSOLUTE PREDICTION, for two reasons that matter:
      * static only — no inertial term, and our motion reaches 0.85 of J4's
        rate limit, so acceleration torque is not negligible;
      * no torque constant is published, so N*m cannot be compared with amps
        except by ORDERING and by how each responds to the payload.
    """
    print("I. GRAVITY TORQUE vs MEASURED CURRENT — F30")
    cur = joints(rows, "current")
    if not qs or not cur:
        print("   [SKIP] needs joint positions and a current channel\n")
        return
    try:
        import robot_model as rm
    except Exception as exc:                                # noqa: BLE001
        print("   [SKIP] %s\n" % exc)
        return
    tool = (meta.get("commanded") or {}).get("tool_frame")
    m, cx, cy, cz = payload if payload else (0.0, 0.0, 0.0, 0.0)
    step = max(1, len(t) // 150)
    idx = list(range(0, len(t), step))
    tau_arm, tau_pl = [[0.0] * 7 for _ in range(2)]
    for i in idx:
        q = [qs[j][i] for j in range(7)]
        ta = rm.gravity_torque(q)
        tp = (rm.gravity_torque(q, m, (cx, cy, cz), tool) if m > 0 else ta)
        for j in range(7):
            tau_arm[j] = max(tau_arm[j], abs(ta[j]))
            tau_pl[j] = max(tau_pl[j], abs(tp[j]))
    pk = [max(abs(v) for v in cur[j]) / 1000.0 for j in range(7)]
    print("   payload %s" % ("%.3f kg at (%.3f, %.3f, %.3f) in %s"
                             % (m, cx, cy, cz, tool) if m > 0 else
                             "not supplied — pass --payload M,CX,CY,CZ "
                             "(read it with payload_audit.py; it is "
                             "CONTROLLER state, not in run.json)"))
    print("   %-5s %11s %11s %10s %9s" % ("joint", "tau arm Nm",
                                          "+payload", "peak A", "effort Nm"))
    for j in range(7):
        print("   J%-4d %11.2f %11.2f %10.2f %9.0f"
              % (j + 1, tau_arm[j], tau_pl[j], pk[j], rm.JOINT_EFFORT_NM[j]))
    order_model = sorted(range(7), key=lambda j: -tau_pl[j])
    order_meas = sorted(range(7), key=lambda j: -pk[j])
    print("   model ranks joints J%s ; the arm draws most on J%s"
          % (">J".join(str(j + 1) for j in order_model[:3]),
             ">J".join(str(j + 1) for j in order_meas[:3])))
    if order_model[0] != order_meas[0]:
        print("                  ^ they DISAGREE on the top joint. That is "
              "F30's direction if the gap is on J4: gravity alone does not "
              "explain its draw. It is also what a missing inertial term "
              "looks like, so this is a lead, not a finding.")
    if m <= 0:
        print("                  ^ without the payload this is the ARM ONLY. "
              "F30 is a claim ABOUT the payload record, so the comparison is "
              "not yet the one that tests it.")
    print()



# ── H. faults ──────────────────────────────────────────────────────────────
def section_faults(rows, t, spd=None, meta_ref=None, pos_ref=None):
    meta_ref = meta_ref or {}
    print("H. FAULTS AND STATE")
    err = joints(rows, "err")
    en = joints(rows, "en")
    clean = True
    if err:
        bad = {j + 1: sorted({int(v) for v in err[j] if v}) for j in range(7)}
        bad = {k: v for k, v in bad.items() if v}
        if bad:
            clean = False
            print("   [FAIL] joint error flags: %s" % bad)
            print("          0x0004 = undervoltage (F29 took J3-J7 down "
                  "this way and they refused to re-enable)")
        else:
            print("   joint error flags   all zero")
    if en:
        off = [j + 1 for j in range(7) if any(v == 0 for v in en[j])]
        if off:
            clean = False
            print("   [FAIL] joints not enabled at some point: %s" % off)
        else:
            print("   joint enables       all high throughout")
    st = col(rows, "arm_status", int)
    if st is not None:
        seq, last = [], None
        for i, v in enumerate(st):
            if v != last:
                seq.append((t[i], v))
                last = v
        print("   arm_status          %s%s"
              % (" -> ".join("%s@%.1fs" % (ARM_STATUS.get(v, v), tt)
                             for tt, v in seq[:10]),
                 " ..." if len(seq) > 10 else ""))
        # `arm_status == 0` IS NOT "STOPPED", and reading it that way inverts
        # the answer. Measured on 20260813T205300 (r=50): the controller drops
        # out of MOVE_L five times mid-run, and the TCP is doing 137-287 mm/s
        # through every one of those spans — full cruise. It leaves MOVE_L
        # while it traverses the blend arc BETWEEN two queued trajectories.
        #
        # Which makes the pattern a blend detector, and a far better one than
        # retention: it is binary, it comes from the controller's own state
        # machine, and it needs nothing from the noisy position channel.
        #
        #   IDLE spans WITH the tool moving  -> the blend is being applied
        #   one unbroken MOVE_L              -> no blending; on the same path
        #                                       at r=0 the whole chain ran as
        #                                       a single 3.13 s MOVE_L and the
        #                                       tool stopped dead at each
        #                                       waypoint INSIDE it
        #   IDLE spans with the tool stopped -> a real stop between
        #                                       trajectories
        if spd is not None and len(spd) == len(st):
            mv = sorted(v for v in spd if v > 0.005)
            cruise = pct(mv, 0.5) if mv else 0.0
            spans, i = [], 0
            while i < len(st):
                j = i
                while j < len(st) and st[j] == st[i]:
                    j += 1
                spans.append((st[i], i, j - 1))
                i = j
            mid = [s for s in spans[1:-1] if s[0] == 0]
            moving = [s for s in mid
                      if cruise > 0 and max(spd[s[1]:s[2] + 1]) > 0.25 * cruise]
            if moving:
                print("   [BLEND ACTIVE] the controller left MOVE_L %d time(s) "
                      "mid-run WITH THE TOOL STILL MOVING (up to %.0f mm/s). "
                      "That is the blend arc between two queued trajectories "
                      "— independent confirmation, from the controller's own "
                      "state machine, that the radius is being applied."
                      % (len(moving),
                         1000 * max(max(spd[s[1]:s[2] + 1]) for s in moving)))
            if len(mid) > len(moving):
                print("   [NOTE] %d mid-run IDLE span(s) with the tool at rest "
                      "— real stops between trajectories."
                      % (len(mid) - len(moving)))
            if not mid:
                print("   one unbroken move state from first motion to last. "
                      "No blend transitions: on this controller that is what "
                      "r=0 looks like, and any deceleration is inside the "
                      "single trajectory.")
            # WHICH corners got one. This is the strongest evidence in the
            # recording, because it is the controller's own state machine and
            # owes nothing to the position channel. It is how the first-corner
            # result was confirmed: on 20260813T205253 and ...5300 the tool
            # blended corners 2-5 and NOT corner 1, matching, independently,
            # the retention measurement of 11 % at position 1 against 59 %
            # everywhere else.
            cmdp = (meta_ref.get("commanded") or {}).get("poses")
            if cmdp and len(cmdp) > 2 and pos_ref:
                vidx = [min(range(len(pos_ref)),
                            key=lambda i: math.dist(pos_ref[i], cmdp[k][:3]))
                        for k in range(1, len(cmdp) - 1)]
                got = []
                for k, vi in enumerate(vidx):
                    hit = any(s[1] - 6 <= vi <= s[2] + 6 for s in moving)
                    got.append("%d%s" % (k + 1, "" if hit else "*"))
                missing = [g for g in got if g.endswith("*")]
                print("   corners that blended: %s   (* = none, the tool ran "
                      "through inside a single trajectory)" % " ".join(got))
                if missing and got and got[0].endswith("*"):
                    print("                  ^ the FIRST corner is the one "
                          "without a blend. Measured across the 2026-08-13 "
                          "runs this is positional, not angular: reversing "
                          "the path moved the effect onto whichever corner "
                          "came first.")
        if any(v in (9, 10) for _tt, v in seq):
            clean = False
            print("   [FAIL] the controller entered a STOP state. F26/F27 "
                  "saw RM_STOP_E arrive with the joints clean and no error "
                  "code — a planning/kinematics rejection, not a drive fault.")
    if clean:
        print("   nothing reported on any fault channel.")
        print("                  ^ this is NOT proof the run was fine. The "
              "arm has completed a run at 105.9 % of J4's limit, drawing "
              "16.7 A, with every one of these channels clean (H39). The "
              "pendant showed 'Out Of Reach, reason: Joint4 overspeed' for a "
              "refusal that none of these columns reported.")
    print()


# ── plots ──────────────────────────────────────────────────────────────────
def make_plot(d, meta, t, p, spd, qs, qds, rows, out):
    try:
        import matplotlib
        # Always Agg: this tool only ever writes a file, and an interactive
        # backend emits Qt thread warnings into the middle of the report.
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                # noqa: BLE001
        print("   [SKIP] --plot needs matplotlib (%s)" % exc)
        return None
    lim = ((meta.get("limits_in_force") or {}).get("joint_speed")
           or FALLBACK_JOINT_LIMIT)
    cur = joints(rows, "current")
    cum = [0.0]
    for i in range(1, len(p)):
        cum.append(cum[-1] + math.dist(p[i], p[i - 1]))

    fig, ax = plt.subplots(3, 2, figsize=(15, 11))
    fig.suptitle("%s   %s   cap %s m/s   r=%s"
                 % (d.name, meta.get("mode", "?"),
                    meta.get("line_speed_cap_m_s", "?"),
                    (meta.get("speeds") or {}).get("blend_pct", "?")))

    # TCP speed against ARC, with the commanded waypoints marked. Arc, not
    # time, because a dip's position on the path is the thing being read.
    a = ax[0][0]
    a.plot([1000 * c for c in cum], [1000 * v for v in spd], lw=0.8)
    cmd = (meta.get("commanded") or {}).get("poses")
    if cmd:
        for k in range(1, len(cmd) - 1):
            i = min(range(len(p)), key=lambda i: math.dist(p[i], cmd[k][:3]))
            a.axvline(1000 * cum[i], color="r", ls=":", lw=0.8)
        a.plot([], [], "r:", label="waypoints (by closest approach)")
        a.legend(fontsize=7)
    capv = meta.get("line_speed_cap_m_s")
    if capv:
        a.axhline(1000 * capv, color="g", ls="--", lw=0.8)
    a.set_xlabel("arc along the path, mm")
    a.set_ylabel("TCP speed, mm/s")
    a.set_title("TCP speed vs position — a dip at a red line is a corner")

    a = ax[0][1]
    for j in range(7):
        a.plot(t, [100 * abs(v) / lim[j] for v in qds[j]], lw=0.7,
               label="J%d" % (j + 1))
    a.axhline(100, color="r", ls="--", lw=1)
    a.axhline(98, color="orange", ls=":", lw=1)
    a.set_xlabel("s")
    a.set_ylabel("% of that joint's limit")
    a.set_title("Joint rate against its OWN limit (98 % = H63 dwell line)")
    a.legend(fontsize=7, ncol=4)

    a = ax[1][0]
    travel = [sum(abs(qs[j][i] - qs[j][i - 1]) for i in range(1, len(t)))
              for j in range(7)]
    a.bar(range(1, 8), travel)
    a.set_xlabel("joint")
    a.set_ylabel("total travel, deg")
    a.set_title("How far each joint travelled")

    a = ax[1][1]
    if cur:
        for j in range(7):
            a.plot(t, [abs(v) / 1000.0 for v in cur[j]], lw=0.7,
                   label="J%d" % (j + 1))
        a.set_ylabel("A")
        a.set_title("Joint current — R10's discriminator is WHICH joint peaks")
        a.legend(fontsize=7, ncol=4)
    else:
        a.text(0.5, 0.5, "no current channel", ha="center")
    a.set_xlabel("s")

    a = ax[2][0]
    a.plot(t, [1000 * v for v in spd], lw=0.8)
    a.set_xlabel("s")
    a.set_ylabel("TCP speed, mm/s")
    a.set_title("TCP speed vs time — pre-motion latency shows as a flat head")

    a = ax[2][1]
    a.plot([1000 * x[0] for x in p], [1000 * x[1] for x in p], lw=0.8)
    if cmd:
        a.plot([1000 * c[0] for c in cmd], [1000 * c[1] for c in cmd],
               "r.--", ms=6, lw=0.7, label="commanded")
        a.legend(fontsize=7)
    a.set_xlabel("X mm")
    a.set_ylabel("Y mm")
    a.set_title("Path in plan view — traced against commanded")
    a.set_aspect("equal", adjustable="datalim")

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def summarise(d, meta, rows, t, p, spd, qs, qds):
    """The machine-readable digest — one flat dict per run.

    Exists so a 7-rung ladder or a 5x acceptance set can be judged without
    re-reading terminal output, and so `trend()` has one shape to work from.
    Every field here is printed somewhere above; nothing is computed only for
    the JSON, because a number that appears in one output and not the other
    is a number nobody reconciles.
    """
    lim = ((meta.get("limits_in_force") or {}).get("joint_speed")
           or FALLBACK_JOINT_LIMIT)
    cur = joints(rows, "current")
    tmp = joints(rows, "temperature")
    err = joints(rows, "err")
    st = col(rows, "arm_status", int)
    cmd = (meta.get("commanded") or {}).get("poses")
    mv = sorted(v for v in spd if v > 0.005)
    arc = sum(math.dist(p[i], p[i - 1]) for i in range(1, len(p)))
    out = {
        "run": d.name,
        "mode": meta.get("mode"),
        "path": meta.get("path_file"),
        "line_speed_cap_m_s": meta.get("line_speed_cap_m_s"),
        "blend_pct": (meta.get("speeds") or {}).get("blend_pct"),
        "connect": (meta.get("speeds") or {}).get("connect"),
        "rung": (meta.get("ladder") or {}).get("rung"),
        "samples": len(rows),
        "duration_s": round(t[-1] - t[0], 3),
        "traced_m": round(arc, 4),
        "tcp_typical_mm_s": round(1000 * pct(mv, 0.5), 1) if mv else None,
        "tcp_p95_mm_s": round(1000 * pct(mv, 0.95), 1) if mv else None,
    }
    if cmd:
        out["commanded_m"] = round(sum(
            math.dist(cmd[i][:3], cmd[i + 1][:3])
            for i in range(len(cmd) - 1)), 4)
        out["start_err_mm"] = round(1000 * math.dist(p[0], cmd[0][:3]), 2)
        out["end_err_mm"] = round(1000 * math.dist(p[-1], cmd[-1][:3]), 2)
    if qs:
        out["joint_travel_deg"] = [round(sum(
            abs(qs[j][i] - qs[j][i - 1]) for i in range(1, len(t))), 1)
            for j in range(7)]
    if qds and any(any(v for v in q) for q in qds):
        pk = [max(abs(v) for v in qds[j]) for j in range(7)]
        out["joint_peak_deg_s"] = [round(v, 1) for v in pk]
        out["joint_peak_pct"] = [round(100 * pk[j] / lim[j], 1)
                                 for j in range(7)]
        f, jn = max((pk[j] / lim[j], j + 1) for j in range(7))
        out["worst_joint"] = jn
        out["worst_joint_pct"] = round(100 * f, 1)
        dt = (t[-1] - t[0]) / max(len(t) - 1, 1)
        util = [max(abs(qds[j][i]) / lim[j] for j in range(7))
                for i in range(len(t))]
        out["dwell_ms"] = {str(int(100 * thr)):
                           round(1000 * dt * sum(1 for u in util if u >= thr))
                           for thr in (0.95, 0.98, 1.00)}
    if cur:
        out["current_peak_A"] = [round(max(abs(v) for v in cur[j]) / 1000.0, 3)
                                 for j in range(7)]
    if tmp:
        out["temperature_C"] = [max(tmp[j]) for j in range(7)]
    if err:
        out["joint_err_nonzero"] = sorted(
            {j + 1 for j in range(7) for v in err[j] if v})
    if st is not None:
        out["entered_stop_state"] = any(v in (9, 10) for v in st)
    return out


def trend(summaries):
    """Across runs — the questions a single run cannot answer.

    PHASE_PLAN's definition of done is FIVE CONSECUTIVE clean runs per task,
    which is a property of a set, not of a run. Temperature is the same: it
    is quantised to 1 C and constant inside a 25 s run, so drift only ever
    shows between runs in a session.
    """
    if len(summaries) < 2:
        return
    print("=" * 78)
    print("  ACROSS %d RUNS" % len(summaries))
    print("=" * 78)
    print("  %-30s %6s %5s %7s %8s %9s %8s"
          % ("run", "cap", "r", "worst", "peak A", "end mm", "faults"))
    for s in summaries:
        wj = ("J%s %s%%" % (s.get("worst_joint"), s.get("worst_joint_pct"))
              if s.get("worst_joint") else "-")
        pa = (max(s["current_peak_A"]) if s.get("current_peak_A") else None)
        # "clean" means every fault channel was quiet, which is NOT the same
        # as a good run — 20260813T205319 ended 44 mm short having drawn
        # 26 A, with every channel clean. So the verdict here also weighs the
        # end pose and the current, the two columns that actually saw it.
        flt = ("ERR%s" % s["joint_err_nonzero"]
               if s.get("joint_err_nonzero") else
               ("STOP" if s.get("entered_stop_state") else
                ("ABORT" if (s.get("end_err_mm") or 0) > 10 else
                 ("HI-AMP" if (max(s["current_peak_A"])
                               if s.get("current_peak_A") else 0) >= 10
                  else "clean"))))
        print("  %-30s %6s %5s %7s %8s %9s %8s"
              % (s["run"][:30], s.get("line_speed_cap_m_s"),
                 s.get("blend_pct"), wj,
                 "%.2f" % pa if pa is not None else "-",
                 s.get("end_err_mm", "-"), flt))
    # Temperature drift across the session — 1 C resolution, so only a
    # multi-run view can see it at all.
    temps = [s["temperature_C"] for s in summaries if s.get("temperature_C")]
    if len(temps) >= 2:
        rise = [temps[-1][j] - temps[0][j] for j in range(7)]
        print("\n  temperature first -> last run: %s  (delta %s C)"
              % (" ".join("%d" % v for v in temps[-1]),
                 " ".join("%+d" % v for v in rise)))
        if max(rise) >= 3:
            print("  [NOTE] J%d rose %d C across this set. Worth watching "
                  "against duty cycle — the acceptance gate is five "
                  "consecutive runs, which is longer than anything run so far."
                  % (rise.index(max(rise)) + 1, max(rise)))
    # The gate itself.
    clean = [s for s in summaries
             if not s.get("joint_err_nonzero")
             and not s.get("entered_stop_state")
             and (s.get("end_err_mm") is None or s["end_err_mm"] <= 10)]
    print("\n  clean by the fault channels: %d of %d"
          % (len(clean), len(summaries)))
    print("  ^ PHASE_PLAN 6.4 wants FIVE CONSECUTIVE clean runs per task. "
          "This counts runs, not consecutiveness across sessions, and a "
          "clean fault channel is not a clean run — H39 completed at 105.9 % "
          "of J4's limit with every channel clean.")
    # Joint rate against speed, which is what the ladder gate uses.
    byspeed = {}
    for s in summaries:
        if s.get("worst_joint_pct") and s.get("line_speed_cap_m_s"):
            byspeed.setdefault(s["line_speed_cap_m_s"], []).append(
                s["worst_joint_pct"])
    if len(byspeed) >= 2:
        ks = sorted(byspeed)
        print("\n  worst joint % of limit vs commanded speed:")
        for k in ks:
            print("     %.2f m/s -> %s" % (k, max(byspeed[k])))
        lo, hi = ks[0], ks[-1]
        a, b = max(byspeed[lo]), max(byspeed[hi])
        if a > 0:
            print("     scaling %.2fx over a speed ratio of %.2fx%s"
                  % (b / a, hi / lo,
                     "  — linear, so the next rung is predictable"
                     if abs((b / a) / (hi / lo) - 1) < 0.15 else
                     "  — NOT linear, so do not extrapolate"))
    print()


# ── one run ────────────────────────────────────────────────────────────────
def analyse(run_dir, plot=False, payload=None, quiet=False, rates="auto"):
    d, meta, rows = load(run_dir)
    if len(rows) < 2:
        print("%s: fewer than 2 samples, nothing to analyse" % d.name)
        return
    t = col(rows, "t_mono")
    px, py, pz = (col(rows, "tcp_x"), col(rows, "tcp_y"), col(rows, "tcp_z"))
    if px is None:
        print("%s: no TCP columns in this stream" % d.name)
        return
    p = list(zip(px, py, pz))
    qs = joints(rows, "position")
    qds = joints(rows, "speed")
    # A DEAD `speed{n}` CHANNEL MUST NOT READ AS A SAFE RUN — see
    # `resolve_joint_rates`. Resolved once, here, so section_joints,
    # section_electrical, summarise and the plot all use the same rates.
    #
    # COMPARING SIM AGAINST REAL: pass `--rates derived`. Left on `auto` a
    # SIM run falls back to d(position)/dt while its REAL twin keeps the
    # reported channel, and the two estimators differ by ~20 % (H78) — so an
    # auto/auto comparison prices the estimator, not the mode.
    if rates == "derived" and qs:
        qds, qd_src = derive_joint_rates(t, qs), "d(position)/dt — forced"
    elif rates == "reported":
        qd_src = "reported speed{n} — forced"
    else:
        qds, qd_src = resolve_joint_rates(t, qs, qds)
    spd = windowed_speed(t, p)

    if not quiet:
        print("=" * 78)
        print("  %s" % d.name)
        print("=" * 78)
        section_provenance(d, meta)
    if not quiet:
        section_health(meta, rows, t, p)
    if not quiet:
        section_motion(meta, t, p, spd)
    if not quiet:
        section_tcp(meta, spd)
    if qs and not quiet:
        section_joints(meta, rows, t, p, qs, qds, qd_src)
        section_jacobian(meta, p, qs)
    sig = None
    if qs:
        if not quiet:
            sig = section_conditioning(meta, t, qs)
        else:
            try:
                import robot_model as rm
                st_ = max(1, len(t) // 400)
                vv = [rm.sigma_min([qs[j][i] for j in range(7)])
                      for i in range(0, len(t), st_)]
                vv = [v for v in vv if v is not None]
                sig = min(vv) if vv else None
            except Exception:                               # noqa: BLE001
                sig = None
    if not quiet:
        section_electrical(rows, t, qds, spd)
        section_faults(rows, t, spd, meta, p)
        section_payload(meta, rows, t, qs, payload)
    summary = summarise(d, meta, rows, t, p, spd, qs, qds)
    if sig is not None:
        summary["sigma_min"] = round(sig, 5)
    if plot and qs and qds:
        out = d / "analysis.png"
        got = make_plot(d, meta, t, p, spd, qs, qds, rows, out)
        if got and not quiet:
            print("   plot -> %s\n" % got)
    return summary


def main() -> int:
    if wants_help() or len(sys.argv) < 2:
        print(__doc__)
        return 0
    plot = "--plot" in sys.argv
    quiet = "--quiet" in sys.argv
    payload = None
    if "--payload" in sys.argv:
        try:
            payload = [float(x) for x in
                       sys.argv[sys.argv.index("--payload") + 1].split(",")]
            if len(payload) != 4:
                raise ValueError
        except (IndexError, ValueError):
            print("--payload wants MASS,CX,CY,CZ in kg and metres")
            return 1
    js = None
    if "--json" in sys.argv:
        i = sys.argv.index("--json")
        js = (sys.argv[i + 1] if i + 1 < len(sys.argv)
              and not sys.argv[i + 1].startswith("--") else "-")
    rates = "auto"
    if "--rates" in sys.argv:
        i = sys.argv.index("--rates")
        rates = sys.argv[i + 1] if i + 1 < len(sys.argv) else "auto"
        if rates not in ("auto", "derived", "reported"):
            print("--rates wants auto | derived | reported")
            return 1
    # Flags that take a value must have that value removed from the
    # directory list, or it gets opened as a run and reported as missing.
    taken = set()
    for f in ("--json", "--payload", "--rates"):
        if f in sys.argv and sys.argv.index(f) + 1 < len(sys.argv):
            nxt = sys.argv[sys.argv.index(f) + 1]
            if not nxt.startswith("--"):
                taken.add(nxt)
    dirs = [a for a in sys.argv[1:]
            if not a.startswith("--") and a not in taken]
    out = []
    for i, dd in enumerate(dirs):
        if i and not quiet:
            print()
        try:
            s = analyse(dd, plot=plot, payload=payload, quiet=quiet,
                        rates=rates)
        except (OSError, KeyError) as exc:
            print("%s: %s" % (dd, exc))
            continue
        if s:
            out.append(s)
    trend(out)
    if js:
        text = json.dumps(out, indent=2)
        if js == "-":
            print(text)
        else:
            open(js, "w").write(text + "\n")
            print("  json -> %s (%d runs)" % (js, len(out)))
    return 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    raise SystemExit(main())
