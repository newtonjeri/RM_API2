#!/usr/bin/env python3
"""Which joint binds on each cleaning path — over the WHOLE corpus, at once.

    python3 survey_binding.py                    # r=10, the production radius
    python3 survey_binding.py --radius all       # every radius, grouped
    python3 survey_binding.py --radius 10 --per-run
    python3 survey_binding.py --family toplid --per-run
    python3 survey_binding.py --mode sim         # SIM only, derived rates
    python3 survey_binding.py --pairs            # SIM vs REAL, same estimator

WHY THIS FILE EXISTS, AND IT IS NOT A CONVENIENCE.

Three findings were recorded against this corpus in two days and all three
were wrong, each from a one-off script that selected the wrong subset and
was then thrown away:

  * H79/H80 quoted ONE run — `20260814T200622`, `blend_r25_v250_right` — as
    a property of `hinge_area`. It is one outlier in six, at a blend radius
    §0 forbids, and its same-configuration twin reads J1 at 71 %.
  * A re-derivation aggregated peak-per-joint with `max` across all runs of a
    (family, speed) pair, POOLING r=10/25/50. That is how J4's 95-98 % — which
    happens only at the forbidden or aborted radii — became "the binding joint
    moves with speed".
  * The correction to that then resolved a run's family from its DIRECTORY
    NAME, so it saw only the `blend_*` runs of 2026-08-14 and missed the
    task-named runs of 2026-08-10/11 on byte-identical waypoints. n=8 became
    n=1, and `toplid` at 0.45 was reported J1-bound when 7 of its 8 runs
    bind J4.

All three have one shape: A SUBSET MISTAKEN FOR THE POPULATION. Pooling took
too much, labelling took too little, single-run citation took one. So this
tool prints its SELECTION RULE and its n on every table it produces, and
resolves the family from `run.json`'s `path_file` rather than any label.
A number from here is quotable; a number from a fresh one-off is not, until
it has been reconciled against this.

  READ THE FAMILY FROM `path_file`. Run-naming convention changed between
  sessions. `20260814T193318_blend_r10_v450_left` and
  `20260811T183500_toplid_left_left` are the same path — verified identical
  in `commanded.poses` — and a label filter drops the second.

ESTIMATOR, AND WHY SIM IS IN. Imports `analyse_run` rather than re-deriving,
so there is exactly one estimator in the repo.

  REAL  -> reported `speed{n}`. The controller's own number, and what H63 was
           calibrated on. Never d(position)/dt for a REAL safety verdict: the
           two differ WITH A SIGN (H78, derived runs high, up to 5.7 points and
           worst on J1), and H63 keys on dwell at >=98 %, so a derived verdict
           false-positives exactly where the rule decides something.
  SIM   -> d(position)/dt. SIM's `speed{n}` really is dead (~0.4 deg/s while
           the arm moves hundreds of degrees) but its POSITION channel is
           faithful, so the rates are recoverable — MODE_CHARACTERIZATION 1.

An earlier version of this file EXCLUDED SIM outright, reasoning from the dead
speed channel. That was wrong and it is worth recording why, because it is the
same error class as the other three: **it discarded 106 of the corpus's 200
runs — more than half, and more runs than REAL has** — on the strength of one
unusable channel, when the repo's own tooling (`analyse_run --rates derived`,
written for exactly this) recovers them from another. Three families exist ONLY
in SIM (`blend_r25_capp`, `chain`, `hinge_area_right` at some configurations),
and the whole `chain_rmix_vmix_capp` family — mixed radius, mixed speed, cap
applied, i.e. the production motion form — is largely SIM. A screen that cannot
see the production form is not a screen.

It also discarded the best evidence there is that SIM predicts REAL. On the
`chain_rmix_vmix_capp_v250_left` pair (SIM `20260815T153521`, REAL
`20260815T153752`), forced derived on both, every joint agrees within ONE
point and both name J4:

    J1 41/40  J2 36/36  J3 43/42  J4 62/63  J5 12/13  J6 32/33  J7 20/21

Rule that follows, and `--pairs` exists to keep it: comparing SIM against REAL
requires the SAME estimator on both sides, or the comparison prices the
estimator rather than the mode. What SIM must NOT be used for is an H63 dwell
verdict — dwell from derived rates is advisory, and every table below says
which estimator produced it.

WHAT IT PRINTS. Per (family, line speed, blend radius): the peak of every
joint as % of the limit in force, which joint binds and in how many runs of
the group, and the worst H63 dwell. Ranges are shown when runs disagree,
because run-to-run spread is the thing single-run citation hides.

OFFLINE AND READ-ONLY — `run.json` and `stream.csv`, no arm, no controller.
"""

import argparse
import collections
import hashlib
import json
import os
import pathlib
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyse_run                                        # noqa: E402

RUNS = pathlib.Path(__file__).resolve().parent.parent / "runs"


def logic_stamp():
    """Short hash of THIS FILE — print it beside every figure this tool emits.

    Exists because of a failure the other checks cannot catch. A pooling defect
    in `--pairs` was found and fixed; `--radius 10` was re-run and gave 6/8;
    `--radius all` was NOT re-run, and its pre-fix figure (33/34, actually
    31/34) was quoted to four sessions as if it came from the corrected tool.

    Re-deriving a number does not help here — the number was CORRECT WHEN
    COMPUTED and went stale when the code beneath it changed. Plausibility does
    not help either: nothing about 33 looks wrong. The only check that works is
    provenance — *when did I last run the thing that produced this, and has it
    changed since*. So every figure carries the hash of the logic that made it,
    and a figure quoted with a stale stamp is visibly stale.

    Hashing the whole file is deliberate: selection, estimator choice, grouping
    and the binding rule are all here, and any of them can move a number.
    """
    try:
        src = pathlib.Path(__file__).read_bytes()
    except OSError:
        return "unknown"
    return hashlib.sha256(src).hexdigest()[:12]

# Longest first: `top_left` must win over `top`, `hinge_area` over `area`.
FAMILIES = ("hinge_area", "toplid", "top_left", "top_right", "underside",
            "bowl", "seat", "lever", "top")

# H63: dwell at >=98 % of a limit separated the outcomes — 0 ms on every
# completed run, 110 ms before a silent stall, 330 ms before the violent one.
DWELL_THRESHOLD = 0.98


def family_of(meta, run_name):
    """The cleaning path a run exercises. `path_file` FIRST — see module doc.

    `path_file` is written either as `task:toplid_left` (a cleaning task) or
    as `../paths/blend_corner_001.py` (a synthetic characterisation path with
    no cleaning geometry in it at all). The second kind is returned tagged
    `synth:<stem>` rather than left blank, so it can never be read as a
    cleaning family with missing metadata — 40 of the 94 REAL runs are
    synthetic, and silently merging one into a cleaning family is the exact
    misattribution this tool exists to prevent.

    The directory name is the last resort and the one that caused a
    documented error, so it is tried only when the recording carries neither
    `path_file` nor `task_name`.
    """
    srcs = (meta.get("path_file"),
            (meta.get("commanded") or {}).get("path_file"),
            meta.get("task_name"), run_name)
    for src in srcs:
        if not src:
            continue
        for f in FAMILIES:
            if f in str(src):
                return f
    for src in srcs[:2]:
        if src and str(src).endswith(".py"):
            return "synth:%s" % pathlib.Path(str(src)).stem.replace(
                "_001", "").replace("_006", "")
    return None


def _stats(t, rates, lim):
    """(per-joint fraction of limit, dwell ms) for one estimator's rates."""
    peaks = [max(abs(v) for v in rates[j]) for j in range(7)]
    frac = [peaks[j] / lim[j] if lim[j] else 0.0 for j in range(7)]
    # Dwell is max-over-joints at each SAMPLE, then summed — not per joint.
    # A run can sit at 98 % on J4 for one stretch and on J1 for another; H63
    # counts the arm's exposure, not any one joint's.
    util = [max(abs(rates[j][i]) / lim[j] for j in range(7) if lim[j])
            for i in range(len(t))]
    dt = (t[-1] - t[0]) / max(len(t) - 1, 1)
    dwell = 1000 * dt * sum(1 for u in util if u >= DWELL_THRESHOLD)
    return frac, dwell, max(peaks)


def read_run(d, rates="auto"):
    """One run reduced to (config, per-joint peaks, dwell) — or None.

    Computes BOTH estimators where the channels allow, then picks per the
    `rates` argument. `auto` means reported for REAL and derived for SIM,
    which is the only pairing that is correct on both sides at once.
    """
    try:
        meta = json.load(open(d / "run.json"))
    except (OSError, ValueError):
        return None
    if not (d / "stream.csv").exists():
        return None
    try:
        rows = list(__import__("csv").DictReader(open(d / "stream.csv")))
    except OSError:
        return None
    if len(rows) < 10:
        return None

    t = analyse_run.col(rows, "t_mono")
    if not t:
        return None
    qds = analyse_run.joints(rows, "speed")       # reported
    qs = analyse_run.joints(rows, "position")     # for derived
    lim = ((meta.get("limits_in_force") or {}).get("joint_speed")
           or analyse_run.FALLBACK_JOINT_LIMIT)

    reported = _stats(t, qds, lim) if qds else None
    derived = (_stats(t, analyse_run.derive_joint_rates(t, qs), lim)
               if qs else None)

    real = not (meta.get("sim") or meta.get("mode") == "SIMULATION")
    # The reported channel is "live" only if its peak is consistent with real
    # motion. SIM leaves it at ~0.4 deg/s, which passes any `any(nonzero)`
    # guard — see resolve_joint_rates in analyse_run for the run that cost.
    live_reported = bool(reported) and reported[2] > 1.0

    if rates == "reported":
        chosen, src = reported, "reported"
    elif rates == "derived":
        chosen, src = derived, "derived"
    else:                                          # auto
        if real and live_reported:
            chosen, src = reported, "reported"
        else:
            chosen, src = derived, "derived"
    if chosen is None:
        return None

    cmd = meta.get("commanded") or {}
    return {
        "run": d.name,
        "family": family_of(meta, d.name),
        "real": real,
        "live": live_reported,
        "v": meta.get("line_speed_cap_m_s"),
        "r": cmd.get("blend_pct"),
        "frac": chosen[0],
        "dwell": chosen[1],
        "src": src,
        # An H63 dwell verdict is only meaningful on a REAL run read through
        # the controller's own channel. Everything else is advisory.
        "h63_valid": real and src == "reported",
        "derived_frac": derived[0] if derived else None,
    }


def cell(lo, hi):
    return "%.0f" % hi if abs(hi - lo) < 0.5 else "%.0f-%.0f" % (lo, hi)


def main():
    ap = argparse.ArgumentParser(
        description="Binding joint per cleaning path, over the whole corpus.")
    ap.add_argument("--radius", default="10",
                    help="blend radius to report: a number, or 'all'. "
                         "Default 10 — the radius §0 mandates. r>=25 is a "
                         "characterisation setting and MOTION_FINDINGS §10.3 "
                         "forbids it on dense geometry.")
    ap.add_argument("--family", default=None,
                    help="restrict to one path family, e.g. toplid")
    ap.add_argument("--per-run", action="store_true",
                    help="list every run instead of grouping")
    ap.add_argument("--mode", default="real", choices=("real", "sim", "both"),
                    help="which recordings to report. Default real. SIM is "
                         "read on d(position)/dt because its speed channel is "
                         "dead; its dwell is ADVISORY, not an H63 verdict.")
    ap.add_argument("--rates", default="auto",
                    choices=("auto", "reported", "derived"),
                    help="force an estimator. 'auto' = reported for REAL, "
                         "derived for SIM. Use 'derived' to compare SIM "
                         "against REAL, or the comparison prices the "
                         "estimator rather than the mode (H78).")
    ap.add_argument("--pairs", action="store_true",
                    help="SIM vs REAL on matched (family, v, r), forced "
                         "derived on both sides. Answers 'does SIM predict "
                         "REAL', which is what lets SIM screen all 7 joints.")
    ap.add_argument("--runs", default=str(RUNS), help="runs/ directory")
    a = ap.parse_args()

    root = pathlib.Path(a.runs)
    if not root.is_dir():
        print("no such directory: %s" % root)
        return 2

    if a.pairs:
        return report_pairs(root, a)

    every = [r for r in (read_run(d, a.rates) for d in sorted(root.iterdir())
                         if d.is_dir()) if r]
    # REAL runs must have a live reported channel to be trusted on `reported`;
    # SIM never does, and is admitted on its position channel instead.
    real = [r for r in every if r["real"] and (r["live"] or a.rates == "derived")]
    sim = [r for r in every if not r["real"]]
    sel = {"real": real, "sim": sim, "both": real + sim}[a.mode]
    # A run whose radius is unrecorded must be REPORTED, never quietly
    # dropped by a `== radius` filter — a silent drop is the same failure
    # this tool exists to prevent, just spelled differently. Today every
    # REAL run records `commanded.blend_pct` (0:12, 10:27, 25:28, 35:2,
    # 50:25), so this list is empty; it is here so that stops being luck.
    unknown_r = [r for r in sel if r["r"] is None]
    if a.radius != "all":
        sel = [r for r in sel if r["r"] == int(a.radius)]
    if a.family:
        sel = [r for r in sel if r["family"] == a.family]

    srcs = collections.Counter(r["src"] for r in sel)
    print("SELECTION RULE — quote this with any number taken from below.")
    print("  logic stamp: %s   (hash of survey_binding.py — a figure\n  quoted with a different stamp came from different code)" % logic_stamp())
    print("  %d run directories read; %d REAL, %d SIM; %d selected (mode=%s)."
          % (len(every), len(real), len(sim), len(sel), a.mode))
    print("  radius: %s      family: %s      --rates %s"
          % (a.radius, a.family or "all", a.rates))
    print("  estimator actually used: %s"
          % (", ".join("%s x%d" % (k, n) for k, n in srcs.most_common())
             or "n/a"))
    print("  Nothing pooled across radii. SIM is read on d(position)/dt "
          "(its speed\n  channel is dead); SIM dwell is ADVISORY, never an "
          "H63 verdict.")
    print("  blend radius is read from `commanded.blend_pct` — NOT the "
          "top-level\n  `blend_pct`, which is absent on every run and reads "
          "as 'not recorded'.")
    if unknown_r:
        print()
        print("  !! %d REAL run(s) carry NO recorded blend radius and are "
              "therefore\n     excluded from any --radius filter. Do not read "
              "a radius into them:" % len(unknown_r))
        for r in unknown_r:
            print("       %-42s family=%s v=%s" % (r["run"], r["family"],
                                                   r["v"]))
    print()

    if not sel:
        print("  nothing matched.")
        return 1

    if a.per_run:
        print("%-42s %-5s %-11s %5s %4s  %6s %6s %6s  %-4s %8s"
              % ("run", "mode", "family", "v", "r", "J1", "J4", "J5",
                 "bind", "dwell"))
        for r in sorted(sel, key=lambda x: (str(x["family"]), x["v"] or 0,
                                            x["r"] or 0, x["run"])):
            b = 1 + max(range(7), key=lambda j: r["frac"][j])
            print("%-42s %-5s %-11s %5s %4s  %5.1f%% %5.1f%% %5.1f%%  J%-3d "
                  "%5.0f ms%s"
                  % (r["run"][:42], "REAL" if r["real"] else "SIM",
                     r["family"], r["v"], r["r"],
                     100 * r["frac"][0], 100 * r["frac"][3],
                     100 * r["frac"][4], b, r["dwell"],
                     "" if r["h63_valid"] else "  (advisory)"))
    else:
        groups = collections.defaultdict(list)
        for r in sel:
            groups[(r["family"], r["v"], r["r"],
                    "REAL" if r["real"] else "SIM")].append(r)
        head = ("%-11s %-5s %5s %4s %3s  " % ("family", "mode", "v", "r", "n")
                + " ".join("%5s" % ("J%d" % (j + 1)) for j in range(7))
                + "   %-16s %8s" % ("binds", "dwell"))
        print(head)
        print("-" * len(head))
        for key in sorted(groups, key=lambda k: (str(k[0]), k[1] or 0,
                                                 k[2] or 0, k[3])):
            fam, v, r, mode = key
            g = groups[key]
            cells = [cell(min(100 * x["frac"][j] for x in g),
                          max(100 * x["frac"][j] for x in g))
                     for j in range(7)]
            binds = collections.Counter(
                "J%d" % (1 + max(range(7), key=lambda j: x["frac"][j]))
                for x in g)
            worst = max(x["dwell"] for x in g)
            print("%-11s %-5s %5s %4s %3d  %s   %-16s %5.0f ms%s"
                  % (fam, mode, v, r, len(g),
                     " ".join("%5s" % c for c in cells),
                     ",".join("%s x%d" % (k, n) for k, n in binds.most_common()),
                     worst, "" if g[0]["h63_valid"] else " (adv)"))

    hits = [r for r in sel if r["dwell"] > 0]
    print()
    print("Dwell at >=%.0f %% of a limit in this selection:"
          % (100 * DWELL_THRESHOLD))
    if not hits:
        print("  none.")
    else:
        for h in sorted(hits, key=lambda x: -x["dwell"]):
            print("  %-42s %-4s v=%s r=%s   %.0f ms%s"
                  % (h["run"], "REAL" if h["real"] else "SIM", h["v"], h["r"],
                     h["dwell"], "" if h["h63_valid"] else "   ADVISORY "
                     "(derived rates — not an H63 verdict)"))
        print("  NOTE: state what the exposure tracks before quoting it. The "
              "two REAL\n  exposures are at r=10 and at 0.6/0.8 m/s — they "
              "track LINE SPEED, not\n  radius, and they are the runs H63 was "
              "calibrated on.")
    return 0


def report_pairs(root, a):
    """Does SIM predict REAL? Matched configurations, derived on both sides.

    This is the question that decides whether SIM may be used as the
    seven-joint screen the J4 gate cannot be. It must be asked with ONE
    estimator on both sides — `analyse_run`'s docstring records that an
    auto/auto comparison prices the estimator (~20 %, H78) rather than the
    mode, which is a difference large enough to invent or hide a finding.
    """
    every = [r for r in (read_run(d, "derived") for d in sorted(root.iterdir())
                         if d.is_dir()) if r]
    if a.family:
        every = [r for r in every if r["family"] == a.family]
    if a.radius != "all":
        every = [r for r in every if r["r"] == int(a.radius)]

    groups = collections.defaultdict(lambda: {"SIM": [], "REAL": []})
    for r in every:
        groups[(r["family"], r["v"], r["r"])]["REAL" if r["real"] else "SIM"] \
            .append(r)
    matched = {k: v for k, v in groups.items() if v["SIM"] and v["REAL"]}

    # A configuration whose LINE SPEED is unrecorded is not a matched
    # configuration — it matches on a variable neither side states. The four
    # runs concerned (2026-08-10 16:09-17:31, all `hinge_area_left`) predate
    # the `line_speed_cap_m_s` field entirely: it is ABSENT from their
    # run.json, not null. They may well have been at the same speed; "may
    # well" is not a match. Reported separately rather than dropped — a
    # silent exclusion is the same failure as a wrong sample.
    incomparable = {k: v for k, v in matched.items() if k[1] is None}
    matched = {k: v for k, v in matched.items() if k[1] is not None}

    print("SIM vs REAL — matched (family, v, r), d(position)/dt on BOTH sides.")
    print("  logic stamp: %s" % logic_stamp())
    print("  %d configurations have both; radius=%s family=%s"
          % (len(matched), a.radius, a.family or "all"))
    print("  Cells are SIM/REAL peak %% of limit, worst run of each side.")
    print()
    if not matched:
        print("  no matched configurations.")
        return 1

    def modal_bind(runs):
        """The joint that binds in the MOST RUNS — never max-per-joint.

        Taking the max of each joint across runs and then arg-maxing is
        POOLING, and it is the exact defect retracted from H79: on
        `toplid` 0.45/r=10 it yields J1 (95 vs 94, from two different runs)
        while SEVEN of the eight runs individually bind J4. A binding joint
        is a per-run fact; summarise it by counting, not by mixing runs.
        """
        c = collections.Counter(
            1 + max(range(7), key=lambda j: x["frac"][j]) for x in runs)
        top = c.most_common()
        return top[0][0], top[0][1], len(runs), c

    head = ("%-11s %5s %4s %9s  " % ("family", "v", "r", "n SIM/REAL")
            + " ".join("%7s" % ("J%d" % (j + 1)) for j in range(7))
            + "   %-14s" % "binds SIM/REAL")
    print(head)
    print("-" * len(head))
    worst_gap = (0.0, None, None)
    agree = 0
    for key in sorted(matched, key=lambda k: (str(k[0]), k[1] or 0, k[2] or 0)):
        fam, v, r = key
        g = matched[key]
        # Columns are the WORST run per joint on each side — a envelope, and
        # labelled as such. The binding verdict beside them is per-run modal.
        s = [max(100 * x["frac"][j] for x in g["SIM"]) for j in range(7)]
        q = [max(100 * x["frac"][j] for x in g["REAL"]) for j in range(7)]
        cells = ["%3.0f/%-3.0f" % (s[j], q[j]) for j in range(7)]
        bs, ns, ts, _ = modal_bind(g["SIM"])
        bq, nq, tq, _ = modal_bind(g["REAL"])
        for j in range(7):
            if abs(s[j] - q[j]) > worst_gap[0]:
                worst_gap = (abs(s[j] - q[j]), "J%d" % (j + 1), key)
        if bs == bq:
            agree += 1
        print("%-11s %5s %4s %4d/%-4d  %s   J%d(%d/%d)/J%d(%d/%d)%s"
              % (fam, v, r, len(g["SIM"]), len(g["REAL"]),
                 " ".join("%7s" % c for c in cells),
                 bs, ns, ts, bq, nq, tq,
                 "" if bs == bq else "  <-- DISAGREE"))

    print()
    print("Joint columns are the WORST RUN PER JOINT on each side (an "
          "envelope, pooled\nacross runs). The binding verdict is per-run "
          "modal — J4(7/8) means seven of\neight runs bound J4. Never read a "
          "binding joint off the envelope columns.")
    print()
    print("Worst single-joint disagreement: %.0f points on %s (%s)."
          % (worst_gap[0], worst_gap[1], worst_gap[2]))
    print("Binding joint agreed on %d of %d matched configurations."
          % (agree, len(matched)))
    if incomparable:
        print()
        print("EXCLUDED as not comparable — line speed unrecorded on both "
              "sides (%d config%s):" % (len(incomparable),
                                        "" if len(incomparable) == 1 else "s"))
        for k, g in sorted(incomparable.items(), key=lambda kv: str(kv[0])):
            print("  %-11s v=unrecorded r=%-4s  %d SIM / %d REAL"
                  % (k[0], k[2], len(g["SIM"]), len(g["REAL"])))
        print("  These predate the `line_speed_cap_m_s` field (absent, not "
              "null). Matching\n  on a variable neither side records is not a "
              "match — counted as neither\n  agreement nor miss, and named "
              "here rather than dropped silently.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
