"""C19 self-test — does the corner measurement RECOVER a known answer?

Offline, no arm, no emulator. Synthesises TCP streams that walk
`paths/blend_corner_001.py` at a prescribed speed with a PRESCRIBED
retention at each corner, runs the real `corner_speeds`, and checks what
comes back against what went in. A measurement pipeline that has never been
run against a known input is an assumption, not an instrument.

    python3 verify_blend_measure.py

WHAT IT CAUGHT (2026-08-13), and why it exists:

  THE CHORD ARTIFACT. Speed is |dp|/dt between consecutive samples — a
  chord. Where an interval straddles a vertex the chord cuts the corner and
  reads short, by a geometric amount that GROWS WITH THE TURN ANGLE: 15 deg
  read 2 % low, 90 deg read 25 % low, on a stream holding constant speed
  through every corner. Uncorrected, C19 would have manufactured exactly the
  deceleration it exists to detect, worst at the sharpest corners — and the
  result would have looked like a clean confirmation. `corner_speeds` now
  drops the one interval per vertex that straddles it.

  THE RESOLUTION FLOOR. Accuracy tracks SAMPLES INSIDE THE DIP and nothing
  else — not sample rate or speed on their own:

      20 samples -> 2 pt error     4 samples -> 13 pt
       8 samples -> 5 pt           2 samples -> 20 pt

  Hence `MIN_CORNER_SAMPLES = 8`, and a corner measured from fewer is
  printed with "?" rather than reported as a number.

REMAINING KNOWN BIAS: with the straddling sample dropped, the sample nearest
the vertex is gone too, so retention reads ~3-5 points HIGH. That direction
is deliberate — it UNDER-reports deceleration, so the test cannot flatter
the hypothesis it is testing.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help                      # noqa: E402
import test_blend_corner as tb                        # noqa: E402

# 10 points — the MEASURED envelope under the real sampling behaviour, not an
# aspiration. Was 8 while every case here assumed a perfect sampler; adding
# the measured position-field aliasing showed the true envelope. The response
# is monotonic and slightly COMPRESSED — a prescribed 100/70/50/30/20 comes
# back as 91/73/54/36/24 — so the top end reads a little low and the bottom a
# little high, and the ordering between corners, which is what the test is
# read for, survives intact. Setting the tolerance tighter than the method
# delivers would only make the self-test dishonest.
TOL_PT = 0.10

# Once the blend passes ~40 % of a segment the dips from its two ends MEET:
# the approach never returns to cruise, so retention is measured against a
# depressed reference and reads high. That is geometry, not a defect in the
# analysis — no estimator can recover a plateau that the motion does not
# contain. Measured at a blend of 50 % of a 65 mm segment: 12 points high
# against a known 50 %. It is allowed a wider band here, and `main` prints a
# warning on any run that sweeps a blend this wide, so the number is never
# read as if it carried the same weight as the others.
TOL_PT_WIDE = 0.15
WIDE_DIP = 0.40


def build(poses):
    seg = [math.dist(poses[i][:3], poses[i + 1][:3])
           for i in range(len(poses) - 1)]
    vert, acc = [], 0.0
    for s in seg[:-1]:
        acc += s
        vert.append(acc)
    return seg, vert, sum(seg)


# The MEASURED aliasing of the UDP position field, as a fraction of one
# sample step. The push clock is clean (dt = 10.0 +-0.14 ms) but the position
# it carries is not synchronous with it, so the reported point runs ahead of
# and behind the true one. Differencing consecutive samples then gives step
# ratios with median 1.00, p10 0.39, p90 1.63 (run 20260813T183639, J2/J4/J6
# against the controller's own joint-speed channel, which is smooth).
#
# The sequence is FIXED, not random: a self-test that passes on a different
# draw each time is not a regression test. Its mean is zero, so it moves
# where each sample is reported without changing where the path goes.
POS_JITTER = [0.0, .30, -.30, .10, .45, -.45, .20, -.10, .35, -.35,
              .05, .40, -.40, .15, -.15, .25, -.25, .50, -.50, .10]


def synth(poses, seg, vert, total, retain, cruise=0.25, hz=100.0,
          dip_frac=0.10, frame_rot=False, jitter=0.0):
    """Walk the path at `cruise`, dipping to retain[k]*cruise at vertex k.

    `jitter` scales POS_JITTER: 1.0 reproduces the aliasing measured on
    hardware. It perturbs WHERE ALONG THE PATH each sample is reported, not
    the path or the speed profile — so the prescribed retention is still the
    right answer, and any error it causes is the instrument's, not the
    stream's.
    """
    def speed_at(s):
        v = cruise
        for k, vs in enumerate(vert):
            half = dip_frac * min(seg[k], seg[k + 1])
            d = abs(s - vs)
            if d <= half:
                f = d / half
                v = min(v, cruise * (retain[k] + (1 - retain[k]) * f))
        return max(v, 1e-4)

    rows, s, t, n = [], 0.0, 0.0, 0
    while s <= total - 1e-9:
        step = speed_at(s) / hz
        # Report the position the push ACTUALLY carries: the true arc offset
        # by the sampling phase error. Clamped into the path so the last
        # sample cannot run past the end.
        s_rep = s + jitter * POS_JITTER[n % len(POS_JITTER)] * step
        s_rep = min(max(s_rep, 0.0), total - 1e-9)
        rem, i = s_rep, 0
        while i < len(seg) - 1 and rem > seg[i]:
            rem -= seg[i]
            i += 1
        f = rem / seg[i]
        p = [poses[i][j] + (poses[i + 1][j] - poses[i][j]) * f
             for j in range(3)]
        if frame_rot:                     # World -> base: (x,y,z)->(z,y,-x)
            p = [p[2], p[1], -p[0]]
        rows.append((t, p[0], p[1], p[2]))
        s += step
        t += 1.0 / hz
        n += 1
    return rows


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    path = tb.load_path(tb.DEFAULT_PATH)
    poses, angles = path["poses"], path["angles"]
    seg, vert, total = build(poses)
    # SYNTHESISE AT THE PATH'S OWN SETTINGS, not a fixed guess. The dip a
    # blend actually produces scales with the blend radius, and the sample
    # spacing with the speed — so a self-test hardcoding either would be
    # validating conditions the run will never see.
    cruise = path["speed"]
    blends = [b for b in path["blends"] if b > 0] or [25]
    dip = min(blends) / 100.0          # the NARROWEST dip that will be swept
    base = {"cruise": cruise, "dip_frac": dip}
    print("synthesised at the path's own settings: %.2f m/s, dip = %d %% of a "
          "segment (the narrowest blend in BLEND_SWEEP)\n" % (cruise, 100 * dip))

    cases = [
        ("no dip anywhere",              [1.0] * 5, {}),
        ("uniform 50 %",                 [0.5] * 5, {}),
        ("uniform 20 %",                 [0.2] * 5, {}),
        ("full stop (2 %)",              [0.02] * 5, {}),
        ("graded 90/70/50/30/10",        [.9, .7, .5, .3, .1], {}),
        ("graded, BASE frame reported",  [.9, .7, .5, .3, .1],
         {"frame_rot": True}),
        ("widest blend in the sweep",    [0.5] * 5,
         {"dip_frac": max(blends) / 100.0}),
        # THE CASE THAT WAS MISSING until 2026-08-13. Every case above
        # assumes a perfect sampler, which is why this self-test passed while
        # the hardware runs it was vouching for read a p95 of 162 % of cap.
        # A measurement pipeline validated only on clean input is validated
        # against a stream that does not exist.
        ("uniform 50 %, MEASURED aliasing", [0.5] * 5, {"jitter": 1.0}),
        ("graded, MEASURED aliasing",    [.9, .7, .5, .3, .1],
         {"jitter": 1.0}),
        ("no dip, MEASURED aliasing",    [1.0] * 5, {"jitter": 1.0}),
    ]
    cases = [(n, r, dict(base, **kw)) for n, r, kw in cases]
    print("C19 measurement self-test — prescribed vs recovered retention")
    print("tolerance %.0f points at >= %d samples per corner\n"
          % (100 * TOL_PT, tb.MIN_CORNER_SAMPLES))
    print("  %-30s" % "case" + "".join("%6s deg" % a for a in angles)
          + "   max err")
    print("  " + "-" * 76)
    failures = 0
    for name, retain, kw in cases:
        rows = synth(poses, seg, vert, total, retain, **kw)
        ok, rec, cmd = tb.path_followed(rows, poses)
        if not ok:
            print("  %-30s path_followed REJECTED (%.3f/%.3f m)"
                  % (name, rec, cmd))
            failures += 1
            continue
        res = tb.corner_speeds(rows, poses, 0.0, rows[-1][0])
        # A corner the tool marks "?" is NOT a reported measurement — it is
        # the tool declining to report one. Scoring its value against the
        # tolerance would fail the self-test for behaving correctly. What is
        # checked instead is that the marking appears when it should.
        worst, cells, marked = 0.0, [], 0
        for want, r in zip(retain, res):
            if r is None:
                cells.append("   none ")
                worst = 9.0
                continue
            thin = r[3] < tb.MIN_CORNER_SAMPLES
            marked += thin
            cells.append("%6.0f%%%s" % (100 * r[2], "?" if thin else " "))
            if not thin:
                worst = max(worst, abs(r[2] - want))
        tol = TOL_PT_WIDE if kw.get("dip_frac", 0) >= WIDE_DIP else TOL_PT
        bad = worst > tol
        failures += bad
        note = ("  (%d corner%s declined — correct)"
                % (marked, "" if marked == 1 else "s")) if marked else ""
        if tol != TOL_PT and not marked:
            note += "  (dip >= %d %% of the segment: no undisturbed cruise, " \
                    "%d pt band)" % (100 * WIDE_DIP, 100 * tol)
        print("  %-30s%s  %5.1f pt %s%s"
              % (name, "".join(cells), 100 * worst,
                 "FAIL" if bad else "ok", note))

    # The stall and wrong-stream guards must reject, not measure.
    print()
    truncated = synth(poses, seg, vert, total, [1.0] * 5)
    truncated = truncated[:int(0.77 * len(truncated))]      # H45: stopped at 77 %
    ok, rec, cmd = tb.path_followed(truncated, poses)
    print("  %-30s%s" % ("H45 stall (path truncated 77 %)",
                         "REJECTED — correct" if not ok else "ACCEPTED — FAIL"))
    failures += ok
    doubled = synth(poses, seg, vert, total, [1.0] * 5) * 2
    ok2, _, _ = tb.path_followed(doubled, poses)
    print("  %-30s%s" % ("stream is not the path",
                         "REJECTED — correct" if not ok2 else "ACCEPTED — FAIL"))
    failures += ok2

    print("\n%s  (%d failure%s)"
          % ("ALL CHECKS PASSED" if not failures else "SELF-TEST FAILED",
             failures, "" if failures == 1 else "s"))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
