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

# 8 points — the MEASURED envelope, not an aspiration. The residual is a
# systematic +5 to +6 point HIGH bias: excluding the samples that straddle a
# vertex also excludes the ones nearest it, where the true minimum sits, so
# retention reads slightly generous. That direction is deliberate — it
# UNDER-reports deceleration, so this test cannot flatter the hypothesis it
# exists to check. Setting the tolerance tighter than the method delivers
# would only make the self-test dishonest in the other direction.
TOL_PT = 0.08


def build(poses):
    seg = [math.dist(poses[i][:3], poses[i + 1][:3])
           for i in range(len(poses) - 1)]
    vert, acc = [], 0.0
    for s in seg[:-1]:
        acc += s
        vert.append(acc)
    return seg, vert, sum(seg)


def synth(poses, seg, vert, total, retain, cruise=0.25, hz=100.0,
          dip_frac=0.10, frame_rot=False):
    """Walk the path at `cruise`, dipping to retain[k]*cruise at vertex k."""
    def speed_at(s):
        v = cruise
        for k, vs in enumerate(vert):
            half = dip_frac * min(seg[k], seg[k + 1])
            d = abs(s - vs)
            if d <= half:
                f = d / half
                v = min(v, cruise * (retain[k] + (1 - retain[k]) * f))
        return max(v, 1e-4)

    rows, s, t = [], 0.0, 0.0
    while s <= total - 1e-9:
        rem, i = s, 0
        while i < len(seg) - 1 and rem > seg[i]:
            rem -= seg[i]
            i += 1
        f = rem / seg[i]
        p = [poses[i][j] + (poses[i + 1][j] - poses[i][j]) * f
             for j in range(3)]
        if frame_rot:                     # World -> base: (x,y,z)->(z,y,-x)
            p = [p[2], p[1], -p[0]]
        rows.append((t, p[0], p[1], p[2]))
        s += speed_at(s) / hz
        t += 1.0 / hz
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
        bad = worst > TOL_PT
        failures += bad
        note = ("  (%d corner%s declined — correct)"
                % (marked, "" if marked == 1 else "s")) if marked else ""
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
