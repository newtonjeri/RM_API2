#!/usr/bin/env python3
"""Plot an rm_pose_t points file. 3D plus the three orthographic views.

    python3 plot_alix_points.py alix_points.yaml
    python3 plot_alix_points.py a.yaml b.yaml -o compare.png
    python3 plot_alix_points.py alix_points.yaml --show

Standalone: PyYAML and matplotlib, nothing else.

The 3D view takes two thirds of the figure; TOP (XY), FRONT (XZ) and SIDE
(YZ) share the remaining third. UNIFORM SCALE on every axis in every panel —
one millimetre of X is one millimetre of Y is one millimetre of Z, so a flat
surface looks flat and a slope has its real slope.

Points are drawn in the order the `sequence` traverses them, which is the
order they would be dispatched.
"""

import argparse
import math
import pathlib
import sys

import matplotlib
import yaml

MAX_FILES = 3
SURFACE, INK, INK2 = "#fcfcfb", "#0b0b0b", "#52514e"
GRID, AXIS = "#dcdcd8", "#b8b8b2"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a")     # blue, orange, aqua


def short(name):
    """point14 -> p14. Anything without a number keeps its own name."""
    digits = "".join(ch for ch in name if ch.isdigit())
    return "p" + digits if digits else name


def label_indices_2d(names, a, b, tol):
    """First visit to each point, MINUS the ones that project onto a label
    already placed in this panel.

    An elevation drops an axis, so points differing only in that axis land
    on the identical spot — in FRONT the five points sharing X and Z pile
    up exactly. Their labels then overprint into things like "p1p13", which
    reads as a point name that does not exist. One label per visible
    position is the honest maximum.
    """
    seen, placed, out = set(), [], []
    for i, n in enumerate(names):
        if n in seen:
            continue
        seen.add(n)
        if any(abs(a[i] - pa) < tol and abs(b[i] - pb) < tol
               for pa, pb in placed):
            continue
        placed.append((a[i], b[i]))
        out.append(i)
    return out


def load(path):
    """(label, xyz in mm, side) — the traversal, in dispatch order."""
    p = pathlib.Path(path)
    if not p.is_file():
        raise SystemExit("no such points file: %s" % p)
    try:
        doc = yaml.safe_load(p.read_text())
    except yaml.YAMLError as exc:
        # Reached most often by passing the script itself, or any non-YAML
        # file. The parser's own traceback says "expected <document start>",
        # which does not tell you that.
        hint = ("\n  That is this script, not a points file — you probably "
                "meant alix_points.yaml." if p.suffix == ".py" else "")
        raise SystemExit("%s is not valid YAML.%s\n  %s"
                         % (p, hint, str(exc).splitlines()[0]))
    if not isinstance(doc, dict):
        raise SystemExit("%s: top level must be a mapping" % p)

    points = doc.get("points") or doc.get("cleaning_points")
    sequence = doc.get("sequence") or doc.get("cleaning_sequence")
    if not points or not sequence:
        raise SystemExit("%s: needs both `points` and `sequence`" % p)
    if any(isinstance(v, dict) for v in points.values()):
        raise SystemExit(
            "%s is the translation/rotation format, not rm_pose_t. Plot it "
            "with RMDemo_PointSequence/points/plot_points.py instead." % path)

    walk, prev = [], None
    for seg in sequence:
        a, b = seg[0], seg[1]
        for n in (a, b):
            if n not in points:
                raise SystemExit("%s: sequence names %r, which is not in "
                                 "`points`" % (path, n))
        if prev is None or a != prev:
            walk.append(a)
        walk.append(b)
        prev = b

    # EVERY point in the file is plotted, not only the ones the sequence
    # names. A point the sequence never visits is still a point in the file
    # — leaving it out makes the plot disagree with what you are reading.
    # The PATH still follows the sequence; the markers show the whole set.
    return {"label": pathlib.Path(path).name,
            "side": str(doc.get("side") or "?"),
            "names": list(points),
            "xyz": [[1000.0 * v for v in points[n][:3]] for n in points],
            "path": [[1000.0 * v for v in points[n][:3]] for n in walk],
            "unvisited": [n for n in points if n not in set(walk)]}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("Standalone")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="rm_pose_t YAML points file(s)")
    ap.add_argument("-o", "--out", default=None, help="output PNG")
    ap.add_argument("--show", action="store_true", help="open a window")
    args = ap.parse_args()
    if len(args.files) > MAX_FILES:
        raise SystemExit("%d files given; this plot takes at most %d."
                         % (len(args.files), MAX_FILES))

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D                   # noqa: F401
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MaxNLocator

    data = [load(f) for f in args.files]
    allp = [p for d in data for p in d["xyz"]]
    lo = [min(p[i] for p in allp) for i in range(3)]
    hi = [max(p[i] for p in allp) for i in range(3)]

    fig = plt.figure(figsize=(15, 9), facecolor=SURFACE)
    # Three columns: 3D spans two of them, the orthographic views stack in
    # the third.
    gs = fig.add_gridspec(3, 3, hspace=0.42, wspace=0.28,
                          left=0.04, right=0.965, top=0.855, bottom=0.06)

    ax3 = fig.add_subplot(gs[:, 0:2], projection="3d")
    ax3.set_facecolor(SURFACE)
    for d, c in zip(data, SERIES):
        px = [p[0] for p in d["path"]]
        py = [p[1] for p in d["path"]]
        pz = [p[2] for p in d["path"]]
        ax3.plot(px, py, pz, color=c, linewidth=1.8)
        x = [p[0] for p in d["xyz"]]
        y = [p[1] for p in d["xyz"]]
        z = [p[2] for p in d["xyz"]]
        ax3.scatter(x, y, z, color=c, s=26, edgecolors=SURFACE,
                    linewidths=1.0, depthshade=False)
        for i, n in enumerate(d["names"]):
            ax3.text(x[i], y[i], z[i], "  " + short(n),
                     color=INK2, fontsize=7)
    ax3.set_xlabel("X [mm]", color=INK2, fontsize=8)
    ax3.set_ylabel("Y [mm]", color=INK2, fontsize=8)
    ax3.set_zlabel("Z [mm]", color=INK2, fontsize=8, labelpad=8)
    ax3.tick_params(colors=INK2, labelsize=7)
    # At true proportions the short axis is physically small and the default
    # locator crams its labels together. Fewer ticks — the scale is
    # unchanged, only how often it is labelled.
    for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        axis.set_major_locator(MaxNLocator(4))
    ax3.set_title("3D — Z up   ·   uniform scale on all axes",
                  color=INK, fontsize=11, loc="left", y=0.90)
    ax3.view_init(elev=26, azim=-60)
    # UNIFORM SCALE: the box carries the data's own proportions, so a
    # millimetre is the same length on all three axes.
    pad = 0.04 * max(max(h - l for h, l in zip(hi, lo)), 1e-6)
    ax3.set_xlim(lo[0] - pad, hi[0] + pad)
    ax3.set_ylim(lo[1] - pad, hi[1] + pad)
    ax3.set_zlim(lo[2] - pad, hi[2] + pad)
    ax3.set_box_aspect([max(h - l + 2 * pad, 1e-6) for h, l in zip(hi, lo)])
    for pane in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        pane.pane.set_facecolor(SURFACE)
        pane.pane.set_edgecolor(AXIS)
        pane.pane.set_alpha(1.0)
        pane._axinfo["grid"]["color"] = GRID
        pane._axinfo["grid"]["linewidth"] = 0.6

    for row, (dims, xl, yl, title) in enumerate(
            (((0, 1), "X [mm]", "Y [mm]", "TOP (XY) — looking down"),
             ((0, 2), "X [mm]", "Z [mm]", "FRONT (XZ) — looking along -Y"),
             ((1, 2), "Y [mm]", "Z [mm]", "SIDE (YZ) — looking along +X"))):
        ax = fig.add_subplot(gs[row, 2])
        for d, c in zip(data, SERIES):
            ax.plot([p[dims[0]] for p in d["path"]],
                    [p[dims[1]] for p in d["path"]],
                    color=c, linewidth=1.8, zorder=2, solid_capstyle="round")
            a = [p[dims[0]] for p in d["xyz"]]
            b = [p[dims[1]] for p in d["xyz"]]
            ax.plot(a, b, linestyle="none", marker="o", markersize=4.5,
                    color=c, markeredgecolor=SURFACE, markeredgewidth=1.0,
                    zorder=3)
            tol = 0.012 * max(max(a) - min(a), max(b) - min(b), 1e-9)
            for i in label_indices_2d(d["names"], a, b, tol):
                ax.annotate(short(d["names"][i]), (a[i], b[i]),
                            textcoords="offset points", xytext=(4, 3),
                            fontsize=6, color=INK2, zorder=5)
        ax.set_facecolor(SURFACE)
        ax.set_title(title, color=INK, fontsize=9, pad=9, loc="left")
        ax.set_xlabel(xl, color=INK2, fontsize=7.5)
        ax.set_ylabel(yl, color=INK2, fontsize=7.5)
        ax.tick_params(colors=INK2, labelsize=6.5)
        ax.xaxis.set_major_locator(MaxNLocator(6))
        ax.yaxis.set_major_locator(MaxNLocator(5))
        ax.grid(True, color=GRID, linewidth=0.6, zorder=0)
        ax.set_axisbelow(True)
        for sp in ax.spines.values():
            sp.set_color(AXIS)
            sp.set_linewidth(0.8)
        # UNIFORM SCALE, with the axes keeping their size: matplotlib
        # expands the SHORT axis's limits to match, rather than shrinking
        # the box. Same millimetre on both axes either way, but shrinking
        # left an elevation of this surface about ten pixels tall, with its
        # tick labels printed on top of each other. The empty space above
        # and below the path is the honest picture — the Z variation really
        # is that small next to the X and Y extent.
        ax.set_aspect("equal", adjustable="datalim")

    fig.suptitle("rm_pose_t points — %s"
                 % ", ".join(d["label"] for d in data),
                 color=INK, fontsize=13, x=0.04, ha="left", y=0.975)
    handles = [Line2D([], [], color=c, linewidth=2.4, marker="o",
                      markersize=6, markeredgecolor=SURFACE,
                      label="%s  (%s arm, %d points, %d in the path)"
                            % (d["label"], d["side"], len(d["xyz"]),
                               len(d["path"])))
               for d, c in zip(data, SERIES)]
    leg = fig.legend(handles=handles, loc="upper left",
                     bbox_to_anchor=(0.04, 0.945), frameon=False,
                     fontsize=9, ncol=len(data), columnspacing=2.4)
    for txt in leg.get_texts():
        txt.set_color(INK2)

    out = pathlib.Path(args.out) if args.out else \
        pathlib.Path(__file__).resolve().parent / (
            "%s.png" % "_".join(pathlib.Path(f).stem for f in args.files)[:60])
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print("wrote %s" % out)
    for d in data:
        print("  %-24s %s arm, %3d points, %3d in the path, %7.0f mm"
              % (d["label"], d["side"], len(d["xyz"]), len(d["path"]),
                 sum(math.dist(d["path"][i], d["path"][i + 1])
                     for i in range(len(d["path"]) - 1))))
        if d["unvisited"]:
            shown = [short(n) for n in d["unvisited"][:10]]
            print("    %d of %d points are not visited by the sequence: %s%s"
                  % (len(d["unvisited"]), len(d["xyz"]), ", ".join(shown),
                     " ..." if len(d["unvisited"]) > len(shown) else ""))
    print("  extents  X %.0f..%.0f   Y %.0f..%.0f   Z %.0f..%.0f mm"
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
