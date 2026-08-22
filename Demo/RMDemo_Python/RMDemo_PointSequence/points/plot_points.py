#!/usr/bin/env python3
"""Plot cleaning points in 3D, with XY as the top view. Up to three files
overlaid, one colour each.

    python3 plot_points.py example_points.yaml
    python3 plot_points.py a.yaml b.yaml c.yaml -o compare.png
    python3 plot_points.py a.yaml b.yaml --show

LAYOUT. The 3D view takes two thirds of the figure; the three orthographic
views share the remaining third, stacked:

    3D          two thirds, Z up
    TOP  (XY)   looking down
    FRONT (XZ)  looking along -Y
    SIDE (YZ)   looking along +X

UNIFORM SCALE ON EVERY AXIS, in every panel. One millimetre of X is one
millimetre of Y is one millimetre of Z. Nothing is exaggerated or
normalised, so a flat surface looks flat and a slope has its real slope.

The points are plotted as given, in the traversal order the sequence
defines.
"""

import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import matplotlib                                            # noqa: E402
import point_sequence as ps                                  # noqa: E402

MAX_FILES = 3

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#dcdcd8"
AXIS = "#b8b8b2"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a")     # blue, orange, aqua


def load(path):
    """The waypoints of one file, in traversal order, in millimetres."""
    points, sequence, _ = ps.load_points(path)
    traversal = ps.resolved_traversal(points, sequence)
    xyz = [[1000.0 * v for v in points[n][:3]] for n, _ in traversal]
    return {"label": pathlib.Path(path).name, "xyz": xyz}


def equalise_2d(ax):
    """Uniform scale: one millimetre is the same length on both axes.

    `adjustable="box"` shrinks the axes box to whatever shape the data
    actually has, rather than padding the data out to fill a fixed frame.
    An elevation of a near-flat surface therefore draws as a thin strip —
    which is what a near-flat surface looks like at true proportions.
    """
    ax.set_aspect("equal", adjustable="box")


def style_2d(ax, xlabel, ylabel, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=9, pad=9, loc="left")
    ax.set_xlabel(xlabel, color=INK2, fontsize=7.5)
    ax.set_ylabel(ylabel, color=INK2, fontsize=7.5)
    ax.tick_params(colors=INK2, labelsize=6.5)
    ax.grid(True, color=GRID, linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_color(AXIS)
        s.set_linewidth(0.8)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("LAYOUT")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+",
                    help="up to %d points files" % MAX_FILES)
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

    data = [load(f) for f in args.files]
    allp = [p for d in data for p in d["xyz"]]
    lo = [min(p[i] for p in allp) for i in range(3)]
    hi = [max(p[i] for p in allp) for i in range(3)]
    fig = plt.figure(figsize=(15, 9), facecolor=SURFACE)
    # Three columns: the 3D view spans two of them (two thirds), and the
    # three orthographic views stack in the third.
    gs = fig.add_gridspec(3, 3, hspace=0.42, wspace=0.28,
                          left=0.04, right=0.965, top=0.88, bottom=0.06)

    # ── 3D, two thirds ──
    ax3 = fig.add_subplot(gs[:, 0:2], projection="3d")
    ax3.set_facecolor(SURFACE)
    for d, c in zip(data, SERIES):
        x = [p[0] for p in d["xyz"]]
        y = [p[1] for p in d["xyz"]]
        z = [p[2] for p in d["xyz"]]
        ax3.plot(x, y, z, color=c, linewidth=1.8)
        ax3.scatter(x, y, z, color=c, s=26, edgecolors=SURFACE,
                    linewidths=1.0, depthshade=False)
    ax3.set_xlabel("X [mm]", color=INK2, fontsize=8)
    ax3.set_ylabel("Y [mm]", color=INK2, fontsize=8)
    ax3.set_zlabel("Z [mm]", color=INK2, fontsize=8, labelpad=8)
    ax3.tick_params(colors=INK2, labelsize=7)
    # At true proportions the short axis is physically small, and the
    # default locator crams its labels into an unreadable smear. Fewer
    # ticks — the scale is unchanged, only how often it is labelled.
    from matplotlib.ticker import MaxNLocator
    for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        axis.set_major_locator(MaxNLocator(4))
    ax3.set_title("3D — Z up   ·   uniform scale on all axes", color=INK,
                  fontsize=11, loc="left", y=0.90)
    ax3.view_init(elev=26, azim=-60)
    # UNIFORM SCALE: the box carries the data's OWN proportions, so a
    # millimetre is the same length on all three axes. (A cube with a
    # common span is equally uniform but leaves most of the volume empty
    # when one axis is short, which is the case for these surfaces.)
    pad = 0.04 * max(max(h - l for h, l in zip(hi, lo)), 1e-6)
    ax3.set_xlim(lo[0] - pad, hi[0] + pad)
    ax3.set_ylim(lo[1] - pad, hi[1] + pad)
    ax3.set_zlim(lo[2] - pad, hi[2] + pad)
    ax3.set_box_aspect([max(h - l + 2 * pad, 1e-6)
                        for h, l in zip(hi, lo)])
    for pane in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        pane.pane.set_facecolor(SURFACE)
        pane.pane.set_edgecolor(AXIS)
        pane.pane.set_alpha(1.0)
        pane._axinfo["grid"]["color"] = GRID
        pane._axinfo["grid"]["linewidth"] = 0.6

    # ── the three orthographic views, sharing the last third ──
    for row, (dims, xl, yl, title) in enumerate(
            (((0, 1), "X [mm]", "Y [mm]", "TOP (XY) — looking down"),
             ((0, 2), "X [mm]", "Z [mm]", "FRONT (XZ) — looking along -Y"),
             ((1, 2), "Y [mm]", "Z [mm]", "SIDE (YZ) — looking along +X"))):
        ax = fig.add_subplot(gs[row, 2])
        for d, c in zip(data, SERIES):
            a = [p[dims[0]] for p in d["xyz"]]
            b = [p[dims[1]] for p in d["xyz"]]
            ax.plot(a, b, color=c, linewidth=1.8, zorder=2,
                    solid_capstyle="round")
            ax.plot(a, b, linestyle="none", marker="o", markersize=4.5,
                    color=c, markeredgecolor=SURFACE, markeredgewidth=1.0,
                    zorder=3)
        style_2d(ax, xl, yl, title)
        equalise_2d(ax)

    fig.suptitle("Cleaning points — %s"
                 % ", ".join(d["label"] for d in data),
                 color=INK, fontsize=13, x=0.04, ha="left", y=0.975)
    handles = [Line2D([], [], color=c, linewidth=2.4, marker="o",
                      markersize=6, markeredgecolor=SURFACE,
                      label="%s  (%d points)" % (d["label"], len(d["xyz"])))
               for d, c in zip(data, SERIES)]
    leg = fig.legend(handles=handles, loc="upper left",
                     bbox_to_anchor=(0.04, 0.945), frameon=False,
                     fontsize=9, ncol=len(data), columnspacing=2.4)
    for txt in leg.get_texts():
        txt.set_color(INK2)

    out = pathlib.Path(args.out) if args.out else \
        pathlib.Path(__file__).resolve().parent / (
            "points_%s.png" % "_".join(pathlib.Path(f).stem
                                       for f in args.files)[:60])
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print("wrote %s" % out)
    for d in data:
        print("  %-28s %3d points, %7.0f mm of path"
              % (d["label"], len(d["xyz"]),
                 sum(math.dist(d["xyz"][i], d["xyz"][i + 1])
                     for i in range(len(d["xyz"]) - 1))))
    print("  extents  X %.0f..%.0f   Y %.0f..%.0f   Z %.0f..%.0f mm"
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
