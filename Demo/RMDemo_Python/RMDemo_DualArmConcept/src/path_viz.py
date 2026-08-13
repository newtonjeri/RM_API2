#!/usr/bin/env python3
"""Extract, visualise and export the Cartesian points of a motion program.

Offline and STANDALONE. Reads a file; talks to no arm, no controller, no
emulator, and imports neither the RealMan SDK nor this repo's task configs.
Point it at any motion program — this one, a future one, one from another
machine — and it produces the same three things:

    a CSV of the points          <name>.csv           (m, rad — API units)
    a CSV of the traversal       <name>_sequence.csv
    a report + a map you can read in a terminal (and, with --plot, a PNG)

WHY STATIC PARSING. The obvious way to read `POSES_MM` out of a motion
program is to import it. In THIS repo that is a hazard, not a convenience:
a program whose module level connects and dispatches would move real metal
the moment an "analysis" tool imported it, and the program this was written
for carries `SIMULATION = False` at the top. So the points are lifted with
`ast.literal_eval` over module-level assignments — nothing in the source is
executed, and a program that cannot be imported (missing SDK, wrong Python)
is read anyway.

WHAT IT LOOKS FOR, in order, unless named with --points / --sequence:
    points    POSES_MM, POSES, POINTS_MM, POINTS, WAYPOINTS_MM, WAYPOINTS
              as {label: [x, y, z, rx, ry, rz]} or [[x, y, z, rx, ry, rz]]
    sequence  SEQUENCE, ORDER, PATH, WAYPOINT_ORDER  as [label, label, ...]
    speeds    SEGMENT_SPEEDS {(from, to): pct} and DEFAULT_SPEED — optional,
              only annotates the segment table
A sequence entry with no matching point (e.g. "startpose", a joint target)
is carried through as a non-Cartesian step rather than dropped or fatal.

WHAT IT REPORTS beyond the coordinates:
    planarity     the spread on the third axis. These paths are usually a
                  raster on one plane; a Z that is not flat is worth seeing.
    duplicates    two labels at the same pose (a stop point aliasing a
                  cleaning point is normal — silently having two names for
                  one place is not).
    orientation   the TRUE rotation between consecutive poses, as the
                  geodesic angle between rotation matrices, alongside the
                  naive per-component Euler delta. Those disagree wildly
                  near +/-pi: rx=+3.024 and rx=-3.046 read as a 350 deg
                  flip component-wise and are 12 deg apart in reality. Only
                  the geodesic number means anything, and orientation
                  change is what a movel chain spends joint speed on
                  (see orientation_cost.py, SPEED_INVESTIGATION.md).

EULER CONVENTION. R = Rz(rz) @ Ry(ry) @ Rx(rx), the convention used
everywhere else in this repo (cleaning_path.R_to_euler and the poses
`rm_movel` accepts). Positions are metres and orientations radians in every
output, because that is what the API itself takes; the terminal tables
print mm for reading.

USAGE
  python3 path_viz.py --source ../paths/test_motion_001.py
  python3 path_viz.py --source ../paths/test_motion_001.py --plot
  python3 path_viz.py --source <prog>.py --name my_path --out ../paths
  python3 path_viz.py --source ../paths/test_motion_001.csv    # re-read
  python3 path_viz.py --source <prog>.py --no-csv              # report only
  python3 path_viz.py --source <prog>.py --tool L_glove_4 --frame World

CAVEAT. The points are as WRITTEN, not as reached. Nothing here does IK, so
a point can be perfectly plotted and still be unreachable, in collision, or
over a joint speed limit on the way in. Screen those with predict_task.py /
orientation_cost.py and a SIM run.
"""

import ast
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:                                  # standalone: usable outside src/ too
    from log_utils import wants_help, setup_log
except ImportError:                   # pragma: no cover - fallback only
    def wants_help(argv=None):
        return any(a in ("-h", "--help")
                   for a in (sys.argv[1:] if argv is None else argv))

    def setup_log(_):
        return None

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(HERE, "paths")

POINT_VARS = ("POSES_MM", "POSES", "POINTS_MM", "POINTS",
              "WAYPOINTS_MM", "WAYPOINTS")
SEQ_VARS = ("SEQUENCE", "ORDER", "PATH", "WAYPOINT_ORDER")
JOINT_VARS = ("START_JOINTS", "HOME_JOINTS", "STARTPOSE_JOINTS")

# A pose is metres for the API but authored in mm by every tool that shows
# these numbers (FRAME_MAP.md, blockly_points.py, the Web GUI). Anything
# past this on any axis cannot be metres: the RM75's reach is ~0.9 m.
MM_THRESHOLD_M = 10.0

# "Same place" for duplicate detection. 1 um is far below anything the
# controller resolves, and 0.01 deg is comfortably above geodesic_deg's
# noise floor near identity (~1e-6 deg) while far below any real difference.
SAME_POSE_M = 1e-6
SAME_POSE_DEG = 0.01

USAGE = """\
Usage: python3 path_viz.py --source FILE [options]
  --source FILE     motion program (.py) or a points CSV this tool wrote
  --name NAME       output basename (default: the source file's stem)
  --out DIR         output directory (default: ../paths)
  --points VAR      variable holding the points (default: auto-detect)
  --sequence VAR    variable holding the traversal (default: auto-detect)
  --units mm|m|auto position units in the source (default: auto)
  --frame NAME      work frame these poses are expressed in (default: World)
  --tool NAME       tool/ik frame the poses command (default: unset)
  --width N         terminal map width in columns (default: 74)
  --plot [FILE]     also write a PNG (needs matplotlib; default <out>/<name>.png)
  --show            open the plot in a window instead of only writing it
  --no-csv          report only; write nothing
  --no-sequence-csv write the points CSV but not the traversal CSV
  -h, --help        show this documentation and exit (writes nothing)

This is an OFFLINE tool: it accepts no --mode flag, by design. Nothing it
does can move an arm, and a run recorded as "--mode REAL" that never
touched hardware is exactly the false evidence these scripts avoid."""


# ─── CLI ────────────────────────────────────────────────────────────────────
# Hand-rolled rather than dual_arm_common.handle_cli: that helper pulls in
# the SDK through dual_arm_common, and this script's whole point is that it
# runs anywhere. The two rules that matter are kept — help prints before
# anything happens, and an unknown argument is an ERROR (exit 2) rather than
# silently ignored, because an ignored typo here means exporting a CSV that
# is not the one you asked for.
def parse_args(argv):
    opts = {"source": None, "name": None, "out": DEFAULT_OUT, "points": None,
            "sequence": None, "units": "auto", "frame": "World", "tool": "",
            "width": 74, "plot": None, "show": False, "csv": True,
            "sequence_csv": True}
    value_flags = {"--source": "source", "--name": "name", "--out": "out",
                   "--points": "points", "--sequence": "sequence",
                   "--units": "units", "--frame": "frame", "--tool": "tool",
                   "--width": "width"}
    i = 0
    while i < len(argv):
        a = argv[i]
        key, val = (a.split("=", 1) + [None])[:2] if "=" in a else (a, None)
        if key in value_flags:
            if val is None:
                i += 1
                if i >= len(argv):
                    raise SystemExit(f"{key} needs a value\n\n{USAGE}")
                val = argv[i]
            opts[value_flags[key]] = val
        elif key == "--plot":
            # optional value: --plot, --plot=FILE, or --plot FILE
            if val is None and i + 1 < len(argv) \
                    and not argv[i + 1].startswith("-"):
                i += 1
                val = argv[i]
            opts["plot"] = val or True
        elif key == "--show":
            opts["show"] = True
            opts["plot"] = opts["plot"] or True
        elif key == "--no-csv":
            opts["csv"] = False
        elif key == "--no-sequence-csv":
            opts["sequence_csv"] = False
        else:
            print(f"unknown argument: {a!r}\n")
            print(USAGE)
            raise SystemExit(2)
        i += 1
    if opts["units"] not in ("mm", "m", "auto"):
        raise SystemExit(f"--units must be mm|m|auto (got {opts['units']!r})")
    opts["width"] = max(24, min(200, int(opts["width"])))
    return opts


# ─── extraction ─────────────────────────────────────────────────────────────
def module_literals(path):
    """{name: value} for every module-level assignment that is a literal.

    Nothing is executed. Non-literal assignments (the mm->m dict
    comprehension in these programs, anything built by a call) are skipped
    silently — they are derived from a literal that IS captured.
    """
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        tree = ast.parse(fh.read(), filename=path)
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            value = node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None \
                and isinstance(node.target, ast.Name):
            names, value = [node.target.id], node.value
        else:
            continue
        if not names:
            continue
        try:
            literal = ast.literal_eval(value)
        except (ValueError, SyntaxError, TypeError, MemoryError):
            continue
        for name in names:
            out[name] = literal
    return out


def _is_pose(v):
    return (isinstance(v, (list, tuple)) and len(v) >= 6
            and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                    for x in v[:6]))


def _as_points(value):
    """[(label, [x,y,z,rx,ry,rz])] from a dict or list of poses, else None."""
    if isinstance(value, dict) and value:
        if all(isinstance(k, str) and _is_pose(v) for k, v in value.items()):
            return [(k, [float(x) for x in v[:6]]) for k, v in value.items()]
        return None
    if isinstance(value, (list, tuple)) and value:
        if all(_is_pose(v) for v in value):
            return [(f"p{i}", [float(x) for x in v[:6]])
                    for i, v in enumerate(value)]
    return None


def find_points(lits, named):
    """(var_name, [(label, pose)]) — the named variable, or the best guess."""
    if named:
        if named not in lits:
            raise SystemExit(f"--points {named!r}: no module-level literal of "
                             f"that name.\nfound: "
                             f"{', '.join(sorted(lits)) or '(nothing)'}")
        pts = _as_points(lits[named])
        if pts is None:
            raise SystemExit(f"--points {named!r} is not a set of 6-value "
                             f"poses (got {type(lits[named]).__name__})")
        return named, pts
    for name in POINT_VARS:
        if name in lits:
            pts = _as_points(lits[name])
            if pts:
                return name, pts
    for name, value in lits.items():      # anything shaped like poses
        pts = _as_points(value)
        if pts and len(pts) >= 2:
            return name, pts
    raise SystemExit("no Cartesian points found. Name the variable "
                     "explicitly with --points VAR.\n"
                     f"module-level literals: {', '.join(sorted(lits))}")


def find_sequence(lits, named, labels):
    """(var_name, [label, ...]) or (None, definition order)."""
    def ok(v):
        return (isinstance(v, (list, tuple)) and len(v) >= 2
                and all(isinstance(x, str) for x in v))
    if named:
        if named not in lits or not ok(lits[named]):
            raise SystemExit(f"--sequence {named!r}: not a module-level list "
                             "of label strings")
        return named, list(lits[named])
    for name in SEQ_VARS:
        if name in lits and ok(lits[name]):
            return name, list(lits[name])
    # Any list of strings that mostly names our points.
    for name, value in lits.items():
        if ok(value) and sum(1 for s in value if s in labels) >= len(value) / 2:
            return name, list(value)
    return None, list(labels)


def resolve_units(unit_opt, var_name, points):
    """('mm'|'m', scale to metres, why) — never guessed silently."""
    if unit_opt != "auto":
        return unit_opt, (0.001 if unit_opt == "mm" else 1.0), "--units"
    if var_name.upper().endswith("_MM"):
        return "mm", 0.001, f"variable name {var_name}"
    biggest = max((abs(c) for _, p in points for c in p[:3]), default=0.0)
    if biggest > MM_THRESHOLD_M:
        return "mm", 0.001, f"largest coordinate {biggest:.1f} > " \
                            f"{MM_THRESHOLD_M:g} (no arm reaches that in m)"
    return "m", 1.0, f"largest coordinate {biggest:.3f} is within arm reach"


def read_points_csv(path):
    """Re-read a points CSV this tool wrote (metres/radians).

    The points CSV holds each point ONCE, so it cannot by itself say that
    the path visits point 4 six times. The companion `_sequence.csv` can,
    and it is written next to it — so use it when it is there, and say
    which of the two the traversal came from rather than quietly
    presenting first-visit order as the path.
    """
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path}: no rows")
    need = ("label", "x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad")
    missing = [c for c in need if c not in rows[0]]
    if missing:
        raise SystemExit(f"{path}: missing column(s) {missing}")
    pts = [(r["label"], [float(r[c]) for c in need[1:]]) for r in rows]
    frame = rows[0].get("work_frame", "") or ""
    tool = rows[0].get("tool_frame", "") or ""

    sibling = f"{os.path.splitext(path)[0]}_sequence.csv"
    if os.path.isfile(sibling):
        with open(sibling, newline="", encoding="utf-8") as fh:
            srows = list(csv.DictReader(fh))
        if srows and "label" in srows[0]:
            seq = [r["label"] for r in sorted(
                srows, key=lambda r: int(r.get("step") or 0))]
            return pts, seq, frame, tool, os.path.basename(sibling)
    seq = [lbl for lbl, _ in sorted(
        ((r["label"], int(r.get("first_step") or i))
         for i, r in enumerate(rows)), key=lambda t: t[1])]
    return pts, seq, frame, tool, "first_step order (no _sequence.csv " \
                                  "beside it — revisits are NOT recovered)"


# ─── rotations (pure stdlib; convention R = Rz @ Ry @ Rx) ───────────────────
def rot(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                              math.sin(ry), math.cos(rz), math.sin(rz))
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]


def geodesic_deg(a, b):
    """Smallest rotation angle carrying orientation a onto b, in degrees.

    Beware the tail: acos is infinitely steep at 1, so near-identical
    orientations come back with only half the available precision — two
    poses that are bit-for-bit equal measure ~1e-6 deg apart, not 0. Never
    test this against an exact zero (SAME_POSE_DEG exists for that).
    """
    Ra, Rb = rot(*a[3:6]), rot(*b[3:6])
    trace = sum(sum(Ra[k][i] * Rb[k][j] for k in range(3))
                for i, j in ((0, 0), (1, 1), (2, 2)))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def euler_delta_deg(a, b):
    """Naive per-component Euler difference — the misleading number."""
    return math.degrees(max(abs(a[3 + k] - b[3 + k]) for k in range(3)))


def tool_axis(pose):
    """The tool frame's own +Z expressed in the work frame (approach dir)."""
    R = rot(*pose[3:6])
    return [R[0][2], R[1][2], R[2][2]]


def dist(a, b):
    return math.dist(a[:3], b[:3])


# ─── terminal map ───────────────────────────────────────────────────────────
AXES = ("X", "Y", "Z")
MARKERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"


def choose_plane(points):
    """(h, v, n, spans) — the two widest axes, then the flattest."""
    spans = [max(p[a] for _, p in points) - min(p[a] for _, p in points)
             for a in range(3)]
    order = sorted(range(3), key=lambda a: -spans[a])
    h, v = sorted(order[:2])          # keep natural axis order (X before Y)
    return h, v, order[2], spans


def assign_markers(points):
    used, markers = set(), {}
    for label, _ in points:
        ch = label[0] if len(label) == 1 and label[0].isalnum() else None
        if ch is None or ch in used:
            ch = next((m for m in MARKERS if m not in used), "?")
        used.add(ch)
        markers[label] = ch
    return markers


def render_map(points, segments, markers, width):
    """The path drawn on a character grid. Returns a list of lines."""
    h, v, n, spans = choose_plane(points)
    lo = [min(p[a] for _, p in points) for a in range(3)]
    hi = [max(p[a] for _, p in points) for a in range(3)]
    if spans[h] <= 0 and spans[v] <= 0:
        return ["(all points coincide — nothing to draw)"]

    W = width
    # Terminal cells are about twice as tall as wide, so the vertical scale
    # is halved to keep the drawing's proportions honest.
    aspect = (spans[v] / spans[h]) if spans[h] > 0 else 1.0
    H = max(5, min(34, int(round(W * aspect * 0.5)) or 5))

    def cell(p):
        c = 0 if spans[h] <= 0 else \
            int(round((p[h] - lo[h]) / spans[h] * (W - 1)))
        r = 0 if spans[v] <= 0 else \
            int(round((p[v] - lo[v]) / spans[v] * (H - 1)))
        return (H - 1) - r, c          # screen row 0 is the TOP

    grid = [[" "] * W for _ in range(H)]

    for a, b in segments:              # segments first, points stamp over
        r1, c1 = cell(a)
        r2, c2 = cell(b)
        dr, dc = r2 - r1, c2 - c1
        glyph = ("-" if dr == 0 else "|" if dc == 0
                 else "/" if dr * dc < 0 else "\\")
        steps = max(abs(dr), abs(dc))
        for s in range(steps + 1):
            r = r1 + int(round(dr * s / steps))
            c = c1 + int(round(dc * s / steps))
            cur = grid[r][c]
            grid[r][c] = glyph if cur == " " else \
                (cur if cur == glyph else "+")

    occupants = {}
    for label, p in points:
        r, c = cell(p)
        occupants.setdefault((r, c), []).append(label)
        grid[r][c] = markers[label]
    collisions = [v for v in occupants.values() if len(v) > 1]

    scale = 1000.0                     # the map is labelled in mm
    body = ["    +" + "-" * W + "+"]
    body += [f"    |{''.join(row)}|" for row in grid]
    body += ["    +" + "-" * W + "+"]
    head = (f"    {AXES[v]} {hi[v]*scale:+.1f} mm at the top, "
            f"{lo[v]*scale:+.1f} mm at the bottom")
    foot = (f"    {AXES[h]} {lo[h]*scale:+.1f} mm at the left, "
            f"{hi[h]*scale:+.1f} mm at the right"
            f"   |   {AXES[n]} spread {spans[n]*scale:.1f} mm")
    lines = [head] + body + [foot]
    for group in collisions:
        # Only the LAST label of a group is the visible marker; say so
        # rather than let a point look absent from its own map.
        lines.append(f"    [NOTE] {' and '.join(group)} share one cell — "
                     f"only {markers[group[-1]]} is drawn "
                     "(identical points, or raise --width)")
    return lines


# ─── analysis ───────────────────────────────────────────────────────────────
def analyse(points, sequence, speeds, default_speed):
    """Everything derived: visits, duplicates, per-segment geometry."""
    pose_of = dict(points)
    visits, first = {}, {}
    for step, label in enumerate(sequence):
        visits[label] = visits.get(label, 0) + 1
        first.setdefault(label, step)

    duplicates = []
    for i, (la, pa) in enumerate(points):
        for lb, pb in points[i + 1:]:
            if dist(pa, pb) < SAME_POSE_M \
                    and geodesic_deg(pa, pb) < SAME_POSE_DEG:
                duplicates.append((la, lb))

    steps, cum, prev_label = [], 0.0, None
    for step, label in enumerate(sequence):
        pose = pose_of.get(label)
        rec = {"step": step, "label": label, "pose": pose,
               "cartesian": pose is not None,
               "move": "MOVEL" if pose is not None else "MOVEJ/other",
               "speed": None, "len": None, "rot": None, "euler": None}
        # The program looks up speeds by (previous label, this label); its
        # startpose leaves `current_label` as "start", so honour that alias.
        key = ("start" if prev_label in (None, "startpose") else prev_label,
               label)
        if rec["cartesian"]:
            rec["speed"] = speeds.get(key, default_speed) if speeds \
                else default_speed
        if pose is not None and prev_label is not None \
                and prev_label in pose_of:
            a = pose_of[prev_label]
            rec["len"] = dist(a, pose)
            rec["rot"] = geodesic_deg(a, pose)
            rec["euler"] = euler_delta_deg(a, pose)
            cum += rec["len"]
        rec["cum"] = cum
        steps.append(rec)
        prev_label = label

    segments = []
    seen = set()
    for a, b in zip(steps, steps[1:]):
        if a["cartesian"] and b["cartesian"]:
            key = tuple(sorted((a["label"], b["label"])))
            if key not in seen:
                seen.add(key)
                segments.append((pose_of[a["label"]], pose_of[b["label"]]))
    return {"visits": visits, "first": first, "duplicates": duplicates,
            "steps": steps, "segments": segments, "total_len": cum}


# ─── writers ────────────────────────────────────────────────────────────────
def write_points_csv(path, points, an, frame, tool):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["index", "label", "x_m", "y_m", "z_m",
                    "rx_rad", "ry_rad", "rz_rad",
                    "visits", "first_step", "work_frame", "tool_frame"])
        for i, (label, p) in enumerate(points):
            w.writerow([i, label] + [f"{v:.6f}" for v in p[:3]]
                       + [f"{v:.6f}" for v in p[3:6]]
                       + [an["visits"].get(label, 0),
                          an["first"].get(label, ""), frame, tool])
    return path


def write_sequence_csv(path, an, frame, tool):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["step", "label", "move", "speed_pct",
                    "x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad",
                    "seg_len_m", "seg_rot_deg", "cum_len_m",
                    "work_frame", "tool_frame"])
        for s in an["steps"]:
            p = s["pose"]
            w.writerow(
                [s["step"], s["label"], s["move"],
                 "" if s["speed"] is None else s["speed"]]
                + ([f"{v:.6f}" for v in p[:6]] if p else [""] * 6)
                + ["" if s["len"] is None else f"{s['len']:.6f}",
                   "" if s["rot"] is None else f"{s['rot']:.3f}",
                   f"{s['cum']:.6f}", frame, tool])
    return path


def write_plot(path, points, an, markers, frame, tool, name, show):
    """3D view + the flat projection. matplotlib only, imported on demand."""
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        # No `from mpl_toolkits.mplot3d import Axes3D` — matplotlib has
        # registered the '3d' projection itself since 3.2, and the import
        # only survives in older examples as a side effect.
    except ImportError as exc:
        print(f"  [SKIP] --plot needs matplotlib ({exc}); "
              "the CSV and the terminal map are unaffected")
        return None

    xs = [p[0] * 1000 for _, p in points]
    ys = [p[1] * 1000 for _, p in points]
    zs = [p[2] * 1000 for _, p in points]
    ranges = [max(a) - min(a) for a in (xs, ys, zs)]
    span = max(max(ranges), 1.0)
    arrow = span * 0.14
    order = [s for s in an["steps"] if s["cartesian"]]

    def offsets(pairs):
        """Label offsets that step apart when two points coincide."""
        seen, out = {}, []
        for key in pairs:
            k = tuple(round(c, 6) for c in key)
            out.append(seen.get(k, 0))
            seen[k] = seen.get(k, 0) + 1
        return out

    fig = plt.figure(figsize=(19, 6.0))
    # Explicit gridspec, NOT tight_layout: mplot3d draws its axis labels
    # outside the subplot rectangle and tight_layout does not know it, so
    # the 3D panel's "Z (mm)" lands on top of the next panel's y-label.
    # Reserve the gap instead of discovering it afterwards.
    gs = fig.add_gridspec(1, 3, width_ratios=(1.05, 1.45, 1.1), wspace=0.34,
                          left=0.035, right=0.965, top=0.86, bottom=0.11)

    # ── 1. the path in space, with the tool's approach direction ──
    ax = fig.add_subplot(gs[0, 0], projection="3d")
    ax.plot([s["pose"][0] * 1000 for s in order],
            [s["pose"][1] * 1000 for s in order],
            [s["pose"][2] * 1000 for s in order],
            "-", lw=1.0, alpha=0.55, color="tab:blue")
    ax.scatter(xs, ys, zs, s=40, color="tab:red", depthshade=False)
    for (label, p), x, y, z, k in zip(points, xs, ys, zs,
                                      offsets([p[:3] for _, p in points])):
        ax.text(x, y, z + span * (0.04 + 0.05 * k), label, fontsize=8)
        ux, uy, uz = tool_axis(p)
        ax.quiver(x, y, z, ux * arrow, uy * arrow, uz * arrow,
                  color="tab:green", lw=1.2, arrow_length_ratio=0.25)
    ax.set_xlabel("X (mm)", labelpad=2)
    ax.set_ylabel("Y (mm)", labelpad=2)
    ax.set_zlabel("Z (mm)", labelpad=2)
    ax.tick_params(labelsize=7, pad=1)
    ax.set_title(f"{name} — {len(points)} points, {len(order)} moves\n"
                 "green = tool +Z (approach direction)", fontsize=10)
    try:
        # A planar path has one range of zero; a true-proportion box would
        # collapse to a line and take its axis labels with it. Floor each
        # axis at 15 % of the widest so a flat path draws as a slab and
        # still reads as flat.
        ax.set_box_aspect(tuple(max(r, 0.15 * span) for r in ranges))
    except Exception:
        pass

    # ── 2. the flat projection: the shape, to scale ──
    h, v, n, spans = choose_plane(points)
    ax2 = fig.add_subplot(gs[0, 1])
    fh = [s["pose"][h] * 1000 for s in order]
    fv = [s["pose"][v] * 1000 for s in order]
    for i in range(len(order) - 1):    # direction of travel
        ax2.annotate("", xy=(fh[i + 1], fv[i + 1]), xytext=(fh[i], fv[i]),
                     arrowprops=dict(arrowstyle="->", color="tab:blue",
                                     alpha=0.45, lw=0.9))
    ax2.scatter([p[h] * 1000 for _, p in points],
                [p[v] * 1000 for _, p in points],
                s=50, color="tab:red", zorder=5)
    for (label, p), k in zip(points, offsets([(p[h], p[v]) for _, p in points])):
        ax2.annotate(label, (p[h] * 1000, p[v] * 1000),
                     textcoords="offset points", xytext=(7, 6 - 11 * k),
                     fontsize=9)
    ax2.set_xlabel(f"{AXES[h]} (mm)")
    ax2.set_ylabel(f"{AXES[v]} (mm)")
    ax2.set_title(f"projection on {AXES[h]}{AXES[v]}, to scale — "
                  f"{AXES[n]} spread {spans[n]*1000:.1f} mm\n"
                  f"frame {frame or '?'}"
                  + (f", tool {tool}" if tool else ""), fontsize=10)
    ax2.grid(alpha=0.3)
    # 'box', not 'datalim': shrink the AXES to the data's true proportions
    # instead of padding the data out to fill a wide box.
    ax2.set_aspect("equal", adjustable="box")

    # ── 3. what each step costs: distance, and rotation ──
    ax3 = fig.add_subplot(gs[0, 2])
    steps = [s["step"] for s in an["steps"] if s["len"] is not None]
    lens = [s["len"] * 1000 for s in an["steps"] if s["len"] is not None]
    rots = [s["rot"] for s in an["steps"] if s["rot"] is not None]
    ax3.bar(steps, lens, color="tab:blue", alpha=0.55, label="distance (mm)")
    ax3.set_xlabel("sequence step")
    ax3.set_ylabel("segment distance (mm)", color="tab:blue")
    ax3.tick_params(axis="y", labelcolor="tab:blue")
    ax4 = ax3.twinx()
    ax4.plot(steps, rots, "o-", ms=3, lw=1.2, color="tab:orange",
             label="rotation (deg)")
    ax4.set_ylabel("orientation change (deg)", color="tab:orange")
    ax4.tick_params(axis="y", labelcolor="tab:orange")
    ax4.set_ylim(0, max(max(rots, default=1.0) * 1.3, 1.0))
    ax3.set_title("cost per segment — distance vs orientation change\n"
                  f"total {an['total_len']*1000:.0f} mm, "
                  f"max rotation {max(rots, default=0.0):.1f} deg",
                  fontsize=10)
    ax3.grid(alpha=0.25, axis="y")

    # Pull the 3D panel in from its cell: what overflows is proportional to
    # the box, so a narrower box keeps the labels inside the reserved gap.
    box = ax.get_position()
    ax.set_position([box.x0, box.y0 + box.height * 0.04,
                     box.width * 0.86, box.height * 0.92])
    fig.savefig(path, dpi=140)
    if show:
        plt.show()
    plt.close(fig)
    return path


# ─── report ─────────────────────────────────────────────────────────────────
def report(opts, src, points, seq_var, sequence, an, markers, unit, why,
           joints, lits):
    frame, tool = opts["frame"], opts["tool"]
    print("=" * 78)
    print(f"PATH — {opts['name']}")
    print("=" * 78)
    print(f"  source        {src}")
    print(f"  points        {len(points)} unique, from {opts['_points_var']}"
          f"   (units {unit}: {why})")
    print(f"  sequence      {len(sequence)} steps, from "
          f"{seq_var or 'definition order (no sequence variable found)'}")
    print(f"  work frame    {frame or '(unset)'}")
    print(f"  tool frame    {tool or '(unset — pass --tool to record it)'}")
    for extra in ("ARM_IP", "BLEND", "DEFAULT_SPEED", "SIMULATION"):
        if extra in lits:
            print(f"  {extra.lower():13s} {lits[extra]}")
    print()

    print(f"POINTS — positions mm, orientation rad, in the {frame or '?'} "
          f"frame" + (f" for {tool}" if tool else ""))
    print(f"  {'':2s} {'label':10s} {'X':>10s} {'Y':>10s} {'Z':>10s} "
          f"{'RX':>8s} {'RY':>8s} {'RZ':>8s} {'visits':>7s} {'first':>6s}")
    for label, p in points:
        print(f"  {markers[label]:2s} {label:10s} "
              f"{p[0]*1000:10.3f} {p[1]*1000:10.3f} {p[2]*1000:10.3f} "
              f"{p[3]:8.4f} {p[4]:8.4f} {p[5]:8.4f} "
              f"{an['visits'].get(label, 0):7d} "
              f"{an['first'].get(label, '-'):>6}")
    print()

    lo = [min(p[a] for _, p in points) for a in range(3)]
    hi = [max(p[a] for _, p in points) for a in range(3)]
    print("EXTENT (mm)")
    for a in range(3):
        uniq = sorted({round(p[a] * 1000, 3) for _, p in points})
        flat = "  <-- constant" if len(uniq) == 1 else \
               (f"  {len(uniq)} distinct: "
                + ", ".join(f"{u:g}" for u in uniq) if len(uniq) <= 6 else "")
        print(f"  {AXES[a]}  {lo[a]*1000:10.3f} .. {hi[a]*1000:10.3f}"
              f"   span {(hi[a]-lo[a])*1000:8.3f}{flat}")
    # Count DISTINCT positions, not points: an alias like a stop pose parked
    # on a cleaning point would otherwise hide the grid it belongs to.
    places = {tuple(round(c * 1000, 3) for c in p[:3]) for _, p in points}
    grid = [len({round(p[a] * 1000, 3) for _, p in points}) for a in range(3)]
    if grid[0] * grid[1] * grid[2] == len(places) and min(grid[:2]) > 1:
        print(f"  the {len(places)} distinct positions form a full "
              f"{grid[0]}x{grid[1]}x{grid[2]} grid")
    print()

    if joints:
        name, vals = joints
        print(f"JOINT TARGET — {name} (degrees), not a Cartesian point")
        print("  " + "  ".join(f"J{i+1} {v:8.3f}"
                               for i, v in enumerate(vals)))
        print("  not plotted: turning joints into a pose needs FK with the "
              "tool frame,\n  which would make this script depend on the "
              "SDK. Its Cartesian\n  neighbours are the first and last "
              "points below.")
        print()

    if an["duplicates"]:
        print("DUPLICATE POSES — different labels, identical pose")
        for a, b in an["duplicates"]:
            print(f"  {a} == {b}")
        print()

    unref = [l for l, _ in points if an["visits"].get(l, 0) == 0]
    missing = sorted({s["label"] for s in an["steps"] if not s["cartesian"]})
    if unref:
        print(f"DEFINED BUT NEVER VISITED: {', '.join(unref)}\n")
    if missing:
        print(f"SEQUENCE STEPS WITH NO CARTESIAN POINT: {', '.join(missing)}")
        print("  carried through as non-Cartesian steps (a joint move, a "
              "hand step, ...)\n")

    print("MAP — the traversal, drawn on the two widest axes")
    for line in render_map(points, an["segments"], markers, opts["width"]):
        print(line)
    print("    legend: " + "  ".join(f"{m}={l}" for l, m in markers.items()))
    print()

    print("SEGMENTS — geometry per step of the traversal")
    print(f"  {'step':>4s} {'move':12s} {'from':>9s} -> {'to':<9s} "
          f"{'speed':>5s} {'len mm':>9s} {'rot deg':>8s} {'euler':>8s} "
          f"{'cum mm':>9s}")
    prev = None
    wrapped = 0
    for s in an["steps"]:
        rot_s = f"{'-':>8}" if s["rot"] is None else f"{s['rot']:8.2f}"
        eul_s = f"{'-':>8}" if s["euler"] is None else f"{s['euler']:8.2f}"
        len_s = f"{'-':>9}" if s["len"] is None else f"{s['len']*1000:9.3f}"
        flag = ""
        if s["rot"] is not None and s["euler"] - s["rot"] > 90:
            flag = "  <-- Euler wrap"
            wrapped += 1
        print(f"  {s['step']:4d} {s['move']:12s} {str(prev or '-'):>9s} -> "
              f"{s['label']:<9s} {str(s['speed'] or '-'):>5s} {len_s} "
              f"{rot_s} {eul_s} {s['cum']*1000:9.3f}{flag}")
        prev = s["label"]
    print()

    rots = [s["rot"] for s in an["steps"] if s["rot"] is not None]
    pair = max(((geodesic_deg(a[1], b[1]), a[0], b[0])
                for i, a in enumerate(points) for b in points[i + 1:]),
               default=(0.0, "-", "-"))
    print("TOTALS")
    print(f"  path length            {an['total_len']*1000:.1f} mm "
          f"({an['total_len']:.4f} m) over "
          f"{sum(1 for s in an['steps'] if s['len'] is not None)} segments")
    if rots:
        print(f"  rotation per segment   max {max(rots):.2f} deg, "
              f"mean {sum(rots)/len(rots):.2f} deg")
    print(f"  widest orientation     {pair[0]:.2f} deg, between "
          f"{pair[1]} and {pair[2]}")
    if wrapped:
        print(f"  Euler wraps            {wrapped} segment(s) where the "
              "component-wise delta exceeds")
        print("                         the true rotation by >90 deg — the "
              "poses straddle +/-pi.")
        print("                         Harmless as data; only the 'rot deg' "
              "column is real.")
    print()


# ─── main ───────────────────────────────────────────────────────────────────
def main() -> int:
    if wants_help():
        print(__doc__.strip())
        print()
        print(USAGE)
        return 0
    opts = parse_args(sys.argv[1:])
    if not opts["source"]:
        print("--source is required\n")
        print(USAGE)
        return 2
    src = os.path.abspath(opts["source"])
    if not os.path.isfile(src):
        print(f"no such file: {src}")
        return 1
    opts["name"] = opts["name"] or os.path.splitext(os.path.basename(src))[0]

    lits, joints, speeds, default_speed = {}, None, {}, None
    if src.lower().endswith(".csv"):
        points, sequence, frame, tool, seq_var = read_points_csv(src)
        unit, why = "m", "CSV columns are metres by construction"
        opts["_points_var"] = "the CSV"
        opts["frame"] = opts["frame"] if opts["frame"] != "World" else frame
        opts["tool"] = opts["tool"] or tool
    else:
        lits = module_literals(src)
        pvar, raw = find_points(lits, opts["points"])
        opts["_points_var"] = pvar
        unit, scale, why = resolve_units(opts["units"], pvar, raw)
        points = [(l, [p[0] * scale, p[1] * scale, p[2] * scale,
                       p[3], p[4], p[5]]) for l, p in raw]
        seq_var, sequence = find_sequence(lits, opts["sequence"],
                                          [l for l, _ in points])
        for name in JOINT_VARS:
            v = lits.get(name)
            if isinstance(v, (list, tuple)) and 6 <= len(v) <= 7 \
                    and all(isinstance(x, (int, float)) for x in v):
                joints = (name, [float(x) for x in v])
                break
        raw_speeds = lits.get("SEGMENT_SPEEDS")
        if isinstance(raw_speeds, dict):
            speeds = {k: v for k, v in raw_speeds.items()
                      if isinstance(k, tuple) and len(k) == 2}
        default_speed = lits.get("DEFAULT_SPEED")

    an = analyse(points, sequence, speeds, default_speed)
    markers = assign_markers(points)
    report(opts, src, points, seq_var, sequence, an, markers, unit, why,
           joints, lits)

    written = []
    if opts["csv"]:
        os.makedirs(opts["out"], exist_ok=True)
        written.append(write_points_csv(
            os.path.join(opts["out"], f"{opts['name']}.csv"),
            points, an, opts["frame"], opts["tool"]))
        if opts["sequence_csv"]:
            written.append(write_sequence_csv(
                os.path.join(opts["out"], f"{opts['name']}_sequence.csv"),
                an, opts["frame"], opts["tool"]))
    if opts["plot"]:
        png = opts["plot"] if isinstance(opts["plot"], str) else \
            os.path.join(opts["out"], f"{opts['name']}.png")
        os.makedirs(os.path.dirname(os.path.abspath(png)) or ".",
                    exist_ok=True)
        got = write_plot(png, points, an, markers, opts["frame"],
                         opts["tool"], opts["name"], opts["show"])
        if got:
            written.append(got)
    if written:
        print("WROTE")
        for path in written:
            print(f"  {path}")
    else:
        print("(nothing written — --no-csv)")
    return 0


if __name__ == "__main__":
    setup_log(__file__)
    raise SystemExit(main())
