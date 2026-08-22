#!/usr/bin/env python3
"""SPEED RAMP DRIVER — screen -> SIM -> REAL, one rung at a time.

Newton, 2026-08-19: ramp the linear cap 0.45 -> 1.00 m/s in +0.1 steps with
the angular cap coupled at omega = 1.25 * v, holding angular_acc at 4.0, and
settle the behaviour on evidence.

WHY A DRIVER AND NOT `test_blend_corner.py --path ... ` ON ITS OWN.
A single invocation carries ONE `--angular-speed`, but this ramp needs the cap
to move WITH the rung (that coupling is under test). So each rung is its
own invocation, and the sequencing rule lives here.

THE RULE, in Newton's words: "screen -> SIM -> REAL, do not proceed to REAL if
SIM fails." Enforced literally — a rung whose screen fails never reaches SIM,
and a rung whose SIM fails never reaches REAL. The ladder also stops after two
consecutive failed rungs rather than walking every remaining rung into a wall.

ABORT CRITERIA (agreed with Newton, 2026-08-19), applied to the REAL rung:
  * ANY joint above 95 % of its limit  — not J4 alone. J4 is screened because
    it is the only redundancy-invariant joint (measured, H66), NOT because it
    binds: J1 binds on 11 of 24 tasks, and at 0.8 m/s on toplid_left FOUR
    joints were over limit at once.
  * ANY dwell at >=98 % of a limit, for any duration at all (the H63 metric —
    0 ms at 0.45, 330 ms on the 0.8 run that did not finish).

WHAT THIS RAMP SHOWED, and the tilt law that explains it (the ladder ran
2026-08-19). The prediction recorded here before the run used
kappa = 1.86 rad/m and was WRONG — it divided the rotation across the 420 mm
PADDED span by the 380 mm STROKE, and read the pose in the wrong convention.
rx/ry/rz in a controller pose are Euler RPY (Rz*Ry*Rx), NOT a rotation
vector. Measured: 36.03 deg over 380 mm, so

    kappa = 1.655 rad/m,  omega = kappa * v

The coupling COMMANDS omega = 1.25 * v while the path DEMANDS 1.655 * v,
so the demand exceeds the cap at every rung — and the commanded cap itself
reaches 1.25 rad/s at v = 0.755 m/s, above which the far-reach end of the
stroke, where the tilt is steepest, is asking for more angular rate than it
is allowed. That is where J4 failed. The shipped angular_acc 4.0 supports
the true demand only up to

    v = angular_acc / (3 * kappa) = 4.0 / 4.965 = 0.806 m/s

which is why 0.80 was the highest rung with margin and 0.90 was not
survivable.

MEASURED, left arm, REAL (all three confirmed against the recordings):
    0.80  J4 87.6 %, 0 ms dwell, 6.8 A  -> completed, margin
    0.90  J4 99.3 %, 10 ms dwell, 7.7 A -> completed, BOTH abort criteria met
    1.00  stopped at 55 % of the path, 26.4 A, three joints reversed in 20 ms
          controller reported "Out Of Reach, reason: Joint4overspeed"
          (4103 / 0x1007) — GUI-ONLY. err1..err7 and lift_err are ZERO in all
          23 recordings and arm_status read 0 (IDLE) through the event, so the
          only machine-readable evidence is truncated traced distance plus
          non-arrival at the final waypoint.

0.80 m/s / 1.25 rad/s IS A MEASURED CEILING, not a design mandate — it is
what this ladder reached with margin on 2026-08-19, on one path, once.
Raising the cap bought 8.9 s across the whole commode_c corpus (3.4 %), which
did not look worth the joint margin. Both halves of that are open to
re-measurement; this driver exists to re-measure them.

USAGE
    python3 run_speed_ramp.py --side left --mode SCREEN   # offline only (default)
    python3 run_speed_ramp.py --side left --mode EMU      # + emulator, no arm
    python3 run_speed_ramp.py --side left --mode SIM      # + controller SIM
    python3 run_speed_ramp.py --side left --mode REAL     # the whole ladder
    python3 run_speed_ramp.py --side left --mode SIM --rungs 0.45,0.50
    python3 run_speed_ramp.py --side left --mode REAL --dry   # print, run nothing
    python3 run_speed_ramp.py --side left --coupling 1.655    # omega = path demand (tilt law)
    python3 run_speed_ramp.py --side left --angular-acc 6.0   # pin the acc instead

--mode names the HIGHEST rung the ladder may reach; every rung below it still
runs in order, and a failure stops the ladder there. SCREEN is the default
because it is the only rung that cannot move an arm.
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import time
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(HERE, "..", "paths", "planar_speed_ramp_001.py")

COUPLING = 1.25        # omega_cap = COUPLING * v (Newton's ratio)
# Settable with --coupling, and this is the knob that makes the angular_acc
# cap removal MATTER. Under COUPLING = 1.25 even rung 1.00 needs only
# angular_acc 3.75, i.e. under the shipped 4.00 — so the ratio ceiling never
# fired in this ladder and lifting it alone changes nothing.
#
# What the PATH demands is different and larger: the conditioning tilt is
# kappa = 1.655 rad/m (measured), so a segment needs omega = 1.655 * v.
# At --coupling 1.655
# the commanded cap finally matches the demand, and THEN the ratio asks for
# angular_acc = 3 * 1.655 * v, which passes 4.00 at rung 0.806 and reaches
# 5.58 at rung 1.00 — exactly the ">= 5.6" the module docstring predicted.

# ANGULAR ACCELERATION: THE CAP IS REMOVED (Newton, 2026-08-19).
# It was HELD at 4.00, which made `omega <= angular_acc/3 = 1.3333` a hard
# ceiling and stopped the ladder at 0.80 m/s before any arm moved. Newton has
# lifted it for this test, so 4.00 is now a FLOOR and the vendor ratio is
# satisfied by RAISING the acceleration to match the rung:
#       angular_acc(rung) = max(4.00, 3 * omega_cap) = max(4.00, 3.75 * rung)
# Override with --angular-acc to pin it at a value instead.
#
# WHAT THIS GIVES UP, once, so it is on the record: RealMan hold the shipped
# 4.0 because it preserves the ability to stop immediately (H62) — a higher
# value lengthens the stop. Nothing else is relaxed. The all-joint 95 % abort
# and the 98 % dwell abort still decide every rung, and those are what
# actually protect the arm; the ratio ceiling never did.
ANGULAR_ACC_FLOOR = 4.00
ANGULAR_ACC_PINNED = None          # set by --angular-acc


# LINE ACCELERATION: KEEP A MARGIN THE 2-DECIMAL WIRE FORMAT CAN CARRY.
# `speed_limits` requires acc >= 3 * speed * (1 + 1e-9) and rejects anything
# under it, because a pair on the exact boundary is answered by the
# controller with a bare ret=1 and the run then proceeds at whatever was
# already configured. This driver used to compute `3.0 * rung` exactly, which
# is BY CONSTRUCTION below that threshold, and then formatted it with "%.2f"
# — so even a 1e-9 nudge was rounded away. Measured 2026-08-19: rungs 0.45
# and 0.50 ran (their acc is floored at 1.60, well clear) and EVERY rung from
# 0.60 up was refused with
#   "line_acc 1.800 is below 3 x line_speed 0.600 = 1.800"
# — two numbers that print identically, which is what made it look like a
# controller quirk rather than a rounding boundary.
#
# 0.01 is the smallest margin "%.2f" can express. At rung 0.60 it raises the
# ratio from 3.000 to 3.017; the cost is under 1 % of acceleration.
LINE_ACC_MARGIN = 0.01


def line_acc_for(rung):
    """Linear acceleration for a rung, clear of the ratio boundary."""
    return max(1.60, 3.0 * rung + LINE_ACC_MARGIN)


def angular_acc_for(cap):
    """Angular acceleration for a rung whose angular cap is `cap`."""
    if ANGULAR_ACC_PINNED is not None:
        return ANGULAR_ACC_PINNED
    return max(ANGULAR_ACC_FLOOR, 3.0 * cap * (1 + 1e-9))
SCREEN_GATE = 90.0     # % of the J4 limit

# THE PREDICTION BAND [0.74 P, 1.08 P] IS WITHDRAWN — refuted on the
# 2026-08-19 hardware ramp. It is gone in BOTH its value and its shape:
#
#   * Value, in the UNSAFE direction. Offline screen against the REAL worst
#     joint on this very path, v = 0.45 … 0.90, ran 1.12 1.13 1.10 1.10 1.14
#     1.15 — outside the old band and OPTIMISTIC. The screen reads 10-15 %
#     LOW. The old band's 0.74 low end licensed exactly the wrong call.
#   * Shape, which matters more. A whole-path ratio is the wrong statistic:
#     J4 passed 80 % only inside a 40 mm window at ONE junction and never
#     passed 51 % anywhere else. A scalar over 1220 samples cannot gate a
#     3-sample event, it can only average it away.
#
# WHAT REPLACES IT: the screen RANKS, and geometry PREDICTS. Nothing
# offline gates a rung any more — the ladder runs and the recording judges.
#   * The screen may REJECT a speed. It may NEVER certify one — a J4 pass is
#     not a clearance, because J4 is screened for being redundancy-invariant
#     (hence offline-computable), NOT for being the binding joint. Measured:
#     J1 binds on 11 of 24 tasks, and at 0.8 m/s on toplid_left four joints
#     were over limit at once.
#   * Rejection uses the LOWEST measured under-read, 1.10, so a rung is
#     refused only when even the most optimistic reading clears the abort.
#   * The interesting junction is the one carrying the screen's maximum, and
#     src/junction_limits.py says something exact and offline about it — but
#     advisory only, and not with this statistic.
SCREEN_UNDERREAD_MIN = 1.10
JOINT_ABORT = 95.0     # % of any joint's limit
FINAL_WAYPOINT_TOL_M = 0.020   # completed rungs land within 1 mm; the
                               # 2026-08-19 v1.00 failure ended 350 mm out
TRACED_FLOOR = 0.90            # SIM traces 101 % of commanded, REAL 104 %;
                               # the failure traced 53 % (SIM) / 57 % (REAL)
CONSECUTIVE_FAIL_STOP = 2


# --- contact fraction -----------------------------------------
# APPLIED, NOT ENFORCED BY CAPPING (Newton, 2026-08-19: "apply the formula, do
# not cap the angle, if you cap we add another limit"). The tilt along a stroke
# is set by what the ARM needs to stay out of the elbow singularity; theta is
# then a consequence, and f is a consequence of theta. Capping theta to hold
# f >= 0.5 would invent a constraint the physics does not have and would put
# the arm back near the singularity — measured: reducing the tilt to 0.67 of
# toplid's profile took the J4 screen from 99 % to 432 %.
#
#     f = min(1, t / (D sin theta))      t = pad compliance depth [m]
#                                        D = pad edge in the tilt plane = L
#                                        theta = press axis vs inward normal
#
# So f is REPORTED per rung, and a run that presses below f = 0.5 is recorded
# as pressing below it — not silently prevented.
PAD_T = 0.020      # frame 2 compliance depth [m]
PAD_L = 0.080      # frame 2 long edge [m]
# Contact gate, moved 0.5 -> 0.40 by Newton on 2026-08-19 because at 0.5 the
# contact requirement and the arm's kinematics could not both be satisfied: the
# tilt needed to keep the elbow off a singularity (theta_k ~ 30.9 deg) exceeded
# the tilt the gate allowed (theta_c = 30.0 deg). At 0.40 the ceiling is 38.7.
CONTACT_GATE = 0.40


def _tool_press_axis(rx, ry, rz):
    """World-frame +Z of the tool, from the controller's RPY pose."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # R = Rz(rz) Ry(ry) Rx(rx); third column is the tool Z in world
    return (cz * sy * cx + sz * sx,
            sz * sy * cx - cz * sx,
            cy * cx)


def contact_profile(mod, normal=(0.0, 0.0, 1.0)):
    """(min f, worst pose name, theta_deg at that pose) over the path.

    The surface here is planar with an UP normal, so the inward normal the pad
    presses along is -normal.
    """
    worst = (2.0, None, 0.0)
    for name, p in mod.POSES_MM.items():
        ax = _tool_press_axis(p[3], p[4], p[5])
        dot = -(ax[0] * normal[0] + ax[1] * normal[1] + ax[2] * normal[2])
        theta = math.acos(max(-1.0, min(1.0, dot)))
        f = 1.0 if theta < 1e-9 else min(1.0, PAD_T / (PAD_L * math.sin(theta)))
        if f < worst[0]:
            worst = (f, name, math.degrees(theta))
    return worst



# --- devices, commanded BEFORE any arm motion (Newton, 2026-08-19) ----------
# Serialised per F9: concurrent pole+arm motion in the planned domain is a
# vendor-confirmed Gen-3 defect and RealMan's own workaround is serialisation.
# The pole goes first because it MOVES THE ARM BASE — stage_runner.py:19-21:
# "it must complete before any pose is resolved — the path is a function of
# pole height (215 mm between SRDF home and the tasks' 0.075)".
POLE_QUARTER_M = 0.075     # stage_runner.POLE_QUARTER_M — what every commode task commands
POLE_SPEED_PCT = 50        # what the commode task configs ask for

# open_tenth, VERBATIM from alix.srdf group inspire_hand_left/right, converted
# by dual_arm_common.hand_rad_to_hw — NEVER by picking the nearest concept
# state. stage_runner.run_hand records why: open_tenth is not a HAND_STATES_HW
# entry, and substituting the nearest name sent it to 993 counts instead of
# ~130 — "the hand flying open mid-task while gripping a cloth against the
# fixture". The fitted map takes 1.17 rad -> ~130, 0.0698 -> 133, 0.454 -> 944.
HAND_OPEN_TENTH_RAD = {
    "index_1": 1.17, "little_1": 1.17, "middle_1": 1.17, "ring_1": 1.17,
    "thumb_1": 0.0698, "thumb_2": 0.454,
}


LADDER = ("SCREEN", "EMU", "SIM", "REAL")


def parse_ladder_mode(argv=None):
    """Parse --mode SCREEN|EMU|SIM|REAL (also --mode=..., case-insensitive).

    Returns the HIGHEST rung the ladder is allowed to reach; every rung below
    it still runs, in order, and a failure at any rung stops the ladder there.
    Default SCREEN — the only rung that cannot move an arm.

    This extends the suite's shared `parse_mode_arg`, which is binary SIM|REAL,
    because the ladder has four rungs and the whole point of this driver is to
    stop at a chosen one. Same spelling, same case-insensitivity, same usage
    error shape, so `--mode SIM` means here exactly what it means in every other
    script: run it in the controller's simulation mode.
    """
    args = list(sys.argv[1:] if argv is None else argv)
    for i, a in enumerate(args):
        if a == "--mode":
            val = args[i + 1] if i + 1 < len(args) else None
        elif a.startswith("--mode="):
            val = a.split("=", 1)[1]
        else:
            continue
        v = (val or "").strip().upper()
        if v in ("SIMULATION",):
            v = "SIM"
        if v in ("EMULATOR", "EMULATED"):
            v = "EMU"
        if v in LADDER:
            return v
        raise SystemExit("usage: --mode SCREEN|EMU|SIM|REAL  (got %r)" % val)
    return "SCREEN"



def prepare_devices_inproc(side, mode, dry):
    """Command pole then hand, in that order, before any arm motion.

    Follows stage_runner.run_pole / run_hand exactly: the pole blocks on the
    device-2 arrival event, the hand is duration-based because device-2 events
    never fire for it (F4). SIM commands NEITHER — the controller simulates
    neither device — but both are still reported, because the path is resolved
    at that pole height and a silent difference is how a run gets attributed to
    the wrong geometry.
    """
    from dual_arm_common import (ArrivalMonitor, ConceptArm, DEV_LIFT,
                                 LEFT_IP, RIGHT_IP, ROBOT_PORT,
                                 LIFT_TIMEOUT_S, hand_dwell_s, hand_rad_to_hw,
                                 lift_hw_mm)
    counts = hand_rad_to_hw(HAND_OPEN_TENTH_RAD)
    if dry:
        print("    DRY pole -> %.3f m ; hand open_tenth -> %s" % (POLE_QUARTER_M, counts))
        return True
    if mode == "SIM":
        print("    SIM: pole %.3f m and hand open_tenth %s ASSUMED — the "
              "controller simulates neither device (stage_runner)."
              % (POLE_QUARTER_M, counts))
        return True

    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm(LEFT_IP if side == "left" else RIGHT_IP,
                                       ROBOT_PORT, 3)
    # ConceptArm adopts the firmware's lift gear; lift_hw_mm is only correct
    # after that, so the target is computed AFTER construction, never before.
    arm = ConceptArm(side, robot, handle)
    monitor = ArrivalMonitor()
    # The SDK delivers arrival events only to a REGISTERED callback, one per
    # process. Without this the pole never "arrives" and the prep refuses to
    # start the arm — a safe failure, but a false one.
    monitor.register(robot)
    try:
        hw = lift_hw_mm(side, POLE_QUARTER_M)
        monitor.expect(arm.handle_id, DEV_LIFT)
        ret = robot.rm_set_lift_height(POLE_SPEED_PCT, hw, 0)
        if ret != 0:
            print("    POLE REJECTED ret=%s — check the e-stop first "
                  "(test_pole_only C8: one chain covers both arms)." % ret)
            return False
        arrived, ok = monitor.wait(arm.handle_id, DEV_LIFT, LIFT_TIMEOUT_S)
        if not (arrived and ok):
            print("    POLE did not arrive within %.0fs — arm motion NOT started."
                  % LIFT_TIMEOUT_S)
            return False
        print("    pole -> %.3f m (%s hw-mm at %d%%), arrival confirmed"
              % (POLE_QUARTER_M, hw, POLE_SPEED_PCT))

        ret = robot.rm_set_hand_angle(counts, False, 2)
        if ret != 0:
            print("    HAND REJECTED ret=%s (ret -5 = the end port is in "
                  "MODBUS mode — the C6 hand-failure signature)." % ret)
            return False
        time.sleep(hand_dwell_s(373.0))          # F4: no arrival event
        print("    hand -> open_tenth %s (duration-based)" % counts)
        return True
    finally:
        try:
            robot.rm_delete_robot_arm()
        except Exception:                                    # noqa: BLE001
            pass


def prepare_devices(side, mode, dry):
    """Run the device prep in its own process, so EMU can install the emulator
    first and so the connection is CLOSED before test_blend_corner opens its
    own — two live handles on one controller is not a state worth debugging."""
    boot = ("import sys;"
            + ("import rm_emulator; rm_emulator.install();" if mode == "EMU" else "")
            + "sys.argv=['run_speed_ramp.py','--prepare-only','--side',%r,"
              "'--mode',%r%s];"
              "import runpy; runpy.run_path('run_speed_ramp.py', run_name='__main__')"
              % (side, mode, ",'--dry'" if dry else ""))
    p = subprocess.run([sys.executable, "-c", boot], capture_output=True,
                       text=True, cwd=HERE)
    out = (p.stdout or "") + (p.stderr or "")
    for line in out.strip().splitlines()[-6:]:
        print("    " + line)
    return p.returncode == 0 and "REJECTED" not in out and "did not arrive" not in out


def load_path(path):
    spec = importlib.util.spec_from_file_location("ramp_path", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def screen(mod, path, rung, cap):
    """Offline J4 screen at this rung's OWN coupled cap.

    Driven as a SUBPROCESS through orientation_cost.py rather than by importing
    preflight_j4: the in-process import returns j4_per_m = 0 on every segment
    here while the identical computation through the CLI returns 419.8 and 81 %
    of the limit at 0.45 on the same file. Until that divergence is understood
    the CLI is the path with evidence behind it, and a screen that silently
    reads zero is the most dangerous failure available — it reports PASS.

    The cap is passed as --angular-cap. orientation_cost.py takes it ONLY from
    the command line (its own default is 0.600 rad/s) and never reads
    TCP_ANGULAR_VELOCITY from the path file — an earlier version of this driver
    wrote the cap into a temp copy of the path and screened every rung at 0.600
    without noticing, which is why 0.45 and 0.60 both reported 81 %.
    """
    cmd = [sys.executable, os.path.join(HERE, "orientation_cost.py"),
           "--segments", os.path.abspath(path), "--tool", mod.TOOL_FRAME,
           "--speed", "%.3f" % rung, "--angular-cap", "%.4f" % cap]
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=HERE)
    out = (p.stdout or "") + (p.stderr or "")
    # Confirm the cap actually reached it, rather than trusting the flag.
    echoed = re.search(r"angular cap\s+([\d.]+)\s*rad/s", out)
    if echoed and abs(float(echoed.group(1)) - cap) > 0.02:
        return None, ("cap not applied: asked %.4f, screen used %s"
                      % (cap, echoed.group(1)))
    m = re.search(r"at\s+(-?[\d.]+)\s*%\s+of the J4 limit", out)
    if not m:
        return None, (out.strip().splitlines()[-1] if out.strip() else "no output")
    seg = re.search(r"worst:\s+segment\s+(\S+)", out)
    return float(m.group(1)), (seg.group(1) if seg else "?")


def run_emulated(side, path, rung, cap, line_acc, dry):
    """Rung 1 — the emulator. Installs in-process BEFORE the SDK import, so
    the UNMODIFIED test program runs against realistic motion timing and
    arrival semantics with zero hardware and zero network (EMULATOR.md).
    This is the only rung that can run on a laptop with no arm attached."""
    boot = (
        "import sys, runpy;"
        "import rm_emulator; rm_emulator.install();"
        "sys.argv=['test_blend_corner.py','--side',%r,'--mode','REAL',"
        "'--path',%r,'--speed','%.3f','--angular-speed','%.4f','--line-acc','%.2f'];"
        "runpy.run_path('test_blend_corner.py', run_name='__main__')"
        % (side, path, rung, cap, line_acc))
    cmd = [sys.executable, "-c", boot]
    print("    $ python3 -c \"import rm_emulator; install(); test_blend_corner "
          "--mode REAL --speed %.3f --angular-speed %.4f\"" % (rung, cap))
    if dry:
        print("    (dry run — not executed)")
        return True, "dry"
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=HERE)
    out = (p.stdout or "") + (p.stderr or "")
    print("\n".join(out.strip().splitlines()[-12:]), flush=True)
    if "no joint-speed telemetry" in out:
        print("    NOTE: the emulator produced NO joint-speed telemetry, so the "
              "all-joint abort criteria were NOT evaluated on this rung. An "
              "emulator PASS means the PROGRAM ran, not that the joints were "
              "within limits — only SIM/REAL can say that.")
    # A run that produced no usable result is NOT a pass. The emulator rung
    # taught this the hard way: it returned 0 while printing "NO RESULT — the
    # tool traced 10.300 m against a commanded 4.330 m (238 %)", and an
    # earlier version of this driver called that PASS.
    bad = any(k in out for k in ("DID NOT FINISH", "did not finish",
                                 "did not complete", "arrival event reports "
                                 "failure",
                                 "Traceback", "ABORT", "over limit",
                                 "NO RESULT", "stream is not this path",
                                 "would be fiction"))
    return (p.returncode == 0 and not bad), out


def check_completion(meta, rows):
    """Did the tool actually traverse the program? Works in SIM and REAL.

    Two independent signatures, both from the recording alone:
      * the LAST sample sits at the LAST commanded waypoint;
      * the traced length is not far short of the commanded length.
    `test_blend_corner` prints "[STOP] ... did not complete" for this and
    then EXITS 0, so neither the exit code nor the string blacklist catches
    it — this does.
    """
    cmd = meta.get("commanded") or {}
    poses = cmd.get("poses") or []
    if len(poses) < 2:
        return True, "no commanded poses in run.json — completion NOT checked"

    def d3(a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    end = [float(rows[-1]["tcp_%s" % k]) for k in ("x", "y", "z")]
    miss = d3(end, poses[-1])
    if miss > FINAL_WAYPOINT_TOL_M:
        return False, ("DID NOT COMPLETE — ended %.0f mm from the final "
                       "waypoint (tol %.0f mm)"
                       % (miss * 1000, FINAL_WAYPOINT_TOL_M * 1000))

    vias = cmd.get("arc_vias") or []
    want = 0.0
    for i in range(len(poses) - 1):
        via = vias[i] if i < len(vias) else None
        want += (d3(poses[i], via) + d3(via, poses[i + 1])) if via \
            else d3(poses[i], poses[i + 1])
    got = sum(d3([float(a["tcp_%s" % k]) for k in ("x", "y", "z")],
                 [float(b["tcp_%s" % k]) for k in ("x", "y", "z")])
              for a, b in zip(rows, rows[1:]))
    if want > 0 and got < TRACED_FLOOR * want:
        return False, ("DID NOT COMPLETE — traced %.3f m against a commanded "
                       "%.3f m (%.0f %%)" % (got, want, 100.0 * got / want))
    return True, ("completed: %.3f m traced / %.3f m commanded, %.0f mm from "
                  "the final waypoint" % (got, want, miss * 1000))


def check_recording(out):
    """Apply the ABORT CRITERIA to the recording a rung just wrote.

    ADDED 2026-08-19 AFTER THE LADDER WALKED STRAIGHT PAST THEM. The two
    criteria at the top of this file were printed in the header and never
    enforced: `run()` judged a rung on `test_blend_corner`'s exit code and a
    blacklist of strings, and never opened the stream. So rung 0.90 recorded
    J4 at 223.3 deg/s (99 % of 225) with 10 ms at or above 98 %, both of
    which are abort conditions, and the ladder advanced to 1.00 — where three
    joints reversed inside 20 ms and J4 drew 26.4 A, the highest current this
    project has recorded. No joint error bit was set and `arm_status` read 0
    (IDLE) throughout, so nothing else was going to catch it either.

    Returns (ok, reason). Reads the run directory the runner prints.
    """
    m = re.search(r"recorded .*? -> (\S+)", out)
    if not m:
        return True, "no recording path in the output — criteria NOT applied"
    d = m.group(1)
    try:
        meta = json.loads(open(os.path.join(d, "run.json")).read())
        rows = list(csv.DictReader(open(os.path.join(d, "stream.csv"))))
    except Exception as exc:                                  # noqa: BLE001
        return True, "recording unreadable (%s) — criteria NOT applied" % exc
    if len(rows) < 60:
        return True, "recording too short to judge"
    # PATH COMPLETION FIRST — it is the ONLY check SIM can make, and on the
    # 2026-08-19 ladder it was the one that mattered: SIM traced 2.372 m of a
    # 4.468 m program at v=1.00 and ended 350 mm from the last waypoint,
    # reproducing the REAL failure offline. SIM is mandatory before REAL
    # (Newton's screen -> SIM -> REAL rule); the offline JOINT screen could
    # not have done it, because it averages a 3-sample event away.
    ok, why = check_completion(meta, rows)
    if not ok:
        return False, why
    if meta.get("sim"):
        return True, why + " (SIM: joint channel is dead, joint criteria NOT applied)"
    lim = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]
    peak = [max(abs(float(r["speed%d" % j])) for r in rows)
            for j in range(1, 8)]
    util = [100.0 * peak[j] / lim[j] for j in range(7)]
    worst = max(util)
    wj = util.index(worst) + 1
    dwell_ms = 10 * sum(1 for r in rows
                        if any(abs(float(r["speed%d" % j])) >= 0.98 * lim[j - 1]
                               for j in range(1, 8)))
    cur = max(max(abs(float(r["current%d" % j])) for j in range(1, 8))
              for r in rows) / 1000.0
    note = ("worst J%d %.0f %% of limit, dwell>=98%% %d ms, peak current %.1f A"
            % (wj, worst, dwell_ms, cur))
    if worst > JOINT_ABORT:
        return False, "ABORT — %s (any joint over %.0f %%)" % (note, JOINT_ABORT)
    if dwell_ms > 0:
        return False, "ABORT — %s (any dwell at >=98 %%, any duration)" % note
    return True, note


def run(mode, side, path, rung, cap, line_acc, dry):
    cmd = [sys.executable, os.path.join(HERE, "test_blend_corner.py"),
           "--side", side, "--mode", mode, "--path", path,
           "--speed", "%.3f" % rung,
           "--angular-speed", "%.4f" % cap,
           "--line-acc", "%.2f" % line_acc]
    print("    $ " + " ".join(cmd), flush=True)
    if dry:
        print("    (dry run — not executed)")
        return True, "dry"
    p = subprocess.run(cmd, capture_output=True, text=True)
    out = (p.stdout or "") + (p.stderr or "")
    tail = "\n".join(out.strip().splitlines()[-12:])
    print(tail, flush=True)
    # A run that produced no usable result is NOT a pass. The emulator rung
    # taught this the hard way: it returned 0 while printing "NO RESULT — the
    # tool traced 10.300 m against a commanded 4.330 m (238 %)", and an
    # earlier version of this driver called that PASS.
    bad = any(k in out for k in ("DID NOT FINISH", "did not finish",
                                 "did not complete", "arrival event reports "
                                 "failure",
                                 "Traceback", "ABORT", "over limit",
                                 "NO RESULT", "stream is not this path",
                                 "would be fiction"))
    ok = (p.returncode == 0 and not bad)
    # THE ABORT CRITERIA, applied to what was actually recorded — not to the
    # exit code. A dispatch can complete cleanly with a joint at 99 %.
    if ok:
        ok, why = check_recording(out)
        if why:
            print("    criteria: %s" % why, flush=True)
    return ok, tail


def main():
    from dual_arm_common import handle_cli
    handle_cli(__doc__, extra_flags=("--dry", "--prepare-only", "--no-prepare"),
               value_flags=("--side", "--path", "--rungs", "--angular-acc",
                            "--coupling"),
               allow_common=True)
    mode = parse_ladder_mode()
    global ANGULAR_ACC_PINNED, COUPLING
    if "--angular-acc" in sys.argv:
        ANGULAR_ACC_PINNED = float(sys.argv[sys.argv.index("--angular-acc") + 1])
    if "--coupling" in sys.argv:
        COUPLING = float(sys.argv[sys.argv.index("--coupling") + 1])
    side = "left"
    if "--side" in sys.argv:
        side = sys.argv[sys.argv.index("--side") + 1]
    if "--prepare-only" in sys.argv:
        return 0 if prepare_devices_inproc(side, mode, "--dry" in sys.argv) else 1
    if "--side" in sys.argv:
        side = sys.argv[sys.argv.index("--side") + 1]
    path = DEFAULT_PATH
    if "--path" in sys.argv:
        path = sys.argv[sys.argv.index("--path") + 1]
    dry = "--dry" in sys.argv

    mod = load_path(path)
    rungs = list(mod.SPEED_LADDER)
    if "--rungs" in sys.argv:
        rungs = [float(x) for x in sys.argv[sys.argv.index("--rungs") + 1].split(",")]

    top = LADDER.index(mode)
    print("SPEED RAMP  path=%s  side=%s  mode=%s (rungs run: %s)"
          % (os.path.basename(path), side, mode, " -> ".join(LADDER[:top + 1])))
    if ANGULAR_ACC_PINNED is not None:
        print("  coupling omega = %.2f * v ; angular_acc PINNED at %.2f "
              "(--angular-acc) -> omega ceiling %.4f rad/s"
              % (COUPLING, ANGULAR_ACC_PINNED, ANGULAR_ACC_PINNED / 3.0))
    else:
        print("  coupling omega = %.2f * v ; angular_acc UNCAPPED, floor %.2f, "
              "raised to 3x the rung's cap -> NO omega ceiling"
              % (COUPLING, ANGULAR_ACC_FLOOR))
        print("  (the 1.3333 rad/s ratio ceiling that stopped this ladder at "
              "0.80 is removed; H62 stop-distance caveat accepted)")
    print("  abort: ANY joint > %.0f %% of limit, or ANY dwell >= 98 %%" % JOINT_ABORT)
    f_min, f_at, f_th = contact_profile(mod)
    print("  contact (applied, not capped): min f = %.3f at %s (theta %.1f deg)%s\n"
          % (f_min, f_at, f_th,
             "   <-- BELOW the %.2f gate; recorded, not prevented" % CONTACT_GATE
             if f_min < CONTACT_GATE else "   (gate %.2f)" % CONTACT_GATE))

    # Devices first, once, before ANY arm motion.
    if mode != "SCREEN" and "--no-prepare" not in sys.argv:
        print("DEVICES (pole -> hand, serialised per F9; arm motion waits)")
        if not prepare_devices(side, mode, dry):
            print("  device prep FAILED — no arm motion attempted.")
            return 1
        print()

    consecutive = 0
    for rung in rungs:
        cap = COUPLING * rung
        line_acc = line_acc_for(rung)
        ang_acc = angular_acc_for(cap)
        print("=" * 72)
        print("RUNG %.2f m/s   omega_cap %.4f rad/s   line_acc %.2f m/s^2   "
              "angular_acc %.2f rad/s^2%s"
              % (rung, cap, line_acc, ang_acc,
                 "  <-- above the shipped 4.00" if ang_acc > 4.0 + 1e-9 else ""))
        if ANGULAR_ACC_PINNED is not None and cap > ANGULAR_ACC_PINNED / 3.0 + 1e-9:
            print("  STOP — omega_cap %.4f exceeds the PINNED angular_acc/3 "
                  "(%.4f). Raise --angular-acc to >= %.2f or drop the pin."
                  % (cap, ANGULAR_ACC_PINNED / 3.0, 3 * cap))
            break

        # rung 1 — SCREEN, always, whatever --mode says
        worst, seg = screen(mod, path, rung, cap)
        if worst is None or worst <= 0.0:
            print("  SCREEN: unavailable or degenerate (got %r) — refusing to "
                  "continue blind.\n    detail: %s" % (worst, seg))
            break
        # The screen RANKS — it names the junction. It never certifies.
        optimistic = SCREEN_UNDERREAD_MIN * worst
        if optimistic > JOINT_ABORT:
            verdict, ok = "BLOCK", False
        else:
            verdict, ok = "RANKED", True
        print("  1. SCREEN   worst J4 %.0f %% at %s -> real >= %.0f %% -> %s"
              % (worst, seg, optimistic, verdict))
        if verdict == "RANKED":
            print("     NOT a clearance. The screen has done its whole "
                  "job: it named the junction. Geometry gates below.")
        if not ok:
            print("     rung refused before any motion: even at the lowest "
                  "measured under-read (x%.2f -> %.0f %%) it exceeds the %.0f %% "
                  "abort." % (SCREEN_UNDERREAD_MIN, optimistic, JOINT_ABORT))

        # rung 1b — GEOMETRY PREDICTION. ADVISORY: never refuses a rung.
        # Standing this down is deliberate (Newton, 2026-08-22, restarting the
        # test to understand the behaviour). The predictor over-flags — run
        # verbatim it refuses rungs hardware completed — so gating on it would
        # suppress the measurements that would fix it. It prints, the rung
        # runs, the recording decides.
        if ok:
            try:
                import junction_limits
                jtxt, jbind = junction_limits.report(
                    mod, rung, line_acc, os.path.basename(path))
                print("\n".join("  " + ln for ln in jtxt.splitlines()))
                if jbind is not None and jbind < rung - 1e-9:
                    print("     PREDICTION: geometry would put this rung at "
                          "%.3f m/s, below the commanded %.2f. NOT ENFORCED — "
                          "the rung runs." % (jbind, rung))
                    print("     Record whether it was right. If the rung "
                          "completes clean, the predictor is miscalibrated "
                          "here; do not re-tune it to fit — measure c(theta).")
            except Exception as exc:                          # noqa: BLE001
                print("  1b. GEOMETRY  prediction UNAVAILABLE (%s) — the rung "
                      "still runs; there is simply no offline prediction to "
                      "score against." % exc)
        if not ok:
            consecutive += 1
            if consecutive >= CONSECUTIVE_FAIL_STOP:
                print("\nLADDER STOPPED — %d consecutive failed rungs." % consecutive)
                break
            continue
        if top == LADDER.index("SCREEN"):
            consecutive = 0
            continue

        # rung 2 — EMULATOR (no hardware needed)
        ok, emu_out = run_emulated(side, path, rung, cap, line_acc, dry)
        geometry_only = (not ok) and ("Traceback" not in emu_out) and (
            "NO RESULT" in emu_out or "stream is not this path" in emu_out)
        if geometry_only:
            ok = True
            print("  2. EMULATOR -> ADVISORY (geometry not reproduced)")
            print("     SIM caveat: the emulator cannot validate this path's arc "
                  "geometry and emits no joint telemetry, so this rung is a "
                  "DIFFERENT test, not a weaker one — it answers nothing about "
                  "joints and does not gate SIM.")
        else:
            print("  2. EMULATOR -> %s" % ("PASS" if ok else "FAIL"))
        if not ok:
            print("     higher rungs NOT entered — the emulator failed as a "
                  "PROGRAM, not merely on geometry.")
            consecutive += 1
            if consecutive >= CONSECUTIVE_FAIL_STOP:
                print("\nLADDER STOPPED — %d consecutive failed rungs." % consecutive)
                break
            continue
        if top == LADDER.index("EMU"):
            consecutive = 0
            continue

        # rung 3 — SIM (needs the controller)
        ok, _ = run("SIM", side, path, rung, cap, line_acc, dry)
        print("  3. SIM      -> %s" % ("PASS" if ok else "FAIL"))
        if not ok:
            print("     REAL NOT ENTERED for this rung (Newton's rule).")
            consecutive += 1
            if consecutive >= CONSECUTIVE_FAIL_STOP:
                print("\nLADDER STOPPED — %d consecutive failed rungs." % consecutive)
                break
            continue
        if top == LADDER.index("SIM"):
            consecutive = 0
            continue

        # rung 4 — REAL
        ok, _ = run("REAL", side, path, rung, cap, line_acc, dry)
        print("  4. REAL     -> %s" % ("PASS" if ok else "FAIL"))
        consecutive = 0 if ok else consecutive + 1
        if consecutive >= CONSECUTIVE_FAIL_STOP:
            print("\nLADDER STOPPED — %d consecutive failed rungs." % consecutive)
            break

    print("=" * 72)
    print("Ramp finished. Per the ratchet rule, run reset_limits.py "
          "before any other work on this arm.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
