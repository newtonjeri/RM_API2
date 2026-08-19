#!/usr/bin/env python3
"""SPEED RAMP DRIVER — screen -> SIM -> REAL, one rung at a time.

Newton, 2026-08-19: ramp the linear cap 0.45 -> 1.00 m/s in +0.1 steps with
the angular cap coupled by C2 (omega = 1.25 * v), holding angular_acc at 4.0,
and settle what the contract asserts without evidence.

WHY A DRIVER AND NOT `test_blend_corner.py --path ... ` ON ITS OWN.
A single invocation carries ONE `--angular-speed`, but this ramp needs the cap
to move WITH the rung (that is the C2 coupling under test). So each rung is its
own invocation, and the sequencing rule lives here.

THE RULE, in Newton's words: "screen -> SIM -> REAL, do not proceed to REAL if
SIM fails." Enforced literally — a rung whose screen fails never reaches SIM,
and a rung whose SIM fails never reaches REAL. The ladder also stops after two
consecutive failed rungs rather than walking every remaining rung into a wall.

ABORT CRITERIA (agreed with Newton, 2026-08-19), applied to the REAL rung:
  * ANY joint above 95 % of its limit  — not J4 alone. J4 is screened because
    it is the only redundancy-invariant joint (contract A.4), NOT because it
    binds: J1 binds on 11 of 24 tasks, and at 0.8 m/s on toplid_left FOUR
    joints were over limit at once.
  * ANY dwell at >=98 % of a limit, for any duration at all (the H63 metric —
    0 ms at 0.45, 330 ms on the 0.8 run that did not finish).

WHAT THIS RAMP IS EXPECTED TO SHOW (a prediction, recorded before the run so
it can be wrong): the conditioning tilt this arm needs to keep the elbow out
of a singularity is ~40.5 deg over a 380 mm stroke, i.e. kappa = 1.86 rad/m.
A segment demands omega = kappa * v, so under the C2 coupling every stroke
runs at v_eff = 1.25 v / 1.86 = 0.67 * v at EVERY rung, and the absolute
vendor ceiling omega <= angular_acc/3 = 1.333 pins the stroke at
1.333 / 1.86 = 0.72 m/s no matter how high the linear cap goes. If that holds,
the contract's 1.0 m/s target is unreachable without raising angular_acc to
>= 5.6 rad/s^2 — which is untested, and which RealMan advise against because
the shipped values preserve the ability to stop immediately (H62).

USAGE
    python3 run_speed_ramp.py --side left --mode SCREEN   # offline only (default)
    python3 run_speed_ramp.py --side left --mode EMU      # + emulator, no arm
    python3 run_speed_ramp.py --side left --mode SIM      # + controller SIM
    python3 run_speed_ramp.py --side left --mode REAL     # the whole ladder
    python3 run_speed_ramp.py --side left --mode SIM --rungs 0.45,0.50
    python3 run_speed_ramp.py --side left --mode REAL --dry   # print, run nothing

--mode names the HIGHEST rung the ladder may reach; every rung below it still
runs in order, and a failure stops the ladder there. SCREEN is the default
because it is the only rung that cannot move an arm.
"""
from __future__ import annotations

import importlib.util
import math
import time
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(HERE, "..", "paths", "planar_speed_ramp_001.py")

COUPLING = 1.25        # C2: omega_cap = 1.25 * v (Newton's ratio)
ANGULAR_ACC = 4.00     # HELD (Newton, 2026-08-19)
OMEGA_MAX = ANGULAR_ACC / 3.0      # 1.3333 — vendor ratio, enforced by the runner too
SCREEN_GATE = 90.0     # % of the J4 limit (contract C3)
JOINT_ABORT = 95.0     # % of any joint's limit
CONSECUTIVE_FAIL_STOP = 2


# --- contact fraction, contract A.3 -----------------------------------------
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
    because C9's ladder has four rungs and the whole point of this driver is to
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
    """C9 rung 1 — the emulator. Installs in-process BEFORE the SDK import, so
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
                                 "Traceback", "ABORT", "over limit",
                                 "NO RESULT", "stream is not this path",
                                 "would be fiction"))
    return (p.returncode == 0 and not bad), out


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
                                 "Traceback", "ABORT", "over limit",
                                 "NO RESULT", "stream is not this path",
                                 "would be fiction"))
    return (p.returncode == 0 and not bad), tail


def main():
    from dual_arm_common import handle_cli
    handle_cli(__doc__, extra_flags=("--dry", "--prepare-only", "--no-prepare"),
               value_flags=("--side", "--path", "--rungs"),
               allow_common=True)
    mode = parse_ladder_mode()
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
    print("  coupling omega = %.2f * v ; angular_acc HELD at %.2f -> omega_max %.4f rad/s"
          % (COUPLING, ANGULAR_ACC, OMEGA_MAX))
    print("  abort: ANY joint > %.0f %% of limit, or ANY dwell >= 98 %%" % JOINT_ABORT)
    f_min, f_at, f_th = contact_profile(mod)
    print("  contact (A.3, applied not capped): min f = %.3f at %s (theta %.1f deg)%s\n"
          % (f_min, f_at, f_th,
             "   <-- BELOW the 0.5 gate; recorded, not prevented" if f_min < 0.5 else ""))

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
        line_acc = max(1.60, 3.0 * rung)
        print("=" * 72)
        print("RUNG %.2f m/s   omega_cap %.4f rad/s   line_acc %.2f m/s^2"
              % (rung, cap, line_acc))
        if cap > OMEGA_MAX + 1e-9:
            print("  STOP — omega_cap %.4f exceeds the vendor ratio ceiling %.4f "
                  "(angular_acc/3). Raising it needs angular_acc >= %.2f, which is "
                  "untested and which RealMan advise against (H62)."
                  % (cap, OMEGA_MAX, 3 * cap))
            break

        # rung 1 — SCREEN, always, whatever --mode says
        worst, seg = screen(mod, path, rung, cap)
        if worst is None or worst <= 0.0:
            print("  SCREEN: unavailable or degenerate (got %r) — refusing to "
                  "continue blind.\n    detail: %s" % (worst, seg))
            break
        ok = worst <= SCREEN_GATE
        print("  1. SCREEN   worst J4 %.0f %% of limit at %s  -> %s"
              % (worst, seg, "PASS" if ok else "FAIL"))
        if not ok:
            print("     rung refused before any motion. (A J4 pass is not a "
                  "clearance either — A.4: the screen is sound but INCOMPLETE.)")
            consecutive += 1
            if consecutive >= CONSECUTIVE_FAIL_STOP:
                print("\nLADDER STOPPED — %d consecutive failed rungs." % consecutive)
                break
            continue
        if top == LADDER.index("SCREEN"):
            consecutive = 0
            continue

        # rung 2 — EMULATOR (no hardware needed)
        ok, _ = run_emulated(side, path, rung, cap, line_acc, dry)
        print("  2. EMULATOR -> %s" % ("PASS" if ok else "FAIL"))
        if not ok:
            print("     higher rungs NOT entered for this rung.")
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
    print("Ramp finished. Per the ratchet rule (C2), run reset_limits.py "
          "before any other work on this arm.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
