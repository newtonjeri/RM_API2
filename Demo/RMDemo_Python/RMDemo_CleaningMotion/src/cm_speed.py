"""Speed selection, the scaling law, and the offline screens.

THE SCALING LAW IS IN EFFECT HERE (Newton, 2026-08-22) and it is not
optional bookkeeping. The controller REJECTS a (speed, acceleration) pair
whose acceleration is under 3x the speed — with a bare `ret=1`, after which
the run proceeds at whatever was already configured and reads as "the speed
made no difference". A test that raises only the speed measures nothing and
does not say so. So every speed here goes through
`speed_limits.scale_for()`, which returns a legal pair or refuses.

Two further traps that law hides, both already paid for next door and both
inherited here by importing rather than re-deriving:

  * `3 * 0.70` is 2.0999999999999996 in binary floating point, which reads
    as "below 3" to a strict comparison — and the controller applies the
    same rule with arithmetic we cannot inspect. `line_acc_for()` carries a
    0.01 margin, the smallest a "%.2f" wire format can express.
  * `v` IS A PERCENTAGE, not m/s. `rm_movel`'s v is a percent of the arm's
    TCP speed constraint (hardware-confirmed: the arm ran 0.495 while a
    config said 0.8). So a rung is set by CONFIGURING `line_speed` and
    commanding v as a percent of it — never by passing m/s as v.

WHAT THE SCREENS CAN AND CANNOT SAY. The J4 screen is exact for the elbow
and blind to everything else: on an S-R-S arm J4 is fixed by the commanded
pose regardless of how the controller resolves the 7-DOF redundancy, so it
is the one joint predictable offline. The other six are not. A clean screen
is therefore NECESSARY AND NOT SUFFICIENT, and it may REJECT a speed but
never certifies one. The junction audit is advisory and is known to
over-flag; it prints and never refuses.
"""

import math

from cm_common import speed_limits

# The calibrated thresholds, imported from the driver that measured them so
# there is one copy. This module is import-safe next door (its entry point
# is behind `if __name__ == "__main__"`).
from run_speed_ramp import (                                   # noqa: E402
    SCREEN_GATE, SCREEN_UNDERREAD_MIN, JOINT_ABORT,
    line_acc_for, angular_acc_for, COUPLING as COUPLING_DEFAULT,
)

FACTORY_LINE_SPEED = 0.250      # m/s — the pendant "Default", F10/H64
FACTORY_ANGULAR = 0.600         # rad/s


class SpeedPlan:
    """The speeds a run will use, and where each number came from."""

    def __init__(self, rungs, coupling, line_acc=None, angular=None,
                 source="", allow_over=False, keep_controller=False):
        self.rungs = list(rungs)
        self.coupling = float(coupling)
        self.line_acc_pinned = line_acc
        self.angular_pinned = angular
        self.source = source
        self.allow_over = bool(allow_over)
        # True when nobody asked for a speed. The arm then KEEPS whatever
        # its controller already holds — that is the default, per Newton
        # (2026-08-22) — and `rungs` is only an assumption the offline
        # screens need a number for.
        self.keep_controller = bool(keep_controller)

    def limits_to_apply(self, rung):
        """Only the limits someone actually ASKED for.

        An empty dict means "change nothing", which is different from
        "set it to what it already is": the TCP limits ratchet and are
        global controller state shared with the Web GUI, so not writing
        them at all is the safer and more honest default.
        """
        if not self.keep_controller:
            return self.limits_for(rung)[0]
        out = {}
        if self.line_acc_pinned is not None:
            out["line_acc"] = float(self.line_acc_pinned)
        if self.angular_pinned is not None:
            out["angular_speed"] = float(self.angular_pinned)
            out["angular_acc"] = angular_acc_for(float(self.angular_pinned))
        return out

    def limits_for(self, rung):
        """The four controller limits for one rung, all mutually legal."""
        speed, acc, notes = speed_limits.scale_for(
            rung, acc=self.line_acc_pinned or line_acc_for(rung))
        ang = self.angular_pinned if self.angular_pinned is not None \
            else self.coupling * rung
        return {
            "line_speed": speed,
            "line_acc": acc,
            "angular_speed": ang,
            "angular_acc": angular_acc_for(ang),
        }, notes

    def describe(self):
        out = ["speed plan (%s)" % self.source,
               "  rungs      %s m/s" % ", ".join("%.3f" % r for r in self.rungs),
               "  coupling   omega = %.3f * v" % self.coupling]
        if self.line_acc_pinned:
            out.append("  line_acc   PINNED at %.2f m/s^2" % self.line_acc_pinned)
        if self.angular_pinned is not None:
            out.append("  angular    PINNED at %.3f rad/s" % self.angular_pinned)
        for r in self.rungs:
            lim, _ = self.limits_for(r)
            out.append("    %.3f m/s -> line_acc %.2f, angular %.3f rad/s, "
                       "angular_acc %.2f"
                       % (r, lim["line_acc"], lim["angular_speed"],
                          lim["angular_acc"]))
        return "\n".join(out)


def resolve(argv, program):
    """Build a SpeedPlan from the CLI, then the motion's own declaration.

    Precedence, most explicit first: --speed / --speeds, then a ladder the
    motion file declares, then NOTHING — in which case the arm keeps the
    speed its controller already holds and no limit is written at all.
    An unspecified speed must never become a fast one, and it must not
    silently reconfigure the machine either.
    """
    def val(flag, cast=float, default=None):
        if flag in argv:
            i = argv.index(flag) + 1
            if i >= len(argv):
                raise SystemExit("%s needs a value" % flag)
            try:
                return cast(argv[i])
            except ValueError:
                raise SystemExit("%s: %r is not a number" % (flag, argv[i]))
        return default

    coupling = val("--coupling", float, COUPLING_DEFAULT)
    line_acc = val("--line-acc", float, None)
    angular = val("--angular-speed", float, None)
    allow_over = "--allow-over-limit" in argv
    keep = False

    if "--speed" in argv:
        rungs, source = [val("--speed")], "--speed (single, off-ladder)"
    elif "--speeds" in argv:
        raw = argv[argv.index("--speeds") + 1]
        try:
            rungs = [float(x) for x in raw.split(",") if x.strip()]
        except ValueError:
            raise SystemExit("--speeds wants a comma list, got %r" % raw)
        if not rungs:
            raise SystemExit("--speeds is empty")
        source = "--speeds (explicit ladder)"
    elif program is not None and program.speed_ladder:
        rungs = list(program.speed_ladder)
        source = "the motion file's own ladder"
    else:
        # Nobody named a speed, so the arm KEEPS the one its controller
        # already holds. The screens still need a number to compute
        # against; the factory value is used for that and labelled as the
        # assumption it is, and is replaced by the controller's real value
        # as soon as there is a connection to read it from.
        rungs = [FACTORY_LINE_SPEED]
        keep = True
        source = ("no speed given — the arm KEEPS its controller's current "
                  "line_speed; screens assume the factory %.3f m/s until "
                  "the arm is read" % FACTORY_LINE_SPEED)

    for r in rungs:
        if r <= 0:
            raise SystemExit("speeds must be positive, got %.3f" % r)
    if angular is not None and angular <= 0:
        raise SystemExit("--angular-speed must be positive, got %.3f" % angular)
    return SpeedPlan(rungs, coupling, line_acc, angular, source, allow_over,
                     keep_controller=keep)


def apply_limits(robot, plan, rung, allow_raise=False):
    """Set the four limits for one rung. Returns the PREVIOUS values.

    These are GLOBAL controller state, shared with the Web GUI and every
    other program on the arm, so the caller owns putting them back — see
    `restore_limits`, and call it in a `finally`.
    """
    lim, notes = plan.limits_for(rung)
    before = speed_limits.apply(robot, allow_raise=allow_raise, **lim)
    return before, lim, notes


def restore_limits(robot, before):
    if before:
        speed_limits.restore(robot, before)


def screen_j4(poses, tool_frame, rung, v_list=None, angular_cap=None):
    """(worst % of the J4 limit, segment) or None when the screen can't run.

    None means UNKNOWN and must never be read as safe. Delegates to the
    concept tree's `preflight_j4`, which owns the tool-offset transform and
    refuses to opine when its own selfcheck fails.
    """
    try:
        from test_blend_corner import preflight_j4
    except Exception as exc:                                  # noqa: BLE001
        print("  [WARN] J4 screen unavailable (%r) — treat this rung as "
              "UNSCREENED, not as clear." % exc)
        return None
    return preflight_j4(poses, tool_frame, rung, v_list=v_list,
                        angular_cap=angular_cap)


def verdict_j4(worst, allow_over=False):
    """(ok, line) — the only offline check here permitted to REFUSE a rung.

    Rejection uses the LOWEST measured under-read (x1.10): the screen reads
    10-15 % low against hardware, so a rung is refused only when even the
    most optimistic reading still exceeds the abort threshold. That keeps
    the refusal defensible without needing the screen to be accurate.
    """
    if worst is None:
        return True, ("  J4 screen: UNAVAILABLE — rung is UNSCREENED. This "
                      "is not a clearance.")
    optimistic = worst * SCREEN_UNDERREAD_MIN
    if optimistic >= JOINT_ABORT:
        line = ("  J4 screen: %.0f %% -> real >= %.0f %% -> REFUSED (abort is "
                "%.0f %%)%s" % (worst, optimistic, JOINT_ABORT,
                                "" if not allow_over else
                                " [--allow-over-limit: RUNNING ANYWAY]"))
        return bool(allow_over), line
    flag = "" if worst < SCREEN_GATE else "  <-- above the %.0f %% screen gate" \
        % SCREEN_GATE
    return True, ("  J4 screen: %.0f %% of the J4 limit -> real >= %.0f %% -> "
                  "RANKED%s\n    Not a clearance: J4 is screened for being "
                  "redundancy-invariant, not for being the binding joint."
                  % (worst, optimistic, flag))


def audit_junctions(program, poses, rung, line_acc):
    """Advisory geometry audit. Returns text; NEVER refuses a rung.

    The predictor is known to over-flag — run verbatim it refuses rungs
    hardware has completed — so it prints and the recording judges. It needs
    a path-module-shaped object, which is built here from the MotionProgram
    rather than requiring one on disk.
    """
    try:
        import junction_limits
    except Exception as exc:                                  # noqa: BLE001
        return "  junction audit unavailable (%r) — the rung still runs." % exc

    class _Mod:                       # the shape `junction_limits` reads
        POSES_MM = {(program.waypoint_names[i] if i < len(program.waypoint_names)
                     else "p%02d" % i): [1000 * q[0], 1000 * q[1], 1000 * q[2],
                                         q[3], q[4], q[5]]
                    for i, q in enumerate(poses)}
        R_LIST = program.r_list or [0] * max(1, len(poses) - 1)
        V_LIST = program.v_list or [100] * max(1, len(poses) - 1)
        ARC_LIST = None
        VIA_MM = {}

    try:
        txt, _ = junction_limits.report(_Mod, rung, line_acc, program.name)
        return "\n".join("  " + ln for ln in txt.splitlines())
    except Exception as exc:                                  # noqa: BLE001
        return "  junction audit failed (%r) — the rung still runs." % exc


# `build_program` lives in `cm_arm`, next to the code that dispatches it.
# It was briefly here too, delegating elsewhere — two places deriving the
# per-move (v, r, connect) tuples, which is the drift this bed is built to
# avoid. One implementation, beside its only caller.


def angular_for(plan, rung):
    return plan.limits_for(rung)[0]["angular_speed"]


def summarise(program, poses):
    """Static facts a speed decision should be taken against."""
    L = [math.dist(poses[i][:3], poses[i + 1][:3])
         for i in range(len(poses) - 1)]
    if not L:
        return "  (no segments)"
    return ("  segments %d, shortest %.1f mm, longest %.1f mm, total %.0f mm"
            % (len(L), 1000 * min(L), 1000 * max(L), 1000 * sum(L)))
