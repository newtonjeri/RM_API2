"""The arm controller — one method per motion, `v / r / connect / block` on
every one of them, in the shape of `RMDemo_Moves/src/core/demo_moves.py`.

WHY THIS SHAPE. `demo_moves.RobotArmController` makes the two parameters
that decide how a trajectory is executed — the **blend radius** `r` and the
**connect** flag — visible arguments on every call, defaulted but always
overridable. That is exactly what this bed needs: it exists to watch how the
hardware executes a cleaning motion, and `r` and `connect` are the two knobs
that change the answer. Burying them makes the interesting variable
invisible.

    r        blend radius. A PERCENTAGE 0-100 of the shorter adjoining
             segment, not millimetres (vendor: jiao rong ban jing bai fen bi).
    connect  1 = this move joins the next into one continuous trajectory,
             0 = the move is discrete and the arm comes to rest at its end.

THE CHAIN-CLOSE RULE, preserved from the concept tree: the LAST move of a
chain always carries `r=0, connect=0`. A chain whose final move still says
connect=1 never closes — the controller waits for a continuation that never
arrives, and the run hangs rather than failing.

BLOCKING. `block=1` asks the SDK to block until the move completes;
`block=0` returns immediately and arrival is observed on the event stream.
A CHAINED program produces ONE arrival event, from the closing `connect=0`
segment — waiting for one per move is how a chained test hangs. Both are
offered because they are genuinely different things to measure, and the
arrival semantics are hardware-only (the emulator does not reproduce them
faithfully), so a SIM result about blocking proves nothing.
"""

import time

from cm_common import ARM_SPEED_PCT, ConceptArm, DEV_JOINT, error_codes


class CleaningArmController:
    """One arm, arm-only. No pole, no hand — this class cannot command them.

    That is a property of the class, not a discipline: there is no lift or
    hand method here, so no run out of this bed can move either device by
    accident. The pole is assumed at its minimum and that assumption lives
    in the frame transform (`cm_frames`), not in a command.
    """

    def __init__(self, ip, port=8080, level=3, mode=2, side="left",
                 quiet=False):
        from Robotic_Arm.rm_robot_interface import RoboticArm
        from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
        self.side = side
        self.ip = ip
        self.thread_mode = rm_thread_mode_e(mode)
        self.robot = RoboticArm(self.thread_mode)
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)
        self.connected = bool(self.handle and self.handle.id > 0)
        if not self.connected:
            if not quiet:
                print("  [SKIP] arm not reachable at %s" % ip)
            return
        if not quiet:
            print("  connected to %s (handle %d)" % (ip, self.handle.id))
        # ConceptArm carries the calibrated setup this bed must not skip:
        # it adopts the firmware's lift units and enables self-collision
        # detection. Wrapping rather than reimplementing keeps one copy.
        self.arm = ConceptArm(side, self.robot, self.handle)
        self.monitor = None

    # ── plumbing ────────────────────────────────────────────────────────
    def attach_monitor(self, monitor):
        self.monitor = monitor
        return self

    def get_arm_model(self):
        res, info = self.robot.rm_get_robot_info()
        return info["arm_model"] if res == 0 else None

    def disconnect(self):
        if not self.connected:
            return
        r = self.robot.rm_delete_robot_arm()
        print("  disconnected" if r == 0 else "  [WARN] disconnect ret=%s" % r)

    # ── the one place a return code is judged ───────────────────────────
    def _check(self, label, ret):
        if ret == 0:
            return True
        print("  [FAIL] %s ret=%s: %s"
              % (label, ret, error_codes.describe_api2_return(ret)))
        return False

    def _await(self, label, block, timeout=90.0):
        """Wait for arrival when the call was non-blocking.

        With `block=1` the SDK already returned only after the move
        finished, so there is nothing to wait for. With `block=0` the
        arrival event is the only thing that says the move completed —
        and its ABSENCE is the signature of an abandoned chain, which is
        why a timeout here is a failure and not a warning.
        """
        if block or self.monitor is None:
            return True
        arrived, ok = self.monitor.wait(self.arm.handle_id, DEV_JOINT, timeout)
        if not arrived:
            print("  [FAIL] %s: no arrival event in %.0f s — the move was "
                  "abandoned, or it is still running." % (label, timeout))
            return False
        if not ok:
            print("  [FAIL] %s: arrival event reports failure" % label)
            return False
        return True

    def _expect(self, block):
        if not block and self.monitor is not None:
            self.monitor.expect(self.arm.handle_id, DEV_JOINT)

    # ── motions ─────────────────────────────────────────────────────────
    def movej(self, joint, v=ARM_SPEED_PCT, r=0, connect=0, block=0):
        """Joint-space move to `joint` (degrees, one per joint)."""
        self._expect(block)
        if not self._check("movej", self.robot.rm_movej(joint, v, r,
                                                        connect, block)):
            return False
        return self._await("movej", block)

    def movej_p(self, pose, v=ARM_SPEED_PCT, r=0, connect=0, block=0):
        """Reach a CARTESIAN target by interpolating in JOINT space.

        This is the right primitive for getting to the start pose: the tool
        does not travel a commanded straight line, so it cannot be dragged
        through anything on the way, and the controller is free to pick the
        redundancy resolution it likes.
        """
        self._expect(block)
        if not self._check("movej_p", self.robot.rm_movej_p(pose, v, r,
                                                            connect, block)):
            return False
        return self._await("movej_p", block)

    def movel(self, pose, v=ARM_SPEED_PCT, r=0, connect=0, block=0):
        """Straight line in TCP space to `pose`."""
        self._expect(block)
        if not self._check("movel", self.robot.rm_movel(pose, v, r,
                                                        connect, block)):
            return False
        return self._await("movel", block)

    # ── the cleaning path ───────────────────────────────────────────────
    def cleaning_path(self, poses, v=100, r=0, connect=1, block=0,
                      v_list=None, r_list=None, overrides=None,
                      primitive="movel", verbose=True):
        """Dispatch the cleaning sequence. `poses` includes the start pose.

        `poses[0]` is where the arm already is (the movej_p target), so the
        moves dispatched are poses[1:] — one per segment.

        `v` and `r` are the defaults for every move; `v_list` / `r_list`
        override them per move when the config supplies them. Move `i`
        governs the junction at `poses[i+1]`, because `rm_movel` blends at
        the vertex the move ENDS on — an off-by-one here shifts every blend
        by one corner, which is invisible in a plot.

        `primitive` selects `rm_movel` (Cartesian-linear between waypoints,
        the faithful mapping of what the planner intends) or `rm_moves`
        (spline through the points). They are different motions through the
        same waypoints, which is worth being able to compare on hardware.

        Returns (ok, program) where `program` is the [(v, r, connect)] list
        actually dispatched — recorded verbatim, so the analysis checks the
        controller against what was sent, not against a reconstruction.
        """
        n = len(poses) - 1
        if n < 1:
            print("  [FAIL] cleaning path needs at least 2 poses")
            return False, []
        program = build_program(n, v=v, r=r, connect=connect,
                                v_list=v_list, r_list=r_list,
                                overrides=overrides)
        if primitive == "movel":
            call = self.robot.rm_movel
        else:
            # `rm_moves` is in the SDK but NOT in the emulator, so a
            # `--primitive moves` run is hardware-only. Saying that beats an
            # AttributeError mid-dispatch, and beats silently substituting
            # movel — which would report a spline run that never happened.
            call = getattr(self.robot, "rm_moves", None)
            if call is None:
                print("  [FAIL] rm_moves is unavailable here. It exists in "
                      "the SDK but not in the emulator, so --primitive "
                      "moves is a HARDWARE-ONLY run; use --primitive movel "
                      "in SIM.")
                return False, []
        if verbose:
            print("  cleaning path: %d segments via rm_%s, "
                  "v %s, r %s, connect %s"
                  % (n, primitive,
                     "per-move" if v_list else "%d%%" % v,
                     "per-move" if r_list else "%d%%" % r,
                     "chained" if connect else "discrete"))
        t0 = time.perf_counter()
        for i, pose in enumerate(poses[1:]):
            mv, mr, mc = program[i]
            # An event is due only where a move actually closes: a chained
            # program produces ONE, from its final connect=0 segment.
            if mc == 0:
                self._expect(block)
            ret = call(list(pose), mv, mr, mc, block)
            if ret != 0:
                print("  [FAIL] segment %d (rm_%s, v=%d r=%d connect=%d) "
                      "ret=%s: %s"
                      % (i, primitive, mv, mr, mc, ret,
                         error_codes.describe_api2_return(ret)))
                if ret == 1 and mr:
                    print("         ret=1 on a blended move often means the "
                          "blend could not carry the speed step into this "
                          "corner. Lower r, or lower the speed.")
                return False, program
            if mc == 0 and not self._await("segment %d" % i, block):
                return False, program
        if verbose:
            print("  cleaning path completed in %.2f s"
                  % (time.perf_counter() - t0))
        return True, program


def build_program(n_moves, v=100, r=0, connect=1, v_list=None, r_list=None,
                  overrides=None):
    """The per-move (v %, r %, connect) tuples, built ONCE.

    Built here and recorded verbatim so there is no second place the
    per-move parameters are derived.

    Three sources, least specific first: the `v`/`r`/`connect` defaults,
    then whole-path `v_list` / `r_list`, then `overrides` — a
    {move_index: {v, r, connect}} map from a config's per-segment third
    element, which is the most specific and therefore wins.

    Rules:
      * the LAST move always closes the chain: r=0, connect=0;
      * a move with connect=0 cannot blend into anything, so its r is
        forced to 0 — including when an override asked for both;
      * a mid-chain r=0 with connect=1 is legal and deliberate — it asks
        the controller for a latch corner with no blend.
    """
    overrides = overrides or {}
    prog = []
    for i in range(n_moves):
        o = overrides.get(i, {})
        mv = int(o.get("v", v_list[i] if v_list else v))
        mr = int(o.get("r", o.get("blend_r", o.get("blend",
                                                   r_list[i] if r_list
                                                   else r))))
        mc = int(o.get("connect", connect))
        if i == n_moves - 1:
            mr, mc = 0, 0
        elif not mc:
            mr = 0
        prog.append((mv, mr, mc))
    return prog
