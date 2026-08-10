"""Record what the arm ACTUALLY did during a task run, for 1:1 comparison.

Every diagnosis this week was reconstructed hours later from screenshots
and log archaeology. The controller streams the answer at 100 Hz; this
records it, marked with the stage boundaries, so a run explains itself.

WHAT IS RECORDED, AND WHAT IS NOT

    arm    UDP realtime push — position, speed, current, voltage,
           temperature, enable, error, per joint
    pole   UDP `liftState` — height, pos, current, err, en. Same frame as
           the arm, so no cross-device alignment is needed
    pose   UDP `waypoint` — the CONTROLLER's own TCP pose. This is the
           series to compare against the commanded movel poses: it needs
           no FK of ours, and our own FK is where several of this week's
           errors lived
    hand   NOT RECORDED. `bench_udp_fields` (butterfli_hw, both arms, with
           hand=1 explicitly requested) measured `handState` ALL-ZERO —
           the hand does not report on the arm's UDP line. Modbus
           `ANGLE_ACT` at 2-5 Hz is the only feedback path, and the bench
           is explicit that `rm_set_hand_angle` must NEVER be called while
           modbus mode is active — which our dispatcher does. Recording
           the hand therefore means moving hand COMMANDS to ANGLE_SET too.
           (The bench ran on fw < 1.7.3 and its own note says handState
           needs >= 1.7.3; both arms are now V1.7.4, so this is worth
           re-checking rather than assuming.)

RATE. `cycle` is in 5 ms units and the mapping is measured, not inferred:
bench_udp_fields at cycle=2 recorded mean arrival 10.001 ms (p99 11.06,
max 11.19 — about +/-1 ms of jitter, which bounds how tightly stage marks
can be aligned). Default here is cycle=2 = 100 Hz (Newton); cycle=1 gives
200 Hz if a run ever needs finer.

OUTPUT — one directory per run:

    runs/<run_id>/stream.csv    the samples, one row per frame
    runs/<run_id>/run.json      metadata, the COMMANDED program, and the
                                per-stage marks — written in the same
                                human-readable style as the MoveIt plans
                                in ../plans (2-space indent, %.6f floats,
                                scalar arrays inline)

`t_mono` is one monotonic clock shared by the stream and the stage marks,
zeroed when recording starts, so "where did stage X begin and end" is a
lookup rather than a correlation problem.
"""

import csv
import math
import os
import pathlib
import sys
import threading
import time

RUNS_DIR = pathlib.Path(__file__).resolve().parent.parent / "runs"
DEFAULT_CYCLE = 2                       # x5 ms -> 100 Hz (Newton)

_JOINT_FIELDS = ("joint_position", "joint_speed", "joint_current",
                 "joint_voltage", "joint_temperature")


# ── JSON in the same style as the saved MoveIt plans ────────────────────
def _fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return f"{v:.6f}"
    if isinstance(v, int):
        return str(v)
    if v is None:
        return "null"
    s = str(v).replace("\\", "\\\\").replace('"', '\\"')
    return f'"{s}"'


def _is_scalar_list(v):
    return isinstance(v, (list, tuple)) and all(
        isinstance(x, (int, float, str, bool)) or x is None for x in v)


def dumps(obj, indent=0):
    """Plan-style JSON: 2-space indent, %.6f floats, scalar arrays INLINE.

    `json.dumps(indent=2)` puts every array element on its own line, which
    turns a 44-pose program into 300 lines and a joint_names list into 7.
    The saved plans keep scalar arrays on one line and that is what makes
    them readable — match it so a run and a plan can be diffed side by side.
    """
    pad, pad2 = " " * indent, " " * (indent + 2)
    if isinstance(obj, dict):
        if not obj:
            return "{}"
        items = [f'{pad2}"{k}": {dumps(v, indent + 2)}' for k, v in obj.items()]
        return "{\n" + ",\n".join(items) + "\n" + pad + "}"
    if isinstance(obj, (list, tuple)):
        if not obj:
            return "[]"
        if _is_scalar_list(obj):
            return "[" + ", ".join(_fmt(x) for x in obj) + "]"
        items = [pad2 + dumps(v, indent + 2) for v in obj]
        return "[\n" + ",\n".join(items) + "\n" + pad + "]"
    return _fmt(obj)


class RunRecorder:
    """Captures the UDP stream for one task run, marked by stage.

    The realtime callback is PROCESS-GLOBAL in the SDK, so only one
    recorder can be active per process — `start()` says so if a second is
    attempted rather than silently stealing the stream.
    """

    _active = None

    def __init__(self, robot, task, side, host_ip, udp_port,
                 cycle=DEFAULT_CYCLE, out_dir=None):
        self.robot = robot
        self.task = task
        self.side = side
        self.host_ip = host_ip
        self.udp_port = udp_port
        self.cycle = int(cycle)
        self.run_id = f"{time.strftime('%Y%m%dT%H%M%S')}_{task}_{side}"
        # EMULATED output never lands in runs/. That directory holds
        # recorded HARDWARE data and is committed as such; an emulated
        # suite run writing beside it is the same mistake as the emulated
        # C11 capture overwriting the real one (2026-08-08). Auto-redirect
        # rather than rely on every caller remembering.
        self.emulated = "rm_emulator" in sys.modules
        if out_dir:
            self.dir = pathlib.Path(out_dir)
        elif self.emulated:
            self.dir = (pathlib.Path(os.environ.get("TMPDIR", "/tmp"))
                        / "emu_runs" / self.run_id)
        else:
            self.dir = RUNS_DIR / self.run_id
        self._lock = threading.Lock()
        self._rows = []
        self._dropped = 0
        self._first_error = None
        self._t0 = None
        self.stages = []
        self.meta = {}
        self._cb = None

    # ── capture ──
    def _on_state(self, data):
        t = time.perf_counter()
        try:
            js = data.joint_status
            row = [t - self._t0]
            for f in _JOINT_FIELDS:
                row += [float(x) for x in getattr(js, f)]
            row += [int(bool(x)) for x in getattr(js, "joint_en_flag",
                                                  [0] * 7)]
            row += [int(x) for x in getattr(js, "joint_err_code", [0] * 7)]
            ls = getattr(data, "liftState", None)
            row += [int(getattr(ls, "height", 0)), float(getattr(ls, "pos", 0.0)),
                    int(getattr(ls, "current", 0)), int(getattr(ls, "err_flag", 0)),
                    int(getattr(ls, "en_flag", 0))] if ls else [0, 0.0, 0, 0, 0]
            wp = getattr(data, "waypoint", None)
            if wp is not None:
                row += [float(wp.position.x), float(wp.position.y),
                        float(wp.position.z), float(wp.euler.rx),
                        float(wp.euler.ry), float(wp.euler.rz)]
            else:
                row += [0.0] * 6
            row.append(int(getattr(data, "arm_current_status", 0)))
        except Exception as exc:
            # A recorder that silently records nothing is worse than one
            # that errors: count the drops and surface the FIRST reason at
            # stop(), so "0 samples" is never a mystery.
            with self._lock:
                self._dropped += 1
                if self._first_error is None:
                    self._first_error = repr(exc)
            return
        with self._lock:
            self._rows.append(row)

    def header(self):
        h = ["t_mono"]
        for f in _JOINT_FIELDS:
            h += [f"{f[6:]}{i}" for i in range(1, 8)]
        h += [f"en{i}" for i in range(1, 8)]
        h += [f"err{i}" for i in range(1, 8)]
        h += ["lift_height", "lift_pos", "lift_current", "lift_err", "lift_en"]
        h += ["tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"]
        h += ["arm_status"]
        return h

    def start(self):
        from Robotic_Arm.rm_ctypes_wrap import (
            rm_realtime_arm_state_callback_ptr, rm_realtime_push_config_t,
            rm_udp_custom_config_t)
        if RunRecorder._active is not None:
            print("  [WARN] a recorder is already active in this process — "
                  "the realtime callback is global; not starting a second")
            return False
        self._t0 = time.perf_counter()
        self._cb = rm_realtime_arm_state_callback_ptr(self._on_state)
        self.robot.rm_realtime_arm_state_call_back(self._cb)
        # hand_state deliberately NOT requested: measured ALL-ZERO (see
        # the module docstring). lift_state IS populated and rides the
        # same frame as the arm.
        ret = self.robot.rm_set_realtime_push(rm_realtime_push_config_t(
            cycle=self.cycle, enable=True, port=self.udp_port,
            ip=self.host_ip,
            custom_config=rm_udp_custom_config_t(
                joint_speed=1, lift_state=1, arm_current_status=1)))
        RunRecorder._active = self
        print(f"  [INFO] recording -> {self.dir.name}  "
              f"({1000.0 / (self.cycle * 5):.0f} Hz, ret={ret})")
        return True

    def _speed_report(self, rows):
        """What speed did the arm ACTUALLY sustain, against what we asked?

        Added 2026-08-10. The first complete task run took 62 s for a
        4.8 m path — 46% of what the arm's own configured limits allow —
        and the cause was the teach pendant's GLOBAL SPEED OVERRIDE, left
        at 39%. That slider scales every motion the controller executes
        and there is NO SDK getter or setter for it (checked against the
        whole `rm_robot_interface` surface), so a run cannot read its own
        speed scaling. It can, however, MEASURE what it achieved — and a
        measured 43% of cap is the same evidence, visible at the time
        instead of a week later.

        Returns None when the stream is too short to be meaningful.
        """
        h = self.header()
        try:
            ix, iy, iz = (h.index("tcp_x"), h.index("tcp_y"), h.index("tcp_z"))
        except ValueError:
            return None
        if len(rows) < 60:
            return None
        # Scope to the LONGEST arm stage — in practice `execute_path`.
        # A whole-run figure mixes the movej transits, whose speed is a
        # percentage of the JOINT cap, with the movel path, whose speed is
        # a percentage of the LINE cap; comparing that mixture against the
        # line cap overstates the achieved fraction (67% vs the movel
        # stage's real 43% on the 2026-08-10 run).
        arm = [st for st in self.stages if st.get("device") == "arm"]
        span = None
        if arm:
            longest = max(arm, key=lambda st: st["t_end"] - st["t_start"])
            span = (longest["t_start"], longest["t_end"])
            rows = [r for r in rows if span[0] <= r[0] <= span[1]]
            if len(rows) < 60:
                return None
        sp = []
        for i in range(1, len(rows)):
            dt = rows[i][0] - rows[i - 1][0]
            d = math.dist((rows[i][ix], rows[i][iy], rows[i][iz]),
                          (rows[i - 1][ix], rows[i - 1][iy], rows[i - 1][iz]))
            # a tool-frame change reports as a metre-scale jump with the
            # joints unchanged; it is not motion, so drop it
            if dt > 0 and d < 0.05:
                sp.append(d / dt)
        if len(sp) < 60:
            return None
        # TYPICAL CRUISE, not peak and not a windowed minimum. A peak can
        # be one noisy sample; a windowed minimum on a stop-start path
        # lands inside a corner deceleration. Smooth 50 ms, drop the
        # near-stationary samples (corners, dwell), take the median of
        # what is left: "how fast does it go when it is going".
        k = 2                                   # +/-2 samples = 50 ms
        sm = [sum(sp[max(0, i - k):i + k + 1]) /
              len(sp[max(0, i - k):i + k + 1]) for i in range(len(sp))]
        cap = self.meta.get("line_speed_cap_m_s")
        floor = 0.1 * float(cap) if cap else 0.005
        moving = sorted(x for x in sm if x > floor)
        if len(moving) < 30:
            return None
        typical = moving[len(moving) // 2]
        out = {"typical_mm_s": typical * 1000.0,
               "p95_mm_s": moving[int(len(moving) * 0.95)] * 1000.0,
               "moving_fraction": len(moving) / len(sm),
               "measured_over": (longest["stage_name"] if arm
                                 else "whole run")}
        if cap:
            out["cap_mm_s"] = float(cap) * 1000.0
            out["pct_of_cap"] = typical / float(cap) * 100.0
        return out

    def mark(self, stage, device, t_start, t_end, ok, detail):
        """Record a stage span on the SAME clock as the stream."""
        self.stages.append({
            "index": len(self.stages),
            "stage_name": stage,
            "device": device,
            "t_start": float(t_start - self._t0) if self._t0 else 0.0,
            "t_end": float(t_end - self._t0) if self._t0 else 0.0,
            "ok": bool(ok) if ok is not None else None,
            "detail": str(detail),
        })

    def now(self):
        return time.perf_counter()

    def stop(self, extra=None):
        from Robotic_Arm.rm_ctypes_wrap import rm_realtime_push_config_t
        try:
            self.robot.rm_set_realtime_push(rm_realtime_push_config_t(
                cycle=self.cycle, enable=False, port=self.udp_port,
                ip=self.host_ip))
        except Exception:
            pass
        RunRecorder._active = None
        with self._lock:
            rows = list(self._rows)
        self.dir.mkdir(parents=True, exist_ok=True)
        with (self.dir / "stream.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.header())
            for r in rows:
                w.writerow([f"{r[0]:.6f}"] + [
                    f"{x:.6f}" if isinstance(x, float) else str(x)
                    for x in r[1:]])
        dur = rows[-1][0] if rows else 0.0
        speed = self._speed_report(rows)
        meta = {
            "run_id": self.run_id,
            "task_name": self.task,
            "side": self.side,
            "timestamp_human": time.strftime("%Y-%m-%d %H:%M:%S"),
            "stream_file": "stream.csv",
            "udp_cycle": self.cycle,
            "sample_rate_hz": 1000.0 / (self.cycle * 5),
            "num_samples": len(rows),
            "duration_s": float(dur),
            "measured_rate_hz": float(len(rows) / dur) if dur > 0 else 0.0,
            "frames_dropped": int(self._dropped),
            "first_parse_error": self._first_error,
            # Provenance: an emulated run must never be mistaken for a
            # hardware one when the data is read back weeks later.
            "sdk": "emulated" if self.emulated else "hardware",
            "hand_recorded": False,
            "hand_note": ("handState measured ALL-ZERO on the arm UDP line "
                          "(bench_udp_fields, both arms) — Modbus ANGLE_ACT "
                          "is the hand feedback path"),
        }
        meta.update(self.meta)
        if speed:
            meta["speed_achieved"] = speed
        meta["num_stages"] = len(self.stages)
        meta["stages"] = self.stages
        if extra:
            meta.update(extra)
        (self.dir / "run.json").write_text(dumps(meta) + "\n")
        if self._dropped:
            print(f"  [WARN] {self._dropped} frames DROPPED while parsing; "
                  f"first error: {self._first_error}")
        if not rows:
            print("  [WARN] NO samples recorded — check that the push "
                  f"target {self.host_ip}:{self.udp_port} is this machine "
                  "and that the callback stayed registered")
        if speed and speed.get("pct_of_cap") is not None \
                and speed["pct_of_cap"] < 80:
            print(f"  [WARN] the arm reached only "
                  f"{speed['typical_mm_s']:.0f} mm/s typical against a "
                  f"commanded cap of {speed['cap_mm_s']:.0f} mm/s "
                  f"({speed['pct_of_cap']:.0f}%). The pendant has a GLOBAL "
                  "speed override slider that scales everything and that "
                  "the SDK cannot read or set — check it is at 100% before "
                  "trusting any timing from this run.")
        print(f"  [INFO] recorded {len(rows)} samples "
              f"({meta['measured_rate_hz']:.1f} Hz measured) -> "
              f"{self.dir}")
        return self.dir
