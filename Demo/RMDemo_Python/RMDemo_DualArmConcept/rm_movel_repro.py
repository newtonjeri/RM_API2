#!/usr/bin/env python3
"""Chained rm_movel stops with no error at line_speed 0.6; same path is fine at 0.5.

    python3 rm_movel_repro.py --ip 192.168.1.10 --sdk /opt/RM_API2/Python --tool my_tool

RM75-6FB, firmware V1.7.4, C API 1.1.6. Free space, no contact.
The arm moves. Keep the emergency stop in hand.
"""
import argparse
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--ip", required=True)
ap.add_argument("--sdk", required=True, help="path to the RM_API2 Python package")
ap.add_argument("--tool", required=True, help="tool frame name on the controller")
ap.add_argument("--port", type=int, default=8080)
ap.add_argument("--speeds", default="0.5,0.6")
args = ap.parse_args()

sys.path.insert(0, args.sdk)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e, rm_event_callback_ptr

LINE_ACC = 3.6
ANGULAR_SPEED = 1.2
ANGULAR_ACC = 4.0
V_PERCENT = 100
BLEND_PCT = 10

START_JOINTS = [12.2797, -53.9484, -10.2032, 92.2149, -5.7711, 49.0807, -4.4358]

POSES = [
    [0.550000, -0.043700, -0.272500, 3.113933, 0.029752, 3.137781],
    [0.842500, -0.048200, -0.269500, 3.010080, 0.544077, 2.932834],
    [0.512500, -0.016700, -0.272000, 3.113732, 0.082113, 3.137769],
    [0.847500, -0.016700, -0.269000, 3.111692, 0.553364, 3.137114],
    [0.477500, 0.019800, -0.271500, 3.125841, 0.098548, -2.970108],
    [0.846000, 0.025300, -0.269500, 3.060536, 0.550935, 3.034690],
    [0.456500, 0.047800, -0.272000, 3.118310, 0.081930, -3.057894],
    [0.845000, 0.064300, -0.270000, -3.140741, 0.552676, -3.084570],
    [0.446000, 0.072300, -0.272000, 3.125813, 0.116213, -3.004911],
    [0.849000, 0.096800, -0.272000, 3.009247, 0.550625, 3.083359],
    [0.442500, 0.099800, -0.271500, 3.068965, 0.088626, -3.009812],
    [0.845500, 0.137800, -0.270500, 2.948047, 0.546742, 3.051432],
    [0.441000, 0.128300, -0.271500, 3.058218, -0.139311, -3.017115],
    [0.845500, 0.137800, -0.270500, 2.948047, 0.546742, 3.051432],
    [0.842500, -0.048200, -0.269500, 3.010080, 0.544077, 2.932834],
    [0.550000, -0.043700, -0.272500, 3.113933, 0.029752, 3.137781],
    [0.512500, -0.016700, -0.272000, 3.113732, 0.082113, 3.137769],
    [0.477500, 0.019800, -0.271500, 3.125841, 0.098548, -2.970108],
    [0.456500, 0.047800, -0.272000, 3.118310, 0.081930, -3.057894],
    [0.442500, 0.099800, -0.271500, 3.068965, 0.088626, -3.009812],
    [0.441000, 0.128300, -0.271500, 3.058218, -0.139311, -3.017115],
    [0.456500, 0.047800, -0.272000, 3.118310, 0.081930, -3.057894],
    [0.477500, 0.019800, -0.271500, 3.125841, 0.098548, -2.970108],
    [0.512500, -0.016700, -0.272000, 3.113732, 0.082113, 3.137769],
    [0.550000, -0.043700, -0.272500, 3.113933, 0.029752, 3.137781],
    [0.675500, -0.056200, -0.276000, -3.127307, 0.302303, 2.981742],
    [0.842500, -0.048200, -0.269500, 3.010080, 0.544077, 2.932834],
    [0.750000, -0.054200, -0.232500, 3.120276, 0.508383, 2.985947],
]


class Arrival:
    def __init__(self):
        self.events = []
        self.done = False
        self.state = None
        self.ptr = rm_event_callback_ptr(self._cb)

    def _cb(self, data):
        self.events.append((int(data.event_type), int(data.device),
                            int(data.trajectory_state),
                            int(data.trajectory_connect)))
        if int(data.event_type) == 1 and int(data.device) == 0:
            self.state = int(data.trajectory_state)
            if int(data.trajectory_connect) == 0:
                self.done = True


def errors(robot):
    out = []
    ret, st = robot.rm_get_current_arm_state()
    if ret == 0:
        out += [str(c) for c in (st.get("err") or []) if str(c) not in ("0", "")]
    jd = robot.rm_get_joint_err_flag()
    if isinstance(jd, dict):
        out += ["J%d=%s" % (i + 1, f) for i, f in
                enumerate(jd.get("err_flag") or []) if f]
    return out


def run(robot, arrival, line_speed):
    print("\n=== line_speed %.2f  line_acc %.1f ===" % (line_speed, LINE_ACC))
    for fn, v in ((robot.rm_set_arm_max_line_acc, LINE_ACC),
                  (robot.rm_set_arm_max_line_speed, line_speed),
                  (robot.rm_set_arm_max_angular_acc, ANGULAR_ACC),
                  (robot.rm_set_arm_max_angular_speed, ANGULAR_SPEED)):
        assert fn(float(v)) == 0

    assert robot.rm_change_tool_frame(args.tool) == 0
    assert robot.rm_movej(START_JOINTS, 30, 0, 0, 1) == 0
    time.sleep(1.0)

    arrival.events.clear()
    arrival.done = False
    arrival.state = None

    n = len(POSES) - 1
    rejects = []
    t0 = time.time()
    for i in range(1, len(POSES)):
        last = i == n
        r = robot.rm_movel(POSES[i], V_PERCENT,
                           0 if last else BLEND_PCT,
                           0 if last else 1, 0)
        if r != 0:
            rejects.append((i, r))
    print("%d rm_movel queued, rejects: %s" % (n, rejects or "none"))

    deadline = time.time() + 120
    seen = set()
    while time.time() < deadline and not arrival.done:
        for e in errors(robot):
            seen.add(e)
        time.sleep(0.05)

    print("elapsed          %.1f s" % (time.time() - t0))
    print("arrival event    %s" % ("yes" if arrival.done else "NO"))
    print("trajectory_state %s" % arrival.state)
    print("events received  %s" % (arrival.events or "none"))
    print("error codes      %s" % (sorted(seen) or "none"))
    ret, st = robot.rm_get_current_arm_state()
    print("final joints     %s" % [round(float(v), 1) for v in st.get("joint", [])])
    return arrival.done


robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
h = robot.rm_create_robot_arm(args.ip, args.port, 3)
assert h is not None and h.id > 0, "connect failed"
arrival = Arrival()
robot.rm_get_arm_event_call_back(arrival.ptr)
robot.rm_set_arm_run_mode(1)

try:
    for s in [float(x) for x in args.speeds.split(",")]:
        ok = run(robot, arrival, s)
        if not ok:
            print("\nSTOPPED: motion ceased, no arrival event, no error code.")
            break
        time.sleep(2.0)
finally:
    robot.rm_set_arm_stop()
    robot.rm_delete_robot_arm()

print("\nQuestions and measurements: QUESTIONS_FOR_REALMAN.md")
