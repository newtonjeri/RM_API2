"""RealMan error codes, local and decodable — the thing the SDK does not ship.

WHY THIS EXISTS

`rm_get_current_arm_state()["err"]["err"]` returns bare DECIMAL integers.
Nothing in the SDK, its headers, or its Python wrapper decodes them. On
2026-08-10 `system 4109` cost two hardware sessions of guessing before the
table was found on the web; it means **0x100D, arm collision detected**,
which pointed straight at a tool frame whose payload centroid had been
written as 128 METRES.

Three DIFFERENT numbering schemes are in play and they must not be mixed:

  API2 RETURN VALUES    what an SDK call returns: 0, 1, -1 .. -6.
                        Small integers. `1` is not "error 1", it is
                        "the controller said false".
  SYSTEM / ARM CODES    what the CONTROLLER latches: 0x1001 .. 0x5009,
                        reported as decimal 4097 .. 20489.
  JOINT CODES           per joint, a BITMASK: 0x0004 = undervoltage,
                        0x4000 = Position Command Step Warning. Several
                        bits can be set at once, so decode bit by bit.

And a fourth case that is easy to get wrong: ~23 interfaces do NOT use the
API2 return values at all and document their own codes. Passing one of
their returns through the API2 decoder yields a confident wrong answer, so
`describe_api2_return()` refuses when told which function it came from.

SOURCES — refresh with `python3 update_error_codes.py`
  https://develop.realman-robotics.com/en/robot/apierrorList2/   (API2)
  https://develop.realman-robotics.com/robot/json/errorList/     (system,
                                                                  joint)
Generated content lives between the BEGIN/END markers below; hand-written
notes outside them survive an update.
"""

# --- GENERATED: api2_return  BEGIN ---
API2_RETURN = {
    0: ('Success.',
        '-'),
    1: ('The controller returns false, indicating that the parameters are sent incorrectly or the robotic arm state is wrong.',
        '- Validate JSON Command: ① Enable DEBUG logs for the API to capture the raw JSON data. ② Check JSON syntax: Ensure correct formatting of parentheses, quotes, commas, etc. (You can use a JSON validation tool). ③ Verify against the API documentation that parameter names, data types, and value ranges comply with the specifications. ④ After fixing the issues, resend the command and check if the controller returns a normal status code and business data.- Check Robot Arm Status: ① Check for real-time error messages in the robot arm controller or logs (such as hardware failures, over-limit conditions), and reset, calibrate, or troubleshoot hardware issues according to the prompts. ② After fixing the issues, resend the command and check if the controller returns a normal status code and business data.'),
    -1: ('The data transmission fails, indicating that a problem occurs during the communication.',
        'Check Network Connectivity: Use tools like ping/telnet to check if the communication link with the controller is normal.'),
    -2: ('The data reception fails, indicating that a problem occurs during the communication, or the controller has a return timeout.',
        '- Check Network Connectivity: Use tools like ping/telnet to check if the communication link with the controller is normal. - Verify Version Compatibility: ① Check if the controller firmware version supports the current API functions. For specific version compatibility, refer to the Version Description. ② If the version is too low, upgrade the controller or use an API version that is compatible.- Call the ModbusTCP interface: Applicable only when reading from or writing to the ModbusTCP device of the controller. After creating the robot arm control handle, you must call the rm_set_modbustcp_mode() interface; otherwise, no return value will be received.'),
    -3: ('The return value parse fails, indicating that the received data format is incorrect or incomplete.',
        'Verify Version Compatibility: ① Check if the controller firmware version supports the current API functions. For specific version compatibility, refer to the Version Description. ② If the version is too low, upgrade the controller or use an API version that is compatible.'),
    -4: ('The current in-position equipment verification fails, indicating the current in-position equipment is not the joints/elevators/grippers/dexterous hands.',
        '- Detect Concurrent Control by Multiple Devices: Check if other devices are sending motion commands to the robot arm, including the motion of the robot arm, gripper, dexterous hand, and elevator. - Monitor Command Events in Real-Time: Register the callback function rm_get_arm_event_call_back: ① Capture device arrival events (such as motion completion, timeout, etc.). ② Determine the specific device type that triggered the event through the device parameter in the callback.'),
    -5: ('The single-thread mode does not receive a return value after the timeout, indicating that the timeout period may be improper.',
        "- Check Timeout Setting: In single-thread blocking mode, it supports configuring the timeout for waiting for the device to complete its motion. Ensure that the timeout is set longer than the device's motion time. - Check Network Connectivity: Use tools like ping/telnet to check if the communication link with the controller is normal."),
    -6: ('The robotic arm has stopped its motion planning due to an external stop command.',
        '- Check for External E-Stop Commands: Investigate whether there are any external calls to the E-Stop command, such as sending an E-Stop JSON protocol, triggering an IO E-Stop, or triggering the E-Stop on the teach pendant.'),
}
# --- GENERATED: api2_return  END ---

# --- GENERATED: system_err  BEGIN ---
SYSTEM_ERR = {
    0x0000: {
        "dec": 0,
        "zh": '系统正常。',
        "en": 'Normal system.',
        "effect": '-',
        "fix": 'You can use our robotic arm as normal.',
        "severity": '-',
    },
    0x1001: {
        "dec": 4097,
        "zh": '关节通信异常。',
        "en": 'Abnormal joint communication.',
        "effect": '-',
        "fix": 'Please contact our technical support promptly.',
        "severity": 'Severe',
    },
    0x1002: {
        "dec": 4098,
        "zh": '目标角度超过限位。',
        "en": 'Target angle out of limit.',
        "effect": 'The command sent exceeds the maximum angle limit of the robotic arm joints',
        "fix": 'Modify the sent command according to the angle limits of the robotic arm joints.',
        "severity": 'General',
    },
    0x1003: {
        "dec": 4099,
        "zh": '该处不可达，为奇异点。',
        "en": 'Point unreachable as a singularity.',
        "effect": 'Pose reaches the singularity point of the robotic arm',
        "fix": 'It is recommended to refer to the user manual for the explanation of singularity points and avoid singular regions.',
        "severity": 'General',
    },
    0x1004: {
        "dec": 4100,
        "zh": '实时内核通信错误。',
        "en": 'Real-time kernel communication error.',
        "effect": 'The hardware and software versions of the robotic arm do not match',
        "fix": 'It is recommended to contact our technical support promptly to re-flash the corresponding firmware.',
        "severity": 'Severe',
    },
    0x1005: {
        "dec": 4101,
        "zh": '关节通信总线错误。',
        "en": 'Joint communication bus error.',
        "effect": '-',
        "fix": 'Please contact our technical support promptly.',
        "severity": 'Severe',
    },
    0x1006: {
        "dec": 4102,
        "zh": '规划层内核错误。',
        "en": 'Planning layer kernel error.',
        "effect": 'The hardware and software versions of the robotic arm do not match',
        "fix": 'It is recommended to contact our technical support promptly to re-flash the corresponding firmware.',
        "severity": 'Severe',
    },
    0x1007: {
        "dec": 4103,
        "zh": '关节超速。',
        "en": 'Joint overspeed.',
        "effect": 'The robotic arm is moving too fast, causing joint overspeed',
        "fix": 'This usually occurs during transparency and requires reducing the planned trajectory speed.',
        "severity": 'General',
    },
    0x1008: {
        "dec": 4104,
        "zh": '末端接口板无法连接。',
        "en": 'End interface board disconnected.',
        "effect": '1. Check if the end light is always on.2. Check if there is voltage output from the end interface board.',
        "fix": '1. If the end light is always on, you can perform drag teaching. Connect the teach pendant and click on the expansion on the page. Turn on the end power output.2. If the end light is not on, you cannot perform drag teaching. Please contact our technical support promptly.',
        "severity": 'Severe',
    },
    0x1009: {
        "dec": 4105,
        "zh": '超速度限制。',
        "en": 'Speed out of limit.',
        "effect": "The robotic arm's movement speed exceeds the speed limit of the robotic arm",
        "fix": 'This usually occurs during transparency and requires reducing the planned trajectory speed.',
        "severity": 'General',
    },
    0x100A: {
        "dec": 4106,
        "zh": '超加速度限制。',
        "en": 'Acceleration out of limit.',
        "effect": "The robotic arm's movement speed exceeds the acceleration limit of the robotic arm",
        "fix": "This usually occurs during transparency and requires appropriately reducing the planned trajectory's acceleration.",
        "severity": 'General',
    },
    0x100B: {
        "dec": 4107,
        "zh": '关节抱闸未打开。',
        "en": 'Joint brake release failed.',
        "effect": 'The joint brake is not released, which is usually related to the startup current',
        "fix": 'Solution approach: 1. After power-on is complete, press and hold the green button at the end of the robotic arm for an extended period, observe whether the corresponding joint can be normally dragged.2. If it cannot be normally dragged, perform disable and enable operations on the abnormal joint in the security configuration interface.',
        "severity": 'General',
    },
    0x100C: {
        "dec": 4108,
        "zh": '拖动示教时超速。',
        "en": 'Overspeed during drag teaching.',
        "effect": 'Dragging the teach pendant too fast has caused this issue',
        "fix": 'Slow down the speed when dragging the robotic arm.',
        "severity": 'General',
    },
    0x100D: {
        "dec": 4109,
        "zh": '机械臂发生碰撞。',
        "en": 'Robotic arm collision.',
        "effect": 'The robotic arm stopped operating after a collision occurred',
        "fix": '1. Reduce the collision protection level.2. If there is a load at the end, it is necessary to re-establish the tool coordinate system.',
        "severity": 'General',
    },
    0x100E: {
        "dec": 4110,
        "zh": '无该工作坐标系。',
        "en": 'Null work frame.',
        "effect": '-',
        "fix": 'When running online programming, a non-existent work coordinate system was entered.',
        "severity": 'General',
    },
    0x100F: {
        "dec": 4111,
        "zh": '无该工具坐标系。',
        "en": 'Null tool frame.',
        "effect": '-',
        "fix": 'When running online programming, a non-existent work coordinate system was entered.',
        "severity": 'General',
    },
    0x1010: {
        "dec": 4112,
        "zh": '关节发生掉使能错误。',
        "en": 'Joint disabling error.',
        "effect": 'Take appropriate measures based on the specific situation at the time',
        "fix": 'Solution approach:1. Click on System Information in the teach pendant to clear the current error message, and then go to Configuration - Robot Arm Configuration - Security Configuration to select the corresponding joint and click on Enable.2. If the error message cannot be cleared or there are other issues, Please contact our technical support promptly.',
        "severity": 'General',
    },
    0x1011: {
        "dec": 4113,
        "zh": '圆弧规划错误。',
        "en": 'Arc planning error.',
        "effect": 'The circular arc motion trajectory includes the singularity area of the robotic arm',
        "fix": 'Reselect the waypoints.',
        "severity": 'General',
    },
    0x1012: {
        "dec": 4114,
        "zh": '自碰撞错误。',
        "en": 'Self-collision error.',
        "effect": 'After adding a load to the end of the robotic arm, it collides with the arm body, causing the robotic arm to stop moving',
        "fix": 'Choose reasonable waypoints to avoid the risk of self-collision.',
        "severity": 'General',
    },
    0x1013: {
        "dec": 4115,
        "zh": '碰撞到电子围栏错误。',
        "en": 'Electronic fence collision error.',
        "effect": 'The robotic arm stops moving after detecting a collision with the electronic fence',
        "fix": '1. Manually clear the error and move the robotic arm within/outside the set electronic fence.2. The range of the electronic fence can be adjusted as needed.',
        "severity": 'General',
    },
    0x1014: {
        "dec": 4116,
        "zh": '超关节软限位错误。',
        "en": 'Joint out of soft limit error.',
        "effect": 'Joint exceeds soft limit',
        "fix": 'Press and hold the green button at the end of the robotic arm to automatically recover within the limit range.',
        "severity": 'General',
    },
    0x1016: {
        "dec": 4118,
        "zh": '电流环拖动使能失败，关节处于限位附近。',
        "en": 'Current loop drag enable failed; the joint is near the limit position.',
        "effect": 'When enabling the current loop drag function, it is detected that the robotic arm joint has entered or is close to the software-set limit area.',
        "fix": 'First move the relevant joint out of the limit area to keep it away from the soft limit range, then try enabling the function again.',
        "severity": 'General',
    },
    0x1017: {
        "dec": 4119,
        "zh": '系统外受力数据校验失败。',
        "en": 'External force data verification failed.',
        "effect": 'External force detected exceeds the normal range before enabling force control or performing 6D force drag teaching.',
        "fix": 'Re-perform 6D Force Center of Gravity Calibration in Configuration – Robotic Arm Configuration – Force Sensor Configuration.',
        "severity": 'General',
    },
    0x1101: {
        "dec": 4353,
        "zh": '碰撞到虚拟墙错误。',
        "en": 'Collision with virtual wall error.',
        "effect": 'The robotic arm detects a collision with the virtual wall.',
        "fix": '1. Manually clear the error and move the robotic arm to within the set virtual wall.2. Adjust the range of the virtual wall as needed.',
        "severity": 'General',
    },
    0x2001: {
        "dec": 8193,
        "zh": '夹爪异常。',
        "en": 'Abnormal gripper.',
        "effect": '-',
        "fix": 'Use the upper computer software for the gripper to check the specific error code and message, and take corresponding action or contact our technical support.',
        "severity": 'General',
    },
    0x2002: {
        "dec": 8194,
        "zh": '灵巧手异常。',
        "en": 'Abnormal dexterous hand.',
        "effect": '-',
        "fix": 'Use the upper computer software for the dexterous hand to check the specific error code and message, and take corresponding action or contact our technical support.',
        "severity": 'General',
    },
    0x2003: {
        "dec": 8195,
        "zh": '六维力模块异常。',
        "en": 'Abnormal 6-DoF force module.',
        "effect": '-',
        "fix": 'Contact our technical support for factory repair and inspection.',
        "severity": 'Severe',
    },
    0x2004: {
        "dec": 8196,
        "zh": '一维力模块异常。',
        "en": 'Abnormal 1-DoF force module.',
        "effect": '-',
        "fix": 'Contact our technical support for factory repair and inspection.',
        "severity": 'Severe',
    },
    0x2005: {
        "dec": 8197,
        "zh": '输出电流异常。',
        "en": 'Abnormal output current.',
        "effect": 'The end effector might be jammed, increasing the demand for current, and the end effector continuously outputs a large current for an extended period, leading to overcurrent',
        "fix": 'Please contact our technical support for repair and handling.',
        "severity": 'Severe',
    },
    0x5001: {
        "dec": 20481,
        "zh": '预留。',
        "en": 'Reserved.',
        "effect": '-',
        "fix": '-',
        "severity": '-',
    },
    0x5002: {
        "dec": 20482,
        "zh": '预留。',
        "en": 'Reserved.',
        "effect": '-',
        "fix": '-',
        "severity": '-',
    },
    0x5003: {
        "dec": 20483,
        "zh": '控制器过温。',
        "en": 'Controller over-temperature.',
        "effect": 'The controller temperature has reached the alarm temperature',
        "fix": 'The controller temperature has reached the alarm level, please let it rest or power it off to cool down.',
        "severity": 'Severe',
    },
    0x5004: {
        "dec": 20484,
        "zh": '预留。',
        "en": 'Reserved.',
        "effect": '-',
        "fix": '-',
        "severity": '-',
    },
    0x5005: {
        "dec": 20485,
        "zh": '控制器过流。',
        "en": 'Controller overcurrent.',
        "effect": 'The controller current is too high',
        "fix": "It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. Please contact our technical support promptly.",
        "severity": 'Severe',
    },
    0x5006: {
        "dec": 20486,
        "zh": '控制器欠流。',
        "en": 'Controller undercurrent.',
        "effect": 'Controller current is too low',
        "fix": "It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. Please contact our technical support promptly.",
        "severity": 'Severe',
    },
    0x5007: {
        "dec": 20487,
        "zh": '控制器过压。',
        "en": 'Controller overvoltage.',
        "effect": "Input voltage exceeds the robotic arm's voltage tolerance range",
        "fix": 'Check if the voltage input is between 20-27V, the Gen 3 controller supports up to 30V.',
        "severity": 'Severe',
    },
    0x5008: {
        "dec": 20488,
        "zh": '控制器欠压。',
        "en": 'Controller undervoltage.',
        "effect": "Input voltage is below the robotic arm's voltage tolerance range",
        "fix": "It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. Please contact our technical support promptly.",
        "severity": 'Severe',
    },
    0x5009: {
        "dec": 20489,
        "zh": '实时层无法通讯。',
        "en": 'Real-time layer communication error.',
        "effect": 'The hardware and software versions of the robotic arm do not match',
        "fix": 'Please contact our technical support promptly to re-flash the corresponding firmware.',
        "severity": 'Severe',
    },
}
# --- GENERATED: system_err  END ---

# --- GENERATED: joint_err  BEGIN ---
JOINT_ERR_BITS = {
    0x0000: {
        "dec": 0,
        "zh": '关节正常。',
        "en": 'Normal joint.',
        "effect": '-',
        "fix": 'You can use our robotic arm normally.',
        "severity": '-',
    },
    0x0001: {
        "dec": 1,
        "zh": 'FOC错误。',
        "en": 'FOC error.',
        "effect": 'FOC frequency too high',
        "fix": 'Please contact our technical support promptly.',
        "severity": 'Severe',
    },
    0x0002: {
        "dec": 2,
        "zh": '过压。',
        "en": 'Overvoltage.',
        "effect": "Input voltage exceeds the joint's voltage rating",
        "fix": 'Check if the voltage input is between 20-27V.',
        "severity": 'General',
    },
    0x0004: {
        "dec": 4,
        "zh": '欠压。',
        "en": 'Undervoltage.',
        "effect": "Input voltage is below the joint's voltage rating range",
        "fix": '1. Commonly occurs when the battery power supply is insufficient, such as during low battery levels, when starting the joints, or during rapid movements under high load conditions.2. Check the power supply output. It is recommended to use a switch power supply with an output of 20-27V, over 600W of power, and with features like pulse mode and constant current output for 1 second, to allow the robotic arm to perform self-inspection and self-calibration after powering up.',
        "severity": 'General',
    },
    0x0008: {
        "dec": 8,
        "zh": '过温。',
        "en": 'Over-temperature.',
        "effect": 'The joint temperature reaches the alarm temperature.',
        "fix": '1. The joint temperature reaches the overtemperature threshold of 85°C, please rest or power off to cool down.2. It is recommended to check whether the joint is overloaded or has collided.',
        "severity": 'General',
    },
    0x0010: {
        "dec": 16,
        "zh": '启动失败。',
        "en": 'Start failed.',
        "effect": 'The joint did not start up normally',
        "fix": 'Check the power supply output power. It is recommended to use a switch power supply with an output of 20-27V, over 600W of power, and with features like pulse mode, constant current output for 1 second, for self-inspection and self-calibration after powering on the robotic arm.',
        "severity": 'General',
    },
    0x0020: {
        "dec": 32,
        "zh": '编码器错误。',
        "en": 'Encoder error.',
        "effect": 'The joint self-inspection fails or cannot recognize the current angle',
        "fix": '1. If multiple joints of the robotic arm show encoder errors, it is recommended to check the power supply output power and restart the robotic arm. The reason for this phenomenon is insufficient power supply output power, and the robotic arm cannot perform normal self-inspection and self-calibration.2. A single joint shows encoder error, you can try to clear it; if it cannot be cleared it may be due to excessive load, error reported after startup, it is recommended to reduce the load and power on again to solve.3. The encoder read position continuously deviates more than 1 degree from the last data (consider the flange position shift if there is no hard stop output).4. Unable to eliminate and the drive board lights up in blue (not visible externally), encoder fault.5. The target position sent by the upper machine exceeds the set limit position.',
        "severity": 'General',
    },
    0x0040: {
        "dec": 64,
        "zh": '过流。',
        "en": 'Overcurrent.',
        "effect": 'Instantaneous current is too high',
        "fix": '1. Check if the power supply and load are normal;2. Check if there is a large position command step causing excessive instantaneous current.',
        "severity": 'General',
    },
    0x0080: {
        "dec": 128,
        "zh": '软件错误。',
        "en": 'Software error.',
        "effect": '1. Hardware and software do not corresponding2. Current detection error',
        "fix": 'Please contact our technical support in time.Re-recording the corresponding firmware can solve the problem of hardware and software not corresponding.',
        "severity": 'Severe',
    },
    0x0100: {
        "dec": 256,
        "zh": '温度传感器错误。',
        "en": 'Temperature Sensor Error.',
        "effect": 'The temperature sensor fails to normally obtain the joint temperature',
        "fix": 'Please contact our technical support in time.',
        "severity": 'Severe',
    },
    0x0200: {
        "dec": 512,
        "zh": '位置超限错误。',
        "en": 'Position Limit Error.',
        "effect": 'The current position of the joint exceeds the joint limit',
        "fix": 'Solution ideas:1. Check the current joint angle on the single joint upper machine;2. Adjust the joint limit to be greater than the current angle;3. Clear the joint error.',
        "severity": 'General',
    },
    0x0400: {
        "dec": 1024,
        "zh": '关节ID非法。',
        "en": 'Invalid joint ID.',
        "effect": 'Incorrect joint ID input',
        "fix": 'Enter the correct joint ID information.The correct setting range for joint ID is 1-7, if the joint ID is less than 1 or greater than 7, this error will occur.',
        "severity": 'General',
    },
    0x0800: {
        "dec": 2048,
        "zh": '位置跟踪错误。',
        "en": 'Position Tracking Error.',
        "effect": '1. Position error tracking limit exceeded2. The difference between the target position and the current position exceeds the threshold value',
        "fix": 'The error angle is greater than the maximum angle following angle limit (the difference between the target motor position and the encoder read position is greater than 80 degrees).',
        "severity": 'General',
    },
    0x1000: {
        "dec": 4096,
        "zh": '电流检测错误。',
        "en": 'Current Detection Error.',
        "effect": 'Current sensor detection error when powering on',
        "fix": 'Please contact our technical support in time.Circuit board detects current beyond the correct range (most likely circuit board issue).',
        "severity": 'Severe',
    },
    0x2000: {
        "dec": 8192,
        "zh": '抱闸打开失败。',
        "en": 'Brake Release Failure.',
        "effect": '1. Joint brake did not release normally',
        "fix": "It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up.",
        "severity": 'General',
    },
    0x4000: {
        "dec": 16384,
        "zh": '位置指令阶跃警告。',
        "en": 'Position Command Step Jump Warning.',
        "effect": 'The current position and target position error is large',
        "fix": 'which often occurs during transmission. It is recommended to check whether the transmission trajectory is smooth and whether the communication cycle is stable.1. The target position sent by the upper machine and the current motor position angle is greater than 10 degrees.2. FOC status error.',
        "severity": 'General',
    },
    0x8000: {
        "dec": 32768,
        "zh": '多圈关节丢圈数。',
        "en": 'Multi-coil joint data loss.',
        "effect": 'Encoder battery is low on power',
        "fix": 'The end joint of the robotic arm uses a multi-turn encoder. Check whether the battery interface is loose and whether the battery power is insufficient.',
        "severity": 'Severe',
    },
}
JOINT_ERR_COMBINED = {
    0xF000: {
        "dec": 61440,
        "zh": '通信丢帧。',
        "en": 'Communication frame loss.',
        "effect": 'The joint cannot communicate normally',
        "fix": 'Clear joint errors on the single joint upper machine, check the CAN line connection and 120 ohm shunt resistor.',
        "severity": 'General',
    },
}
# --- GENERATED: joint_err  END ---

# --- GENERATED: special_interfaces  BEGIN ---
SPECIAL_INTERFACES = (
    "rm_algo_inverse_kinematics",
    "rm_algo_ikine_select_ik_solve",
    "rm_algo_ikine_check_joint_position_limit",
    "rm_algo_ikine_check_joint_velocity_limit",
    "rm_algo_calculate_arm_angle_from_config_rm75",
    "rm_algo_inverse_kinematics_rm75_for_arm_angle",
    "rm_algo_universal_singularity_analyse",
    "rm_algo_kin_robot_singularity_analyse",
    "rm_algo_safety_robot_self_collision_detection",
    "rm_save_trajectory",
    "rm_set_force_drag_mode",
    "rm_get_drag_teach_sensitivity",
    "rm_set_gripper_release",
    "rm_set_gripper_pick",
    "rm_set_gripper_pick_on",
    "rm_set_gripper_position",
    "rm_set_hand_posture",
    "rm_set_hand_seq",
    "rm_send_project",
    "rm_set_program_id_run",
    "rm_init",
    "rm_delete_robot_arm",
    "rm_get_robot_info",
)
# --- GENERATED: special_interfaces  END ---

SOURCES = {
    "api2_return": "https://develop.realman-robotics.com/en/robot/"
                   "apierrorList2/",
    # BOTH languages: the Chinese page is authoritative and complete,
    # the English one supplies the translations. Merged per code.
    "system_err": "https://develop.realman-robotics.com/robot/json/errorList/",
    "system_err_en": "https://develop.realman-robotics.com/en/robot/json/"
                     "errorList/",
    "joint_err": "https://develop.realman-robotics.com/robot/json/errorList/",
    "joint_err_en": "https://develop.realman-robotics.com/en/robot/json/"
                    "errorList/",
    "special_interfaces": "https://develop.realman-robotics.com/en/robot/"
                          "apierrorList2/",
}

# Codes THIS project has actually observed on hardware, and what they were.
# Kept by hand: it is the difference between a reference and a diagnosis.
SEEN_HERE = {
    ("system", 4109):
        "every execute_path abort, 2026-08-08 and 2026-08-10. Root cause "
        "was a tool frame whose payload centroid had been written 1000x too "
        "large (128 m), so the controller's torque model predicted a force "
        "the joints never produced and called it a collision.",
    ("system", 4098):
        "C15 pole/arm concurrency, together with joint 0x4000 on J2/J4/J6 "
        "— the F9 truncation.",
    ("joint", 0x0004):
        "J3-J7 on the left arm, 2026-08-10 16:56: five joints undervoltage, "
        "clear succeeded but enable was refused (ret=1).",
    ("joint", 0x4000):
        "Position Command Step Warning, seen on J2/J4/J6 with system 4098.",
    ("api2", 1):
        "returned by rm_set_joint_en_state when re-enabling an "
        "undervoltage joint — the arm state is wrong, so the enable is "
        "refused rather than the parameters being bad.",
}


def _label(rec) -> str:
    """One short human label from a record, English preferred."""
    if isinstance(rec, dict):
        return (rec.get("en") or rec.get("zh") or "").rstrip("。.") or "?"
    if isinstance(rec, (tuple, list)):
        return str(rec[-1] or rec[0])
    return str(rec)


def remedy(code, kind="system") -> str:
    """RealMan's OWN documented fix for a code, if the table carries one.

    Worth reading before theorising. For 0x100D it says: lower the
    collision protection level, and *if there is a load at the end,
    re-establish the tool coordinate system* — which is precisely what
    fixed it here on 2026-08-10.
    """
    n = _coerce(code)
    table = {"system": SYSTEM_ERR, "joint": JOINT_ERR_BITS}.get(kind, {})
    rec = table.get(n)
    if not isinstance(rec, dict):
        return ""
    bits = [b for b in (rec.get("effect"), rec.get("fix")) if b]
    return "  ".join(bits)


def _coerce(code):
    """Accept 4109, '4109', '0x100D' — the SDK is not consistent."""
    if isinstance(code, bool):
        return None
    if isinstance(code, int):
        return code
    try:
        return int(str(code).strip(), 0)
    except (TypeError, ValueError):
        return None


def describe_api2_return(code, func=None) -> str:
    """Decode an SDK call's return value.

    `func` is the interface it came from. If that interface is one of the
    ~23 that do NOT use these codes, say so instead of decoding — a
    confident wrong answer is worse than none.
    """
    if func:
        name = str(func).replace("()", "").strip()
        if name in SPECIAL_INTERFACES:
            return (f"{code} — {name}() does NOT use the API2 return "
                    "values; see its own interface documentation")
    n = _coerce(code)
    if n is None or n not in API2_RETURN:
        return f"{code} (not an API2 return value)"
    desc, fix = API2_RETURN[n]
    return f"{code} ({desc}{'  FIX: ' + fix if fix else ''})"


def describe_system_err(code) -> str:
    """'4109' -> '4109 (0x100D ARM COLLISION DETECTED)'."""
    n = _coerce(code)
    if n is None:
        return str(code)
    hit = SYSTEM_ERR.get(n)
    if not hit:
        return f"{code} (unknown system code)"
    return f"{code} (0x{n:04X} {_label(hit)})"


def describe_joint_err(flag) -> str:
    """A joint flag is a BITMASK: 16388 -> 'UNDERVOLTAGE|Position Command
    Step Warning'. Decoding it as a single value loses half the fault."""
    n = _coerce(flag)
    if n is None:
        return str(flag)
    if n == 0:
        return "0"
    combined = JOINT_ERR_COMBINED.get(n)
    if combined:
        return f"{flag} ({_label(combined)})"
    names = [_label(rec) for bit, rec in JOINT_ERR_BITS.items() if n & bit]
    left = n & ~sum(bit for bit in JOINT_ERR_BITS if n & bit)
    if left:
        names.append(f"unknown bits 0x{left:04X}")
    return f"{flag} ({'|'.join(names)})" if names else str(flag)


def explain(code, kind="system") -> str:
    """Full explanation, plus anything THIS hardware has shown for it.

    The note is keyed by (kind, code) on purpose: 4109 is a real system
    code and a meaningless joint bitmask, and attaching the same history
    to both is the very mixing this module exists to prevent.
    """
    n = _coerce(code)
    base = {"system": describe_system_err,
            "joint": describe_joint_err,
            "api2": describe_api2_return}[kind](code)
    note = SEEN_HERE.get((kind, n))
    return f"{base}\n            SEEN HERE: {note}" if note else base


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        for c in sys.argv[1:]:
            for kind in ("system", "joint", "api2"):
                print(f"  {kind:7s} {explain(c, kind)}")
            print()
    else:
        print(__doc__)
        print(f"  {len(API2_RETURN)} API2 return values")
        print(f"  {len(SYSTEM_ERR)} system/arm codes")
        print(f"  {len(JOINT_ERR_BITS)} joint bits")
        print(f"  {len(SPECIAL_INTERFACES)} interfaces with their own codes")
