# RealMan error codes

**Generated — do not hand-edit.** Refresh with `python3 src/update_error_codes.py --apply --md`.
Hand-written notes belong in `src/error_codes.py` (`SEEN_HERE`), which survives regeneration.

Sources:

- `api2_return` — <https://develop.realman-robotics.com/en/robot/apierrorList2/>
- `joint_err` — <https://develop.realman-robotics.com/robot/json/errorList/>
- `joint_err_en` — <https://develop.realman-robotics.com/en/robot/json/errorList/>
- `special_interfaces` — <https://develop.realman-robotics.com/en/robot/apierrorList2/>
- `system_err` — <https://develop.realman-robotics.com/robot/json/errorList/>
- `system_err_en` — <https://develop.realman-robotics.com/en/robot/json/errorList/>

Three schemes, never interchangeable: an SDK call's **return value**, the controller's latched **system code**, and a per-joint **bitmask**.

## API2 return values — what an SDK call returns

| code | meaning | handling |
|---|---|---|
| `0` | Success. | - |
| `1` | The controller returns false, indicating that the parameters are sent incorrectly or the robotic arm state is wrong. | - Validate JSON Command: ① Enable DEBUG logs for the API to capture the raw JSON data. ② Check JSON syntax: Ensure correct formatting of parentheses, quotes, commas, etc. (You can use a JSON validation tool). ③ Verify against the API documentation that parameter names, data types, and value ranges comply with the specifications. ④ After fixing the issues, resend the command and check if the controller returns a normal status code and business data.- Check Robot Arm Status: ① Check for real-time error messages in the robot arm controller or logs (such as hardware failures, over-limit conditions), and reset, calibrate, or troubleshoot hardware issues according to the prompts. ② After fixing the issues, resend the command and check if the controller returns a normal status code and business data. |
| `-1` | The data transmission fails, indicating that a problem occurs during the communication. | Check Network Connectivity: Use tools like ping/telnet to check if the communication link with the controller is normal. |
| `-2` | The data reception fails, indicating that a problem occurs during the communication, or the controller has a return timeout. | - Check Network Connectivity: Use tools like ping/telnet to check if the communication link with the controller is normal. - Verify Version Compatibility: ① Check if the controller firmware version supports the current API functions. For specific version compatibility, refer to the Version Description. ② If the version is too low, upgrade the controller or use an API version that is compatible.- Call the ModbusTCP interface: Applicable only when reading from or writing to the ModbusTCP device of the controller. After creating the robot arm control handle, you must call the rm_set_modbustcp_mode() interface; otherwise, no return value will be received. |
| `-3` | The return value parse fails, indicating that the received data format is incorrect or incomplete. | Verify Version Compatibility: ① Check if the controller firmware version supports the current API functions. For specific version compatibility, refer to the Version Description. ② If the version is too low, upgrade the controller or use an API version that is compatible. |
| `-4` | The current in-position equipment verification fails, indicating the current in-position equipment is not the joints/elevators/grippers/dexterous hands. | - Detect Concurrent Control by Multiple Devices: Check if other devices are sending motion commands to the robot arm, including the motion of the robot arm, gripper, dexterous hand, and elevator. - Monitor Command Events in Real-Time: Register the callback function rm_get_arm_event_call_back: ① Capture device arrival events (such as motion completion, timeout, etc.). ② Determine the specific device type that triggered the event through the device parameter in the callback. |
| `-5` | The single-thread mode does not receive a return value after the timeout, indicating that the timeout period may be improper. | - Check Timeout Setting: In single-thread blocking mode, it supports configuring the timeout for waiting for the device to complete its motion. Ensure that the timeout is set longer than the device's motion time. - Check Network Connectivity: Use tools like ping/telnet to check if the communication link with the controller is normal. |
| `-6` | The robotic arm has stopped its motion planning due to an external stop command. | - Check for External E-Stop Commands: Investigate whether there are any external calls to the E-Stop command, such as sending an E-Stop JSON protocol, triggering an IO E-Stop, or triggering the E-Stop on the teach pendant. |

## System / arm codes — `rm_get_current_arm_state()["err"]["err"]`

Reported as DECIMAL. The decimal column is what a log shows.
**Bold** rows are ones this project has actually hit.

| dec | hex | 中文 | English | effect | RealMan's remedy | sev |
|---|---|---|---|---|---|---|
| 0 | `0x0000` | 系统正常。 | Normal system. | - | You can use our robotic arm as normal. | - |
| 4097 | `0x1001` | 关节通信异常。 | Abnormal joint communication. | - | Please contact our technical support promptly. | Severe |
| 4098 | `0x1002` | 目标角度超过限位。 | **Target angle out of limit.** | The command sent exceeds the maximum angle limit of the robotic arm joints | Modify the sent command according to the angle limits of the robotic arm joints. | General |
| 4099 | `0x1003` | 该处不可达，为奇异点。 | Point unreachable as a singularity. | Pose reaches the singularity point of the robotic arm | It is recommended to refer to the user manual for the explanation of singularity points and avoid singular regions. | General |
| 4100 | `0x1004` | 实时内核通信错误。 | Real-time kernel communication error. | The hardware and software versions of the robotic arm do not match | It is recommended to contact our technical support promptly to re-flash the corresponding firmware. | Severe |
| 4101 | `0x1005` | 关节通信总线错误。 | Joint communication bus error. | - | Please contact our technical support promptly. | Severe |
| 4102 | `0x1006` | 规划层内核错误。 | Planning layer kernel error. | The hardware and software versions of the robotic arm do not match | It is recommended to contact our technical support promptly to re-flash the corresponding firmware. | Severe |
| 4103 | `0x1007` | 关节超速。 | Joint overspeed. | The robotic arm is moving too fast, causing joint overspeed | This usually occurs during transparency and requires reducing the planned trajectory speed. | General |
| 4104 | `0x1008` | 末端接口板无法连接。 | End interface board disconnected. | 1. Check if the end light is always on.2. Check if there is voltage output from the end interface board. | 1. If the end light is always on, you can perform drag teaching. Connect the teach pendant and click on the expansion on the page. Turn on the end power output.2. If the end light is not on, you cannot perform drag teaching. Please contact our technical support promptly. | Severe |
| 4105 | `0x1009` | 超速度限制。 | Speed out of limit. | The robotic arm's movement speed exceeds the speed limit of the robotic arm | This usually occurs during transparency and requires reducing the planned trajectory speed. | General |
| 4106 | `0x100A` | 超加速度限制。 | Acceleration out of limit. | The robotic arm's movement speed exceeds the acceleration limit of the robotic arm | This usually occurs during transparency and requires appropriately reducing the planned trajectory's acceleration. | General |
| 4107 | `0x100B` | 关节抱闸未打开。 | Joint brake release failed. | The joint brake is not released, which is usually related to the startup current | Solution approach: 1. After power-on is complete, press and hold the green button at the end of the robotic arm for an extended period, observe whether the corresponding joint can be normally dragged.2. If it cannot be normally dragged, perform disable and enable operations on the abnormal joint in the security configuration interface. | General |
| 4108 | `0x100C` | 拖动示教时超速。 | Overspeed during drag teaching. | Dragging the teach pendant too fast has caused this issue | Slow down the speed when dragging the robotic arm. | General |
| 4109 | `0x100D` | 机械臂发生碰撞。 | **Robotic arm collision.** | The robotic arm stopped operating after a collision occurred | 1. Reduce the collision protection level.2. If there is a load at the end, it is necessary to re-establish the tool coordinate system. | General |
| 4110 | `0x100E` | 无该工作坐标系。 | Null work frame. | - | When running online programming, a non-existent work coordinate system was entered. | General |
| 4111 | `0x100F` | 无该工具坐标系。 | Null tool frame. | - | When running online programming, a non-existent work coordinate system was entered. | General |
| 4112 | `0x1010` | 关节发生掉使能错误。 | Joint disabling error. | Take appropriate measures based on the specific situation at the time | Solution approach:1. Click on System Information in the teach pendant to clear the current error message, and then go to Configuration - Robot Arm Configuration - Security Configuration to select the corresponding joint and click on Enable.2. If the error message cannot be cleared or there are other issues, Please contact our technical support promptly. | General |
| 4113 | `0x1011` | 圆弧规划错误。 | Arc planning error. | The circular arc motion trajectory includes the singularity area of the robotic arm | Reselect the waypoints. | General |
| 4114 | `0x1012` | 自碰撞错误。 | Self-collision error. | After adding a load to the end of the robotic arm, it collides with the arm body, causing the robotic arm to stop moving | Choose reasonable waypoints to avoid the risk of self-collision. | General |
| 4115 | `0x1013` | 碰撞到电子围栏错误。 | Electronic fence collision error. | The robotic arm stops moving after detecting a collision with the electronic fence | 1. Manually clear the error and move the robotic arm within/outside the set electronic fence.2. The range of the electronic fence can be adjusted as needed. | General |
| 4116 | `0x1014` | 超关节软限位错误。 | Joint out of soft limit error. | Joint exceeds soft limit | Press and hold the green button at the end of the robotic arm to automatically recover within the limit range. | General |
| 4118 | `0x1016` | 电流环拖动使能失败，关节处于限位附近。 | Current loop drag enable failed; the joint is near the limit position. | When enabling the current loop drag function, it is detected that the robotic arm joint has entered or is close to the software-set limit area. | First move the relevant joint out of the limit area to keep it away from the soft limit range, then try enabling the function again. | General |
| 4119 | `0x1017` | 系统外受力数据校验失败。 | External force data verification failed. | External force detected exceeds the normal range before enabling force control or performing 6D force drag teaching. | Re-perform 6D Force Center of Gravity Calibration in Configuration – Robotic Arm Configuration – Force Sensor Configuration. | General |
| 4353 | `0x1101` | 碰撞到虚拟墙错误。 | Collision with virtual wall error. | The robotic arm detects a collision with the virtual wall. | 1. Manually clear the error and move the robotic arm to within the set virtual wall.2. Adjust the range of the virtual wall as needed. | General |
| 8193 | `0x2001` | 夹爪异常。 | Abnormal gripper. | - | Use the upper computer software for the gripper to check the specific error code and message, and take corresponding action or contact our technical support. | General |
| 8194 | `0x2002` | 灵巧手异常。 | Abnormal dexterous hand. | - | Use the upper computer software for the dexterous hand to check the specific error code and message, and take corresponding action or contact our technical support. | General |
| 8195 | `0x2003` | 六维力模块异常。 | Abnormal 6-DoF force module. | - | Contact our technical support for factory repair and inspection. | Severe |
| 8196 | `0x2004` | 一维力模块异常。 | Abnormal 1-DoF force module. | - | Contact our technical support for factory repair and inspection. | Severe |
| 8197 | `0x2005` | 输出电流异常。 | Abnormal output current. | The end effector might be jammed, increasing the demand for current, and the end effector continuously outputs a large current for an extended period, leading to overcurrent | Please contact our technical support for repair and handling. | Severe |
| 20481 | `0x5001` | 预留。 | Reserved. | - | - | - |
| 20482 | `0x5002` | 预留。 | Reserved. | - | - | - |
| 20483 | `0x5003` | 控制器过温。 | Controller over-temperature. | The controller temperature has reached the alarm temperature | The controller temperature has reached the alarm level, please let it rest or power it off to cool down. | Severe |
| 20484 | `0x5004` | 预留。 | Reserved. | - | - | - |
| 20485 | `0x5005` | 控制器过流。 | Controller overcurrent. | The controller current is too high | It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. Please contact our technical support promptly. | Severe |
| 20486 | `0x5006` | 控制器欠流。 | Controller undercurrent. | Controller current is too low | It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. Please contact our technical support promptly. | Severe |
| 20487 | `0x5007` | 控制器过压。 | Controller overvoltage. | Input voltage exceeds the robotic arm's voltage tolerance range | Check if the voltage input is between 20-27V, the Gen 3 controller supports up to 30V. | Severe |
| 20488 | `0x5008` | 控制器欠压。 | Controller undervoltage. | Input voltage is below the robotic arm's voltage tolerance range | It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. Please contact our technical support promptly. | Severe |
| 20489 | `0x5009` | 实时层无法通讯。 | Real-time layer communication error. | The hardware and software versions of the robotic arm do not match | Please contact our technical support promptly to re-flash the corresponding firmware. | Severe |

## Joint codes — `rm_get_joint_err_flag()`, a BITMASK

Several bits can be set at once; decode bit by bit.

| dec | hex | 中文 | English | effect | RealMan's remedy |
|---|---|---|---|---|---|
| 0 | `0x0000` | 关节正常。 | Normal joint. | - | You can use our robotic arm normally. |
| 1 | `0x0001` | FOC错误。 | FOC error. | FOC frequency too high | Please contact our technical support promptly. |
| 2 | `0x0002` | 过压。 | Overvoltage. | Input voltage exceeds the joint's voltage rating | Check if the voltage input is between 20-27V. |
| 4 | `0x0004` | 欠压。 | Undervoltage. | Input voltage is below the joint's voltage rating range | 1. Commonly occurs when the battery power supply is insufficient, such as during low battery levels, when starting the joints, or during rapid movements under high load conditions.2. Check the power supply output. It is recommended to use a switch power supply with an output of 20-27V, over 600W of power, and with features like pulse mode and constant current output for 1 second, to allow the robotic arm to perform self-inspection and self-calibration after powering up. |
| 8 | `0x0008` | 过温。 | Over-temperature. | The joint temperature reaches the alarm temperature. | 1. The joint temperature reaches the overtemperature threshold of 85°C, please rest or power off to cool down.2. It is recommended to check whether the joint is overloaded or has collided. |
| 16 | `0x0010` | 启动失败。 | Start failed. | The joint did not start up normally | Check the power supply output power. It is recommended to use a switch power supply with an output of 20-27V, over 600W of power, and with features like pulse mode, constant current output for 1 second, for self-inspection and self-calibration after powering on the robotic arm. |
| 32 | `0x0020` | 编码器错误。 | Encoder error. | The joint self-inspection fails or cannot recognize the current angle | 1. If multiple joints of the robotic arm show encoder errors, it is recommended to check the power supply output power and restart the robotic arm. The reason for this phenomenon is insufficient power supply output power, and the robotic arm cannot perform normal self-inspection and self-calibration.2. A single joint shows encoder error, you can try to clear it; if it cannot be cleared it may be due to excessive load, error reported after startup, it is recommended to reduce the load and power on again to solve.3. The encoder read position continuously deviates more than 1 degree from the last data (consider the flange position shift if there is no hard stop output).4. Unable to eliminate and the drive board lights up in blue (not visible externally), encoder fault.5. The target position sent by the upper machine exceeds the set limit position. |
| 64 | `0x0040` | 过流。 | Overcurrent. | Instantaneous current is too high | 1. Check if the power supply and load are normal;2. Check if there is a large position command step causing excessive instantaneous current. |
| 128 | `0x0080` | 软件错误。 | Software error. | 1. Hardware and software do not corresponding2. Current detection error | Please contact our technical support in time.Re-recording the corresponding firmware can solve the problem of hardware and software not corresponding. |
| 256 | `0x0100` | 温度传感器错误。 | Temperature Sensor Error. | The temperature sensor fails to normally obtain the joint temperature | Please contact our technical support in time. |
| 512 | `0x0200` | 位置超限错误。 | Position Limit Error. | The current position of the joint exceeds the joint limit | Solution ideas:1. Check the current joint angle on the single joint upper machine;2. Adjust the joint limit to be greater than the current angle;3. Clear the joint error. |
| 1024 | `0x0400` | 关节ID非法。 | Invalid joint ID. | Incorrect joint ID input | Enter the correct joint ID information.The correct setting range for joint ID is 1-7, if the joint ID is less than 1 or greater than 7, this error will occur. |
| 2048 | `0x0800` | 位置跟踪错误。 | Position Tracking Error. | 1. Position error tracking limit exceeded2. The difference between the target position and the current position exceeds the threshold value | The error angle is greater than the maximum angle following angle limit (the difference between the target motor position and the encoder read position is greater than 80 degrees). |
| 4096 | `0x1000` | 电流检测错误。 | Current Detection Error. | Current sensor detection error when powering on | Please contact our technical support in time.Circuit board detects current beyond the correct range (most likely circuit board issue). |
| 8192 | `0x2000` | 抱闸打开失败。 | Brake Release Failure. | 1. Joint brake did not release normally | It is recommended to check the power supply output. It is suggested to use a switching power supply with an output of 20-27V, over 600W, and equipped with hiccup mode and constant current output for 1 second, for the robotic arm's self-inspection and self-calibration after powering up. |
| 16384 | `0x4000` | 位置指令阶跃警告。 | Position Command Step Jump Warning. | The current position and target position error is large | which often occurs during transmission. It is recommended to check whether the transmission trajectory is smooth and whether the communication cycle is stable.1. The target position sent by the upper machine and the current motor position angle is greater than 10 degrees.2. FOC status error. |
| 32768 | `0x8000` | 多圈关节丢圈数。 | Multi-coil joint data loss. | Encoder battery is low on power | The end joint of the robotic arm uses a multi-turn encoder. Check whether the battery interface is loose and whether the battery power is insufficient. |
| 61440 | `0xF000` | 通信丢帧。 | Communication frame loss. | The joint cannot communicate normally | Clear joint errors on the single joint upper machine, check the CAN line connection and 120 ohm shunt resistor. |

## Interfaces that do NOT use the API2 return values

Each documents its own codes. Passing one of their returns through the API2 decoder gives a confident wrong answer, which is why `describe_api2_return(code, func=...)` refuses.

- `rm_algo_inverse_kinematics()`
- `rm_algo_ikine_select_ik_solve()`
- `rm_algo_ikine_check_joint_position_limit()`
- `rm_algo_ikine_check_joint_velocity_limit()`
- `rm_algo_calculate_arm_angle_from_config_rm75()`
- `rm_algo_inverse_kinematics_rm75_for_arm_angle()`
- `rm_algo_universal_singularity_analyse()`
- `rm_algo_kin_robot_singularity_analyse()`
- `rm_algo_safety_robot_self_collision_detection()`
- `rm_save_trajectory()`
- `rm_set_force_drag_mode()`
- `rm_get_drag_teach_sensitivity()`
- `rm_set_gripper_release()`
- `rm_set_gripper_pick()`
- `rm_set_gripper_pick_on()`
- `rm_set_gripper_position()`
- `rm_set_hand_posture()`
- `rm_set_hand_seq()`
- `rm_send_project()`
- `rm_set_program_id_run()`
- `rm_init()`
- `rm_delete_robot_arm()`
- `rm_get_robot_info()`

## Codes this project has actually hit

| scheme | code | what it was |
|---|---|---|
| api2 | `1` (`0x0001`) | returned by rm_set_joint_en_state when re-enabling an undervoltage joint — the arm state is wrong, so the enable is refused rather than the parameters being bad. |
| joint | `4` (`0x0004`) | J3-J7 on the left arm, 2026-08-10 16:56: five joints undervoltage, clear succeeded but enable was refused (ret=1). |
| joint | `16384` (`0x4000`) | Position Command Step Warning, seen on J2/J4/J6 with system 4098. |
| system | `4098` (`0x1002`) | C15 pole/arm concurrency, together with joint 0x4000 on J2/J4/J6 — the F9 truncation. |
| system | `4109` (`0x100D`) | every execute_path abort, 2026-08-08 and 2026-08-10. Root cause was a tool frame whose payload centroid had been written 1000x too large (128 m), so the controller's torque model predicted a force the joints never produced and called it a collision. |
