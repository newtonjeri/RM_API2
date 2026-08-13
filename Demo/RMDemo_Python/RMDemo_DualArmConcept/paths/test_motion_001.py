import time
from Robotic_Arm.rm_robot_interface import *

# ============================================================
# ROBOT CONNECTION
# ============================================================

ARM_IP = "192.168.1.10"
ARM_PORT = 8080


# ============================================================
# SIMULATION SWITCH
# ============================================================

# True  -> simulation mode
# False -> real robot mode

SIMULATION = False


# ============================================================
# MOTION SETTINGS
# ============================================================

DEFAULT_SPEED = 100

# Blend radius percentage
BLEND = 25

STARTPOSE_SPEED = 100


# ============================================================
# TCP SPEED LIMITS
# ============================================================

# These values are configured in the ARM controller/software.

TCP_LINEAR_VELOCITY = 0.60
TCP_LINEAR_ACCELERATION = 1.80

TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00


# ============================================================
# PER-SEGMENT SPEEDS
# ============================================================

SEGMENT_SPEEDS = {

    ("start", "1"): 100,

    ("1", "2"): 100,
    ("2", "3"): 100,
    ("3", "4"): 100,
    ("4", "5"): 100,
    ("5", "6"): 100,

    ("6", "1"): 100,

    ("4", "9"): 100,
    # 60 and 90 until 2026-08-12 -- the hand-found speeds that kept the
    # elbow inside its limit while point 8 carried the old tilt. With the
    # revised tilt both run at 100 on hardware.
    ("9", "8"): 100,
    ("8", "7"): 100,
    ("7", "6"): 100,
    ("6", "5"): 100,
    ("5", "4"): 100,
}


# ============================================================
# DIAGNOSTIC SETTINGS
# ============================================================

POSITION_TOLERANCE = 0.005       # meters
SEGMENT_TIMEOUT = 30             # seconds
POLL_INTERVAL = 0.1


# ============================================================
# START JOINT POSITION
# ============================================================

START_JOINTS = [
    -0.112,
    -100.501,
    6.424,
    67.532,
    -1.656,
    95.585,
    171.774,
]


# ============================================================
# CARTESIAN POSES
# ============================================================

POSES_MM = {

    "1": [
        861.572,
        -79.923,
        -323.628,
        3.024,
        -0.749,
        0.278,
    ],

    "2": [
        688.329,
        -79.923,
        -323.628,
        -3.046,
        -0.291,
        0.168,
    ],

    "3": [
        472.855,
        -79.923,
        -323.628,
        -3.112,
        0.017,
        0.158,
    ],

    "4": [
        472.855,
        -21.000,
        -323.628,
        -3.053,
        -0.162,
        -0.008,
    ],

    "5": [
        688.329,
        -21.000,
        -323.628,
        -3.090,
        -0.376,
        0.083,
    ],

    "6": [
        861.572,
        -21.000,
        -323.628,
        3.129,
        -0.679,
        0.112,
    ],

    "7": [
        861.572,
        52.660,
        -323.628,
        -3.108,
        -0.662,
        0.072,
    ],

    "8": [
        688.329,
        52.660,
        -323.628,
        -3.117,
        # -0.218 until 2026-08-12. At that tilt the tool stayed almost
        # rigid across 9->8, so the WRIST had to cover 202 of the 215 mm
        # itself and the elbow swept 73 deg -- 329 deg/s against J4's 225
        # limit (146%). That is why this row could not run at 100%.
        # -0.400 lets the glove tilt ~12.8 deg across the segment and walk
        # part of the distance, exactly as rows y=-79.9 and y=-21.0 always
        # did. Elbow demand drops to 55%, and 9->8 / 8->7 balance at
        # 55%/56% instead of 146%/9%. Verified on hardware at 100%.
        -0.400,
        0.077,
    ],

    "9": [
        472.855,
        52.660,
        -323.628,
        -3.111,
        -0.177,
        0.004,
    ],
    "stoppose": [
            472.855,
            52.660,
            -323.628,
            -3.111,
            -0.177,
            0.004,
        ],
}


# ============================================================
# CONVERT MM -> METERS
# ============================================================

POSES = {
    label: [
        values[0] / 1000.0,
        values[1] / 1000.0,
        values[2] / 1000.0,
        values[3],
        values[4],
        values[5],
    ]
    for label, values in POSES_MM.items()
}


# ============================================================
# CLEANING SEQUENCE
# ============================================================
SEQUENCE = [
    "startpose",

    # ---------- ROUND 1 ----------
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "1",
    "2",
    "3",
    "4",
    "9",
    "8",
    "7",
    "6",
    "5",
    "4",

    # ---------- ROUND 2 ----------
    "3",
    "5",
    "6",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "1",
    "2",
    "3",
    "4",
    "9",
    "8",
    "7",
    "6",
    "5",
    "4",

    # ---------- STOP ----------
    "stoppose"
]


# ============================================================
# FIND NEAREST POSE
# ============================================================

def nearest_pose_label(current):

    best_label = None
    best_distance = None

    for label, pose in POSES.items():

        dx = current[0] - pose[0]
        dy = current[1] - pose[1]
        dz = current[2] - pose[2]

        distance = (
            dx * dx +
            dy * dy +
            dz * dz
        ) ** 0.5

        if (
            best_distance is None
            or distance < best_distance
        ):
            best_label = label
            best_distance = distance

    return best_label, best_distance


# ============================================================
# GET SEGMENT SPEED
# ============================================================

def get_segment_speed(from_label, to_label):

    return SEGMENT_SPEEDS.get(
        (from_label, to_label),
        DEFAULT_SPEED
    )


# ============================================================
# WAIT FOR FINAL TARGET
# ============================================================

def wait_for_target(
    arm,
    target_label,
    from_label,
    to_label,
    timeout=SEGMENT_TIMEOUT
):

    target = POSES[target_label]

    deadline = time.time() + timeout

    last_print_time = 0

    while time.time() < deadline:

        ret, state = arm.rm_get_current_arm_state()

        if ret == 0:

            current = state.get("pose")

            if current is not None:

                # ----------------------------------------
                # Position error
                # ----------------------------------------

                distance = max(
                    abs(current[k] - target[k])
                    for k in range(3)
                )

                # ----------------------------------------
                # Nearest pose
                # ----------------------------------------

                nearest_label, nearest_distance = (
                    nearest_pose_label(current)
                )

                # ----------------------------------------
                # Print every 1 second
                # ----------------------------------------

                now = time.time()

                if now - last_print_time >= 1.0:

                    print(
                        f"    current near {nearest_label} "
                        f"(dist={nearest_distance:.4f} m) "
                        f"target={target_label} "
                        f"(target_error={distance:.4f} m)"
                    )

                    print(
                        f"    pose={current}"
                    )

                    last_print_time = now

                # ----------------------------------------
                # Target reached
                # ----------------------------------------

                if distance <= POSITION_TOLERANCE:

                    print(
                        f"    ARRIVED at pose{target_label}"
                    )

                    print(
                        f"    final pose={current}"
                    )

                    return True

        time.sleep(POLL_INTERVAL)


    # ====================================================
    # TIMEOUT
    # ====================================================

    print(
        f"\n    TIMEOUT: "
        f"{from_label} -> {to_label}"
    )

    ret, state = arm.rm_get_current_arm_state()

    if ret == 0:

        current = state.get("pose")
        err = state.get("err")

        print(
            "    Current pose:",
            current
        )

        print(
            "    Error field:",
            err
        )

        if current is not None:

            nearest_label, nearest_distance = (
                nearest_pose_label(current)
            )

            print(
                f"    Nearest pose: "
                f"{nearest_label} "
                f"(distance={nearest_distance:.4f} m)"
            )

    else:

        print(
            "    Could not read arm state."
        )

    return False


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # Connect
    # --------------------------------------------------------

    arm = RoboticArm(
        rm_thread_mode_e.RM_TRIPLE_MODE_E
    )

    handle = arm.rm_create_robot_arm(
        ARM_IP,
        ARM_PORT
    )

    print(
        "Connected. Arm handle ID:",
        handle.id
    )


    # --------------------------------------------------------
    # Set simulation / real mode
    # --------------------------------------------------------

    mode_ret = arm.rm_set_arm_run_mode(
        0 if SIMULATION else 1
    )

    print(
        "rm_set_arm_run_mode result "
        "(0 = success):",
        mode_ret
    )


    check_ret, current_mode = (
        arm.rm_get_arm_run_mode()
    )


    if current_mode == 0:

        mode_label = (
            "SIMULATION "
            "(arm will NOT physically move)"
        )

    else:

        mode_label = (
            "REAL "
            "(arm WILL physically move)"
        )


    print(
        f"Confirmed mode: {mode_label}\n"
    )


    # --------------------------------------------------------
    # Print configuration
    # --------------------------------------------------------

    print(
        "========== MOTION SETTINGS =========="
    )

    print(
        f"TCP Linear Velocity:      "
        f"{TCP_LINEAR_VELOCITY:.3f} m/s"
    )

    print(
        f"TCP Linear Acceleration:  "
        f"{TCP_LINEAR_ACCELERATION:.3f} m/s²"
    )

    print(
        f"TCP Angular Velocity:     "
        f"{TCP_ANGULAR_VELOCITY:.3f} rad/s"
    )

    print(
        f"TCP Angular Acceleration: "
        f"{TCP_ANGULAR_ACCELERATION:.3f} rad/s²"
    )

    print(
        f"Blend Radius:             "
        f"{BLEND}"
    )

    print(
        f"Default Point Speed:      "
        f"{DEFAULT_SPEED}"
    )

    print(
        f"Start Pose Speed:         "
        f"{STARTPOSE_SPEED}"
    )

    print(
        "====================================\n"
    )


    # --------------------------------------------------------
    # Print segment speeds
    # --------------------------------------------------------

    print(
        "========== SEGMENT SPEEDS =========="
    )

    for segment, speed in SEGMENT_SPEEDS.items():

        print(
            f"{segment[0]} -> {segment[1]} : "
            f"{speed}"
        )

    print(
        "====================================\n"
    )


    # ========================================================
    # FIND FINAL CARTESIAN POINT
    # ========================================================

    cartesian_sequence = [
        label
        for label in SEQUENCE
        if label != "startpose"
    ]


    if not cartesian_sequence:

        print(
            "ERROR: No Cartesian poses in sequence."
        )

        return


    final_point = cartesian_sequence[-1]


    print(
        f"FINAL CARTESIAN POINT: POSE {final_point}"
    )

    print(
        "All previous Cartesian points: connect=1"
    )

    print(
        f"Final Cartesian point: POSE {final_point}, "
        f"connect=0"
    )

    print()


    # ========================================================
    # EXECUTE SEQUENCE
    # ========================================================

    current_label = "start"


    try:

        for index, next_label in enumerate(SEQUENCE):


            # =================================================
            # STARTPOSE
            # =================================================

            if next_label == "startpose":

                print(
                    "\n========================================"
                )

                print(
                    f"Moving {current_label} -> startpose"
                )

                print(
                    "Target joints:",
                    START_JOINTS
                )

                print(
                    "========================================"
                )


                ret = arm.rm_movej(
                    START_JOINTS,
                    STARTPOSE_SPEED,
                    0,
                    0,
                    1
                )


                print(
                    f"movej startpose "
                    f"(speed={STARTPOSE_SPEED}) "
                    f"result "
                    f"(0 = success): {ret}"
                )


                if ret != 0:

                    print(
                        "Failed to reach startpose."
                    )

                    return


                print(
                    "STARTPOSE reached."
                )


                current_label = "start"

                continue


            # =================================================
            # GET SEGMENT SPEED
            # =================================================

            speed = get_segment_speed(
                current_label,
                next_label
            )


            # =================================================
            # FINAL POINT DETECTION
            # =================================================

            is_last_point = (
                index == len(SEQUENCE) - 1
            )


            # =================================================
            # CONNECT SETTING
            #
            # Intermediate point -> connect=1
            # Final point        -> connect=0
            # =================================================

            if is_last_point:

                connect = 0

            else:

                connect = 1


            # =================================================
            # PRINT SEGMENT
            # =================================================

            print(
                "\n========================================"
            )

            print(
                f"SEGMENT: "
                f"{current_label} -> {next_label}"
            )

            print(
                f"SPEED: {speed}"
            )

            print(
                f"BLEND: {BLEND}"
            )

            print(
                f"CONNECT: {connect}"
            )

            print(
                f"POINT {index + 1} / "
                f"{len(SEQUENCE)}"
            )

            print(
                "========================================"
            )


            # =================================================
            # GET TARGET POSE
            # =================================================

            pose = POSES[next_label]


            # =================================================
            # SEND MOVEL
            # =================================================

            ret = arm.rm_movel(
                pose,
                speed,
                BLEND,
                connect,
                0
            )


            print(
                f"movel pose{next_label} "
                f"(speed={speed}) "
                f"(blend={BLEND}) "
                f"(connect={connect}) "
                f"result "
                f"(0 = accepted): {ret}"
            )


            # =================================================
            # COMMAND REJECTED
            # =================================================

            if ret != 0:

                print(
                    "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )

                print(
                    f"TRAJECTORY COMMAND REJECTED: "
                    f"{current_label} -> {next_label}"
                )

                print(
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )


                ret_state, state = (
                    arm.rm_get_current_arm_state()
                )


                if ret_state == 0:

                    print(
                        "Current pose:",
                        state.get("pose")
                    )

                    print(
                        "Error:",
                        state.get("err")
                    )


                return


            # =================================================
            # INTERMEDIATE POINT
            #
            # DO NOT WAIT.
            #
            # Immediately send the next point so that
            # the controller can blend the trajectory.
            # =================================================

            if not is_last_point:

                current_label = next_label

                continue


            # =================================================
            # FINAL POINT ONLY
            #
            # Wait until final point is actually reached.
            # =================================================

            print(
                f"\nWaiting for final Pose "
                f"{final_point} to finish..."
            )


            success = wait_for_target(
                arm,
                final_point,
                current_label,
                final_point,
                timeout=SEGMENT_TIMEOUT
            )


            if not success:

                print(
                    "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )

                print(
                    f"POSE {final_point} "
                    f"WAS NOT REACHED"
                )

                print(
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                )

                return


            current_label = final_point


        # ====================================================
        # COMPLETE
        # ====================================================

        print(
            "\n========================================"
        )

        print(
            "CLEANING SEQUENCE COMPLETED."
        )

        print(
            f"Robot stopped at POSE {final_point}."
        )

        print(
            "========================================"
        )


    finally:

        # ----------------------------------------------------
        # Stop robot
        # ----------------------------------------------------

        arm.rm_set_arm_stop()


        # ----------------------------------------------------
        # Disconnect
        # ----------------------------------------------------

        arm.rm_delete_robot_arm()


        print(
            "Disconnected."
        )


# ============================================================
# PROGRAM ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
