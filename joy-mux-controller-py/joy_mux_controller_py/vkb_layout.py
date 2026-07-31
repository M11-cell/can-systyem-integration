from enum import IntEnum


class VKBButtonLayout(IntEnum):
    """Button index mapping for the VKB NXT joystick as reported by joy_node.

    These values are zero-based indices into sensor_msgs/Joy.buttons[].
    They document the expected mapping — use joy_button_probe to verify that
    the indices match your specific hardware / driver enumeration before
    relying on them for safety-critical params (e.g. can_safety_node).
    """

    FIRST_FIRE   = 0
    SECOND_FIRE  = 1
    A2           = 2
    B2           = 3
    D1           = 4
    A3_UP        = 5
    A3_RIGHT     = 6
    A3_DOWN      = 7
    A3_LEFT      = 8
    A3_PRESS     = 9
    A4_UP        = 10
    A4_RIGHT     = 11
    A4_DOWN      = 12
    A4_LEFT      = 13
    A4_PRESS     = 14
    C1_UP        = 15
    C1_RIGHT     = 16
    C1_DOWN      = 17
    C1_LEFT      = 18
    C1_PRESS     = 19
    TRIGGER_UP   = 20
    TRIGGER_DOWN = 21
    EN1_UP       = 22
    EN1_DOWN     = 23
    EN2_UP       = 24
    EN2_DOWN     = 25
    # F1/F2/F3 double as arm sub-mode selectors in ik_mux_vkb (FK / Servo /
    # Poses). can_safety_node defaults F1/F2 to wheel force-stop/resume, so
    # arm_ik_teleop.launch.py remaps the safety buttons when both run together.
    F1           = 26  # ik_mux_vkb: arm FK; else can_safety_node wheel_force_stop_button
    F2           = 27  # ik_mux_vkb: arm Servo; else can_safety_node wheel_resume_button
    F3           = 28  # ik_mux_vkb: arm Poses


class VKBAxesLayout(IntEnum):
    """Axis index mapping for the VKB NXT joystick as reported by joy_node.

    In ``ik_mux_vkb`` F2 Cartesian mode the arm's operator-facing base axes are
    cyclically rotated: STICK_Y push commands base Z (physical forward/back),
    STICK_Z lean commands base X (physical left/right), and STICK_X twist
    commands base Y (physical up/down). A3 up/down and left/right command pitch
    and yaw about the end-effector; C1 left/right commands end-effector roll.
    Axis indices and direction signs are ROS parameters on ``ik_mux_vkb`` so
    controller firmware variants can be corrected without rebuilding.
    """

    STICK_X        = 0
    STICK_Y        = 1
    STICK_Z        = 5
    MIDDLE_SCROLL  = 2
    A1_X_LED_ON    = 3
    A1_Y_LED_ON    = 4
    A1_X_LED_OFF   = 8
    A1_Y_LED_OFF   = 9
