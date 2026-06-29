import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

from .vkb_layout import VKBButtonLayout, VKBAxesLayout  # noqa: F401 (re-exported for callers)


# Must cover highest VKBButtonLayout index (F3 = 28) so C1 and F-keys are in range.
_JOY_MIN_BUTTONS = 29
_JOY_MIN_AXES = 8
_ARM_JOINT_COUNT = 7


class DriveMode:
    """/rover/drive_mode (std_msgs/UInt8) values consumed by skid_steer_mux."""

    NORMAL = 0
    PIVOT_LEFT = 1
    PIVOT_RIGHT = 2


class ArmVelocityScale:
    """Max velocity scale per arm motor (normalized stick/button input in [-1, 1])."""

    M1_STICK_Z = 1.0
    M2_A3_VERTICAL = 0.4
    M3_STICK_Y = 0.7
    M4_STICK_X = -0.7
    M5_A3_HORIZONTAL = 0.8
    M6_SPIN = 1.0
    M7_GRIPPER = -1.0


class ThrottleAxisMap:
    """VKB middle scroll (axis 2) -> A3 speed multiplier for M2/M5."""

    AXIS_MIN = -1.0
    AXIS_MAX = 1.0
    MULT_MIN = 0.2
    MULT_MAX = 1.0


def _throttle_multiplier(axis: float) -> float:
    axis_span = ThrottleAxisMap.AXIS_MAX - ThrottleAxisMap.AXIS_MIN
    t = (axis - ThrottleAxisMap.AXIS_MIN) / axis_span
    mult = ThrottleAxisMap.MULT_MIN + t * (
        ThrottleAxisMap.MULT_MAX - ThrottleAxisMap.MULT_MIN
    )
    return max(ThrottleAxisMap.MULT_MIN, min(ThrottleAxisMap.MULT_MAX, mult))


class JoyMuxController(Node):

    def __init__(self):
        super().__init__('joy_mux_controller')

        max_cmd_publish_hz = self.declare_parameter("max_cmd_publish_hz", 100.0).value

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.drive_mode_pub = self.create_publisher(UInt8, '/rover/drive_mode', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_xyz_cmd', 10)

        period_s = 1.0 / max(0.1, max_cmd_publish_hz)
        self._publish_timer = self.create_timer(period_s, self._tick)

        self.current_mode = 0
        self.last_toggle = 0
        self._last_mode_toggle_at_s = -1e9
        self._mode_toggle_cooldown_s = self.declare_parameter(
            "mode_toggle_cooldown_s", 0.35
        ).value
        self._deadman_held = False
        self._prev_deadman = False

        # Pivot is press-to-toggle: pressing A3_LEFT/A3_RIGHT latches PIVOT_LEFT/
        # PIVOT_RIGHT, pressing the same side again returns to NORMAL. Edge state
        # and a cooldown debounce mirror the A2 Rover/Arm toggle above.
        self._latched_pivot = DriveMode.NORMAL
        self._prev_a3_left = 0
        self._prev_a3_right = 0
        self._last_pivot_toggle_at_s = -1e9

        # _cached_twist holds the UNBOOSTED stick twist; the boost factor is
        # applied (with ramp-down) in _tick so release decays smoothly.
        self._cached_twist: Twist | None = None
        self._cached_joint: JointState | None = None
        self._drive_mode = DriveMode.NORMAL

        self._stop_burst_until: float = 0.0
        self._stop_burst_duration_s = self.declare_parameter(
            "stop_burst_duration_s", 1.0
        ).value

        # Boost ramp-down: target boost is set instantly on press; on release
        # the effective boost decays back toward 1.0 over boost_release_decay_s.
        self._target_boost = 1.0
        self._effective_boost = 1.0
        self._boost_release_decay_s = self.declare_parameter(
            "boost_release_decay_s", 0.4
        ).value
        self._last_tick_s = self.get_clock().now().nanoseconds * 1e-9

        # Mode-switch stop window: publish zeros on the mode we just left so
        # firmware without a watchdog actually halts.
        self._mode_switch_stop_until: float = 0.0
        self._mode_switch_stop_duration_s = self.declare_parameter(
            "mode_switch_stop_duration_s", 0.5
        ).value

        self._arm_button_min_hold_s = self.declare_parameter(
            "arm_button_min_hold_s", 0.08
        ).value
        self._m4_latched_cmd = 0.0
        self._m4_hold_until = 0.0

        self._rover_boost_trigger_up = self.declare_parameter(
            "rover_boost_trigger_up", 1.5
        ).value
        self._rover_boost_trigger_down = self.declare_parameter(
            "rover_boost_trigger_down", 2.0
        ).value

        self.get_logger().info(
            f"joy_mux_controller ready — max_cmd_publish_hz={max_cmd_publish_hz}, "
            f"stop_burst_duration_s={self._stop_burst_duration_s}, "
            f"mode_switch_stop_duration_s={self._mode_switch_stop_duration_s}, "
            f"mode_toggle_cooldown_s={self._mode_toggle_cooldown_s}, "
            f"arm_button_min_hold_s={self._arm_button_min_hold_s}, "
            f"rover_boost_trigger_up={self._rover_boost_trigger_up}, "
            f"rover_boost_trigger_down={self._rover_boost_trigger_down}, "
            f"boost_release_decay_s={self._boost_release_decay_s}"
        )

    def _publish_all_stop(self) -> None:
        self.rover_pub.publish(Twist())
        stopped = JointState()
        stopped.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
        stopped.velocity = [0.0] * _ARM_JOINT_COUNT
        stopped.position = []
        stopped.effort = []
        self.arm_pub.publish(stopped)

    def _rover_boost(self, buttons) -> float:
        if buttons[VKBButtonLayout.TRIGGER_DOWN]:
            return self._rover_boost_trigger_down
        if buttons[VKBButtonLayout.TRIGGER_UP]:
            return self._rover_boost_trigger_up
        return 1.0

    def _update_boost(self, dt: float) -> None:
        """Track _effective_boost toward _target_boost.

        Ramp up is instantaneous on press; release decays linearly back to 1.0
        over boost_release_decay_s so the rover slows gradually instead of
        dropping speed abruptly when the trigger is let go.
        """
        if self._target_boost >= self._effective_boost:
            self._effective_boost = self._target_boost
            return
        span = max(self._rover_boost_trigger_down - 1.0, 1e-6)
        decay_s = max(self._boost_release_decay_s, 1e-3)
        rate = span / decay_s
        self._effective_boost = max(self._target_boost, self._effective_boost - rate * dt)

    def _boosted_twist(self) -> Twist:
        out = Twist()
        out.linear.x = self._cached_twist.linear.x * self._effective_boost
        out.angular.z = self._cached_twist.angular.z * self._effective_boost
        return out

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) < _JOY_MIN_BUTTONS or len(msg.axes) < _JOY_MIN_AXES:
            self.get_logger().warning(
                (
                    f'Joy message too short (buttons={len(msg.buttons)}, axes={len(msg.axes)}); '
                    f'need at least {_JOY_MIN_BUTTONS} and {_JOY_MIN_AXES}. Publishing stop.'
                ),
                throttle_duration_sec=2.0,
            )
            self._publish_all_stop()
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        home_down = msg.buttons[VKBButtonLayout.A2] == 1
        home_rising = home_down and self.last_toggle == 0
        if (
            home_rising
            and (now_s - self._last_mode_toggle_at_s) >= self._mode_toggle_cooldown_s
        ):
            self.current_mode = 1 - self.current_mode
            self._last_mode_toggle_at_s = now_s
            self._cached_twist = None
            self._cached_joint = None
            self._m4_latched_cmd = 0.0
            self._m4_hold_until = 0.0
            self._latched_pivot = DriveMode.NORMAL
            self._mode_switch_stop_until = now_s + self._mode_switch_stop_duration_s
            self._publish_all_stop()
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = 1 if home_down else 0

        # A3 pivot is press-to-toggle (rover only). A3 doubles as arm joint5 in
        # arm mode, so only update the latch when driving the rover.
        a3_left_btn = msg.buttons[VKBButtonLayout.A3_LEFT] == 1
        a3_right_btn = msg.buttons[VKBButtonLayout.A3_RIGHT] == 1
        if self.current_mode == 0:
            cooldown_ok = (
                now_s - self._last_pivot_toggle_at_s
            ) >= self._mode_toggle_cooldown_s
            if a3_left_btn and self._prev_a3_left == 0 and cooldown_ok:
                self._latched_pivot = (
                    DriveMode.NORMAL
                    if self._latched_pivot == DriveMode.PIVOT_LEFT
                    else DriveMode.PIVOT_LEFT
                )
                self._last_pivot_toggle_at_s = now_s
            elif a3_right_btn and self._prev_a3_right == 0 and cooldown_ok:
                self._latched_pivot = (
                    DriveMode.NORMAL
                    if self._latched_pivot == DriveMode.PIVOT_RIGHT
                    else DriveMode.PIVOT_RIGHT
                )
                self._last_pivot_toggle_at_s = now_s
        self._prev_a3_left = 1 if a3_left_btn else 0
        self._prev_a3_right = 1 if a3_right_btn else 0

        self._deadman_held = msg.buttons[VKBButtonLayout.D1] == 1

        if self._deadman_held:
            if self.current_mode == 0:
                twist = Twist()
                stick_y = float(msg.axes[VKBAxesLayout.STICK_Y])
                stick_z = float(msg.axes[VKBAxesLayout.STICK_Z])
                tank_turn = (1 if msg.buttons[VKBButtonLayout.A4_LEFT] else 0) - (1 if msg.buttons[VKBButtonLayout.A4_RIGHT] else 0)

                # Priority: A4 tank (hold) > latched pivot > normal.
                if tank_turn != 0:
                    self._drive_mode = DriveMode.NORMAL
                    twist.linear.x = 0.0
                    twist.angular.z = float(tank_turn)
                elif self._latched_pivot != DriveMode.NORMAL:
                    self._drive_mode = self._latched_pivot
                    twist.linear.x = 0.0
                    twist.angular.z = stick_z
                else:
                    self._drive_mode = DriveMode.NORMAL
                    twist.linear.x = stick_y
                    twist.angular.z = stick_z

                # Boost is applied (and ramped on release) in _tick, not baked in.
                self._target_boost = self._rover_boost(msg.buttons)
                self._cached_twist = twist
            else:
                joint_state = JointState()
                joint_state.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
                m4_raw = float(msg.axes[VKBAxesLayout.STICK_X])
                if m4_raw != 0.0:
                    self._m4_latched_cmd = m4_raw
                    self._m4_hold_until = now_s + self._arm_button_min_hold_s
                elif now_s < self._m4_hold_until:
                    m4_raw = self._m4_latched_cmd
                else:
                    self._m4_latched_cmd = 0.0
                a3_throttle = _throttle_multiplier(
                    float(msg.axes[VKBAxesLayout.MIDDLE_SCROLL])
                )
                joint_state.velocity = [
                    float(msg.axes[VKBAxesLayout.STICK_Z]) * ArmVelocityScale.M1_STICK_Z,
                    float(msg.buttons[VKBButtonLayout.A3_UP] - msg.buttons[VKBButtonLayout.A3_DOWN])
                    * ArmVelocityScale.M2_A3_VERTICAL * a3_throttle,
                    -float(msg.axes[VKBAxesLayout.STICK_Y]) * ArmVelocityScale.M3_STICK_Y,
                    m4_raw * ArmVelocityScale.M4_STICK_X,
                    float(msg.buttons[VKBButtonLayout.A3_LEFT] - msg.buttons[VKBButtonLayout.A3_RIGHT])
                    * ArmVelocityScale.M5_A3_HORIZONTAL * a3_throttle,
                    float(msg.buttons[VKBButtonLayout.C1_RIGHT] - msg.buttons[VKBButtonLayout.C1_LEFT])
                    * ArmVelocityScale.M6_SPIN,
                    float(msg.buttons[VKBButtonLayout.C1_UP] - msg.buttons[VKBButtonLayout.C1_DOWN])
                    * ArmVelocityScale.M7_GRIPPER,
                ]
                joint_state.position = []
                joint_state.effort = []
                self._cached_joint = joint_state
        elif self._prev_deadman:
            self._stop_burst_until = (
                self.get_clock().now().nanoseconds * 1e-9
                + self._stop_burst_duration_s
            )
            self._latched_pivot = DriveMode.NORMAL
            self._drive_mode = DriveMode.NORMAL
            self._publish_all_stop()

        self._prev_deadman = self._deadman_held

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        dt = max(0.0, now_s - self._last_tick_s)
        self._last_tick_s = now_s
        self._update_boost(dt)
        in_stop_burst = now_s < self._stop_burst_until
        in_mode_switch_stop = now_s < self._mode_switch_stop_until

        if in_mode_switch_stop:
            if self.current_mode == 0:
                stopped = JointState()
                stopped.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
                stopped.velocity = [0.0] * _ARM_JOINT_COUNT
                stopped.position = []
                stopped.effort = []
                self.arm_pub.publish(stopped)
            else:
                self.rover_pub.publish(Twist())

        if not self._deadman_held:
            if in_stop_burst:
                self._publish_all_stop()
            return

        if self.current_mode == 0:
            if self._cached_twist is not None:
                self.rover_pub.publish(self._boosted_twist())
                dm = UInt8()
                dm.data = self._drive_mode
                self.drive_mode_pub.publish(dm)
        else:
            if self._cached_joint is not None:
                self.arm_pub.publish(self._cached_joint)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
