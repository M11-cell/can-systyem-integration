import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist

from .vkb_layout import VKBButtonLayout, VKBAxesLayout  # noqa: F401 (re-exported for callers)


# Must cover highest VKBButtonLayout index (F3 = 28) so C1 and F-keys are in range.
_JOY_MIN_BUTTONS = 29
_JOY_MIN_AXES = 8
_ARM_JOINT_COUNT = 7

# Rover drive geometry — must match half_track in can_controller_node.cpp.
# Track width = 591 mm (left–right wheel centers); half_track = 591 mm / 2.
_ROVER_HALF_TRACK_M: float = 0.591 / 2.0  # 0.2955 m

# Minimum STICK_Z deflection before tank-mode uses the stick value instead of ±1.
_ROVER_TANK_Z_DEADZONE: float = 0.05


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

        self._cached_twist: Twist | None = None
        self._cached_joint: JointState | None = None

        self._stop_burst_until: float = 0.0
        self._stop_burst_duration_s = self.declare_parameter(
            "stop_burst_duration_s", 0.5
        ).value

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
        self._rover_boost_slew_up_per_s = self.declare_parameter(
            "rover_boost_slew_up_per_s", 10.0
        ).value
        self._rover_boost_slew_down_per_s = self.declare_parameter(
            "rover_boost_slew_down_per_s", 2.0
        ).value
        self._current_rover_boost = 1.0
        self._last_boost_update_at_s: float | None = None

        self.get_logger().info(
            f"joy_mux_controller ready — max_cmd_publish_hz={max_cmd_publish_hz}, "
            f"stop_burst_duration_s={self._stop_burst_duration_s}, "
            f"mode_switch_stop_duration_s={self._mode_switch_stop_duration_s}, "
            f"mode_toggle_cooldown_s={self._mode_toggle_cooldown_s}, "
            f"arm_button_min_hold_s={self._arm_button_min_hold_s}, "
            f"rover_boost_trigger_up={self._rover_boost_trigger_up}, "
            f"rover_boost_trigger_down={self._rover_boost_trigger_down}, "
            f"rover_boost_slew_up_per_s={self._rover_boost_slew_up_per_s}, "
            f"rover_boost_slew_down_per_s={self._rover_boost_slew_down_per_s}"
        )

    def _publish_all_stop(self) -> None:
        self.rover_pub.publish(Twist())
        stopped = JointState()
        stopped.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
        stopped.velocity = [0.0] * _ARM_JOINT_COUNT
        stopped.position = []
        stopped.effort = []
        self.arm_pub.publish(stopped)

    def _target_rover_boost(self, buttons) -> float:
        if buttons[VKBButtonLayout.TRIGGER_DOWN]:
            return self._rover_boost_trigger_down
        if buttons[VKBButtonLayout.TRIGGER_UP]:
            return self._rover_boost_trigger_up
        return 1.0

    def _update_rover_boost(self, now_s: float, buttons) -> float:
        target = self._target_rover_boost(buttons)
        if self._last_boost_update_at_s is None:
            self._last_boost_update_at_s = now_s
            self._current_rover_boost = target
            return self._current_rover_boost

        dt = max(0.0, now_s - self._last_boost_update_at_s)
        self._last_boost_update_at_s = now_s

        slew_per_s = (
            self._rover_boost_slew_up_per_s
            if target >= self._current_rover_boost
            else self._rover_boost_slew_down_per_s
        )
        max_step = max(0.0, slew_per_s) * dt
        delta = target - self._current_rover_boost

        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self._current_rover_boost += delta
        return self._current_rover_boost

    def _build_rover_twist(self, msg: Joy, now_s: float) -> Twist:
        """Build a Twist for rover mode.

        Priority (highest first):
          1. A3 left/right — pivot: one track stopped, other driven by STICK_Y.
          2. A4 left/right — tank:  linear.x = 0, yaw from STICK_Z (or ±1 if
                                    near-zero stick).
          3. default        — arc:  both sticks pass through; can_controller
                                    mixes them into left/right RPM.

        Boost triggers are applied to the final (vx, wz) pair.
        """
        stick_y = float(msg.axes[VKBAxesLayout.STICK_Y])
        stick_z = float(msg.axes[VKBAxesLayout.STICK_Z])

        pivot_left  = bool(msg.buttons[VKBButtonLayout.A3_LEFT])
        pivot_right = bool(msg.buttons[VKBButtonLayout.A3_RIGHT])
        tank_left   = bool(msg.buttons[VKBButtonLayout.A4_LEFT])
        tank_right  = bool(msg.buttons[VKBButtonLayout.A4_RIGHT])

        if pivot_left:
            # Pivot about the right track: right side stops, left side driven.
            # vx/wz chosen so can_controller's skid-steer mix zeroes right_cmd.
            vx = -stick_y / 2.0
            wz =  stick_y / (2.0 * _ROVER_HALF_TRACK_M)
        elif pivot_right:
            # Pivot about the left track: left side stops, right side driven.
            vx =  stick_y / 2.0
            wz = -stick_y / (2.0 * _ROVER_HALF_TRACK_M)
        elif tank_left:
            # Tank spin left: modulate with STICK_Z if deflected, else full speed.
            vx = 0.0
            wz = stick_z if abs(stick_z) > _ROVER_TANK_Z_DEADZONE else 1.0
        elif tank_right:
            # Tank spin right: modulate with STICK_Z if deflected, else full speed.
            vx = 0.0
            wz = stick_z if abs(stick_z) > _ROVER_TANK_Z_DEADZONE else -1.0
        else:
            # Default arc drive: pass both axes through; can_controller mixes
            # them into different left/right speeds for curved motion.
            vx = stick_y
            wz = stick_z

        boost = self._update_rover_boost(now_s, msg.buttons)
        twist = Twist()
        twist.linear.x = vx * boost
        twist.angular.z = wz * boost
        return twist

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
            self._mode_switch_stop_until = now_s + self._mode_switch_stop_duration_s
            self._publish_all_stop()
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = 1 if home_down else 0

        self._deadman_held = msg.buttons[VKBButtonLayout.D1] == 1

        if self._deadman_held:
            if self.current_mode == 0:
                self._cached_twist = self._build_rover_twist(msg, now_s)
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
            self._current_rover_boost = 1.0
            self._last_boost_update_at_s = None
            self._publish_all_stop()

        self._prev_deadman = self._deadman_held

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
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
                self.rover_pub.publish(self._cached_twist)
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
