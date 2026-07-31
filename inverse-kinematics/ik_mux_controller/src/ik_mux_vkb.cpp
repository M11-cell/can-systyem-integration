// ik_mux_vkb — VKB NXT teleop mux for the rover arm IK stack.
//
// One node arbitrates every teleop channel for arm_ik_teleop.launch.py:
//
//   B2 (latch)  Wheels <-> Arm top-level mode
//   F1 (latch)  Arm FK      — per-joint jogging   -> /servo_node/delta_joint_cmds
//   F2 (latch)  Arm Servo   — Cartesian IK teleop -> /servo_node/delta_twist_cmds
//   F3 (latch)  Arm Poses   — move_group Plan & Execute (no stick output)
//   D1 (hold)   Deadman — required to publish motion in Wheels/FK/Servo
//
// On every arm sub-mode change the node switches ros2_control controllers
// (arm_servo_controller <-> arm_trajectory_controller) and starts/stops MoveIt
// Servo so exactly one owner drives the hardware at a time.
//
// Button/axis indices mirror joy_mux_controller_py/vkb_layout.py and the arm
// velocity scales match joy_mux_controller_py so FK feels identical to the
// legacy /arm_xyz_cmd teleop.

#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ik_mux
{

// VKB NXT button indices (sensor_msgs/Joy.buttons[]), see vkb_layout.py.
namespace vkb_button
{
constexpr int A2 = 2;
constexpr int B2 = 3;
constexpr int D1 = 4;
constexpr int A3_UP = 5;
constexpr int A3_RIGHT = 6;
constexpr int A3_DOWN = 7;
constexpr int A3_LEFT = 8;
constexpr int A4_LEFT = 13;
constexpr int A4_RIGHT = 11;
constexpr int C1_UP = 15;
constexpr int C1_RIGHT = 16;
constexpr int C1_DOWN = 17;
constexpr int C1_LEFT = 18;
constexpr int TRIGGER_UP = 20;
constexpr int TRIGGER_DOWN = 21;
constexpr int F1 = 26;
constexpr int F2 = 27;
constexpr int F3 = 28;
constexpr int COUNT = 29;  // highest index (F3) + 1
}  // namespace vkb_button

// VKB NXT axis indices.
namespace vkb_axis
{
constexpr int STICK_X = 0;
constexpr int STICK_Y = 1;
constexpr int MIDDLE_SCROLL = 2;
constexpr int STICK_Z = 5;
constexpr int COUNT = 6;  // highest index (STICK_Z) + 1
}  // namespace vkb_axis

// Per-motor velocity scales, identical to joy_mux_controller_py ArmVelocityScale.
namespace arm_scale
{
constexpr double M1_STICK_Z = 1.0;
constexpr double M2_A3_VERTICAL = 0.4;
constexpr double M3_STICK_Y = 0.7;
constexpr double M4_STICK_X = -0.7;
constexpr double M5_A3_HORIZONTAL = 0.8;
constexpr double M6_SPIN = 1.0;
constexpr double M7_GRIPPER = -1.0;
}  // namespace arm_scale

enum class TopMode
{
  WHEELS,
  ARM,
};

enum class ArmMode
{
  FK,
  SERVO,
  POSES,
};

// /rover/drive_mode values consumed by skid_steer_mux.
enum DriveMode : uint8_t
{
  NORMAL = 0,
  PIVOT_LEFT = 1,
  PIVOT_RIGHT = 2,
};

class IkMuxVkb : public rclcpp::Node
{
public:
  IkMuxVkb()
  : Node("ik_mux_vkb")
  {
    base_frame_ = declare_parameter<std::string>("planning_frame", "base_structure_link");
    cartesian_frame_ = declare_parameter<std::string>("cartesian_frame", "gripper_claw_link");
    servo_controller_ = declare_parameter<std::string>("servo_controller", "arm_servo_controller");
    trajectory_controller_ =
      declare_parameter<std::string>("trajectory_controller", "arm_trajectory_controller");
    start_in_arm_mode_ = declare_parameter<bool>("start_in_arm_mode", false);
    mode_toggle_cooldown_s_ = declare_parameter<double>("mode_toggle_cooldown_s", 0.35);

    // F2 Cartesian mapping. The arm's operator-facing base axes are rotated
    // relative to the usual mobile-base convention: base Z is rover forward,
    // base X is left/right, and base Y is up/down. Keep axes and signs
    // configurable because joystick enumeration and mounting can vary.
    linear_x_axis_ =
      declare_parameter<int>("cartesian.linear_x_axis", vkb_axis::STICK_X);//X
    linear_y_axis_ =
      declare_parameter<int>("cartesian.linear_y_axis", vkb_axis::STICK_Y);//Y
    linear_z_axis_ =
      declare_parameter<int>("cartesian.linear_z_axis", vkb_axis::STICK_Z);//Z
    linear_x_sign_ = declare_parameter<double>("cartesian.linear_x_sign", -1.0);
    linear_y_sign_ = declare_parameter<double>("cartesian.linear_y_sign", -1.0);
    linear_z_sign_ = declare_parameter<double>("cartesian.linear_z_sign", -1.0);
    pitch_sign_ = declare_parameter<double>("cartesian.pitch_sign", -1.0);
    yaw_sign_ = declare_parameter<double>("cartesian.yaw_sign", 1.0);
    roll_sign_ = declare_parameter<double>("cartesian.roll_sign", 1.0);

    // Bounds are expressed in raw base_structure_link coordinates. Leave the
    // guard disabled until those limits are measured in the real arm frame;
    // the previous mobile-base-oriented AABB clipped normal up/side motion.
    workspace_enabled_ = declare_parameter<bool>("workspace.enabled", false);
    workspace_min_x_ = declare_parameter<double>("workspace.min_x", 0.05);
    workspace_max_x_ = declare_parameter<double>("workspace.max_x", 0.75);
    workspace_min_y_ = declare_parameter<double>("workspace.min_y", -0.45);
    workspace_max_y_ = declare_parameter<double>("workspace.max_y", 0.45);
    workspace_min_z_ = declare_parameter<double>("workspace.min_z", 0.02);
    workspace_max_z_ = declare_parameter<double>("workspace.max_z", 0.55);
    if (linear_x_axis_ < 0 || linear_y_axis_ < 0 || linear_z_axis_ < 0) {
      throw std::invalid_argument("Cartesian joystick axis indices must be non-negative");
    }
    if (workspace_min_x_ >= workspace_max_x_ ||
        workspace_min_y_ >= workspace_max_y_ ||
        workspace_min_z_ >= workspace_max_z_)
    {
      throw std::invalid_argument("Each workspace minimum must be less than its maximum");
    }
    required_axes_count_ =
      std::max({vkb_axis::COUNT, linear_x_axis_ + 1, linear_y_axis_ + 1, linear_z_axis_ + 1});

    // Hybrid Cartesian: linear is stamped in base_frame_; angular is authored in
    // cartesian_frame_ and rotated into base before publish (Servo accepts one frame_id).
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    drive_mode_pub_ = create_publisher<std_msgs::msg::UInt8>("/rover/drive_mode", 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_controller/commands", 10);

    start_servo_client_ = create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    stop_servo_client_ = create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    command_type_client_ = create_client<moveit_msgs::srv::ServoCommandType>(
      "/servo_node/switch_command_type");
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) { joyCallback(msg); });
    status_sub_ = create_subscription<moveit_msgs::msg::ServoStatus>(
      "/servo_node/status", rclcpp::SystemDefaultsQoS(),
      [this](const moveit_msgs::msg::ServoStatus::ConstSharedPtr & msg) { onServoStatus(msg); });

    // MoveIt 2 advertises MoveGroupMoveAction as "/move_action" (not "/move_group").
    // See moveit/move_group/capability_names.hpp MOVE_ACTION.
    move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      this, "/move_action");

    top_mode_ = start_in_arm_mode_ ? TopMode::ARM : TopMode::WHEELS;
    if (top_mode_ == TopMode::ARM) {
      applyArmMode(arm_mode_);
    }

    // Controller manager / Servo services may not exist yet when applyArmMode runs;
    // retry until setup succeeds.
    arm_setup_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() { retryArmSetup(); });

    RCLCPP_INFO(
      get_logger(),
      "ik_mux_vkb ready — top mode: %s (B2 toggles). Arm sub-modes: F1=FK, F2=Servo, F3=Poses. "
      "Hold D1 to move.",
      top_mode_ == TopMode::ARM ? "ARM" : "WHEELS");
  }

private:
  static bool pressed(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, int idx)
  {
    return idx >= 0 && idx < static_cast<int>(msg->buttons.size()) && msg->buttons[idx] != 0;
  }

  static double axis(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, int idx)
  {
    return (idx >= 0 && idx < static_cast<int>(msg->axes.size())) ? msg->axes[idx] : 0.0;
  }

  // VKB middle scroll [-1, 1] -> [0.2, 1.0] speed multiplier (matches joy_mux).
  static double throttleMultiplier(double scroll)
  {
    const double t = (scroll - (-1.0)) / 2.0;
    return std::clamp(0.2 + t * (1.0 - 0.2), 0.2, 1.0);
  }

  bool risingEdge(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, int idx, bool & prev)
  {
    const bool now = pressed(msg, idx);
    const bool rising = now && !prev;
    prev = now;
    return rising;
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    if (static_cast<int>(msg->buttons.size()) < vkb_button::COUNT ||
        static_cast<int>(msg->axes.size()) < required_axes_count_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy message too short (buttons=%zu, axes=%zu); need >= %d and %d.",
        msg->buttons.size(), msg->axes.size(), vkb_button::COUNT, required_axes_count_);
      return;
    }

    const double now_s = now().seconds();

    // B2: toggle Wheels <-> Arm (debounced rising edge).
    if (risingEdge(msg, vkb_button::B2, prev_b2_) &&
        (now_s - last_top_toggle_s_) >= mode_toggle_cooldown_s_)
    {
      last_top_toggle_s_ = now_s;
      top_mode_ = (top_mode_ == TopMode::WHEELS) ? TopMode::ARM : TopMode::WHEELS;
      stopBurst();
      if (top_mode_ == TopMode::ARM) {
        arm_mode_ = ArmMode::SERVO;  // default arm sub-mode on entry
        applyArmMode(arm_mode_);
      }
      RCLCPP_INFO(
        get_logger(), "Top mode -> %s", top_mode_ == TopMode::ARM ? "ARM" : "WHEELS");
    }

    // F1/F2/F3: arm sub-mode (only meaningful in arm mode, but track edges always).
    const bool f1 = risingEdge(msg, vkb_button::F1, prev_f1_);
    const bool f2 = risingEdge(msg, vkb_button::F2, prev_f2_);
    const bool f3 = risingEdge(msg, vkb_button::F3, prev_f3_);
    if (top_mode_ == TopMode::ARM) {
      if (f1 && arm_mode_ != ArmMode::FK) {
        arm_mode_ = ArmMode::FK;
        applyArmMode(arm_mode_);
        RCLCPP_INFO(get_logger(), "Arm mode -> FK (joint jog)");
      } else if (f2 && arm_mode_ != ArmMode::SERVO) {
        arm_mode_ = ArmMode::SERVO;
        applyArmMode(arm_mode_);
        RCLCPP_INFO(get_logger(), "Arm mode -> SERVO (Cartesian)");
      } else if (f3 && arm_mode_ != ArmMode::POSES) {
        arm_mode_ = ArmMode::POSES;
        applyArmMode(arm_mode_);
        RCLCPP_INFO(get_logger(), "Arm mode -> POSES (move_group)");
      }

      // A2 (with D1 held): one-button escape to the SRDF "home" pose via
      // move_group. Gated on the deadman so the arm never moves autonomously
      // without the operator holding it.
      const bool a2 = risingEdge(msg, vkb_button::A2, prev_a2_);
      if (a2 && pressed(msg, vkb_button::D1)) {
        triggerHomeEscape();
      }
    } else {
      // Keep A2 edge state consistent when not in arm mode.
      risingEdge(msg, vkb_button::A2, prev_a2_);
    }

    const bool deadman = pressed(msg, vkb_button::D1);
    if (!deadman) {
      if (prev_deadman_) {
        stopBurst();  // publish zeros once on release
      }
      prev_deadman_ = false;
      return;
    }
    prev_deadman_ = true;

    if (top_mode_ == TopMode::WHEELS) {
      publishWheels(msg);
    } else {
      // Gripper (joint7) is driven directly in every arm sub-mode, independent
      // of MoveIt/IK, so it works while jogging, servoing, or executing poses.
      publishGripper(msg);
      if (arm_mode_ == ArmMode::FK) {
        publishJointJog(msg);
      } else if (arm_mode_ == ArmMode::SERVO) {
        publishCartesian(msg);
      }
      // POSES: no arm stick output; RViz + move_group drive execution.
    }
  }

  void publishGripper(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    const double gripper =
      static_cast<double>(pressed(msg, vkb_button::C1_UP) - pressed(msg, vkb_button::C1_DOWN));
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {gripper * arm_scale::M7_GRIPPER};
    gripper_pub_->publish(cmd);
  }

  // Hybrid Cartesian teleop:
  //   linear  — base_structure_link (world-aligned pick & place)
  //   angular — authored about gripper_claw_link axes, then rotated into base
  // MoveIt Servo takes a single TwistStamped.frame_id, so we compose the hybrid
  // twist here and stamp it in base_frame_. keep apply_twist_commands_about_ee_frame
  // true so pure EE-frame publishers (if any) still rotate about the tip.
  void publishCartesian(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    const double throttle = throttleMultiplier(axis(msg, vkb_axis::MIDDLE_SCROLL));

    // Hat / C1 intents in operator space, authored about end-effector axes.
    const double a3_ud =
      static_cast<double>(pressed(msg, vkb_button::A3_UP) - pressed(msg, vkb_button::A3_DOWN));
    const double a3_lr =
      static_cast<double>(pressed(msg, vkb_button::A3_LEFT) - pressed(msg, vkb_button::A3_RIGHT));
    const double c1_lr =
      static_cast<double>(pressed(msg, vkb_button::C1_RIGHT) - pressed(msg, vkb_button::C1_LEFT));

    geometry_msgs::msg::Vector3Stamped ang_ee;
    ang_ee.header.frame_id = cartesian_frame_;
    ang_ee.header.stamp = rclcpp::Time(0);  // use latest TF
    ang_ee.vector.x = pitch_sign_ * a3_ud * throttle;
    ang_ee.vector.y = yaw_sign_ * a3_lr * throttle;
    ang_ee.vector.z = roll_sign_ * c1_lr * throttle;

    geometry_msgs::msg::Vector3Stamped ang_base;
    try {
      ang_base = tf_buffer_->transform(ang_ee, base_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cartesian teleop: TF %s -> %s failed (%s); skipping twist",
        cartesian_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    auto twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist->header.frame_id = base_frame_;
    twist->header.stamp = now();
    twist->twist.linear.x = linear_x_sign_ * axis(msg, linear_x_axis_) * throttle;
    twist->twist.linear.y = linear_y_sign_ * axis(msg, linear_y_axis_) * throttle;
    twist->twist.linear.z = linear_z_sign_ * axis(msg, linear_z_axis_) * throttle;
    twist->twist.angular = ang_base.vector;

    if (workspace_enabled_ && !clampLinearToWorkspace(twist->twist.linear)) {
      return;
    }
    twist_pub_->publish(std::move(twist));
  }

  bool clampLinearToWorkspace(geometry_msgs::msg::Vector3 & linear)
  {
    geometry_msgs::msg::TransformStamped base_to_ee;
    try {
      base_to_ee = tf_buffer_->lookupTransform(
        base_frame_, cartesian_frame_, rclcpp::Time(0), tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Workspace guard: TF %s -> %s failed (%s); skipping twist",
        cartesian_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    const auto & p = base_to_ee.transform.translation;
    bool clamped = false;
    const auto clamp_outward =
      [&clamped](double position, double minimum, double maximum, double & velocity)
      {
        if ((position <= minimum && velocity < 0.0) ||
            (position >= maximum && velocity > 0.0))
        {
          velocity = 0.0;
          clamped = true;
        }
      };
    clamp_outward(p.x, workspace_min_x_, workspace_max_x_, linear.x);
    clamp_outward(p.y, workspace_min_y_, workspace_max_y_, linear.y);
    clamp_outward(p.z, workspace_min_z_, workspace_max_z_, linear.z);

    if (clamped) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cartesian workspace boundary reached at [%.3f, %.3f, %.3f] m; "
        "outward translation clamped.",
        p.x, p.y, p.z);
    }
    return true;
  }

  void publishJointJog(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    const double throttle = throttleMultiplier(axis(msg, vkb_axis::MIDDLE_SCROLL));

    // Latch stick X (joint4) briefly so a quick flick still registers, mirroring
    // joy_mux_controller_py's M4 hold behaviour.
    double m4 = axis(msg, vkb_axis::STICK_X);
    const double now_s = now().seconds();
    if (m4 != 0.0) {
      m4_latched_ = m4;
      m4_hold_until_s_ = now_s + m4_hold_s_;
    } else if (now_s < m4_hold_until_s_) {
      m4 = m4_latched_;
    } else {
      m4_latched_ = 0.0;
    }

    const double a3_vertical =
      static_cast<double>(pressed(msg, vkb_button::A3_UP) - pressed(msg, vkb_button::A3_DOWN));
    const double a3_horizontal =
      static_cast<double>(pressed(msg, vkb_button::A3_LEFT) - pressed(msg, vkb_button::A3_RIGHT));
    const double c1_spin =
      static_cast<double>(pressed(msg, vkb_button::C1_RIGHT) - pressed(msg, vkb_button::C1_LEFT));

    // Only the SRDF "arm" group (joint1-6) is jogged: MoveIt Servo ignores joints
    // outside its planning group. joint7 (gripper) is not part of the group and is
    // driven separately, so it is deliberately omitted here.
    auto jog = std::make_unique<control_msgs::msg::JointJog>();
    jog->header.frame_id = base_frame_;
    jog->header.stamp = now();
    jog->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    jog->velocities = {
      axis(msg, vkb_axis::STICK_Z) * arm_scale::M1_STICK_Z,
      a3_vertical * arm_scale::M2_A3_VERTICAL * throttle,
      axis(msg, vkb_axis::STICK_Y) * arm_scale::M3_STICK_Y,
      m4 * arm_scale::M4_STICK_X,
      a3_horizontal * arm_scale::M5_A3_HORIZONTAL * throttle,
      c1_spin * arm_scale::M6_SPIN,
    };
    joint_pub_->publish(std::move(jog));
  }

  void publishWheels(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    // Pivot latch: A3 left/right toggles a sustained pivot drive mode.
    if (risingEdge(msg, vkb_button::A3_LEFT, prev_a3_left_)) {
      latched_pivot_ = (latched_pivot_ == PIVOT_LEFT) ? NORMAL : PIVOT_LEFT;
    }
    if (risingEdge(msg, vkb_button::A3_RIGHT, prev_a3_right_)) {
      latched_pivot_ = (latched_pivot_ == PIVOT_RIGHT) ? NORMAL : PIVOT_RIGHT;
    }

    const double boost = wheelBoost(msg);
    const double stick_y = axis(msg, vkb_axis::STICK_Y);
    const double stick_z = axis(msg, vkb_axis::STICK_Z);
    const int tank_turn = pressed(msg, vkb_button::A4_LEFT) - pressed(msg, vkb_button::A4_RIGHT);

    geometry_msgs::msg::Twist twist;
    uint8_t drive_mode = NORMAL;
    if (tank_turn != 0) {
      twist.angular.z = static_cast<double>(tank_turn) * boost;
    } else if (latched_pivot_ != NORMAL) {
      drive_mode = latched_pivot_;
      twist.angular.z = stick_z * boost;
    } else {
      twist.linear.x = stick_y * boost;
      twist.angular.z = stick_z * boost;
    }

    cmd_vel_pub_->publish(twist);
    std_msgs::msg::UInt8 mode_msg;
    mode_msg.data = drive_mode;
    drive_mode_pub_->publish(mode_msg);
  }

  double wheelBoost(const sensor_msgs::msg::Joy::ConstSharedPtr & msg) const
  {
    if (pressed(msg, vkb_button::TRIGGER_DOWN)) {
      return 2.0;
    }
    if (pressed(msg, vkb_button::TRIGGER_UP)) {
      return 1.5;
    }
    return 1.0;
  }

  // Publish a single stop on every channel so watchdog-less firmware halts.
  void stopBurst()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    latched_pivot_ = NORMAL;

    auto twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist->header.frame_id = base_frame_;
    twist->header.stamp = now();
    twist_pub_->publish(std::move(twist));

    auto jog = std::make_unique<control_msgs::msg::JointJog>();
    jog->header.frame_id = base_frame_;
    jog->header.stamp = now();
    jog->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    jog->velocities.assign(jog->joint_names.size(), 0.0);
    joint_pub_->publish(std::move(jog));

    std_msgs::msg::Float64MultiArray gripper;
    gripper.data = {0.0};
    gripper_pub_->publish(gripper);
  }

  // Switch controllers + servo so a single owner drives the arm for this mode.
  void applyArmMode(ArmMode mode)
  {
    const bool servo = (mode == ArmMode::FK || mode == ArmMode::SERVO);
    if (servo) {
      pending_servo_arm_mode_ = mode;
      arm_servo_setup_pending_ = true;
      arm_trajectory_setup_pending_ = false;
      controllers_switched_for_servo_ = false;
      retryArmSetup();
    } else {  // POSES
      arm_servo_setup_pending_ = false;
      controllers_switched_for_servo_ = false;
      servo_started_ = false;
      callTrigger(stop_servo_client_);
      arm_trajectory_setup_pending_ = true;
    }
  }

  void retryArmSetup()
  {
    if (top_mode_ != TopMode::ARM) {
      return;
    }

    if (arm_trajectory_setup_pending_) {
      if (switchControllers(trajectory_controller_, servo_controller_)) {
        arm_trajectory_setup_pending_ = false;
        sendHomeEscapeGoal();
      }
      return;
    }

    if (home_escape_pending_ && arm_mode_ == ArmMode::POSES) {
      sendHomeEscapeGoal();
      return;
    }

    if (!arm_servo_setup_pending_) {
      return;
    }
    const ArmMode mode = pending_servo_arm_mode_;
    if (mode != ArmMode::FK && mode != ArmMode::SERVO) {
      arm_servo_setup_pending_ = false;
      return;
    }

    if (!controllers_switched_for_servo_) {
      if (!switchControllers(servo_controller_, trajectory_controller_)) {
        return;
      }
      controllers_switched_for_servo_ = true;
    }

    const int8_t command_type =
      mode == ArmMode::FK ? moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG
                          : moveit_msgs::srv::ServoCommandType::Request::TWIST;
    setServoCommandType(command_type);
  }

  void setServoCommandType(int8_t command_type)
  {
    if (command_type_in_flight_ || !command_type_client_->service_is_ready()) {
      return;
    }
    command_type_in_flight_ = true;
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = command_type;
    command_type_client_->async_send_request(
      req,
      [this, command_type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
        command_type_in_flight_ = false;
        const auto & resp = future.get();
        if (!resp->success) {
          RCLCPP_WARN(get_logger(), "switch_command_type failed; will retry.");
          return;
        }
        arm_servo_setup_pending_ = false;
        RCLCPP_INFO(
          get_logger(), "Servo command type set to %s",
          command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG ? "JOINT_JOG"
                                                                                 : "TWIST");
        if (!servo_started_) {
          callTrigger(start_servo_client_);
          servo_started_ = true;
        }
      });
  }

  bool switchControllers(const std::string & activate, const std::string & deactivate)
  {
    if (controller_switch_in_flight_) {
      return false;
    }

    if (controller_switch_result_ready_) {
      const bool matches =
        controller_switch_activate_ == activate && controller_switch_deactivate_ == deactivate;
      const bool succeeded = controller_switch_result_success_;
      controller_switch_result_ready_ = false;
      if (matches && succeeded) {
        return true;
      }
      // A failed request, or a completed request for a mode that is no longer
      // selected, falls through and sends the currently desired switch.
    }

    if (!switch_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "controller_manager switch service not ready; will retry switch to '%s'.",
        activate.c_str());
      return false;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = {activate};
    req->deactivate_controllers = {deactivate};
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    req->activate_asap = true;
    controller_switch_in_flight_ = true;
    controller_switch_activate_ = activate;
    controller_switch_deactivate_ = deactivate;
    switch_client_->async_send_request(
      req,
      [this, activate, deactivate](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
      {
        controller_switch_in_flight_ = false;
        controller_switch_result_ready_ = true;
        controller_switch_activate_ = activate;
        controller_switch_deactivate_ = deactivate;
        try {
          controller_switch_result_success_ = future.get()->ok;
        } catch (const std::exception & ex) {
          controller_switch_result_success_ = false;
          RCLCPP_WARN(
            get_logger(), "Controller switch response failed (%s); will retry.", ex.what());
          return;
        }
        if (!controller_switch_result_success_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Controller switch to '%s' failed; controllers may still be loading. Will retry.",
            activate.c_str());
        }
      });
    return false;
  }

  void callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
  {
    if (!client->service_is_ready()) {
      return;
    }
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  // Surface the reason motion stopped. Servo only publishes a status code, so
  // translate the singularity codes into a throttled operator warning and point
  // at the escape controls (see singularity_advisor for per-joint hints).
  void onServoStatus(const moveit_msgs::msg::ServoStatus::ConstSharedPtr & msg)
  {
    if (msg->code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Servo halted at a singularity. Press A2 (hold D1) to auto-home, or switch to F1 and "
        "follow /arm/singularity_hint.");
    } else if (msg->code == moveit_msgs::msg::ServoStatus::DECELERATE_FOR_APPROACHING_SINGULARITY) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Approaching a singularity; Servo is decelerating.");
    }
  }

  // Plan and execute to the SRDF "home" pose. Switches to the trajectory
  // controller first so move_group's execution has an active controller.
  void triggerHomeEscape()
  {
    home_escape_pending_ = true;
    if (arm_mode_ != ArmMode::POSES) {
      arm_mode_ = ArmMode::POSES;
      applyArmMode(arm_mode_);
      return;
    }
    sendHomeEscapeGoal();
  }

  void sendHomeEscapeGoal()
  {
    if (!home_escape_pending_ || arm_trajectory_setup_pending_ ||
        controller_switch_in_flight_)
    {
      return;
    }
    if (!move_group_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "move_action not ready; cannot run home escape. Check: ros2 action list | grep move_action");
      return;
    }
    home_escape_pending_ = false;

    moveit_msgs::action::MoveGroup::Goal goal;
    goal.request.group_name = "arm";
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = 5.0;
    goal.request.max_velocity_scaling_factor = 0.3;
    goal.request.max_acceleration_scaling_factor = 0.3;

    // SRDF "home" group state (see rover_arm.srdf / initial_positions.yaml).
    static const std::array<std::pair<const char *, double>, 6> home = {{
      {"joint1", 0.0}, {"joint2", 0.5}, {"joint3", 0.7},
      {"joint4", 0.0}, {"joint5", 0.7}, {"joint6", 0.0}}};
    moveit_msgs::msg::Constraints constraints;
    for (const auto & joint : home) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = joint.first;
      jc.position = joint.second;
      jc.tolerance_above = 0.02;
      jc.tolerance_below = 0.02;
      jc.weight = 1.0;
      constraints.joint_constraints.push_back(jc);
    }
    goal.request.goal_constraints.push_back(constraints);
    goal.planning_options.plan_only = false;

    move_group_client_->async_send_goal(goal);
    RCLCPP_INFO(get_logger(), "Home escape requested (/move_action plan & execute to 'home').");
  }

  // Parameters
  std::string base_frame_;
  std::string cartesian_frame_;
  std::string servo_controller_;
  std::string trajectory_controller_;
  bool start_in_arm_mode_ = false;
  double mode_toggle_cooldown_s_ = 0.35;
  int linear_x_axis_ = vkb_axis::STICK_Z;
  int linear_y_axis_ = vkb_axis::STICK_X;
  int linear_z_axis_ = vkb_axis::STICK_Y;
  int required_axes_count_ = vkb_axis::COUNT;
  double linear_x_sign_ = -1.0;
  double linear_y_sign_ = -1.0;
  double linear_z_sign_ = -1.0;
  double pitch_sign_ = -1.0;
  double yaw_sign_ = 1.0;
  double roll_sign_ = 1.0;
  bool workspace_enabled_ = false;
  double workspace_min_x_ = 0.05;
  double workspace_max_x_ = 0.75;
  double workspace_min_y_ = -0.45;
  double workspace_max_y_ = 0.45;
  double workspace_min_z_ = 0.02;
  double workspace_max_z_ = 0.55;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers / subscription / clients
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drive_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr command_type_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp::TimerBase::SharedPtr arm_setup_timer_;

  // Arm setup: retry controller switches and Servo services until ready.
  bool arm_servo_setup_pending_ = false;
  bool arm_trajectory_setup_pending_ = false;
  bool controllers_switched_for_servo_ = false;
  bool servo_started_ = false;
  bool command_type_in_flight_ = false;
  bool controller_switch_in_flight_ = false;
  bool controller_switch_result_ready_ = false;
  bool controller_switch_result_success_ = false;
  std::string controller_switch_activate_;
  std::string controller_switch_deactivate_;
  bool home_escape_pending_ = false;
  ArmMode pending_servo_arm_mode_ = ArmMode::SERVO;

  // Mode state
  TopMode top_mode_ = TopMode::WHEELS;
  ArmMode arm_mode_ = ArmMode::SERVO;
  double last_top_toggle_s_ = -1e9;

  // Edge-detect state
  bool prev_b2_ = false;
  bool prev_f1_ = false;
  bool prev_f2_ = false;
  bool prev_f3_ = false;
  bool prev_a2_ = false;
  bool prev_deadman_ = false;
  bool prev_a3_left_ = false;
  bool prev_a3_right_ = false;

  // FK joint4 latch
  double m4_latched_ = 0.0;
  double m4_hold_until_s_ = 0.0;
  const double m4_hold_s_ = 0.08;

  // Wheels pivot latch
  uint8_t latched_pivot_ = NORMAL;
};

}  // namespace ik_mux

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ik_mux::IkMuxVkb>());
  rclcpp::shutdown();
  return 0;
}
