// skid_steer_mux: convert teleop Twist + drive_mode into per-wheel velocity
// commands for the ros2_control velocity_controller (ForwardCommandController).
//
// Inputs:
//   /cmd_vel            geometry_msgs/Twist   (from joy_mux_controller)
//   /rover/drive_mode   std_msgs/UInt8        (0 NORMAL, 1 PIVOT_LEFT, 2 PIVOT_RIGHT)
//
// Output:
//   /velocity_controller/commands  std_msgs/Float64MultiArray
//     6 wheel velocities in rad/s at the wheel, ordered to match the
//     velocity_controller joint list: RF, RM, RR, LF, LM, LR.
//
// Kinematics mirror wheel_bench_node.cpp / can_controller_node.cpp skid-steer
// mixing (same sign convention and half-track scaling) so behaviour is
// consistent across stacks. The motor RPM is then converted back to wheel
// rad/s because WheelCanInterface re-applies kRadSToRpm * gear_ratio *
// direction in its write() path.
//
// This node is the ONLY publisher of /velocity_controller/commands in the
// teleop launch. Do NOT run it alongside wheel_bench_node on the same CAN
// interface — they are mutually exclusive wheel owners.

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <vector>

namespace
{
// rad/s -> RPM conversion factor (60 / (2 * pi)); matches WheelCanInterface.
constexpr double kRadSToRpm = 9.5492965855137201;

// Drive-mode enum mirrored from /rover/drive_mode payload.
enum DriveMode : uint8_t
{
  NORMAL = 0,
  PIVOT_LEFT = 1,
  PIVOT_RIGHT = 2,
};

// Wheel command order published to velocity_controller (right side then left).
constexpr size_t kWheelCount = 6;
}  // namespace

class SkidSteerMux : public rclcpp::Node
{
public:
  SkidSteerMux()
  : Node("skid_steer_mux")
  {
    track_width_   = declare_parameter<double>("track_width", 0.591);
    multiplier_    = declare_parameter<double>("multiplier", 750.0);
    deadzone_      = declare_parameter<double>("deadzone", 0.05);
    pure_axis_eps_ = declare_parameter<double>("pure_axis_eps", 1e-5);
    publish_rate_  = declare_parameter<double>("publish_rate", 100.0);
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout", 1.0);

    const std::vector<double> default_dirs = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
    const std::vector<double> default_gears = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    directions_  = declare_parameter<std::vector<double>>("directions", default_dirs);
    gear_ratios_ = declare_parameter<std::vector<double>>("gear_ratios", default_gears);
    if (directions_.size() != kWheelCount || gear_ratios_.size() != kWheelCount) {
      RCLCPP_FATAL(get_logger(),
                   "directions and gear_ratios must each have %zu entries", kWheelCount);
      throw std::runtime_error("skid_steer_mux: bad directions/gear_ratios length");
    }

    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/velocity_controller/commands");

    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, 10);

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_twist_ = *msg;
        last_twist_stamp_ = now();
        have_twist_ = true;
      });

    drive_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/rover/drive_mode", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        drive_mode_ = msg->data;
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishCommands(); });

    RCLCPP_INFO(get_logger(),
                "skid_steer_mux ready — track_width=%.3f, multiplier=%.0f, "
                "publish_rate=%.0f Hz, cmd_vel_timeout=%.2fs, output=%s",
                track_width_, multiplier_, publish_rate_, cmd_vel_timeout_s_,
                output_topic.c_str());
  }

private:
  // Compute the two side commands (right, left) from a shaped twist + mode.
  std::array<double, 2> computeSideCommands(double linear_x, double angular_z,
                                            uint8_t drive_mode) const
  {
    const double half_track = track_width_ * 0.5;

    const bool yaw_only = std::abs(linear_x) < pure_axis_eps_;
    const bool translate_only = std::abs(angular_z) < pure_axis_eps_;

    double right_cmd = 0.0;
    double left_cmd = 0.0;

    if (drive_mode == DriveMode::PIVOT_LEFT) {
      // Left side holds, right side drives; stick Z scales the pivot rate.
      left_cmd = 0.0;
      right_cmd = -(angular_z * half_track);
    } else if (drive_mode == DriveMode::PIVOT_RIGHT) {
      right_cmd = 0.0;
      left_cmd = -(angular_z * half_track);
    } else if (translate_only && yaw_only) {
      right_cmd = 0.0;
      left_cmd = 0.0;
    } else if (translate_only) {
      right_cmd = -linear_x;
      left_cmd = -linear_x;
    } else {
      // NORMAL, tank (A4 twist), and oblique arcs.
      right_cmd = -(linear_x - (-angular_z * half_track));
      left_cmd = -(linear_x + (-angular_z * half_track));
    }

    return {right_cmd, left_cmd};
  }

  void publishCommands()
  {
    geometry_msgs::msg::Twist twist;
    uint8_t drive_mode = DriveMode::NORMAL;
    bool fresh = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      twist = latest_twist_;
      drive_mode = drive_mode_;
      if (have_twist_) {
        const double age_s = (now() - last_twist_stamp_).seconds();
        fresh = age_s <= cmd_vel_timeout_s_;
      }
    }

    std_msgs::msg::Float64MultiArray out;
    out.data.assign(kWheelCount, 0.0);

    if (!fresh) {
      // Stale or never-seen /cmd_vel: command a full stop.
      cmd_pub_->publish(out);
      return;
    }

    double linear_x = twist.linear.x;
    double angular_z = twist.angular.z;
    if (std::abs(linear_x) < deadzone_) {
      linear_x = 0.0;
    }
    if (std::abs(angular_z) < deadzone_) {
      angular_z = 0.0;
    }

    const auto sides = computeSideCommands(linear_x, angular_z, drive_mode);
    const double right_cmd = sides[0];
    const double left_cmd = sides[1];

    // Right side {0,1,2}, left side {3,4,5}. Convert each motor RPM to the
    // wheel rad/s the velocity command interface expects.
    for (size_t i = 0; i < kWheelCount; ++i) {
      const double side_cmd = (i < 3) ? right_cmd : left_cmd;
      const double motor_rpm = side_cmd * multiplier_;
      const double denom = directions_[i] * gear_ratios_[i] * kRadSToRpm;
      out.data[i] = (std::abs(denom) > 1e-9) ? (motor_rpm / denom) : 0.0;
    }

    cmd_pub_->publish(out);
  }

  double track_width_{0.591};
  double multiplier_{750.0};
  double deadzone_{0.05};
  double pure_axis_eps_{1e-5};
  double publish_rate_{100.0};
  double cmd_vel_timeout_s_{1.0};
  std::vector<double> directions_;
  std::vector<double> gear_ratios_;

  std::mutex state_mutex_;
  geometry_msgs::msg::Twist latest_twist_;
  rclcpp::Time last_twist_stamp_;
  bool have_twist_{false};
  uint8_t drive_mode_{DriveMode::NORMAL};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr drive_mode_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<SkidSteerMux>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("skid_steer_mux"), "Node failed: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
