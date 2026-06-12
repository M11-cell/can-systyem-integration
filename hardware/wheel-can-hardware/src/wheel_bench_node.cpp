// Combined wheel bench node: send velocity commands and print SPARK MAX telemetry
// on a single SocketCAN interface (no second CAN owner needed).
//
// Usage examples:
//   ros2 run wheel_can_hardware wheel_bench_node
//   ros2 run wheel_can_hardware wheel_bench_node --ros-args
//       -p can_interface:=can0 -p multiplier:=500
//
// Drive from another terminal:
//   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist
//     "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
//
// Or spin one motor directly (ignores cmd_vel while bench_motor_id is set):
//   ros2 run wheel_can_hardware wheel_bench_node --ros-args
//       -p bench_motor_id:=1 -p bench_rpm:=100.0
//
// Parameters:
//   can_interface            (string, default "can0")
//   print_period_ms          (int,    default 1000)   Telemetry log interval.
//   can_send_rate_hz         (int,    default 100)     Command + heartbeat rate.
//   stale_threshold_ms       (int,    default 100)
//   multiplier               (int,    default 500)     cmd_vel linear.x scale → RPM.
//   heartbeat_motor_mask     (int,    default 0x7E)   SPARK start-motors mask.
//   device_ids               (int[],  default 1..6)
//   enable_status2           (bool,   default true)
//   log_unknown_spark_frames (bool,   default false)
//   bench_motor_id           (int,    default 0)       If 1..6, send bench_rpm
//                                                         to that motor only.
//   bench_rpm                (double, default 0.0)      Fixed RPM when bench_motor_id set.

#include "can-utils/can_connect.hpp"
#include "can-utils/prefixes.hpp"
#include "can-utils/spark_max_feedback.hpp"
#include "can-utils/system_controller.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr uint32_t kSparkPrefix = 0x0205u;

const char * labelForDevice(uint8_t device_id)
{
  switch (device_id) {
    case 1: return "Mot1 (RF)";
    case 2: return "Mot2 (RM)";
    case 3: return "Mot3 (RR)";
    case 4: return "Mot4 (LF)";
    case 5: return "Mot5 (LM)";
    case 6: return "Mot6 (LR)";
    default: return "Mot?";
  }
}

std::vector<uint8_t> defaultDeviceIds()
{
  return {
    static_cast<uint8_t>(DeviceId::ID::WHEEL_MOT1),
    static_cast<uint8_t>(DeviceId::ID::WHEEL_MOT2),
    static_cast<uint8_t>(DeviceId::ID::WHEEL_MOT3),
    static_cast<uint8_t>(DeviceId::ID::WHEEL_MOT4),
    static_cast<uint8_t>(DeviceId::ID::WHEEL_MOT5),
    static_cast<uint8_t>(DeviceId::ID::WHEEL_MOT6),
  };
}

static constexpr std::array<DeviceId::ID, 6> kWheelIds = {
  DeviceId::ID::WHEEL_MOT1,
  DeviceId::ID::WHEEL_MOT2,
  DeviceId::ID::WHEEL_MOT3,
  DeviceId::ID::WHEEL_MOT4,
  DeviceId::ID::WHEEL_MOT5,
  DeviceId::ID::WHEEL_MOT6,
};

std::string formatRow(const char * label,
                      bool vit_fresh,
                      bool vit_ever,
                      bool status0_fresh,
                      bool status2_fresh,
                      bool status2_ever,
                      float cmd_rpm,
                      const spark_max::WheelFeedback & fb)
{
  std::ostringstream ss;
  ss << "  " << std::left << std::setw(12) << label
     << " cmd=" << std::fixed << std::setprecision(1) << std::setw(8) << cmd_rpm << "rpm";

  if (!status2_ever) {
    ss << "  meas=[NO STATUS_2]";
  } else {
    ss << " meas=" << std::fixed << std::setprecision(2) << std::setw(8) << fb.velocity_rpm << "rpm"
       << " pos=" << std::fixed << std::setprecision(3) << std::setw(9) << fb.position_rot << "rot";
  }

  if (!vit_ever) {
    ss << "  V/I/T=[NO VIT]";
  } else {
    ss << " V=" << std::fixed << std::setprecision(2) << std::setw(6) << fb.bus_voltage_v
       << " I=" << std::fixed << std::setprecision(2) << std::setw(6) << fb.current_a
       << " T=" << std::fixed << std::setprecision(1) << std::setw(5) << fb.motor_temperature_c;
    if (!status0_fresh && fb.legacy_vit_seen) {
      ss << " (legacy)";
    }
  }

  if (!status2_ever && !vit_ever) {
    ss << "  [NO TELEM]";
  } else if ((!status2_ever || !status2_fresh) && (!vit_ever || !vit_fresh)) {
    ss << "  [STALE]";
  } else if (!status2_ever || !status2_fresh) {
    ss << "  [status2 stale]";
  } else if (!vit_ever || !vit_fresh) {
    ss << "  [vit stale]";
  } else {
    ss << "  [ok]";
  }

  return ss.str();
}

}  // namespace


class WheelBenchNode : public rclcpp::Node
{
public:
  WheelBenchNode()
  : Node("wheel_bench_node")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    const int print_ms = declare_parameter<int>("print_period_ms", 1000);
    can_send_rate_hz_ = declare_parameter<int>("can_send_rate_hz", 100);
    stale_ms_ = std::chrono::milliseconds(declare_parameter<int>("stale_threshold_ms", 100));
    multiplier_ = declare_parameter<int>("multiplier", 500);
    heartbeat_mask_ = static_cast<uint32_t>(
      declare_parameter<int64_t>("heartbeat_motor_mask", 0x7E));
    log_unknown_ = declare_parameter<bool>("log_unknown_spark_frames", false);
    const bool enable_status2 = declare_parameter<bool>("enable_status2", true);
    bench_motor_id_ = static_cast<uint8_t>(declare_parameter<int>("bench_motor_id", 0));
    bench_rpm_ = static_cast<float>(declare_parameter<double>("bench_rpm", 0.0));

    const auto default_ids = defaultDeviceIds();
    std::vector<int64_t> id_param = declare_parameter<std::vector<int64_t>>(
      "device_ids",
      {default_ids[0], default_ids[1], default_ids[2],
       default_ids[3], default_ids[4], default_ids[5]});

    device_ids_.clear();
    for (int64_t id : id_param) {
      if (id < 1 || id > 62) {
        throw std::runtime_error(
          "device_ids entry " + std::to_string(id) + " out of SPARK MAX range 1..62");
      }
      device_ids_.push_back(static_cast<uint8_t>(id));
    }
    if (device_ids_.empty()) {
      throw std::runtime_error("device_ids must contain at least one motor id");
    }
    if (bench_motor_id_ != 0 && (bench_motor_id_ < 1 || bench_motor_id_ > 6)) {
      throw std::runtime_error("bench_motor_id must be 0 (disabled) or 1..6");
    }

    can_ = can_util::createConfiguredCanController(can_interface_, get_logger());
    if (!can_) {
      throw std::runtime_error(
        "CAN configure failed on '" + can_interface_ + "' — see log for hints");
    }

    rclcpp::on_shutdown([this] { stopAllMotors(); });

    frame_builder_ = std::make_unique<SystemFrameBuilder>(can_);
    feedback_ = std::make_shared<spark_max::SparkMaxFeedback>(can_, device_ids_);

    if (enable_status2) {
      if (!feedback_->enableStatus2()) {
        RCLCPP_WARN(get_logger(), "SET_STATUSES_ENABLED failed on one or more motors");
      }
    }
    frame_builder_->requestStatusFrame();
    frame_builder_->startMotors(heartbeat_mask_);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        latest_twist_ = msg;
      });

    if (log_unknown_) {
      unknown_frame_cb_ = can_->registerFrameCallback(
        [this](uint32_t id, const std::vector<uint8_t> & data) {
          onAnyFrame(id, data);
        });
    }

    const int send_ms = std::max(1, 1000 / std::max(1, can_send_rate_hz_));
    send_timer_ = create_wall_timer(
      std::chrono::milliseconds(send_ms),
      [this]() { sendWheelCommands(); });

    print_timer_ = create_wall_timer(
      std::chrono::milliseconds(print_ms),
      [this]() { printTelemetry(); });

    RCLCPP_INFO(get_logger(),
                "wheel_bench_node ready — iface=%s, send=%d Hz, print=%d ms, "
                "multiplier=%d, bench_motor=%u bench_rpm=%.1f",
                can_interface_.c_str(), can_send_rate_hz_, print_ms,
                multiplier_, bench_motor_id_, bench_rpm_);
  }

  ~WheelBenchNode() override
  {
    stopAllMotors();
    if (can_) {
      can_->stop();
    }
  }

private:
  void stopAllMotors()
  {
    if (!frame_builder_) {
      return;
    }
    for (const auto id : kWheelIds) {
      frame_builder_->sendWheelMotorVelocity(id, 0.0f);
    }
    frame_builder_->startMotors(heartbeat_mask_);
  }

  std::array<float, 6> computeTargetRpm()
  {
    std::array<float, 6> rpm{};

    if (bench_motor_id_ >= 1 && bench_motor_id_ <= 6) {
      rpm[bench_motor_id_ - 1] = bench_rpm_;
      return rpm;
    }

    geometry_msgs::msg::Twist::ConstSharedPtr twist;
    {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      twist = latest_twist_;
    }
    if (!twist) {
      return rpm;
    }

    constexpr float kDeadzone = 0.05f;
    constexpr float kPureAxisEps = 1e-5f;
    constexpr float kHalfTrack = 0.591f * 0.5f;

    float linear_x = static_cast<float>(twist->linear.x);
    float angular_z = static_cast<float>(twist->angular.z);
    if (std::abs(linear_x) < kDeadzone) {
      linear_x = 0.0f;
    }
    if (std::abs(angular_z) < kDeadzone) {
      angular_z = 0.0f;
    }

    float right_cmd = 0.0f;
    float left_cmd = 0.0f;
    const bool yaw_only = std::abs(linear_x) < kPureAxisEps;
    const bool translate_only = std::abs(angular_z) < kPureAxisEps;

    if (translate_only && yaw_only) {
      right_cmd = 0.0f;
      left_cmd = 0.0f;
    } else if (translate_only) {
      right_cmd = -linear_x;
      left_cmd = -linear_x;
    } else {
      right_cmd = -(linear_x - (-angular_z * kHalfTrack));
      left_cmd = -(linear_x + (-angular_z * kHalfTrack));
    }

    const float mult = static_cast<float>(multiplier_);
    rpm = {right_cmd * mult, right_cmd * mult, right_cmd * mult,
           left_cmd * mult, left_cmd * mult, left_cmd * mult};
    return rpm;
  }

  void sendWheelCommands()
  {
    if (!frame_builder_) {
      return;
    }

    const auto targets = computeTargetRpm();
    for (size_t i = 0; i < kWheelIds.size(); ++i) {
      last_cmd_rpm_[i] = targets[i];
      frame_builder_->sendWheelMotorVelocity(kWheelIds[i], targets[i]);
    }
    frame_builder_->startMotors(heartbeat_mask_);
  }

  void printTelemetry()
  {
    std::ostringstream ss;
    ss << "\n--- Wheel Bench (iface=" << can_interface_ << ") ---";

    for (size_t i = 0; i < device_ids_.size(); ++i) {
      const uint8_t device_id = device_ids_[i];
      spark_max::WheelFeedback fb{};
      if (!feedback_->getFeedback(device_id, fb)) {
        ss << "\n  " << std::left << std::setw(12) << labelForDevice(device_id)
           << "  [NOT WATCHED]";
        continue;
      }

      const float cmd = (device_id >= 1 && device_id <= 6)
        ? last_cmd_rpm_[device_id - 1] : 0.0f;

      ss << "\n" << formatRow(
        labelForDevice(device_id),
        feedback_->isVitFresh(device_id, stale_ms_),
        feedback_->hasVitTelemetry(device_id),
        feedback_->isStatus0Fresh(device_id, stale_ms_),
        feedback_->isStatus2Fresh(device_id, stale_ms_),
        fb.status2_seen,
        cmd,
        fb);
    }

    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  void onAnyFrame(uint32_t id, const std::vector<uint8_t> & data)
  {
    const uint32_t arb_id = id & 0x1FFFFFFFu;
    if ((arb_id >> 16) != kSparkPrefix) {
      return;
    }

    const uint8_t device_id = static_cast<uint8_t>(arb_id & spark_max::kDeviceIdMask);
    const uint32_t frame_type = arb_id & spark_max::kFrameTypeMask;

    for (uint8_t d : device_ids_) {
      if (d == device_id && spark_max::isDecodedTelemetryFrameType(frame_type)) {
        return;
      }
    }

    std::ostringstream ss;
    ss << "Unmatched SPARK frame id=0x"
       << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << arb_id
       << " dev=" << std::dec << static_cast<int>(device_id) << " data=";
    for (auto b : data) {
      ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ss.str().c_str());
  }

  std::string can_interface_;
  int can_send_rate_hz_{100};
  int multiplier_{500};
  uint32_t heartbeat_mask_{0x7E};
  std::chrono::milliseconds stale_ms_{100};
  bool log_unknown_{false};
  uint8_t bench_motor_id_{0};
  float bench_rpm_{0.0f};

  std::vector<uint8_t> device_ids_;
  std::array<float, 6> last_cmd_rpm_{};

  std::shared_ptr<can_util::CANController> can_;
  std::unique_ptr<SystemFrameBuilder> frame_builder_;
  std::shared_ptr<spark_max::SparkMaxFeedback> feedback_;
  std::shared_ptr<can_util::CANFrameCallback> unknown_frame_cb_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist::ConstSharedPtr latest_twist_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr print_timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<WheelBenchNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("wheel_bench_node"),
                 "Node failed to start: %s", e.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("wheel_bench_node"),
                 "Node failed to start: unknown exception");
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
