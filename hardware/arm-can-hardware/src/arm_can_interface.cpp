#include "arm_can_hardware/arm_can_interface.hpp"

#include "can-utils/buildAddress.hpp"
#include "can-utils/can_connect.hpp"
#include "can-utils/parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace arm_can_hardware
{

namespace
{
constexpr double kCommandDeadzone = 0.05;

// Standard arm motor velocity payload range (matches buildAddress::ARM_MOTOR_*).
constexpr float kArmMotorPayloadMax = buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MAX;
constexpr float kArmMotorPayloadMin = buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MIN;
}  // namespace

JointKind ArmCanInterface::parseKind(const std::string & s)
{
  if (s == "spin_servo") {
    return JointKind::SPIN_SERVO;
  }
  if (s == "clamp_servo") {
    return JointKind::CLAMP_SERVO;
  }
  return JointKind::ARM_MOTOR;
}

ServoMode ArmCanInterface::parseServoMode(const std::string & /*s*/)
{
  return ServoMode::POSITION;
}

Instructions::Inst ArmCanInterface::parseInstruction(const std::string & s)
{
  // Allow either decimal or 0x-prefixed hex in the URDF param.
  if (s.empty()) {
    return Instructions::Inst::ARM_MOTOR_1;
  }
  return static_cast<Instructions::Inst>(static_cast<uint32_t>(std::stoul(s, nullptr, 0)));
}

std::string ArmCanInterface::getParam(const hardware_interface::ComponentInfo & joint,
                                      const std::string & key,
                                      const std::string & fallback)
{
  auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    return fallback;
  }
  return it->second;
}

hardware_interface::CallbackReturn ArmCanInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Hardware-level parameters (declared at the <hardware><param> level in the
  // URDF). Both are optional.
  auto hw_iter = info_.hardware_parameters.find("can_interface");
  if (hw_iter != info_.hardware_parameters.end()) {
    can_interface_name_ = hw_iter->second;
  }
  hw_iter = info_.hardware_parameters.find("send_heartbeat_on_activate");
  if (hw_iter != info_.hardware_parameters.end()) {
    send_heartbeat_on_activate_ = (hw_iter->second == "true" || hw_iter->second == "1");
  }
  hw_iter = info_.hardware_parameters.find("feedback_freshness_ms");
  if (hw_iter != info_.hardware_parameters.end()) {
    try {
      feedback_freshness_ms_ = std::stof(hw_iter->second);
    } catch (const std::exception &) {
      // keep default
    }
  }

  if (info_.joints.empty()) {
    RCLCPP_FATAL(logger_, "No joints defined in <ros2_control> hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    JointConfig cfg;
    cfg.name = joint.name;
    cfg.kind = parseKind(getParam(joint, "kind", "arm_motor"));

    switch (cfg.kind) {
      case JointKind::ARM_MOTOR: {
        cfg.arm_inst = parseInstruction(getParam(joint, "arm_instruction", "0x12"));
        try {
          cfg.velocity_scale = std::stof(getParam(joint, "velocity_scale", "1.0"));
        } catch (const std::exception &) {
          cfg.velocity_scale = 1.0f;
        }
        try {
          cfg.encoder_device_id = static_cast<uint32_t>(
            std::stoul(getParam(joint, "encoder_device_id", "0"), nullptr, 0));
        } catch (const std::exception &) {
          cfg.encoder_device_id = 0;
        }
        try {
          cfg.direction = std::stof(getParam(joint, "direction", "1.0"));
        } catch (const std::exception &) {
          cfg.direction = 1.0f;
        }
        try {
          cfg.position_scale = std::stod(getParam(joint, "position_scale", "1.0"));
        } catch (const std::exception &) {
          cfg.position_scale = 1.0;
        }
        try {
          cfg.position_offset_rad = std::stod(getParam(joint, "position_offset_rad", "0.0"));
        } catch (const std::exception &) {
          cfg.position_offset_rad = 0.0;
        }

        // Derive default encoder TX IDs from device_id if not explicitly provided.
        // Firmware layout (questions.md §A): TX abs = 0x0108_C_<dev>01, speed = +0x40.
        // Device IDs: BASE=0x07, SHOULDER=0x08, ELBOW=0x09, WRIST=0x0B.
        auto deriveAbsId = [](uint32_t dev_id) -> uint32_t {
          // ID = (0x01 << 24) | (0x08 << 16) | ((dev_id << 2) << 6) | 0x01
          // Precomputed: 0x0108C001 + ((dev_id - 0x07) * 0x100) ... simpler: use the table.
          return (0x0108C000u) | ((dev_id & 0xFFu) << 8) | 0x01u;
        };
        auto deriveSpeedId = [&deriveAbsId](uint32_t dev_id) -> uint32_t {
          return deriveAbsId(dev_id) + 0x40u;
        };

        auto parseOptionalId = [&](const std::string & param_name) -> uint32_t {
          const std::string val = getParam(joint, param_name, "");
          if (val.empty()) {
            return 0;
          }
          try {
            return static_cast<uint32_t>(std::stoul(val, nullptr, 0));
          } catch (const std::exception &) {
            return 0;
          }
        };

        const uint32_t abs_id   = parseOptionalId("encoder_abs_can_id");
        const uint32_t speed_id = parseOptionalId("encoder_speed_can_id");

        if (abs_id != 0) {
          cfg.encoder_abs_can_id   = abs_id;
          cfg.encoder_speed_can_id = (speed_id != 0) ? speed_id : (abs_id + 0x40u);
        } else if (cfg.encoder_device_id != 0) {
          cfg.encoder_abs_can_id   = deriveAbsId(cfg.encoder_device_id);
          cfg.encoder_speed_can_id = deriveSpeedId(cfg.encoder_device_id);
        }
        break;
      }
      case JointKind::SPIN_SERVO:
      case JointKind::CLAMP_SERVO: {
        cfg.servo_mode = parseServoMode(getParam(joint, "servo_mode", "position"));
        const std::string default_max = (cfg.kind == JointKind::SPIN_SERVO) ? "90.0" : "15.0";
        try {
          cfg.servo_max = std::stof(getParam(joint, "servo_max", default_max));
        } catch (const std::exception &) {
          cfg.servo_max = (cfg.kind == JointKind::SPIN_SERVO) ? 90.0f : 15.0f;
        }
        break;
      }
    }

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(logger_,
                   "Joint '%s' must declare exactly one velocity command interface (found %zu).",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joints_.push_back(cfg);
  }

  const size_t n = joints_.size();
  hw_commands_velocity_.assign(n, 0.0);
  hw_states_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());

  // Build the encoder channel table for ArmEncoderFeedback. Only arm motors
  // with a configured absolute-position CAN ID get a channel; joints without an
  // encoder (or whose board is not yet installed) stay open loop. The decoder
  // stores raw radians; sign/scale/offset are applied here in read().
  encoder_channels_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    JointConfig & cfg = joints_[i];
    if (cfg.kind != JointKind::ARM_MOTOR || cfg.encoder_abs_can_id == 0) {
      continue;
    }
    encoder_boards::EncoderChannel ch;
    ch.label        = cfg.name;
    ch.abs_can_id   = cfg.encoder_abs_can_id;
    ch.speed_can_id = (cfg.encoder_speed_can_id != 0)
                        ? cfg.encoder_speed_can_id
                        : (cfg.encoder_abs_can_id + 0x40u);
    ch.direction    = cfg.direction;  // informational; applied in read()
    cfg.has_encoder   = true;
    cfg.channel_index = encoder_channels_.size();
    encoder_channels_.push_back(ch);
    RCLCPP_DEBUG(logger_, "Joint '%s': encoder abs ID 0x%08X, speed ID 0x%08X",
                 cfg.name.c_str(), ch.abs_can_id, ch.speed_can_id);
  }

  RCLCPP_INFO(logger_,
              "Initialised ArmCanInterface with %zu joints (%zu with encoders) on CAN '%s'",
              joints_.size(), encoder_channels_.size(), can_interface_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    can_controller_ = can_util::getSharedCanController(can_interface_name_, logger_);
    if (!can_controller_) {
      can_controller_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
    frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

    // ArmEncoderFeedback registers its own frame callback on the shared
    // controller and decodes the abs/speed frames for every configured channel.
    if (!encoder_channels_.empty()) {
      encoder_feedback_ = std::make_unique<encoder_boards::ArmEncoderFeedback>(
        can_controller_, encoder_channels_);
    }

    RCLCPP_INFO(logger_, "Configured ArmCanInterface on CAN '%s'", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      logger_, "Exception during ArmCanInterface configure on '%s': %s",
      can_interface_name_.c_str(), e.what());
    can_util::logCanSetupRecoveryHints(logger_, can_interface_name_);
    encoder_feedback_.reset();
    frame_builder_.reset();
    can_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_FATAL(
      logger_, "Unknown exception during ArmCanInterface configure on '%s'",
      can_interface_name_.c_str());
    can_util::logCanSetupRecoveryHints(logger_, can_interface_name_);
    encoder_feedback_.reset();
    frame_builder_.reset();
    can_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn ArmCanInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands so the first write() doesn't lurch the arm.
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);

  if (send_heartbeat_on_activate_ && frame_builder_) {
    // The wheel motors share the bus heartbeat; sending it here is harmless
    // and matches the legacy can_controller_node behaviour.
    constexpr uint64_t kWheelMotorMask = 0x7E;
    frame_builder_->startMotors(kWheelMotorMask);
  }

  RCLCPP_INFO(logger_, "ArmCanInterface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  // Best effort: send a zero velocity to every arm motor so the hardware
  // doesn't keep moving after the controller is deactivated.
  if (frame_builder_) {
    for (const auto & cfg : joints_) {
      if (cfg.kind == JointKind::ARM_MOTOR) {
        frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER,
                                             cfg.arm_inst,
                                             DeviceId::ID::ARM_MOTOR_CONTROLLER, 0.0f);
      }
    }
  }
  RCLCPP_INFO(logger_, "ArmCanInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmCanInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Snapshot the latest encoder feedback decoded by ArmEncoderFeedback into the
  // ros2_control state buffers, applying per-joint calibration exactly once.
  const auto fresh_window =
    std::chrono::milliseconds(static_cast<int>(feedback_freshness_ms_));

  for (size_t i = 0; i < joints_.size(); ++i) {
    JointConfig & cfg = joints_[i];

    if (cfg.kind != JointKind::ARM_MOTOR) {
      // Servos do not provide position feedback over CAN today; mirror the
      // commanded value so the controller has something coherent to read.
      hw_states_position_[i] = std::isnan(hw_states_position_[i]) ? 0.0 : hw_states_position_[i];
      hw_states_velocity_[i] = hw_commands_velocity_[i];
      continue;
    }

    // No encoder configured (board not installed yet): stay open loop. The
    // motor still accepts velocity commands; feedback is simply unavailable.
    if (!cfg.has_encoder || !encoder_feedback_) {
      hw_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }

    const size_t ch = cfg.channel_index;
    const double dir_scale = static_cast<double>(cfg.direction) * cfg.position_scale;

    if (encoder_feedback_->absFresh(ch, fresh_window)) {
      hw_states_position_[i] =
        dir_scale * encoder_feedback_->positionRad(ch) + cfg.position_offset_rad;
    } else {
      // Stale or never received: report NaN until live data arrives.
      hw_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
      if (!encoder_feedback_->absEverReceived(ch)) {
        RCLCPP_WARN_THROTTLE(logger_, clock_, 5000,
                             "Joint '%s': no absolute encoder frame received yet (ID 0x%08X)",
                             cfg.name.c_str(), cfg.encoder_abs_can_id);
      }
    }

    if (encoder_feedback_->speedFresh(ch, fresh_window)) {
      hw_states_velocity_[i] = dir_scale * encoder_feedback_->velocityRads(ch);
    } else {
      hw_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmCanInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!frame_builder_) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointConfig & cfg = joints_[i];
    const double raw = hw_commands_velocity_[i];
    if (std::isnan(raw)) {
      continue;
    }

    switch (cfg.kind) {
      case JointKind::ARM_MOTOR: {
        // Treat the velocity command as a normalized [-1, 1] signal scaled to
        // the firmware payload range, matching the legacy can_controller_node
        // behaviour. The scaling is: raw * velocity_scale * ARM_MOTOR_VELOCITY_MAX.
        const double scaled = raw * static_cast<double>(cfg.direction);
        float payload = (std::abs(scaled) < kCommandDeadzone)
                          ? 0.0f
                          : static_cast<float>(scaled * cfg.velocity_scale * kArmMotorPayloadMax);
        payload = std::clamp(payload, kArmMotorPayloadMin, kArmMotorPayloadMax);
        frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER,
                                             cfg.arm_inst,
                                             DeviceId::ID::ARM_MOTOR_CONTROLLER, payload);
        break;
      }
      case JointKind::SPIN_SERVO: {
        const int32_t deg = static_cast<int32_t>(
            std::round(std::clamp(static_cast<float>(raw), -1.0f, 1.0f) * cfg.servo_max));
        if (deg != 0) {
          frame_builder_->sendSpinServoPosition(deg);
        }
        break;
      }
      case JointKind::CLAMP_SERVO: {
        const int32_t deg = static_cast<int32_t>(
            std::round(std::clamp(static_cast<float>(raw), -1.0f, 1.0f) * cfg.servo_max));
        if (deg != 0) {
          frame_builder_->sendClampServoPosition(deg);
        }
        break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ArmCanInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifs;
  ifs.reserve(joints_.size() * 2);
  for (size_t i = 0; i < joints_.size(); ++i) {
    ifs.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    ifs.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return ifs;
}

std::vector<hardware_interface::CommandInterface> ArmCanInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifs;
  ifs.reserve(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    ifs.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
  }
  return ifs;
}

}  // namespace arm_can_hardware

PLUGINLIB_EXPORT_CLASS(arm_can_hardware::ArmCanInterface, hardware_interface::SystemInterface)
