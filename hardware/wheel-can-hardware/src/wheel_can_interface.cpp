#include "wheel_can_hardware/wheel_can_interface.hpp"

#include "can-utils/can_connect.hpp"
#include "can-utils/prefixes.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace wheel_can_hardware
{

namespace
{
// rad/s -> RPM conversion factor (60 / (2 * pi)).
constexpr double kRadSToRpm = 9.5492965855137201;
// RPM -> rad/s conversion factor (2 * pi / 60).
constexpr double kRpmToRadS = 0.10471975511965977;
// rotations -> radians.
constexpr double kRotToRad = 6.283185307179586;
}  // namespace

std::string WheelCanInterface::getParam(const hardware_interface::ComponentInfo & joint,
                                        const std::string & key,
                                        const std::string & fallback)
{
  auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    return fallback;
  }
  return it->second;
}

std::string WheelCanInterface::getParam(const std::unordered_map<std::string, std::string> & params,
                                        const std::string & key,
                                        const std::string & fallback)
{
  auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  return it->second;
}

hardware_interface::CallbackReturn WheelCanInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;
  can_interface_name_         = getParam(hp, "can_interface", can_interface_name_);
  send_heartbeat_on_activate_ = (getParam(hp, "send_heartbeat_on_activate", "true") != "false");
  try {
    heartbeat_motor_mask_ = static_cast<uint64_t>(
      std::stoull(getParam(hp, "heartbeat_motor_mask", "0x7E"), nullptr, 0));
  } catch (const std::exception &) {
    heartbeat_motor_mask_ = 0x7Eu;
  }

  // Two ways to disable anti-slip: anti_slip_enabled=false (legacy) or
  // anti_slip_mode=off. Both are honoured.
  if (getParam(hp, "anti_slip_enabled", "true") == "false") {
    anti_slip_mode_ = AntiSlipMode::OFF;
  } else {
    anti_slip_mode_ = parseMode(getParam(hp, "anti_slip_mode", "pid"));
  }
  try { slip_threshold_        = std::stof(getParam(hp, "slip_threshold",        "0.30")); } catch (...) {}
  try { slip_kp_               = std::stof(getParam(hp, "slip_kp",               "0.10")); } catch (...) {}
  try { max_slip_correction_   = std::stof(getParam(hp, "max_slip_correction",   "0.50")); } catch (...) {}
  try { feedback_freshness_ms_ = std::stof(getParam(hp, "feedback_freshness_ms", "100.0")); } catch (...) {}
  try { stall_current_a_       = std::stof(getParam(hp, "stall_current_a",       "30.0")); } catch (...) {}
  try { stall_relief_factor_   = std::stof(getParam(hp, "stall_relief_factor",   "0.50")); } catch (...) {}

  control_mode_ = parseControlMode(getParam(hp, "control_mode", "velocity"));
  try { max_current_a_ = std::stof(getParam(hp, "max_current_a", "40.0")); } catch (...) {}

  // Traction (Step A of applyWheelCorrection). Defaults keep it off.
  traction_mode_ = parseTractionMode(getParam(hp, "traction_mode", "off"));
  try { traction_slip_free_threshold_  = std::stof(getParam(hp, "traction_slip_free_threshold",  "0.4"));  } catch (...) {}
  try { traction_slip_stall_threshold_ = std::stof(getParam(hp, "traction_slip_stall_threshold", "-0.3")); } catch (...) {}
  try { traction_min_weight_           = std::stof(getParam(hp, "traction_min_weight",           "0.1"));  } catch (...) {}
  try { traction_low_load_a_           = std::stof(getParam(hp, "traction_low_load_a",           "2.0"));  } catch (...) {}
  try { traction_weight_k_             = std::stof(getParam(hp, "traction_weight_k",             "0.5"));  } catch (...) {}

  if (info_.joints.empty()) {
    RCLCPP_FATAL(logger_, "No wheel joints declared in <ros2_control>");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::string expected_cmd_if =
    (control_mode_ == ControlMode::CURRENT) ? hardware_interface::HW_IF_EFFORT
                                            : hardware_interface::HW_IF_VELOCITY;

  wheels_.clear();
  wheels_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != expected_cmd_if) {
      RCLCPP_FATAL(logger_, "Wheel joint '%s' must declare one %s command interface",
                   joint.name.c_str(), expected_cmd_if.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    WheelConfig cfg;
    cfg.name = joint.name;
    try {
      cfg.device_id = static_cast<uint8_t>(std::stoul(getParam(joint, "device_id", "0"), nullptr, 0));
    } catch (const std::exception &) {
      cfg.device_id = 0;
    }
    if (cfg.device_id == 0) {
      RCLCPP_FATAL(logger_, "Wheel joint '%s' is missing a non-zero <param name=\"device_id\">",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto side = getParam(joint, "side", "left");
    cfg.side = (side == "right") ? WheelSide::RIGHT : WheelSide::LEFT;

    try { cfg.direction  = std::stof(getParam(joint, "direction", "1.0")); } catch (...) {}
    try { cfg.gear_ratio = std::stof(getParam(joint, "gear_ratio", "1.0")); } catch (...) {}
    if (cfg.gear_ratio == 0.0f) {
      cfg.gear_ratio = 1.0f;
    }

    wheels_.push_back(cfg);
  }

  const size_t n = wheels_.size();
  hw_commands_velocity_.assign(n, 0.0);
  hw_commands_effort_.assign(n, 0.0);
  hw_states_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());
  last_motor_rpm_cmd_.assign(n, 0.0f);

  const char * mode_str =
    (anti_slip_mode_ == AntiSlipMode::OFF)         ? "off"        :
    (anti_slip_mode_ == AntiSlipMode::CLAMP)       ? "clamp"      :
    (anti_slip_mode_ == AntiSlipMode::PID)         ? "pid"        :
    /*CURRENT_PID*/                                  "current_pid";
  const char * traction_str =
    (traction_mode_ == TractionMode::OFF)        ? "off"    :
    (traction_mode_ == TractionMode::ASSIST)     ? "assist" :
    /*AGGRESSIVE*/                                 "aggressive";
  const char * control_str =
    (control_mode_ == ControlMode::CURRENT) ? "current" : "velocity";
  RCLCPP_INFO(logger_,
              "Initialised WheelCanInterface with %zu wheels on CAN '%s' "
              "(control_mode=%s, traction_mode=%s, anti_slip_mode=%s)",
              wheels_.size(), can_interface_name_.c_str(), control_str, traction_str, mode_str);
  return hardware_interface::CallbackReturn::SUCCESS;
}

WheelCanInterface::AntiSlipMode WheelCanInterface::parseMode(const std::string & s)
{
  if (s == "off")          return AntiSlipMode::OFF;
  if (s == "clamp")        return AntiSlipMode::CLAMP;
  if (s == "current_pid")  return AntiSlipMode::CURRENT_PID;
  return AntiSlipMode::PID;
}

WheelCanInterface::TractionMode WheelCanInterface::parseTractionMode(const std::string & s)
{
  if (s == "assist")      return TractionMode::ASSIST;
  if (s == "aggressive")  return TractionMode::AGGRESSIVE;
  return TractionMode::OFF;
}

WheelCanInterface::ControlMode WheelCanInterface::parseControlMode(const std::string & s)
{
  if (s == "current")  return ControlMode::CURRENT;
  return ControlMode::VELOCITY;
}

hardware_interface::CallbackReturn WheelCanInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    can_controller_ = can_util::getSharedCanController(can_interface_name_, logger_);
    if (!can_controller_) {
      can_controller_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
    frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

    std::vector<uint8_t> ids;
    ids.reserve(wheels_.size());
    for (const auto & w : wheels_) {
      ids.push_back(w.device_id);
    }
    feedback_ = std::make_unique<spark_max::SparkMaxFeedback>(can_controller_, std::move(ids));

    // Tell every SPARK MAX to start broadcasting STATUS_2 (velocity + position).
    // It's idempotent so safe to call again on reconfigure / after a power
    // cycle of the motor controllers.
    if (!feedback_->enableStatus2()) {
      RCLCPP_WARN(logger_, "One or more SPARK MAX SET_STATUSES_ENABLED frames failed to send");
    } else {
      RCLCPP_INFO(logger_, "STATUS_2 enable command sent to %zu SPARK MAX motors", wheels_.size());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      logger_, "Exception during WheelCanInterface configure on '%s': %s",
      can_interface_name_.c_str(), e.what());
    can_util::logCanSetupRecoveryHints(logger_, can_interface_name_);
    can_controller_.reset();
    frame_builder_.reset();
    feedback_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_FATAL(
      logger_, "Unknown exception during WheelCanInterface configure on '%s'",
      can_interface_name_.c_str());
    can_util::logCanSetupRecoveryHints(logger_, can_interface_name_);
    can_controller_.reset();
    frame_builder_.reset();
    feedback_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn WheelCanInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  std::fill(hw_commands_effort_.begin(), hw_commands_effort_.end(), 0.0);
  std::fill(last_motor_rpm_cmd_.begin(), last_motor_rpm_cmd_.end(), 0.0f);

  if (send_heartbeat_on_activate_ && frame_builder_) {
    frame_builder_->startMotors(heartbeat_motor_mask_);
  }
  RCLCPP_INFO(logger_, "WheelCanInterface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheelCanInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send a final zero setpoint to every wheel to make sure nothing keeps
  // moving once the controller is detached.
  if (frame_builder_) {
    for (const auto & w : wheels_) {
      const auto did = static_cast<DeviceId::ID>(w.device_id);
      if (control_mode_ == ControlMode::CURRENT) {
        frame_builder_->sendWheelMotorCurrent(did, 0.0f);
      } else {
        frame_builder_->sendWheelMotorVelocity(did, 0.0f);
      }
    }
  }
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  std::fill(hw_commands_effort_.begin(), hw_commands_effort_.end(), 0.0);
  std::fill(last_motor_rpm_cmd_.begin(), last_motor_rpm_cmd_.end(), 0.0f);
  RCLCPP_INFO(logger_, "WheelCanInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheelCanInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!feedback_) {
    return hardware_interface::return_type::ERROR;
  }

  spark_max::WheelFeedback fb{};
  for (size_t i = 0; i < wheels_.size(); ++i) {
    if (!feedback_->getFeedback(wheels_[i].device_id, fb)) {
      continue;
    }
    // SPARK MAX reports velocity at the motor shaft; divide by the gear
    // reduction to get wheel-side units. Direction inversion is applied so
    // that "positive command -> rover moves forward" stays consistent for
    // both sides.
    const double motor_velocity_rad_s = static_cast<double>(fb.velocity_rpm) * kRpmToRadS;
    const double motor_position_rad   = static_cast<double>(fb.position_rot) * kRotToRad;
    hw_states_velocity_[i] = motor_velocity_rad_s / wheels_[i].gear_ratio * wheels_[i].direction;
    hw_states_position_[i] = motor_position_rad   / wheels_[i].gear_ratio * wheels_[i].direction;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheelCanInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!frame_builder_) {
    return hardware_interface::return_type::ERROR;
  }

  const size_t n = wheels_.size();

  // Current (torque) mode is open loop: clamp the effort command to amperes
  // and send it straight through. Traction and anti-slip do not apply.
  if (control_mode_ == ControlMode::CURRENT) {
    for (size_t i = 0; i < n; ++i) {
      const WheelConfig & w = wheels_[i];
      float amps = static_cast<float>(hw_commands_effort_[i]) * w.direction;
      amps = std::clamp(amps, -max_current_a_, max_current_a_);
      last_motor_rpm_cmd_[i] = amps;
      const auto did = static_cast<DeviceId::ID>(w.device_id);
      frame_builder_->sendWheelMotorCurrent(did, amps);
    }
    frame_builder_->startMotors(heartbeat_motor_mask_);
    return hardware_interface::return_type::OK;
  }

  spark_max::WheelFeedback fb{};
  const auto fresh_window = std::chrono::milliseconds(static_cast<int>(feedback_freshness_ms_));

  // 1) Convert every controller command (rad/s at the wheel) into a SPARK MAX
  //    setpoint (RPM at the motor shaft) and snapshot feedback for all wheels.
  std::vector<float> commanded_rpm(n, 0.0f);
  std::vector<float> measured_rpm(n, 0.0f);
  std::vector<float> current_a(n, 0.0f);
  std::vector<uint8_t> fresh(n, 0u);

  for (size_t i = 0; i < n; ++i) {
    const WheelConfig & w = wheels_[i];
    const double wheel_rad_s = hw_commands_velocity_[i];
    const double motor_rpm   = wheel_rad_s * kRadSToRpm * static_cast<double>(w.gear_ratio);
    commanded_rpm[i] = static_cast<float>(motor_rpm) * w.direction;

    if (feedback_ && feedback_->getFeedback(w.device_id, fb) && fb.status2_seen) {
      measured_rpm[i] = fb.velocity_rpm;
      current_a[i]    = fb.current_a;
      const auto age = std::chrono::steady_clock::now() - fb.status2_stamp;
      fresh[i] = (age <= fresh_window) ? 1u : 0u;
    }
  }

  // 2) commanded -> target (traction) -> output (anti-slip vs target).
  std::vector<float> target_rpm;
  std::vector<float> output_rpm;
  applyWheelCorrection(commanded_rpm, measured_rpm, current_a, fresh, target_rpm, output_rpm);

  // 3) Send.
  for (size_t i = 0; i < n; ++i) {
    last_motor_rpm_cmd_[i] = output_rpm[i];
    const auto did = static_cast<DeviceId::ID>(wheels_[i].device_id);
    frame_builder_->sendWheelMotorVelocity(did, output_rpm[i]);
  }

  // Optional throttled diagnostic when traction is active: log the
  // commanded / target / output / measured RPM per wheel once a second.
  if (traction_mode_ != TractionMode::OFF) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_diag_log_ >= std::chrono::seconds(1)) {
      last_diag_log_ = now;
      for (size_t i = 0; i < n; ++i) {
        RCLCPP_INFO(logger_,
                    "[traction] %s cmd=%.0f tgt=%.0f out=%.0f meas=%.0f I=%.1fA%s",
                    wheels_[i].name.c_str(), commanded_rpm[i], target_rpm[i],
                    output_rpm[i], measured_rpm[i], current_a[i],
                    fresh[i] ? "" : " (stale)");
      }
    }
  }

  // Re-issue the SPARK MAX heartbeat / start-motors frame on every cycle, as
  // the legacy can_controller_node did. Without this the SPARKs latch into a
  // safe-disable state if the heartbeat times out.
  frame_builder_->startMotors(heartbeat_motor_mask_);
  return hardware_interface::return_type::OK;
}

void WheelCanInterface::applyWheelCorrection(
  const std::vector<float> & commanded_rpm,
  const std::vector<float> & measured_rpm,
  const std::vector<float> & current_a,
  const std::vector<uint8_t> & feedback_fresh,
  std::vector<float> & target_rpm,
  std::vector<float> & output_rpm)
{
  const size_t n = wheels_.size();
  target_rpm.assign(n, 0.0f);
  output_rpm.assign(n, 0.0f);

  const bool traction_on = (traction_mode_ != TractionMode::OFF);
  // Aggressive lets a slipping wheel be cut harder (lower floor on its share).
  const float min_weight = (traction_mode_ == TractionMode::AGGRESSIVE)
                             ? std::min(traction_min_weight_, 0.05f)
                             : traction_min_weight_;

  // --- Step A: traction -> target_rpm ---
  // slip_vs_cmd > 0  : spinning faster than commanded (possibly airborne)
  // slip_vs_cmd < 0  : dragging / stalled
  std::vector<float> slip_vs_cmd(n, 0.0f);
  std::vector<float> weight(n, 1.0f);
  for (size_t i = 0; i < n; ++i) {
    const float denom = std::max(std::abs(commanded_rpm[i]), 1.0f);
    slip_vs_cmd[i] = (measured_rpm[i] - commanded_rpm[i]) / denom;

    if (!traction_on || !feedback_fresh[i]) {
      weight[i] = 1.0f;
    } else if (slip_vs_cmd[i] > traction_slip_free_threshold_ &&
               std::abs(current_a[i]) < traction_low_load_a_) {
      weight[i] = min_weight;  // free-spinning at low load -> likely airborne
    } else if (slip_vs_cmd[i] < traction_slip_stall_threshold_ &&
               std::abs(current_a[i]) > stall_current_a_) {
      weight[i] = 1.0f;        // gripping hard -> keep full share
    } else {
      const float reduce = std::clamp(traction_weight_k_ * slip_vs_cmd[i],
                                      0.0f, 1.0f - min_weight);
      weight[i] = 1.0f - reduce;
    }
  }

  if (!traction_on) {
    target_rpm = commanded_rpm;
  } else {
    // Per-side renormalize so each side keeps its overall demand while
    // shifting share toward the gripping wheels.
    for (const WheelSide side : {WheelSide::RIGHT, WheelSide::LEFT}) {
      float sum_cmd = 0.0f;
      float sum_w = 0.0f;
      for (size_t i = 0; i < n; ++i) {
        if (wheels_[i].side != side) { continue; }
        sum_cmd += std::abs(commanded_rpm[i]);
        sum_w   += std::abs(commanded_rpm[i] * weight[i]);
      }
      for (size_t i = 0; i < n; ++i) {
        if (wheels_[i].side != side) { continue; }
        target_rpm[i] = (sum_w > 1e-3f)
                          ? commanded_rpm[i] * weight[i] * (sum_cmd / sum_w)
                          : 0.0f;
      }
    }

    // Motor protect: a wheel that is both dragging and pulling high current is
    // fighting an obstacle; relieve it rather than redistributing more onto it.
    for (size_t i = 0; i < n; ++i) {
      if (feedback_fresh[i] &&
          slip_vs_cmd[i] < traction_slip_stall_threshold_ &&
          std::abs(current_a[i]) > stall_current_a_) {
        target_rpm[i] *= stall_relief_factor_;
      }
    }
  }

  // --- Step B: anti-slip vs target -> output_rpm ---
  // With traction active the stall guard is handled in Step A, so CURRENT_PID
  // degrades to plain PID to avoid double-counting current.
  const AntiSlipMode eff_mode =
    (traction_on && anti_slip_mode_ == AntiSlipMode::CURRENT_PID)
      ? AntiSlipMode::PID
      : anti_slip_mode_;

  for (size_t i = 0; i < n; ++i) {
    output_rpm[i] = applyAntiSlip(eff_mode, target_rpm[i], measured_rpm[i],
                                  current_a[i], feedback_fresh[i] != 0u);
  }
}

float WheelCanInterface::applyAntiSlip(AntiSlipMode mode,
                                       float target_rpm, float measured_rpm,
                                       float motor_current_a, bool feedback_fresh)
{
  if (mode == AntiSlipMode::OFF || !feedback_fresh) {
    return target_rpm;
  }

  // Slip ratio is signed: positive means the wheel is going faster than its
  // target (free-spinning), negative means it's lagging behind (stalled or
  // being dragged). The denominator avoids division by zero at very low
  // target speeds.
  const float denom = std::max(std::abs(target_rpm), 1.0f);
  const float slip_ratio = (measured_rpm - target_rpm) / denom;

  if (std::abs(slip_ratio) < slip_threshold_) {
    return target_rpm;  // within tolerance, no action needed
  }

  switch (mode) {
    case AntiSlipMode::OFF:
      return target_rpm;

    case AntiSlipMode::CLAMP: {
      // Only act on free-spin (positive slip ratio): pull the command down
      // toward the measured speed. Stalled wheels are left alone.
      if (slip_ratio <= 0.0f) {
        return target_rpm;
      }
      const float pull = slip_kp_ * (measured_rpm - target_rpm);
      const float cap = max_slip_correction_ * std::abs(target_rpm);
      const float correction = std::clamp(pull, -cap, cap);
      // pull is positive here (measured > target), so correction increases
      // target toward measured: use the negative direction to back off.
      return target_rpm + std::clamp(-correction, -cap, cap);
    }

    case AntiSlipMode::PID: {
      // Symmetric P controller: nudge the command toward the actual measured
      // velocity. Capped so a single noisy reading can't reverse the wheel.
      const float error = target_rpm - measured_rpm;
      const float cap = max_slip_correction_ * std::abs(target_rpm);
      const float correction = std::clamp(slip_kp_ * error, -cap, cap);
      return target_rpm + correction;
    }

    case AntiSlipMode::CURRENT_PID: {
      // Same PID as above plus a current-based stall guard: when the SPARK
      // MAX is drawing more than stall_current_a_ AND the wheel is dragging
      // (negative slip), reduce demand by stall_relief_factor_ instead of
      // pushing harder. This prevents the inside wheels from cooking the
      // motors during a tank turn against high-friction ground.
      const float error = target_rpm - measured_rpm;
      const float cap = max_slip_correction_ * std::abs(target_rpm);
      float corrected = target_rpm + std::clamp(slip_kp_ * error, -cap, cap);

      const bool stalled = (std::abs(motor_current_a) > stall_current_a_) && (slip_ratio < 0.0f);
      if (stalled) {
        corrected *= stall_relief_factor_;
      }
      return corrected;
    }
  }

  return target_rpm;
}

std::vector<hardware_interface::StateInterface> WheelCanInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifs;
  ifs.reserve(wheels_.size() * 2);
  for (size_t i = 0; i < wheels_.size(); ++i) {
    ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return ifs;
}

std::vector<hardware_interface::CommandInterface> WheelCanInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifs;
  ifs.reserve(wheels_.size());
  for (size_t i = 0; i < wheels_.size(); ++i) {
    if (control_mode_ == ControlMode::CURRENT) {
      ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]);
    } else {
      ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
    }
  }
  return ifs;
}

}  // namespace wheel_can_hardware

PLUGINLIB_EXPORT_CLASS(wheel_can_hardware::WheelCanInterface, hardware_interface::SystemInterface)
