// arm_can_interface.hpp
//
// ros2_control SystemInterface plugin that drives the rover arm motors and
// gripper servos over the team CAN protocol. Replaces the serial-based
// arm_interface from the inverse-kinematics workspace.
//
// Joint configuration is taken from the URDF <ros2_control> block. Per-joint
// parameters declared with <param name="..."> are used to map URDF joints onto
// CAN motor IDs / servo selectors.

#pragma once

#include "can-utils/can_interface.hpp"
#include "can-utils/prefixes.hpp"
#include "can-utils/system_controller.hpp"
#include "encoder-boards/arm_encoder_feedback.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace arm_can_hardware
{

// What kind of CAN device a URDF joint maps onto. ARM_MOTOR uses
// SystemFrameBuilder::sendArmMotorVelocity(); SPIN_SERVO / CLAMP_SERVO use the
// dedicated servo helpers and respect a position-vs-speed mode parameter.
enum class JointKind
{
  ARM_MOTOR,
  SPIN_SERVO,
  CLAMP_SERVO,
};

enum class ServoMode
{
  POSITION,       // cmd × servo_max = degrees, sent via unified servo protocol
};

// Per-joint configuration parsed once during on_init() from the URDF.
struct JointConfig
{
  std::string name;
  JointKind kind{JointKind::ARM_MOTOR};

  // ARM_MOTOR fields
  Instructions::Inst arm_inst{Instructions::Inst::ARM_MOTOR_1};
  // Multiplicative gain applied to the [-1, 1] command before scaling to the
  // ARM_MOTOR_VELOCITY_MAX range. Useful for joints that need to be slowed
  // (e.g. motor 4 historically ran at 0.5 of the others).
  float velocity_scale{1.0f};
  // +1 or -1: aligns URDF/ros2_control sign with motor and encoder (+X joint axis).
  float direction{1.0f};
  // Legacy encoder device id (bits[5:0] of the RX command ID). Kept for
  // documentation compatibility; routing now uses the full 29-bit TX IDs below.
  uint32_t encoder_device_id{0};

  // Full 29-bit arbitration IDs of encoder TX frames (masked, no EFF flag).
  // Populated from URDF params or derived from encoder_device_id defaults.
  // 0 = not connected (feedback disabled for this joint).
  uint32_t encoder_abs_can_id{0};    // absolute position frame
  uint32_t encoder_speed_can_id{0};  // angular velocity frame

  // Calibration applied to the raw encoder reading in read():
  //   joint_position = direction * position_scale * raw_rad + position_offset_rad
  //   joint_velocity = direction * position_scale * raw_rad_s
  // Defaults are identity; real values come from bench calibration once the
  // encoder boards are installed (docs/testing/08-test-arm-encoders.md).
  double position_scale{1.0};
  double position_offset_rad{0.0};

  // Index into ArmCanInterface::encoder_feedback_ when has_encoder is true.
  bool has_encoder{false};
  size_t channel_index{0};

  // SERVO fields
  ServoMode servo_mode{ServoMode::POSITION};
  float servo_max{1.5707963f};
};

class ArmCanInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmCanInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  static JointKind parseKind(const std::string & s);
  static ServoMode parseServoMode(const std::string & s);
  // Parse an instruction byte from a hex/dec string in the URDF param.
  static Instructions::Inst parseInstruction(const std::string & s);

  // Read a string parameter from a joint's <param> map, returning fallback if
  // the key is missing.
  static std::string getParam(const hardware_interface::ComponentInfo & joint,
                              const std::string & key,
                              const std::string & fallback);

  std::string can_interface_name_{"can0"};
  bool send_heartbeat_on_activate_{true};
  // Encoder feedback older than this is treated as stale (position/velocity
  // reported as NaN). Encoder boards transmit at ~2 Hz, so the default is
  // generous. Override with <param name="feedback_freshness_ms"> in the URDF.
  float feedback_freshness_ms_{500.0f};

  std::shared_ptr<can_util::CANController> can_controller_;
  std::unique_ptr<SystemFrameBuilder> frame_builder_;

  // Decodes the arm encoder CAN frames and stores the latest value per channel.
  // Owns its own frame callback on the shared CANController. Built in
  // on_configure() from encoder_channels_ populated during on_init().
  std::unique_ptr<encoder_boards::ArmEncoderFeedback> encoder_feedback_;
  std::vector<encoder_boards::EncoderChannel> encoder_channels_;

  // Indexed parallel to info_.joints / hw_*_ vectors.
  std::vector<JointConfig> joints_;

  // ros2_control state buffers (one entry per URDF joint).
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_velocity_;

  rclcpp::Logger logger_{rclcpp::get_logger("arm_can_interface")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace arm_can_hardware
