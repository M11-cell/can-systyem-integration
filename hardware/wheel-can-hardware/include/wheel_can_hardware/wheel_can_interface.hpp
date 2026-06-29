// wheel_can_interface.hpp
//
// ros2_control SystemInterface plugin that drives the rover's six REV
// SPARK MAX wheel motors over CAN. Velocity commands flow rad/s -> RPM ->
// SystemFrameBuilder::sendWheelMotorVelocity(). Encoder feedback comes back
// from the SPARK MAX STATUS_2 frames decoded by spark_max::SparkMaxFeedback
// and is exposed in the standard ros2_control rad / rad/s units.
//
// The plugin also implements an opt-in anti-slip differential to help with
// skid-steer (tank) turns. See WheelCanInterface::write() for the details.

#pragma once

#include "can-utils/can_interface.hpp"
#include "can-utils/spark_max_feedback.hpp"
#include "can-utils/system_controller.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace wheel_can_hardware
{

enum class WheelSide
{
  LEFT,
  RIGHT,
};

struct WheelConfig
{
  std::string name;
  // SPARK MAX device id (1..62) on the CAN bus.
  uint8_t device_id{0};
  WheelSide side{WheelSide::LEFT};
  // -1.0f or +1.0f. Multiplied into the commanded RPM to flip direction for
  // motors that are physically mounted reversed (the legacy controller node
  // hard-coded the right-hand side as inverted).
  float direction{1.0f};
  // Motor reductions: motor revolutions per wheel revolution. RPM and
  // rotations from the SPARK MAX are at the motor shaft; we divide by this
  // to express feedback at the wheel.
  float gear_ratio{1.0f};
};

class WheelCanInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WheelCanInterface)

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
  // Anti-slip mode selector (string param "anti_slip_mode" in URDF / YAML).
  enum class AntiSlipMode
  {
    OFF,         // pass commands through unchanged
    CLAMP,       // Level 1: only attenuate wheels spinning faster than commanded
    PID,         // Level 2: proportional correction in both directions
    CURRENT_PID  // Level 3: PID + STATUS_0 current threshold to back off stalls
  };

  // Traction mode selector (string param "traction_mode" in URDF / YAML).
  // Traction redistributes a side's RPM share toward gripping wheels before
  // anti-slip runs; see applyWheelCorrection().
  enum class TractionMode
  {
    OFF,         // target_rpm == commanded_rpm
    ASSIST,      // redistribute using traction_min_weight floor
    AGGRESSIVE   // same but with a lower effective min weight
  };

  // Run the full per-cycle correction for all wheels:
  //   commanded_rpm --(traction, per-side renormalize)--> target_rpm
  //                 --(anti-slip vs target only)--------> output_rpm
  // Anti-slip uses target_rpm as its setpoint, never commanded_rpm. All input
  // vectors are indexed by wheel and sized wheels_.size(); outputs are resized
  // to match.
  void applyWheelCorrection(const std::vector<float> & commanded_rpm,
                            const std::vector<float> & measured_rpm,
                            const std::vector<float> & current_a,
                            const std::vector<uint8_t> & feedback_fresh,
                            std::vector<float> & target_rpm,
                            std::vector<float> & output_rpm);

  // Apply the given anti-slip strategy to one wheel using target_rpm as the
  // setpoint and its measured RPM. Returns the corrected motor RPM. If the
  // mode is OFF or feedback is stale, returns target_rpm unchanged.
  //
  // motor_current_a is the SPARK MAX current draw in amperes, used by
  // CURRENT_PID to detect stalled wheels (high current + low velocity) and
  // back off the demand instead of fighting the obstacle.
  float applyAntiSlip(AntiSlipMode mode, float target_rpm, float measured_rpm,
                      float motor_current_a, bool feedback_fresh);

  // Wheel command mode. VELOCITY sends RPM setpoints (with traction +
  // anti-slip); CURRENT sends amperes for torque-like control (open loop,
  // no traction / anti-slip — Phase 4 bench mode).
  enum class ControlMode
  {
    VELOCITY,
    CURRENT
  };

  static AntiSlipMode parseMode(const std::string & s);
  static TractionMode parseTractionMode(const std::string & s);
  static ControlMode parseControlMode(const std::string & s);

  void maybePrintTelemetry(const std::vector<float> & commanded_rpm,
                           const std::vector<float> & target_rpm,
                           const std::vector<float> & output_rpm);

  static std::string getParam(const hardware_interface::ComponentInfo & joint,
                              const std::string & key,
                              const std::string & fallback);
  static std::string getParam(const std::unordered_map<std::string, std::string> & params,
                              const std::string & key,
                              const std::string & fallback);

  // Hardware-level params.
  std::string can_interface_name_{"can0"};
  bool send_heartbeat_on_activate_{true};
  uint64_t heartbeat_motor_mask_{0x7Eu};

  // Command mode. CURRENT exports an effort command interface (amperes).
  ControlMode control_mode_{ControlMode::VELOCITY};
  float max_current_a_{40.0f};   // magnitude clamp on the current setpoint

  // Anti-slip configuration. Defaults match the plan's Phase 5 values.
  AntiSlipMode anti_slip_mode_{AntiSlipMode::PID};
  float  slip_threshold_{0.30f};        // |actual - target| / max(|target|, 1) over which we react
  float  slip_kp_{0.10f};               // proportional gain on the (target - actual) error
  float  max_slip_correction_{0.50f};   // max fraction of |target| to add as correction
  float  feedback_freshness_ms_{100.0f};
  float  stall_current_a_{25.0f};       // current above which we treat the wheel as stalled
  float  stall_relief_factor_{0.50f};   // multiplicative reduction applied when a stall is detected

  // Traction configuration (Step A of applyWheelCorrection). Defaults keep
  // traction off so behaviour is unchanged unless the URDF opts in.
  TractionMode traction_mode_{TractionMode::OFF};
  float  traction_slip_free_threshold_{0.4f};   // slip-vs-cmd above which a wheel is "free spinning"
  float  traction_slip_stall_threshold_{-0.3f}; // slip-vs-cmd below which a wheel is "dragging/stalled"
  float  traction_min_weight_{0.1f};            // floor on a wheel's redistribution weight
  float  traction_low_load_a_{2.0f};            // current below which a free-spinning wheel is airborne
  float  traction_weight_k_{0.5f};              // proportional weight reduction per unit slip

  std::shared_ptr<can_util::CANController>      can_controller_;
  std::unique_ptr<SystemFrameBuilder>           frame_builder_;
  std::unique_ptr<spark_max::SparkMaxFeedback>  feedback_;

  std::vector<WheelConfig> wheels_;

  // ros2_control state buffers (one entry per wheel joint, units rad / rad/s).
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;   // amperes, used in CURRENT mode

  // Last computed motor RPM per wheel; used for diagnostics / debug logging.
  std::vector<float> last_motor_rpm_cmd_;

  bool log_telemetry_{false};
  int print_period_ms_{1000};
  std::chrono::steady_clock::time_point last_telemetry_log_{};

  rclcpp::Logger logger_{rclcpp::get_logger("wheel_can_interface")};
};

}  // namespace wheel_can_hardware
