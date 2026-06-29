// Shared wheel telemetry table formatting for wheel_bench_node and
// WheelCanInterface periodic diagnostics.
#pragma once

#include "can-utils/spark_max_feedback.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace wheel_can_hardware::telemetry
{

inline const char * labelForDevice(uint8_t device_id)
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

inline std::string formatRow(const char * label,
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

// Like formatRow but optionally prefixes ctrl/tgt when traction correction is active.
inline std::string formatRos2ControlRow(const char * label,
                                        bool vit_fresh,
                                        bool vit_ever,
                                        bool status0_fresh,
                                        bool status2_fresh,
                                        bool status2_ever,
                                        float ctrl_rpm,
                                        float tgt_rpm,
                                        float out_rpm,
                                        bool show_traction,
                                        const spark_max::WheelFeedback & fb)
{
  if (!show_traction) {
    return formatRow(
      label, vit_fresh, vit_ever, status0_fresh, status2_fresh, status2_ever,
      out_rpm, fb);
  }

  std::ostringstream ss;
  ss << "  " << std::left << std::setw(12) << label
     << " ctrl=" << std::fixed << std::setprecision(1) << std::setw(8) << ctrl_rpm << "rpm"
     << " tgt=" << std::fixed << std::setprecision(1) << std::setw(8) << tgt_rpm << "rpm"
     << " out=" << std::fixed << std::setprecision(1) << std::setw(8) << out_rpm << "rpm";

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

}  // namespace wheel_can_hardware::telemetry
