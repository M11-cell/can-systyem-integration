// encoder_telemetry_node.cpp
//
// Standalone ROS 2 node that reads arm encoder telemetry from the CAN bus
// and prints decoded position / velocity / status for each joint.
//
// One instance covers all four encoder boards (BASE, SHOULDER, ELBOW, WRIST)
// on a single SocketCAN interface.
//
// Usage:
//   ros2 run encoder-boards encoder_telemetry_node
//   ros2 run encoder-boards encoder_telemetry_node --ros-args
//       -p can_interface:=can0
//       -p print_period_ms:=500
//       -p stale_threshold_ms:=1500
//       -p log_unknown_encoder_frames:=true
//
// Parameters:
//   can_interface              (string, default "can0")   SocketCAN device.
//   print_period_ms            (int,    default 1000)     Print interval (ms).
//   stale_threshold_ms         (int,    default 1500)     Frames older than
//                                                         this are shown as
//                                                         [STALE]. Default 3×
//                                                         the ~500 ms firmware
//                                                         loop period.
//   log_unknown_encoder_frames (bool,   default false)    When true, log a
//                                                         throttled WARN for
//                                                         frames whose devtype
//                                                         (0x01) and mfr (0x08)
//                                                         match encoder boards
//                                                         but whose full ID is
//                                                         not in the watch list.
//
// Notes on data quality (see questions.md §F):
//   * Due to a firmware call-site bug, position frames currently transmit a
//     truncated float (0–3) instead of raw uint16 counts, so position values
//     will be 0 or a tiny non-zero constant until the firmware is fixed.
//   * Velocity is in rad/s with a PDP-endian swap applied by firmware; this
//     node undoes the swap correctly.
//   * validity == 0x00 means the TLE5012B read failed; angle is forced to 0.

#include "encoder-boards/arm_encoder_feedback.hpp"
#include "can-utils/can_connect.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// Encoder TX frames use devtype=0x01 (Jetson field) and mfr=0x08 (TEAM_USE).
// Unknown-frame filtering matches this prefix and warns if the full ID is not
// in our watch list.
constexpr uint8_t kEncTxDevtype = 0x01;
constexpr uint8_t kEncTxMfr     = 0x08;

std::string formatRow(const std::string & label,
                      bool abs_fresh,
                      bool abs_ever,
                      bool speed_fresh,
                      bool speed_ever,
                      int16_t counts,
                      double pos_rad,
                      double vel_rads,
                      float direction,
                      uint16_t status_word,
                      uint8_t validity)
{
  std::ostringstream ss;
  ss << "  " << std::left << std::setw(20) << label;

  if (!abs_ever) {
    ss << "  [NO ABS FRAME YET]";
    return ss.str();
  }

  if (validity != 0x01) {
    ss << "  pos=-- vel=-- status=0x"
       << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << status_word
       << " valid=0 [SENSOR ERROR]";
    return ss.str();
  }

  const double pos_display = pos_rad * static_cast<double>(direction);
  ss << std::dec << std::setfill(' ');
  ss << "  pos=" << std::fixed << std::setprecision(4) << pos_display << "rad"
     << " (counts=" << std::setw(6) << counts << ")";

  if (speed_ever) {
    const double vel_display = vel_rads * static_cast<double>(direction);
    ss << "  vel=" << std::fixed << std::setprecision(4) << vel_display << "rad/s";
  } else {
    ss << "  vel=[NO SPEED YET]";
  }

  ss << "  status=0x"
     << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << status_word
     << " valid=" << std::dec << static_cast<int>(validity);

  if (!abs_fresh) {
    ss << "  [STALE]";
  } else if (!speed_fresh && speed_ever) {
    ss << "  [speed stale]";
  } else {
    ss << "  [ok]";
  }

  return ss.str();
}
}  // namespace


class EncoderTelemetryNode : public rclcpp::Node
{
public:
  EncoderTelemetryNode()
  : Node("encoder_telemetry_node")
  {
    can_interface_   = declare_parameter<std::string>("can_interface", "can0");
    const int period_ms = declare_parameter<int>("print_period_ms", 1000);
    stale_ms_        = std::chrono::milliseconds(declare_parameter<int>("stale_threshold_ms", 1500));
    log_unknown_     = declare_parameter<bool>("log_unknown_encoder_frames", false);

    can_ = can_util::createConfiguredCanController(can_interface_, get_logger());
    if (!can_) {
      throw std::runtime_error(
        "CAN configure failed on '" + can_interface_ + "' — see log for hints");
    }

    rclcpp::on_shutdown([weak_can = std::weak_ptr<can_util::CANController>(can_)] {
      if (auto can = weak_can.lock()) {
        can->stop();
      }
    });

    encoder_ = std::make_shared<encoder_boards::ArmEncoderFeedback>(can_);

    if (log_unknown_) {
      // Build a fast lookup set of all watched IDs for the unknown-frame filter.
      for (const auto & ch : encoder_boards::ArmEncoderFeedback::defaultChannels()) {
        watched_ids_.push_back(ch.abs_can_id);
        watched_ids_.push_back(ch.speed_can_id);
      }
      unknown_frame_cb_ = can_->registerFrameCallback(
        [this](uint32_t id, const std::vector<uint8_t> & data) {
          onAnyFrame(id, data);
        });
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      [this]() { printTelemetry(); });

    RCLCPP_INFO(get_logger(),
                "encoder_telemetry_node ready — interface=%s, period=%d ms, "
                "stale=%ld ms, log_unknown=%s",
                can_interface_.c_str(), period_ms,
                static_cast<long>(stale_ms_.count()),
                log_unknown_ ? "true" : "false");
  }

  ~EncoderTelemetryNode() override
  {
    if (can_) {
      can_->stop();
    }
  }

private:
  void printTelemetry()
  {
    std::ostringstream ss;
    ss << "\n--- Arm Encoder Telemetry (iface=" << can_interface_ << ") ---";

    for (size_t i = 0; i < encoder_->numChannels(); ++i) {
      ss << "\n" << formatRow(
        encoder_->label(i),
        encoder_->absFresh(i, stale_ms_),
        encoder_->absEverReceived(i),
        encoder_->speedFresh(i, stale_ms_),
        encoder_->speedEverReceived(i),
        encoder_->rawCounts(i),
        encoder_->positionRad(i),
        encoder_->velocityRads(i),
        encoder_->direction(i),
        encoder_->statusWord(i),
        encoder_->validity(i));
    }

    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  void onAnyFrame(uint32_t id, const std::vector<uint8_t> & data)
  {
    const uint32_t arb_id = id & 0x1FFFFFFFu;
    const uint8_t devtype = static_cast<uint8_t>((arb_id >> 24) & 0x1Fu);
    if (devtype != kEncTxDevtype) {
      return;
    }
    const uint8_t mfr = static_cast<uint8_t>((arb_id >> 16) & 0xFFu);
    if (mfr != kEncTxMfr) {
      return;
    }
    // Check if this ID is one we already watch.
    for (uint32_t watched : watched_ids_) {
      if (watched == arb_id) {
        return;
      }
    }

    std::ostringstream ss;
    ss << "Unmatched encoder-typed frame id=0x"
       << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << arb_id
       << " data=";
    for (auto b : data) {
      ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ss.str().c_str());
  }

  std::string can_interface_;
  std::chrono::milliseconds stale_ms_{1500};
  bool log_unknown_{false};

  std::shared_ptr<can_util::CANController> can_;
  std::shared_ptr<encoder_boards::ArmEncoderFeedback> encoder_;
  std::shared_ptr<can_util::CANFrameCallback> unknown_frame_cb_;
  std::vector<uint32_t> watched_ids_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<EncoderTelemetryNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("encoder_telemetry_node"),
                 "Node failed to start: %s", e.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("encoder_telemetry_node"),
                 "Node failed to start: unknown exception");
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
