// arm_encoder_feedback.hpp
//
// Decodes arm encoder CAN frames broadcast by the STM32 encoder boards
// (Base, Shoulder, Elbow, Wrist) and stores the latest values per channel.
//
// Each board transmits two frame types at ~2 Hz:
//   Abs  frame — DLC 6, full 29-bit ID per board (e.g. 0x0108C701 for BASE):
//     bytes 0–1  uint16 LE  calibrated angle counts (TLE5012B 15-bit signed)
//     bytes 2–3  uint16 LE  TLE5012B status register (masked 0x1EFE)
//     byte  4    uint8      reserved (0x00)
//     byte  5    uint8      validity flag (0x01 = valid, 0x00 = sensor error)
//   Speed frame — DLC 6, abs ID + 0x40:
//     bytes 0–3  float32    angular velocity rad/s, PDP (middle) endian
//                           firmware swaps within 16-bit halves → [b1,b0,b3,b2]
//                           consumer must undo: swap bytes 0↔1 and 2↔3 before cast
//     byte  4    uint8      sign flag (0 = positive, 1 = negative) — informational
//     bytes 5–7  uint8      reserved
//
// Full ID routing (not device-id-only) per questions.md §D.3.
// direction is a per-channel field applied by the consumer (telemetry node /
// hardware interface) when interpreting sign — not applied inside this class.
//
// Firmware bugs documented in questions.md §F that affect data quality
// (call-site truncation, Encoder_Zero no-op) do not affect the decode path;
// this class correctly decodes whatever the firmware actually transmits.

#pragma once

#include "can-utils/can_interface.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace encoder_boards
{

struct EncoderChannel
{
  std::string label;        // e.g. "joint1 / BASE"
  uint32_t abs_can_id{0};
  uint32_t speed_can_id{0};
  // Sign multiplier for the joint axis convention.  Not applied inside the
  // decoder; used by the caller (telemetry node, hardware interface).
  float direction{1.0f};

  // Decoded state — protected by ArmEncoderFeedback::mutex_.
  int16_t raw_counts{0};                    // TLE5012B raw 15-bit signed counts
  double position_rad{std::numeric_limits<double>::quiet_NaN()};
  double velocity_rads{std::numeric_limits<double>::quiet_NaN()};
  uint16_t status_word{0};
  uint8_t validity{0};

  std::chrono::steady_clock::time_point abs_stamp{};
  std::chrono::steady_clock::time_point speed_stamp{};
  bool abs_ever_received{false};
  bool speed_ever_received{false};
};

class ArmEncoderFeedback
{
public:
  // Construct with a default four-channel table (BASE, SHOULDER, ELBOW, WRIST)
  // taken from questions.md §A.
  explicit ArmEncoderFeedback(std::shared_ptr<can_util::CANController> can);

  // Construct with a caller-supplied channel list (for testing or custom rigs).
  ArmEncoderFeedback(std::shared_ptr<can_util::CANController> can,
                     std::vector<EncoderChannel> channels);

  // Number of channels (4 in the default table).
  size_t numChannels() const { return channels_.size(); }

  // Thread-safe accessors. Index must be < numChannels().
  std::string label(size_t i) const;
  float direction(size_t i) const;

  int16_t rawCounts(size_t i) const;
  double positionRad(size_t i) const;
  double velocityRads(size_t i) const;
  uint16_t statusWord(size_t i) const;
  uint8_t validity(size_t i) const;

  bool absEverReceived(size_t i) const;
  bool speedEverReceived(size_t i) const;

  // Returns true if an abs frame arrived within max_age.
  bool absFresh(size_t i, std::chrono::milliseconds max_age) const;
  // Returns true if a speed frame arrived within max_age.
  bool speedFresh(size_t i, std::chrono::milliseconds max_age) const;

  // Returns the default four-channel list (useful for unknown-frame filtering
  // in the telemetry node).
  static std::vector<EncoderChannel> defaultChannels();

private:
  void onFrame(uint32_t id, const std::vector<uint8_t> & data);
  void decodeAbs(size_t idx, const std::vector<uint8_t> & data);
  void decodeSpeed(size_t idx, const std::vector<uint8_t> & data);

  std::shared_ptr<can_util::CANController> can_;
  std::vector<EncoderChannel> channels_;

  // O(1) lookup from full 29-bit arbitration ID → channel index.
  std::unordered_map<uint32_t, size_t> abs_id_to_idx_;
  std::unordered_map<uint32_t, size_t> speed_id_to_idx_;

  mutable std::mutex mutex_;
  std::shared_ptr<can_util::CANFrameCallback> frame_callback_;
};

}  // namespace encoder_boards
