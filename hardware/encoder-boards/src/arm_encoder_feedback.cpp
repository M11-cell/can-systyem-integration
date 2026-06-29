#include "encoder-boards/arm_encoder_feedback.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace encoder_boards
{

namespace
{
// Scale factor from TLE5012B 15-bit signed counts to radians.
// Full-scale range is ±32768 counts = ±2π rad → 2π/32768 = π/16384.
constexpr double kCountsToRad = M_PI / 16384.0;
}  // namespace

// Default channel table — questions.md §A, matched to rover_arm_can.ros2_control.xacro.
// joint4 / FOREARM_ENCODER (0x0A) omitted: firmware not installed yet.
// Wrist encoder (joint5 / M5) is the fourth channel below.
std::vector<EncoderChannel> ArmEncoderFeedback::defaultChannels()
{
  return {
    {"joint1 / BASE",     0x0108C701u, 0x0108C741u, /* direction */ 1.0f},
    {"joint2 / SHOULDER", 0x0108C801u, 0x0108C841u, /* direction */ 1.0f},
    {"joint3 / ELBOW",    0x0108C901u, 0x0108C941u, /* direction */ -1.0f},
    {"joint5 / WRIST",    0x0108CB01u, 0x0108CB41u, /* direction */ 1.0f},
  };
}

ArmEncoderFeedback::ArmEncoderFeedback(std::shared_ptr<can_util::CANController> can)
: ArmEncoderFeedback(std::move(can), defaultChannels()) {}

ArmEncoderFeedback::ArmEncoderFeedback(std::shared_ptr<can_util::CANController> can,
                                       std::vector<EncoderChannel> channels)
: can_(std::move(can)), channels_(std::move(channels))
{
  if (!can_) {
    throw std::invalid_argument("ArmEncoderFeedback requires a non-null CANController");
  }
  for (size_t i = 0; i < channels_.size(); ++i) {
    abs_id_to_idx_[channels_[i].abs_can_id]     = i;
    speed_id_to_idx_[channels_[i].speed_can_id] = i;
  }
  frame_callback_ = can_->registerFrameCallback(
    [this](uint32_t id, const std::vector<uint8_t> & data) { onFrame(id, data); });
}

// ── private decode ────────────────────────────────────────────────────────────

void ArmEncoderFeedback::onFrame(uint32_t id, const std::vector<uint8_t> & data)
{
  // CANController delivers IDs already stripped of CAN_EFF_FLAG; mask to 29 bits.
  const uint32_t arb_id = id & 0x1FFFFFFFu;

  auto abs_it = abs_id_to_idx_.find(arb_id);
  if (abs_it != abs_id_to_idx_.end()) {
    decodeAbs(abs_it->second, data);
    return;
  }
  auto spd_it = speed_id_to_idx_.find(arb_id);
  if (spd_it != speed_id_to_idx_.end()) {
    decodeSpeed(spd_it->second, data);
  }
}

void ArmEncoderFeedback::decodeAbs(size_t idx, const std::vector<uint8_t> & data)
{
  // DLC must be at least 6.  Shorter frames are malformed; ignore.
  if (data.size() < 6) {
    return;
  }
  // Bytes 0–1: uint16 LE calibrated angle in TLE5012B 15-bit signed counts.
  // Note: firmware call-site bug (questions.md §F) currently sends a truncated
  // float (0–3) here instead of the real counts.  We decode faithfully.
  const uint16_t raw_u16 = static_cast<uint16_t>(data[0]) |
                           (static_cast<uint16_t>(data[1]) << 8);
  const int16_t counts = static_cast<int16_t>(raw_u16);

  // Bytes 2–3: TLE5012B status register (masked 0x1EFE by firmware).
  const uint16_t status = static_cast<uint16_t>(data[2]) |
                          (static_cast<uint16_t>(data[3]) << 8);

  // Byte 5: validity flag.  0x01 = valid; 0x00 = TLE5012B comms error.
  const uint8_t valid = data[5];

  const double pos_rad = static_cast<double>(counts) * kCountsToRad;

  std::lock_guard<std::mutex> lk(mutex_);
  auto & ch = channels_[idx];
  ch.raw_counts          = counts;
  ch.position_rad        = pos_rad;
  ch.status_word         = status;
  ch.validity            = valid;
  ch.abs_stamp           = std::chrono::steady_clock::now();
  ch.abs_ever_received   = true;
}

void ArmEncoderFeedback::decodeSpeed(size_t idx, const std::vector<uint8_t> & data)
{
  // Need at least 4 bytes for the float.
  if (data.size() < 4) {
    return;
  }
  // Firmware applies a within-halfword byte swap when writing the float32 via
  // memcpy: wire order is [b1, b0, b3, b2].  Undo by swapping bytes 0↔1 and
  // 2↔3 before reinterpreting as a LE float32.
  uint8_t buf[4] = {data[1], data[0], data[3], data[2]};
  float vel_rads = 0.0f;
  std::memcpy(&vel_rads, buf, sizeof(float));

  std::lock_guard<std::mutex> lk(mutex_);
  auto & ch = channels_[idx];
  ch.velocity_rads        = static_cast<double>(vel_rads);
  ch.speed_stamp          = std::chrono::steady_clock::now();
  ch.speed_ever_received  = true;
}

// ── thread-safe accessors ─────────────────────────────────────────────────────

std::string ArmEncoderFeedback::label(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).label;
}

float ArmEncoderFeedback::direction(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).direction;
}

int16_t ArmEncoderFeedback::rawCounts(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).raw_counts;
}

double ArmEncoderFeedback::positionRad(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).position_rad;
}

double ArmEncoderFeedback::velocityRads(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).velocity_rads;
}

uint16_t ArmEncoderFeedback::statusWord(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).status_word;
}

uint8_t ArmEncoderFeedback::validity(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).validity;
}

bool ArmEncoderFeedback::absEverReceived(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).abs_ever_received;
}

bool ArmEncoderFeedback::speedEverReceived(size_t i) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return channels_.at(i).speed_ever_received;
}

bool ArmEncoderFeedback::absFresh(size_t i, std::chrono::milliseconds max_age) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  const auto & ch = channels_.at(i);
  if (!ch.abs_ever_received) {
    return false;
  }
  return (std::chrono::steady_clock::now() - ch.abs_stamp) <= max_age;
}

bool ArmEncoderFeedback::speedFresh(size_t i, std::chrono::milliseconds max_age) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  const auto & ch = channels_.at(i);
  if (!ch.speed_ever_received) {
    return false;
  }
  return (std::chrono::steady_clock::now() - ch.speed_stamp) <= max_age;
}

}  // namespace encoder_boards
