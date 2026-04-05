#pragma once

#include "buildAddress.hpp"
#include "prefixes.hpp"

#include <cstdint>

enum class LedState : uint8_t {
    OFF = 0,
    TELEOP,
    AUTONOMY,
    GOAL_REACHED,
    EMERGENCY_STOP
};

struct SilPayload {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
    uint8_t blink_enable;
    uint8_t blink_period;
    uint8_t reserved0;
    uint8_t reserved1;
};

static_assert(sizeof(SilPayload) == 8, "SilPayload must be exactly 8 bytes");

class SilController {
public:
    SilController() = default;

    bool setState(LedState state) {
        switch (state) {
            case LedState::OFF:
                return sendPayload(makePayload(0, 0, 0, 0, 0, 0), severity::SEV_STATUS);

            case LedState::TELEOP:
                // Blue solid
                return sendPayload(makePayload(0, 0, 255, 255, 0, 0), severity::SEV_STATUS);

            case LedState::AUTONOMY:
                // Green solid
                return sendPayload(makePayload(0, 255, 0, 255, 0, 0), severity::SEV_STATUS);

            case LedState::GOAL_REACHED:
                // Green blinking
                return sendPayload(makePayload(0, 255, 0, 255, 1, 50), severity::SEV_STATUS);

            case LedState::EMERGENCY_STOP:
                return emergencyOff();

            default:
                return false;
        }
    }

    bool setRGB(uint8_t red,
                uint8_t green,
                uint8_t blue,
                uint8_t brightness = 255,
                uint8_t blink_enable = 0,
                uint8_t blink_period = 0,
                uint8_t sev = severity::SEV_STATUS) {
        return sendPayload(
            makePayload(red, green, blue, brightness, blink_enable, blink_period),
            sev
        );
    }

    bool emergencyOff() {
        return sendPayload(makePayload(0, 0, 0, 0, 0, 0), severity::SEV_MAN_INTERVENTION);
    }

    uint32_t lastCanId() const {
        return last_can_id_;
    }

    SilPayload lastPayload() const {
        return last_payload_;
    }

private:
    static SilPayload makePayload(uint8_t red,
                                  uint8_t green,
                                  uint8_t blue,
                                  uint8_t brightness,
                                  uint8_t blink_enable,
                                  uint8_t blink_period) {
        return SilPayload{
            red,
            green,
            blue,
            brightness,
            blink_enable,
            blink_period,
            0,
            0
        };
    }

    bool sendPayload(const SilPayload& payload, uint8_t sev) {
        last_payload_ = payload;

        const uint8_t device_type = static_cast<uint8_t>(deviceType::DeviceType::SIL);
        const uint8_t manufacturer = Manufacturer::TEAM_USE;
        const uint8_t instruction = static_cast<uint8_t>(Instructions::Inst::SET_LED);
        const uint8_t device_id = static_cast<uint8_t>(DeviceId::ID::SIL);

        last_can_id_ = BuildAddress::buildCANID(
            device_type,
            manufacturer,
            sev,
            instruction,
            device_id
        );

        builder_.buildAddress(
            device_type,
            manufacturer,
            sev,
            instruction,
            device_id,
            payload
        );

        return true;
    }

private:
    BuildAddress builder_;
    uint32_t last_can_id_{0};
    SilPayload last_payload_{0, 0, 0, 0, 0, 0, 0, 0};
};
