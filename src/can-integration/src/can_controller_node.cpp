#include "can_controller_node.hpp"
#include "prefixes.hpp"

static bool inhibit_command_extraction_ = true;

//------------------------------ Obtaining Joystick input from controller -------------------------------
void CanControllerNode::getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
    
    constexpr int FORCE_STOP_COMPAT = 3;
    constexpr int RESTART_ARM_MOTORS = 2;
    constexpr int FORCE_STOP_WHEELS = 6;
    constexpr int RESTART_WHEEL_MOTORS = 7;

    static bool force_stop_active = false;

    bool force_stop_compat = msg->buttons[FORCE_STOP_COMPAT] == 1;
    bool restart_arm_pressed = msg->buttons[RESTART_ARM_MOTORS] == 1;

    bool force_stop_wheels = msg->buttons[FORCE_STOP_WHEELS] == 1;
    bool restart_wheels = msg->buttons[RESTART_WHEEL_MOTORS] == 1;

    if (force_stop_compat && !force_stop_active) {
        force_stop_active = true;
        inhibit_command_extraction_ = false;
        systemframebuilder_.sendForceStop(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID);
        RCLCPP_WARN(this->get_logger(), "Force stop arm motors request sent");
    }

    if (restart_arm_pressed && force_stop_active) {
        force_stop_active = false;
        inhibit_command_extraction_ = true;
        systemframebuilder_.sendResume(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID);
        RCLCPP_INFO(this->get_logger(), "Restart arm motors request sent");
    }

    if (force_stop_wheels && !force_stop_active) {
        force_stop_active = true;
        inhibit_command_extraction_ = false;
        systemframebuilder_.sendForceStop(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
        RCLCPP_WARN(this->get_logger(), "Wheel force stop request sent");
    }

    if (restart_wheels && force_stop_active) {
        force_stop_active = false;
        inhibit_command_extraction_ = true;
        systemframebuilder_.sendResume(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
        RCLCPP_INFO(this->get_logger(), "Wheel restart request sent");
    }
}

// ------------------------------ Obtaining Wheel twist Data from controller --------------------------------
void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg) {

    auto linear_y = twist_msg->linear.x;
    auto angular_z = twist_msg->angular.z;

    linear_y *= linear_y * linear_y;
    angular_z *= angular_z * angular_z;

    float distance_between_wheels = 1.2f;
    uint32_t right_wheel_velocity = -(linear_y - (-angular_z * distance_between_wheels * 0.5f));
    uint32_t left_wheel_velocity  = -(linear_y + (-angular_z * distance_between_wheels * 0.5f));

    uint32_t right_wheel_velocity_rpm = right_wheel_velocity * 2000;
    uint32_t left_wheel_velocity_rpm  = left_wheel_velocity * 2000;

    systemframebuilder_.sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT1, right_wheel_velocity_rpm);
    systemframebuilder_.sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT2, right_wheel_velocity_rpm);
    systemframebuilder_.sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT3, right_wheel_velocity_rpm);

    systemframebuilder_.sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT4, left_wheel_velocity_rpm);
    systemframebuilder_.sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT5, left_wheel_velocity_rpm);
    systemframebuilder_.sendWheelMotorVelocity(DeviceId::ID::WHEEL_MOT6, left_wheel_velocity_rpm);

    RCLCPP_INFO(this->get_logger(), "Wheel Motor Commands Sent: Right RPM = %u, Left RPM = %u",
                right_wheel_velocity_rpm, left_wheel_velocity_rpm);
}

// ---------------------------- Obtaining Joint State Data from controller --------------------------
void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg) {

    if (joint_state_msg->velocity.size() < 5) {
        RCLCPP_ERROR(this->get_logger(),
                     "Received JointState message with insufficient velocity data. Joint size is %zu, expected at least 5.",
                     joint_state_msg->velocity.size());
        return;
    }

    static constexpr Instructions::Inst MOTOR_MAP[5] = {
        Instructions::Inst::ARM_MOTOR_1,
        Instructions::Inst::ARM_MOTOR_2,
        Instructions::Inst::ARM_MOTOR_3,
        Instructions::Inst::ARM_MOTOR_4,
        Instructions::Inst::ARM_MOTOR_5,
    };

    if (inhibit_command_extraction_) {
        RCLCPP_INFO(this->get_logger(), "Extracting velocity data from JointState");
        for (size_t i = 0; i < 5; i++) {
            const float velocities = static_cast<float>(joint_state_msg->velocity[i]);
            systemframebuilder_.sendArmMotorVelocity(
                deviceType::DeviceType::COMPAT,
                MOTOR_MAP[i],
                DeviceId::ID::COMPAT_BOARD_ID,
                velocities
            );

            RCLCPP_DEBUG(this->get_logger(), "Motor %zu -> %.3f rad/s", i + 1, velocities);
        }
    }
}

// ---------------------------- Obtaining Rover State Data for SIL --------------------------
void CanControllerNode::getRoverState(const std_msgs::msg::String::ConstSharedPtr& msg) {
    const std::string state = msg->data;
    bool attempted = false;

    if (state == "teleop") {
        attempted = sil_controller_.setState(LedState::TELEOP);
    }
    else if (state == "autonomy") {
        attempted = sil_controller_.setState(LedState::AUTONOMY);
    }
    else if (state == "goal_reached") {
        attempted = sil_controller_.setState(LedState::GOAL_REACHED);
    }
    else if (state == "emergency_stop") {
        attempted = sil_controller_.setState(LedState::EMERGENCY_STOP);
    }
    else if (state == "off") {
        attempted = sil_controller_.setState(LedState::OFF);
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Unknown rover_state '%s'", state.c_str());
        return;
    }

    if (attempted) {
        RCLCPP_INFO(
            this->get_logger(),
            "SIL frame sent for state '%s' with CAN ID 0x%08X",
            state.c_str(),
            sil_controller_.lastCanId()
        );
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send SIL frame for state '%s'", state.c_str());
    }
}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<CanControllerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
