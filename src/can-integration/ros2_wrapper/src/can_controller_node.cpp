#include "ros2_wrapper/can_controller_node.hpp"
#include "can-utils/buildAddress.hpp"
#include "can-utils/prefixes.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <thread>
#include <chrono>


CanControllerNode::CanControllerNode(const rclcpp::NodeOptions& options) : 
    Node("can_controller_node", options), logger(this->get_logger().get_child("can_controller_node")){

        this->declare_parameter("can_path", "can0");
        multiplier = this->declare_parameter("multiplier", 2000);
        can_send_rate_hz_ = this->declare_parameter("can_send_rate_hz", 10);
        wheel_rpm_slew_rate_ = static_cast<float>(
            this->declare_parameter("wheel_rpm_slew_rate", 1500.0));
        wheel_rpm_abs_max_ = static_cast<float>(
            this->declare_parameter("wheel_rpm_abs_max", 1500.0));
        wheel_slew_sign_change_boost_ = static_cast<float>(
            this->declare_parameter("wheel_slew_sign_change_boost", 3.0));
        cmd_vel_deadzone_ = static_cast<float>(
            this->declare_parameter("cmd_vel_deadzone", 0.06));
        cmd_vel_angular_deadzone_ = static_cast<float>(
            this->declare_parameter("cmd_vel_angular_deadzone", 0.03));
        wheel_maintain_min_period_s_ = this->declare_parameter(
            "wheel_maintain_min_period_s", 0.0);

        can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");

        can_controller_ = std::make_shared<can_util::CANController>(can_interface_, this->get_logger());
        if (!can_controller_->configureCan()) {
            logger.fatal("Failed to configure CAN on interface {}", can_interface_); 
            throw std::runtime_error("CAN configure failed");
        }
        
        parameter_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto multiplier_callback = [this](const rclcpp::Parameter parameter) {
        if (parameter.get_name() == "multiplier")
        multiplier = parameter.as_int();
        };

        multiplier_callback_handle = parameter_event_handler->add_parameter_callback("multiplier", multiplier_callback);

        frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

        twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), [this]
        (const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {getTwistMessages(msg);});
        joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm_xyz_cmd", rclcpp::SensorDataQoS(), [this]
        (const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {getJointStateMessages(msg);});
        joy_msgs_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(), [this]
        (const sensor_msgs::msg::Joy::ConstSharedPtr& msg){getjoyfeedback(msg);});

        const auto period = std::chrono::milliseconds(1000 / can_send_rate_hz_);
        can_send_timer_ = this->create_wall_timer(period, [this]{ sendCanFrames(); });

        uint64_t mask = 0x7E;
        frame_builder_->startMotors(mask); 
        frame_builder_->requestStatusFrame();

        logger.info("All subscriptions initialized, CAN send rate = {} Hz", can_send_rate_hz_); 
}
        
static bool inhibit_arm_cmds = true; 
static bool inhibit_wheel_cmds = true; 

//------------------------------ Obtaining Joystick input from controller -------------------------------

void CanControllerNode::getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg){
    
    constexpr int FORCE_STOP_COMPAT = 3;
    constexpr int RESTART_ARM_MOTORS = 2; 
    constexpr int FORCE_STOP_WHEELS = 6;
    constexpr int RESTART_WHEEL_MOTORS = 7; 

    static bool arm_force_stop = false;
    static bool wheel_force_stop = false; 

    bool force_stop_compat = msg->buttons[FORCE_STOP_COMPAT] == 1 ;
    bool restart_arm_pressed = msg->buttons[RESTART_ARM_MOTORS] == 1; 

    bool force_stop_wheels = msg->buttons[FORCE_STOP_WHEELS] == 1; 
    bool restart_wheels = msg->buttons[RESTART_WHEEL_MOTORS] == 1;

    // if(force_stop_compat && !arm_force_stop){
    //     arm_force_stop = true; 
    //     inhibit_arm_cmds = false; 
    //     frame_builder_->sendForceStop(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID); 
    //     logger.info("Sending force stop command to arm motors");  
    // }else if(restart_arm_pressed && arm_force_stop){
    //     arm_force_stop = false; 
    //     inhibit_arm_cmds = true; 
    //     frame_builder_->sendResume(deviceType::DeviceType::COMPAT, DeviceId::ID::COMPAT_BOARD_ID); 
    //     logger.info("Sending restart request to arm motors"); 
    // }

    if(force_stop_wheels && !wheel_force_stop){
        wheel_force_stop = true; 
        inhibit_wheel_cmds = false;
        frame_builder_->sendForceStop(deviceType::DeviceType::RELAY, DeviceId::ID::HUB);
        logger.info("Sending force stop command to wheel motors");
    }else if(restart_wheels && wheel_force_stop){
        wheel_force_stop = false; 
        inhibit_wheel_cmds = true;
        frame_builder_->sendResume(deviceType::DeviceType::RELAY, DeviceId::ID::HUB); 
        logger.info("Sending restart command to wheel motors");
    }
}

//------------------------------ Cache incoming topic data (no CAN I/O here) -------------------------------- 

void CanControllerNode::getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg){
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_twist_ = twist_msg;
}

void CanControllerNode::getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg){
    if(joint_state_msg->velocity.size() < 5){
        logger.error("Received joint state messages with insufficient velocity data. joint size is {}, expected at least 5", joint_state_msg->velocity.size()); 
        return; 
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_joint_state_ = joint_state_msg;
    joint_state_dirty_ = true;
}

//------------------------------ Timer: send cached commands at fixed rate -------------------------------- 

void CanControllerNode::sendCanFrames(){

    geometry_msgs::msg::Twist::ConstSharedPtr twist;
    sensor_msgs::msg::JointState::ConstSharedPtr joint_state;
    bool send_arm = false;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        twist = latest_twist_;
        if(joint_state_dirty_){
            joint_state = latest_joint_state_;
            joint_state_dirty_ = false;
            send_arm = true;
        }
    }

    if(twist && inhibit_wheel_cmds){
        const float lin_dz = cmd_vel_deadzone_;
        const float ang_dz = cmd_vel_angular_deadzone_;

        auto linear_x = static_cast<float>(twist->linear.x);
        auto angular_z = static_cast<float>(twist->angular.z);

        if(std::abs(linear_x) < lin_dz) linear_x = 0.0f;
        if(std::abs(angular_z) < ang_dz) angular_z = 0.0f;

        const float half_track = 1.2f * 0.5f;
        float right_cmd = -(linear_x - (-angular_z * half_track));
        float left_cmd = -(linear_x + (-angular_z * half_track));

        const float mult = static_cast<float>(multiplier);
        std::array<float, 6> target_rpm = {
            right_cmd * mult,
            right_cmd * mult,
            right_cmd * mult,
            left_cmd * mult,
            left_cmd * mult,
            left_cmd * mult,
        };

        if(wheel_rpm_abs_max_ > 0.F){
            for(float& t : target_rpm){
                t = std::clamp(t, -wheel_rpm_abs_max_, wheel_rpm_abs_max_);
            }
        }

        const float dt = 1.F / static_cast<float>(std::max(1, can_send_rate_hz_));
        const float base_max_step = (wheel_rpm_slew_rate_ > 0.F)
            ? wheel_rpm_slew_rate_ * dt
            : std::numeric_limits<float>::max();

        static constexpr std::array<DeviceId::ID, 6> kWheelIds = {
            DeviceId::ID::WHEEL_MOT1,
            DeviceId::ID::WHEEL_MOT2,
            DeviceId::ID::WHEEL_MOT3,
            DeviceId::ID::WHEEL_MOT4,
            DeviceId::ID::WHEEL_MOT5,
            DeviceId::ID::WHEEL_MOT6,
        };

        for(size_t i = 0; i < kWheelIds.size(); ++i){
            const float target = target_rpm[i];
            float current = wheel_rpm_smoothed_[i];
            float max_step = base_max_step;
            if(wheel_slew_sign_change_boost_ > 1.F
                && target * current < 0.F
                && std::abs(current) > 1e-3F){
                max_step *= wheel_slew_sign_change_boost_;
            }
            float delta = target - current;
            if(delta > max_step) delta = max_step;
            if(delta < -max_step) delta = -max_step;
            wheel_rpm_smoothed_[i] = current + delta;
            frame_builder_->sendWheelMotorVelocity(kWheelIds[i], wheel_rpm_smoothed_[i]);
        }

        const auto steady_now = std::chrono::steady_clock::now();
        const bool time_for_maintain =
            (wheel_maintain_min_period_s_ <= 0.0)
            || (std::chrono::duration<double>(steady_now - last_wheel_maintain_).count()
                >= wheel_maintain_min_period_s_);
        if(time_for_maintain){
            frame_builder_->startMotors(0x7E);
            last_wheel_maintain_ = steady_now;
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "Wheel RPM smoothed R/L targets: %.1f/%.1f cmd %.3f/%.3f",
            wheel_rpm_smoothed_[0], wheel_rpm_smoothed_[3], right_cmd, left_cmd);
        RCLCPP_DEBUG(
            this->get_logger(),
            "candump wheel velocity frame IDs (29-bit hex): "
            "W1=%08X W2=%08X W3=%08X W4=%08X W5=%08X W6=%08X",
            static_cast<unsigned>(
                (COMMAND_PREFIX_VELOCITY_CONTROL << 8) |
                (static_cast<uint32_t>(DeviceId::ID::WHEEL_MOT1) + 0x80u)),
            static_cast<unsigned>(
                (COMMAND_PREFIX_VELOCITY_CONTROL << 8) |
                (static_cast<uint32_t>(DeviceId::ID::WHEEL_MOT2) + 0x80u)),
            static_cast<unsigned>(
                (COMMAND_PREFIX_VELOCITY_CONTROL << 8) |
                (static_cast<uint32_t>(DeviceId::ID::WHEEL_MOT3) + 0x80u)),
            static_cast<unsigned>(
                (COMMAND_PREFIX_VELOCITY_CONTROL << 8) |
                (static_cast<uint32_t>(DeviceId::ID::WHEEL_MOT4) + 0x80u)),
            static_cast<unsigned>(
                (COMMAND_PREFIX_VELOCITY_CONTROL << 8) |
                (static_cast<uint32_t>(DeviceId::ID::WHEEL_MOT5) + 0x80u)),
            static_cast<unsigned>(
                (COMMAND_PREFIX_VELOCITY_CONTROL << 8) |
                (static_cast<uint32_t>(DeviceId::ID::WHEEL_MOT6) + 0x80u)));
    }

    if(send_arm && inhibit_arm_cmds){
        static constexpr Instructions::Inst MOTOR_MAP[5] = {
            Instructions::Inst::ARM_MOTOR_1,
            Instructions::Inst::ARM_MOTOR_2,
            Instructions::Inst::ARM_MOTOR_3,
            Instructions::Inst::ARM_MOTOR_4,
            Instructions::Inst::ARM_MOTOR_5,
        };
        static constexpr size_t ARM_MOTOR_COUNT = sizeof(MOTOR_MAP) / sizeof(MOTOR_MAP[0]);

        std::array<float, ARM_MOTOR_COUNT> cmd_sent{};
        for(size_t i = 0; i < ARM_MOTOR_COUNT; i++){
            const double raw = joint_state->velocity[i];
            constexpr double DEADZONE = 0.05;
            float velocities = (std::abs(raw) < DEADZONE)
                ? 0.0f
                : static_cast<float>(raw * buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MAX);
                // motor 4 is a special case, it needs to be scaled down to 50% of the other motors because it is faster
            if (i == 3) {
                velocities *= 0.5f;
            }
            const float payload = std::clamp(
                velocities,
                buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MIN,
                buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MAX);
            cmd_sent[i] = payload;
            frame_builder_->sendArmMotorVelocity(
                deviceType::DeviceType::ARM_MOTOR_CONTROLLER, MOTOR_MAP[i], DeviceId::ID::ARM_MOTOR_CONTROLLER, payload);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            RCLCPP_DEBUG(this->get_logger(), "Motor %zu → %.3f (payload)", i + 1, payload);
        }
        RCLCPP_INFO(this->get_logger(), "Arm motor commands sent: Motor 1 = %.2f, Motor 2 = %.2f, Motor 3 = %.2f, Motor 4 = %.2f, Motor 5 = %.2f",
            cmd_sent[0], cmd_sent[1], cmd_sent[2], cmd_sent[3], cmd_sent[4]);
    }
}


int main(int argc, char *argv[]){

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<CanControllerNode>(); 
    rclcpp::spin(node); 

    rclcpp::shutdown(); 
    return 0; 

}
