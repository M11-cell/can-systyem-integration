#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "can-utils/system_controller.hpp"
#include "can-utils/can_interface.hpp"
#include <array>
#include <chrono>
#include <memory>
#include <mutex>

using namespace std::chrono;
using namespace std::literals::chrono_literals; 

class CanControllerNode : public rclcpp::Node{

    public:
        explicit CanControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg); 
        void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);
        void getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg); 

    private: 

        void sendCanFrames();

        std::string can_interface_; 
        
        std::shared_ptr<can_util::CANController> can_controller_; 
        std::unique_ptr<SystemFrameBuilder> frame_builder_; 

        ros2_fmt_logger::Logger logger; 
        int multiplier; 
        int can_send_rate_hz_;

        std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler;
        rclcpp::ParameterCallbackHandle::SharedPtr multiplier_callback_handle;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msgs_; 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_; 

        rclcpp::TimerBase::SharedPtr can_send_timer_; 

        std::mutex cmd_mutex_;
        geometry_msgs::msg::Twist::ConstSharedPtr latest_twist_;
        sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
        bool joint_state_dirty_{false};

        float wheel_rpm_slew_rate_{0.F};
        float wheel_rpm_abs_max_{0.F};
        float wheel_slew_sign_change_boost_{1.F};
        float cmd_vel_deadzone_{0.F};
        float cmd_vel_angular_deadzone_{0.F};
        double wheel_maintain_min_period_s_{0.0};

        std::chrono::steady_clock::time_point last_wheel_maintain_{};
        std::array<float, 6> wheel_rpm_smoothed_{};
}; 

