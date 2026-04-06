#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "system_controller.hpp"
#include "can_interface.hpp"
#include "sil_controller.hpp"

#include <chrono>

using namespace std::chrono;
using namespace std::literals::chrono_literals;

class CanControllerNode : public rclcpp::Node {

public:
    CanControllerNode() : Node("can_controller_node") {

        // check to see if can has been successfully configured
        if (CanManager::configureCan("vcan0") != SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure CAN interface...");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface configured successfully!");

        // initialize subscriptions
        twist_msgs_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
                getTwistMessages(msg);
            });

        joint_state_msgs_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/arm_xyz_cmd",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
                getJointStateMessages(msg);
            });

        joy_msgs_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
                getjoyfeedback(msg);
            });

        rover_state_msgs_ = this->create_subscription<std_msgs::msg::String>(
            "/rover_state",
            rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
                getRoverState(msg);
            });

        RCLCPP_INFO(this->get_logger(), "Subscribed to the required topics");
    }

    void getTwistMessages(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg);
    void getJointStateMessages(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);
    void getjoyfeedback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);
    void getRoverState(const std_msgs::msg::String::ConstSharedPtr& msg);

private:
    SystemFrameBuilder systemframebuilder_;
    SilController sil_controller_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msgs_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msgs_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_msgs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rover_state_msgs_;

    rclcpp::TimerBase::SharedPtr timer_;
};
