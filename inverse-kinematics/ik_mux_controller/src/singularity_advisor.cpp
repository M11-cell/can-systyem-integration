// singularity_advisor — turn MoveIt Servo's generic singularity halt into an
// actionable, per-joint escape hint for the VKB operator.
//
// MoveIt Servo only publishes a status code + message on /servo_node/status
// (e.g. "Very close to a singularity, emergency stop"). It never tells you
// which joints to move. This node reproduces the escape-direction math Servo
// uses internally: it SVD-decomposes the arm Jacobian at the current pose,
// finds the Cartesian direction away from the nearest singularity, projects it
// into joint space with a damped pseudo-inverse, and reports the dominant
// joints as F1 (FK) stick/button actions.
//
// Output: a human-readable std_msgs/String on /arm/singularity_hint plus a
// throttled log. It is advice only — the operator still drives in F1 with the
// deadman held; nothing here commands the arm.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "rclcpp/rclcpp.hpp"
#include <moveit_msgs/msg/servo_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace ik_mux
{

// Maps a joint to the F1 (FK) VKB control that produces a positive joint
// velocity, and the opposite control for a negative one. Signs mirror the FK
// jog scales in ik_mux_vkb.cpp (note joint4 uses a negative scale, so a
// positive velocity comes from Stick X LEFT).
struct JointControlHint
{
  std::string nickname;      // e.g. "shoulder"
  std::string positive_cmd;  // control for velocity > 0
  std::string negative_cmd;  // control for velocity < 0
};

class SingularityAdvisor
{
public:
  explicit SingularityAdvisor(const rclcpp::Node::SharedPtr & node)
  : node_(node), logger_(node->get_logger())
  {
    group_name_ = node_->declare_parameter<std::string>("planning_group", "arm");
    hint_period_s_ = node_->declare_parameter<double>("hint_period_s", 1.0);
    velocity_threshold_ = node_->declare_parameter<double>("velocity_threshold", 0.1);
    max_hint_joints_ = node_->declare_parameter<int>("max_hint_joints", 2);
    damping_ = node_->declare_parameter<double>("pseudo_inverse_damping", 0.01);

    // Hints only need the kinematic model (Jacobian); skip IK plugin load so
    // RobotModelLoader does not spawn a second same-named node (duplicate
    // /singularity_advisor in ros2 node list + rosout WARN).
    robot_model_loader::RobotModelLoader loader(node_, "robot_description", false);
    robot_model_ = loader.getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(logger_, "Failed to load robot model from 'robot_description'; advisor disabled.");
      return;
    }
    jmg_ = robot_model_->getJointModelGroup(group_name_);
    if (!jmg_) {
      RCLCPP_ERROR(logger_, "Planning group '%s' not found; advisor disabled.", group_name_.c_str());
      return;
    }
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    hint_pub_ = node_->create_publisher<std_msgs::msg::String>("/arm/singularity_hint", 10);
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr & msg) { onJointState(msg); });
    status_sub_ = node_->create_subscription<moveit_msgs::msg::ServoStatus>(
      "/servo_node/status", rclcpp::SystemDefaultsQoS(),
      [this](const moveit_msgs::msg::ServoStatus::ConstSharedPtr & msg) { onStatus(msg); });

    RCLCPP_INFO(
      logger_,
      "singularity_advisor ready — group '%s'. Publishes escape hints on /arm/singularity_hint.",
      group_name_.c_str());
  }

private:
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
  {
    if (!robot_state_) {
      return;
    }
    robot_state_->setVariableValues(*msg);
    have_state_ = true;
  }

  void onStatus(const moveit_msgs::msg::ServoStatus::ConstSharedPtr & msg)
  {
    const bool singular =
      msg->code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY ||
      msg->code == moveit_msgs::msg::ServoStatus::DECELERATE_FOR_APPROACHING_SINGULARITY;
    if (!singular || !robot_state_ || !have_state_) {
      return;
    }
    const double now_s = node_->now().seconds();
    if (now_s - last_hint_s_ < hint_period_s_) {
      return;
    }
    last_hint_s_ = now_s;

    const std::string hint = computeHint();
    if (hint.empty()) {
      return;
    }
    std_msgs::msg::String out;
    out.data = hint;
    hint_pub_->publish(out);
    RCLCPP_WARN(logger_, "%s", hint.c_str());
  }

  // Reproduces Servo's escape-direction logic, then projects it into joint
  // space to name the dominant joints and directions.
  std::string computeHint()
  {
    const Eigen::MatrixXd jacobian = robot_state_->getJacobian(jmg_);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd & singular_values = svd.singularValues();
    if (singular_values.size() == 0 || singular_values(singular_values.size() - 1) <= 0.0) {
      return {};
    }

    const double ini_condition =
      singular_values(0) / singular_values(singular_values.size() - 1);

    // Damped (Levenberg-Marquardt) pseudo-inverse, stable near singularities.
    Eigen::VectorXd inv_values(singular_values.size());
    for (int i = 0; i < singular_values.size(); ++i) {
      const double s = singular_values(i);
      inv_values(i) = s / (s * s + damping_ * damping_);
    }
    const Eigen::MatrixXd pseudo_inverse =
      svd.matrixV() * inv_values.asDiagonal() * svd.matrixU().transpose();

    // The last column of U points toward or away from the singularity; its sign
    // is ambiguous, so look ahead one small step and keep the direction that
    // increases the condition number (i.e. that truly points at the singularity).
    Eigen::VectorXd toward_singularity = svd.matrixU().col(svd.matrixU().cols() - 1);
    Eigen::VectorXd delta_x = toward_singularity / 100.0;

    Eigen::VectorXd q_current;
    robot_state_->copyJointGroupPositions(jmg_, q_current);
    moveit::core::RobotState lookahead(*robot_state_);
    lookahead.setJointGroupPositions(jmg_, Eigen::VectorXd(q_current + pseudo_inverse * delta_x));
    const Eigen::MatrixXd new_jacobian = lookahead.getJacobian(jmg_);
    Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(new_jacobian);
    const Eigen::VectorXd & new_singular_values = new_svd.singularValues();
    const double new_condition =
      new_singular_values(0) / new_singular_values(new_singular_values.size() - 1);
    if (ini_condition >= new_condition) {
      toward_singularity *= -1.0;
    }

    // Joint velocities that carry the end effector away from the singularity.
    const Eigen::VectorXd q_dot = pseudo_inverse * (-toward_singularity);
    const std::vector<std::string> & names = jmg_->getVariableNames();

    // Rank joints by |velocity| and keep the strongest few above threshold.
    std::vector<int> order(q_dot.size());
    for (int i = 0; i < q_dot.size(); ++i) {
      order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&q_dot](int a, int b) {
      return std::abs(q_dot(a)) > std::abs(q_dot(b));
    });

    std::string body;
    int shown = 0;
    for (const int idx : order) {
      if (shown >= max_hint_joints_ || std::abs(q_dot(idx)) < velocity_threshold_) {
        break;
      }
      const std::string & joint = names[static_cast<size_t>(idx)];
      const JointControlHint * hint = controlFor(joint);
      const std::string action =
        hint ? (q_dot(idx) > 0.0 ? hint->positive_cmd : hint->negative_cmd) : "(no mapped control)";
      const std::string nick = hint ? (" (" + hint->nickname + ")") : "";
      body += "  " + joint + nick + ": " + action + "\n";
      ++shown;
    }
    if (shown == 0) {
      return {};
    }
    return "SINGULARITY — switch to F1 (FK), hold D1, and:\n" + body;
  }

  static const JointControlHint * controlFor(const std::string & joint)
  {
    // Directions match the FK jog scales in ik_mux_vkb.cpp. joint4's scale is
    // negative, so a positive velocity comes from Stick X LEFT.
    static const std::vector<std::pair<std::string, JointControlHint>> table = {
      {"joint1", {"base yaw", "Stick Z UP", "Stick Z DOWN"}},
      {"joint2", {"shoulder", "A3 UP", "A3 DOWN"}},
      {"joint3", {"elbow", "Stick Y FORWARD", "Stick Y BACK"}},
      {"joint4", {"forearm roll", "Stick X LEFT", "Stick X RIGHT"}},
      {"joint5", {"wrist pitch", "A3 LEFT", "A3 RIGHT"}},
      {"joint6", {"wrist roll", "C1 RIGHT", "C1 LEFT"}},
    };
    for (const auto & entry : table) {
      if (entry.first == joint) {
        return &entry.second;
      }
    }
    return nullptr;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  std::string group_name_;
  double hint_period_s_ = 1.0;
  double velocity_threshold_ = 0.1;
  int max_hint_joints_ = 2;
  double damping_ = 0.01;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * jmg_ = nullptr;
  moveit::core::RobotStatePtr robot_state_;
  bool have_state_ = false;
  double last_hint_s_ = -1e9;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub_;
};

}  // namespace ik_mux

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("singularity_advisor");
  auto advisor = std::make_shared<ik_mux::SingularityAdvisor>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
