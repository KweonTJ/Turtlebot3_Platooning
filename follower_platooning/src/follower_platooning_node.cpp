// Copyright 2026 ktj
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
double clamp(double value, double min_value, double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

geometry_msgs::msg::Twist zero_twist()
{
  return geometry_msgs::msg::Twist();
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const auto siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const auto cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double angle)
{
  constexpr auto pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}
}  // namespace

class FollowerPlatooningNode : public rclcpp::Node
{
public:
  FollowerPlatooningNode()
  : Node("follower_platooning")
  {
    platooning_mode_ = declare_parameter<std::string>("platooning_mode", "vision");

    target_distance_ = declare_parameter<double>("target_distance", 0.45);
    min_distance_ = declare_parameter<double>("min_distance", 0.15);
    max_distance_ = declare_parameter<double>("max_distance", 0.30);
    emergency_stop_distance_ = declare_parameter<double>("emergency_stop_distance", 0.12);

    kp_distance_ = declare_parameter<double>("kp_distance", 0.35);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 0.80);

    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.05);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.25);

    marker_lost_timeout_ = declare_parameter<double>("marker_lost_timeout", 0.5);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    leader_search_timeout_ = declare_parameter<double>("leader_search_timeout", 5.0);
    leader_search_stop_tolerance_ =
      declare_parameter<double>("leader_search_stop_tolerance", 0.03);
    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout", 1.0);
    leader_cmd_timeout_ = declare_parameter<double>("leader_cmd_timeout", 0.5);

    enable_reverse_ = declare_parameter<bool>("enable_reverse", false);
    allow_distance_reverse_ = declare_parameter<bool>("allow_distance_reverse", false);
    search_last_leader_pose_ = declare_parameter<bool>("search_last_leader_pose", true);
    allow_odom_without_heartbeat_ = declare_parameter<bool>("allow_odom_without_heartbeat", true);
    mirror_leader_reverse_turn_ = declare_parameter<bool>("mirror_leader_reverse_turn", true);
    leader_cmd_linear_gain_ = declare_parameter<double>("leader_cmd_linear_gain", 1.0);
    leader_cmd_angular_gain_ = declare_parameter<double>("leader_cmd_angular_gain", 1.0);
    leader_cmd_reverse_threshold_ = declare_parameter<double>("leader_cmd_reverse_threshold", 0.01);
    leader_cmd_turn_threshold_ = declare_parameter<double>("leader_cmd_turn_threshold", 0.05);
    leader_cmd_turn_linear_deadband_ = declare_parameter<double>("leader_cmd_turn_linear_deadband", 0.02);
    use_initial_odom_offset_ = declare_parameter<bool>("use_initial_odom_offset", true);
    initial_leader_offset_x_ = declare_parameter<double>("initial_leader_offset_x", 0.45);
    initial_leader_offset_y_ = declare_parameter<double>("initial_leader_offset_y", 0.0);

    const auto target_visible_topic =
      declare_parameter<std::string>("target_visible_topic", "/follower/target_visible");
    const auto target_offset_x_topic =
      declare_parameter<std::string>("target_offset_x_topic", "/follower/target_offset_x");
    const auto target_distance_topic =
      declare_parameter<std::string>("target_distance_topic", "/follower/target_distance");

    const auto follower_enable_topic =
      declare_parameter<std::string>("follower_enable_topic", "/leader/follower_enable");
    const auto platoon_mode_topic =
      declare_parameter<std::string>("platoon_mode_topic", "/leader/platoon_mode");
    const auto heartbeat_topic =
      declare_parameter<std::string>("heartbeat_topic", "/leader/heartbeat");
    const auto leader_cmd_vel_topic =
      declare_parameter<std::string>("leader_cmd_vel_topic", "/leader/cmd_vel");
    const auto leader_odom_topic =
      declare_parameter<std::string>("leader_odom_topic", "/leader/odom");
    const auto follower_odom_topic =
      declare_parameter<std::string>("follower_odom_topic", "/odom");

    const auto cmd_vel_raw_topic =
      declare_parameter<std::string>("cmd_vel_raw_topic", "/follower/cmd_vel_raw");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/follower/status");
    const auto distance_error_topic =
      declare_parameter<std::string>("distance_error_topic", "/follower/distance_error");

    cmd_vel_raw_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(cmd_vel_raw_topic, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(10));
    distance_error_pub_ =
      create_publisher<std_msgs::msg::Float32>(distance_error_topic, rclcpp::QoS(10));
    target_distance_pub_ =
      create_publisher<std_msgs::msg::Float32>(target_distance_topic, rclcpp::QoS(10));

    target_visible_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_visible_topic, rclcpp::QoS(10),
      std::bind(&FollowerPlatooningNode::target_visible_callback, this, std::placeholders::_1));
    target_offset_x_sub_ = create_subscription<std_msgs::msg::Float32>(
      target_offset_x_topic, rclcpp::QoS(10),
      std::bind(&FollowerPlatooningNode::target_offset_x_callback, this, std::placeholders::_1));
    target_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      target_distance_topic, rclcpp::QoS(10),
      std::bind(&FollowerPlatooningNode::target_distance_callback, this, std::placeholders::_1));
    const auto leader_state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto heartbeat_qos =
      rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
    const auto motion_qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    follower_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      follower_enable_topic, leader_state_qos,
      std::bind(&FollowerPlatooningNode::follower_enable_callback, this, std::placeholders::_1));
    platoon_mode_sub_ = create_subscription<std_msgs::msg::String>(
      platoon_mode_topic, leader_state_qos,
      std::bind(&FollowerPlatooningNode::platoon_mode_callback, this, std::placeholders::_1));
    heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>(
      heartbeat_topic, heartbeat_qos,
      std::bind(&FollowerPlatooningNode::heartbeat_callback, this, std::placeholders::_1));
    leader_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      leader_cmd_vel_topic, motion_qos,
      std::bind(&FollowerPlatooningNode::leader_cmd_vel_callback, this, std::placeholders::_1));
    leader_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_odom_topic, motion_qos,
      std::bind(&FollowerPlatooningNode::leader_odom_callback, this, std::placeholders::_1));
    follower_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_odom_topic, motion_qos,
      std::bind(&FollowerPlatooningNode::follower_odom_callback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      50ms, std::bind(&FollowerPlatooningNode::control_timer_callback, this));
  }

private:
  void target_visible_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    target_visible_ = msg->data;
    have_target_visible_ = true;
    last_target_visible_time_ = now();
  }

  void target_offset_x_callback(const std_msgs::msg::Float32::ConstSharedPtr msg)
  {
    target_offset_x_ = msg->data;
  }

  void target_distance_callback(const std_msgs::msg::Float32::ConstSharedPtr msg)
  {
    measured_distance_ = msg->data;
    have_target_distance_ = true;
    last_target_distance_time_ = now();
  }

  void follower_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    follower_enable_ = msg->data;
    have_follower_enable_ = true;
  }

  void platoon_mode_callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    platoon_mode_state_ = msg->data;
    have_platoon_mode_ = true;
  }

  void heartbeat_callback(const std_msgs::msg::Bool::ConstSharedPtr)
  {
    have_heartbeat_ = true;
    last_heartbeat_time_ = now();
  }

  void leader_cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    latest_leader_cmd_ = *msg;
    have_leader_cmd_ = true;
    last_leader_cmd_time_ = now();
  }

  void leader_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    leader_x_ = msg->pose.pose.position.x;
    leader_y_ = msg->pose.pose.position.y;
    have_leader_odom_ = true;
    last_leader_odom_time_ = now();
  }

  void follower_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    follower_x_ = msg->pose.pose.position.x;
    follower_y_ = msg->pose.pose.position.y;
    follower_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    have_follower_odom_ = true;
    last_follower_odom_time_ = now();
  }

  void control_timer_callback()
  {
    const auto current_time = now();
    std::string status;
    auto cmd = compute_command(current_time, status);

    cmd_vel_raw_pub_->publish(cmd);

    std_msgs::msg::String status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);

    std_msgs::msg::Float32 error_msg;
    error_msg.data = static_cast<float>(last_distance_error_);
    distance_error_pub_->publish(error_msg);

    if (std::isfinite(last_measured_distance_) && last_measured_distance_ > 0.0) {
      std_msgs::msg::Float32 distance_msg;
      distance_msg.data = static_cast<float>(last_measured_distance_);
      target_distance_pub_->publish(distance_msg);
    }
  }

  geometry_msgs::msg::Twist compute_command(const rclcpp::Time & current_time, std::string & status)
  {
    last_distance_error_ = 0.0;
    last_measured_distance_ = -1.0;

    if (!have_follower_enable_) {
      status = "WAITING_ENABLE";
      return zero_twist();
    }
    if (!follower_enable_) {
      status = "DISABLED";
      return zero_twist();
    }
    if (!have_platoon_mode_ || !platoon_mode_allows_distance_control()) {
      status = "WAITING_ENABLE";
      return zero_twist();
    }
    if (
      heartbeat_timed_out(current_time) &&
      !(allow_odom_without_heartbeat_ && platooning_mode_ == "odom" &&
      odom_reference_available(current_time)))
    {
      status = "HEARTBEAT_TIMEOUT";
      return zero_twist();
    }

    if (platooning_mode_ == "odom") {
      return compute_odom_command(current_time, status);
    }
    if (platooning_mode_ != "vision") {
      status = "UNSUPPORTED_MODE";
      return zero_twist();
    }

    geometry_msgs::msg::Twist leader_motion_cmd;
    if (leader_reverse_turn_command(current_time, leader_motion_cmd, status)) {
      return leader_motion_cmd;
    }

    if (target_timed_out(current_time) || !target_visible_) {
      status = "TARGET_LOST";
      return zero_twist();
    }
    if (!valid_distance()) {
      status = "INVALID_DISTANCE";
      return zero_twist();
    }

    last_distance_error_ = measured_distance_ - target_distance_;
    last_measured_distance_ = measured_distance_;

    if (measured_distance_ <= emergency_stop_distance_) {
      status = "TOO_CLOSE_STOP";
      return zero_twist();
    }
    if (!enable_reverse_ && measured_distance_ <= min_distance_) {
      status = "TOO_CLOSE_STOP";
      return zero_twist();
    }

    geometry_msgs::msg::Twist cmd;
    auto linear_x = kp_distance_ * last_distance_error_;
    if (!allow_distance_reverse_) {
      linear_x = std::max(0.0, linear_x);
    }
    const auto min_linear_speed =
      (enable_reverse_ && allow_distance_reverse_) ? -max_linear_speed_ : 0.0;
    cmd.linear.x = clamp(linear_x, min_linear_speed, max_linear_speed_);
    cmd.angular.z = clamp(-kp_yaw_ * target_offset_x_, -max_angular_speed_, max_angular_speed_);

    status = "FOLLOWING";
    return cmd;
  }

  geometry_msgs::msg::Twist compute_odom_command(
    const rclcpp::Time & current_time,
    std::string & status)
  {
    if (!have_leader_odom_ || !have_follower_odom_) {
      status = "ODOM_TIMEOUT";
      return zero_twist();
    }

    const auto leader_age = (current_time - last_leader_odom_time_).seconds();
    const auto follower_age = (current_time - last_follower_odom_time_).seconds();
    if (follower_age > odom_timeout_) {
      status = "ODOM_TIMEOUT";
      return zero_twist();
    }
    const auto using_last_leader_pose = leader_age > odom_timeout_;
    if (
      using_last_leader_pose &&
      (!search_last_leader_pose_ || leader_age > leader_search_timeout_))
    {
      status = "ODOM_TIMEOUT";
      return zero_twist();
    }

    if (use_initial_odom_offset_ && !odom_offset_initialized_) {
      initialize_odom_offset();
    }

    double dx = leader_x_ - follower_x_;
    double dy = leader_y_ - follower_y_;
    if (use_initial_odom_offset_) {
      dx = initial_leader_offset_world_x_ +
        (leader_x_ - leader_origin_x_) - (follower_x_ - follower_origin_x_);
      dy = initial_leader_offset_world_y_ +
        (leader_y_ - leader_origin_y_) - (follower_y_ - follower_origin_y_);
    }
    const auto measured_distance = std::hypot(dx, dy);
    if (!std::isfinite(measured_distance) || measured_distance <= 0.0) {
      status = "INVALID_ODOM_DISTANCE";
      return zero_twist();
    }

    last_distance_error_ = measured_distance - target_distance_;
    last_measured_distance_ = measured_distance;
    if (measured_distance <= emergency_stop_distance_) {
      status = "TOO_CLOSE_STOP";
      return zero_twist();
    }
    if (!enable_reverse_ && measured_distance <= min_distance_) {
      status = "TOO_CLOSE_STOP";
      return zero_twist();
    }
    if (
      using_last_leader_pose &&
      std::abs(last_distance_error_) <= leader_search_stop_tolerance_)
    {
      status = "ODOM_SEARCH_HOLD";
      return zero_twist();
    }

    geometry_msgs::msg::Twist cmd;
    auto linear_x = kp_distance_ * last_distance_error_;
    if (!allow_distance_reverse_) {
      linear_x = std::max(0.0, linear_x);
    }
    bool allow_reverse_command = false;
    if (
      platoon_mode_state_ == "FOLLOW" &&
      have_leader_cmd_ &&
      (current_time - last_leader_cmd_time_).seconds() <= leader_cmd_timeout_)
    {
      linear_x += latest_leader_cmd_.linear.x * leader_cmd_linear_gain_;
      cmd.angular.z += latest_leader_cmd_.angular.z * leader_cmd_angular_gain_;
      allow_reverse_command =
        mirror_leader_reverse_turn_ &&
        latest_leader_cmd_.linear.x < -std::abs(leader_cmd_reverse_threshold_);
    }
    if (!allow_distance_reverse_) {
      linear_x = std::max(0.0, linear_x);
      allow_reverse_command = false;
    }

    const auto min_linear_speed =
      (enable_reverse_ && (allow_distance_reverse_ || allow_reverse_command)) ?
      -max_linear_speed_ : 0.0;
    cmd.linear.x = clamp(linear_x, min_linear_speed, max_linear_speed_);

    const auto bearing_to_leader = std::atan2(dy, dx);
    const auto heading_error = normalize_angle(bearing_to_leader - follower_yaw_);
    cmd.angular.z += kp_yaw_ * heading_error;
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);

    const auto status_prefix = using_last_leader_pose ?
      "ODOM_SEARCH_LAST_LEADER" :
      (measured_distance > max_distance_ ? "ODOM_REACQUIRE" : "ODOM_FOLLOWING");
    std::ostringstream status_stream;
    status_stream << status_prefix << " mode=" << platoon_mode_state_
                  << " distance=" << measured_distance
                  << " error=" << last_distance_error_
                  << " leader_age=" << leader_age;
    status = status_stream.str();
    return cmd;
  }

  bool leader_reverse_turn_command(
    const rclcpp::Time & current_time,
    geometry_msgs::msg::Twist & cmd,
    std::string & status) const
  {
    if (!mirror_leader_reverse_turn_ || !have_leader_cmd_) {
      return false;
    }
    if ((current_time - last_leader_cmd_time_).seconds() > leader_cmd_timeout_) {
      return false;
    }

    const auto leader_linear = latest_leader_cmd_.linear.x;
    const auto leader_angular = latest_leader_cmd_.angular.z;
    const auto reversing = leader_linear < -std::abs(leader_cmd_reverse_threshold_);
    const auto turning =
      std::abs(leader_angular) > std::abs(leader_cmd_turn_threshold_) &&
      std::abs(leader_linear) <= std::abs(leader_cmd_turn_linear_deadband_);

    if (!reversing && !turning) {
      return false;
    }

    const auto min_linear_speed =
      (enable_reverse_ && allow_distance_reverse_) ? -max_linear_speed_ : 0.0;

    if (reversing) {
      cmd.linear.x = clamp(
        leader_linear * leader_cmd_linear_gain_,
        min_linear_speed,
        max_linear_speed_);
      cmd.angular.z = clamp(
        leader_angular * leader_cmd_angular_gain_,
        -max_angular_speed_,
        max_angular_speed_);
      status = "MIRROR_REVERSE";
      return true;
    }

    cmd.angular.z = clamp(
      leader_angular * leader_cmd_angular_gain_,
      -max_angular_speed_,
      max_angular_speed_);
    status = "MIRROR_TURN";
    return true;
  }

  void initialize_odom_offset()
  {
    leader_origin_x_ = leader_x_;
    leader_origin_y_ = leader_y_;
    follower_origin_x_ = follower_x_;
    follower_origin_y_ = follower_y_;

    const auto cos_yaw = std::cos(follower_yaw_);
    const auto sin_yaw = std::sin(follower_yaw_);
    initial_leader_offset_world_x_ =
      cos_yaw * initial_leader_offset_x_ - sin_yaw * initial_leader_offset_y_;
    initial_leader_offset_world_y_ =
      sin_yaw * initial_leader_offset_x_ + cos_yaw * initial_leader_offset_y_;

    odom_offset_initialized_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Initial odom offset enabled: leader starts at follower-frame offset x=%.3f y=%.3f m",
      initial_leader_offset_x_, initial_leader_offset_y_);
  }

  bool heartbeat_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_heartbeat_) {
      return true;
    }
    return (current_time - last_heartbeat_time_).seconds() > heartbeat_timeout_;
  }

  bool platoon_mode_allows_distance_control() const
  {
    return platoon_mode_state_ == "FOLLOW" ||
      platoon_mode_state_ == "STANDBY" ||
      platoon_mode_state_ == "HANDOFF";
  }

  bool target_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_target_visible_ || !have_target_distance_) {
      return true;
    }
    const auto visible_age = (current_time - last_target_visible_time_).seconds();
    const auto distance_age = (current_time - last_target_distance_time_).seconds();
    return visible_age > marker_lost_timeout_ || distance_age > marker_lost_timeout_;
  }

  bool odom_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_leader_odom_ || !have_follower_odom_) {
      return true;
    }
    const auto leader_age = (current_time - last_leader_odom_time_).seconds();
    const auto follower_age = (current_time - last_follower_odom_time_).seconds();
    return leader_age > odom_timeout_ || follower_age > odom_timeout_;
  }

  bool odom_reference_available(const rclcpp::Time & current_time) const
  {
    if (!have_leader_odom_ || !have_follower_odom_) {
      return false;
    }
    const auto leader_age = (current_time - last_leader_odom_time_).seconds();
    const auto follower_age = (current_time - last_follower_odom_time_).seconds();
    if (follower_age > odom_timeout_) {
      return false;
    }
    if (leader_age <= odom_timeout_) {
      return true;
    }
    return search_last_leader_pose_ && leader_age <= leader_search_timeout_;
  }

  bool valid_distance() const
  {
    return std::isfinite(measured_distance_) && measured_distance_ > 0.0;
  }

  std::string platooning_mode_;
  double target_distance_;
  double min_distance_;
  double max_distance_;
  double emergency_stop_distance_;
  double kp_distance_;
  double kp_yaw_;
  double max_linear_speed_;
  double max_angular_speed_;
  double marker_lost_timeout_;
  double odom_timeout_;
  double leader_search_timeout_;
  double leader_search_stop_tolerance_;
  double heartbeat_timeout_;
  double leader_cmd_timeout_;
  bool enable_reverse_;
  bool allow_distance_reverse_;
  bool search_last_leader_pose_;
  bool allow_odom_without_heartbeat_;
  bool mirror_leader_reverse_turn_;
  double leader_cmd_linear_gain_;
  double leader_cmd_angular_gain_;
  double leader_cmd_reverse_threshold_;
  double leader_cmd_turn_threshold_;
  double leader_cmd_turn_linear_deadband_;
  bool use_initial_odom_offset_;
  double initial_leader_offset_x_;
  double initial_leader_offset_y_;

  bool target_visible_{false};
  bool have_target_visible_{false};
  double target_offset_x_{0.0};
  double measured_distance_{-1.0};
  bool have_target_distance_{false};
  bool follower_enable_{false};
  bool have_follower_enable_{false};
  std::string platoon_mode_state_;
  bool have_platoon_mode_{false};
  bool have_heartbeat_{false};
  bool have_leader_cmd_{false};
  bool have_leader_odom_{false};
  bool have_follower_odom_{false};
  geometry_msgs::msg::Twist latest_leader_cmd_;
  double leader_x_{0.0};
  double leader_y_{0.0};
  double follower_x_{0.0};
  double follower_y_{0.0};
  double follower_yaw_{0.0};
  double last_distance_error_{0.0};
  double last_measured_distance_{-1.0};
  bool odom_offset_initialized_{false};
  double leader_origin_x_{0.0};
  double leader_origin_y_{0.0};
  double follower_origin_x_{0.0};
  double follower_origin_y_{0.0};
  double initial_leader_offset_world_x_{0.0};
  double initial_leader_offset_world_y_{0.0};

  rclcpp::Time last_target_visible_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_distance_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_leader_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_leader_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_follower_odom_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_offset_x_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follower_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr platoon_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr leader_cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_distance_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerPlatooningNode>());
  rclcpp::shutdown();
  return 0;
}
