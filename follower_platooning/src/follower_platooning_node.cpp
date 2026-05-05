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
#include <string>

#include "geometry_msgs/msg/twist.hpp"
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
}  // namespace

class FollowerPlatooningNode : public rclcpp::Node
{
public:
  FollowerPlatooningNode()
  : Node("follower_platooning")
  {
    platooning_mode_ = declare_parameter<std::string>("platooning_mode", "vision");

    target_distance_ = declare_parameter<double>("target_distance", 0.20);
    min_distance_ = declare_parameter<double>("min_distance", 0.15);
    max_distance_ = declare_parameter<double>("max_distance", 0.30);
    emergency_stop_distance_ = declare_parameter<double>("emergency_stop_distance", 0.12);

    kp_distance_ = declare_parameter<double>("kp_distance", 0.35);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 0.80);

    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.05);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.25);

    marker_lost_timeout_ = declare_parameter<double>("marker_lost_timeout", 0.5);
    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout", 1.0);

    enable_reverse_ = declare_parameter<bool>("enable_reverse", false);

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

    follower_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      follower_enable_topic, leader_state_qos,
      std::bind(&FollowerPlatooningNode::follower_enable_callback, this, std::placeholders::_1));
    platoon_mode_sub_ = create_subscription<std_msgs::msg::String>(
      platoon_mode_topic, leader_state_qos,
      std::bind(&FollowerPlatooningNode::platoon_mode_callback, this, std::placeholders::_1));
    heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>(
      heartbeat_topic, heartbeat_qos,
      std::bind(&FollowerPlatooningNode::heartbeat_callback, this, std::placeholders::_1));

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
  }

  geometry_msgs::msg::Twist compute_command(const rclcpp::Time & current_time, std::string & status)
  {
    last_distance_error_ = 0.0;

    if (platooning_mode_ != "vision") {
      status = "WAITING_ENABLE";
      return zero_twist();
    }
    if (!have_follower_enable_) {
      status = "WAITING_ENABLE";
      return zero_twist();
    }
    if (!follower_enable_) {
      status = "DISABLED";
      return zero_twist();
    }
    if (!have_platoon_mode_ || platoon_mode_state_ != "FOLLOW") {
      status = "WAITING_ENABLE";
      return zero_twist();
    }
    if (heartbeat_timed_out(current_time)) {
      status = "HEARTBEAT_TIMEOUT";
      return zero_twist();
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

    if (measured_distance_ <= emergency_stop_distance_) {
      status = "TOO_CLOSE_STOP";
      return zero_twist();
    }
    if (measured_distance_ <= min_distance_) {
      status = "TOO_CLOSE_STOP";
      return zero_twist();
    }

    geometry_msgs::msg::Twist cmd;
    auto linear_x = kp_distance_ * last_distance_error_;
    if (!enable_reverse_) {
      linear_x = std::max(0.0, linear_x);
    }
    const auto min_linear_speed = enable_reverse_ ? -max_linear_speed_ : 0.0;
    cmd.linear.x = clamp(linear_x, min_linear_speed, max_linear_speed_);
    cmd.angular.z = clamp(-kp_yaw_ * target_offset_x_, -max_angular_speed_, max_angular_speed_);

    status = "FOLLOWING";
    return cmd;
  }

  bool heartbeat_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_heartbeat_) {
      return true;
    }
    return (current_time - last_heartbeat_time_).seconds() > heartbeat_timeout_;
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
  double heartbeat_timeout_;
  bool enable_reverse_;

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
  double last_distance_error_{0.0};

  rclcpp::Time last_target_visible_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_distance_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_offset_x_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follower_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr platoon_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_error_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerPlatooningNode>());
  rclcpp::shutdown();
  return 0;
}
