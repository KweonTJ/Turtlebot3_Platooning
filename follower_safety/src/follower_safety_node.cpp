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
#include "sensor_msgs/msg/laser_scan.hpp"
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

class FollowerSafetyNode : public rclcpp::Node
{
public:
  FollowerSafetyNode()
  : Node("follower_safety")
  {
    front_obstacle_distance_ = declare_parameter<double>("front_obstacle_distance", 0.12);
    side_obstacle_distance_ = declare_parameter<double>("side_obstacle_distance", 0.08);
    front_angle_deg_ = declare_parameter<double>("front_angle_deg", 25.0);
    use_scan_safety_ = declare_parameter<bool>("use_scan_safety", false);

    marker_required_ = declare_parameter<bool>("marker_required", true);
    heartbeat_required_ = declare_parameter<bool>("heartbeat_required", true);
    allow_fresh_cmd_without_heartbeat_ =
      declare_parameter<bool>("allow_fresh_cmd_without_heartbeat", true);
    use_distance_safety_ = declare_parameter<bool>("use_distance_safety", true);

    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout", 1.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    distance_timeout_ = declare_parameter<double>("distance_timeout", 0.5);
    min_distance_ = declare_parameter<double>("min_distance", 0.25);
    emergency_stop_distance_ = declare_parameter<double>("emergency_stop_distance", 0.15);

    allow_reverse_ = declare_parameter<bool>("allow_reverse", false);
    allow_untracked_reverse_or_turn_ =
      declare_parameter<bool>("allow_untracked_reverse_or_turn", true);
    untracked_reverse_threshold_ = declare_parameter<double>("untracked_reverse_threshold", 0.01);
    untracked_turn_threshold_ = declare_parameter<double>("untracked_turn_threshold", 0.05);
    untracked_turn_linear_deadband_ =
      declare_parameter<double>("untracked_turn_linear_deadband", 0.02);

    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.05);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.25);

    const auto cmd_vel_raw_topic =
      declare_parameter<std::string>("cmd_vel_raw_topic", "/follower/cmd_vel_raw");
    const auto target_visible_topic =
      declare_parameter<std::string>("target_visible_topic", "/follower/target_visible");
    const auto target_distance_topic =
      declare_parameter<std::string>("target_distance_topic", "/follower/target_distance");
    const auto heartbeat_topic =
      declare_parameter<std::string>("heartbeat_topic", "/leader/heartbeat");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");

    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto safety_state_topic =
      declare_parameter<std::string>("safety_state_topic", "/follower/safety_state");

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));
    safety_state_pub_ =
      create_publisher<std_msgs::msg::String>(safety_state_topic, rclcpp::QoS(10));

    cmd_vel_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_raw_topic, rclcpp::QoS(10),
      std::bind(&FollowerSafetyNode::cmd_vel_raw_callback, this, std::placeholders::_1));
    target_visible_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_visible_topic, rclcpp::QoS(10),
      std::bind(&FollowerSafetyNode::target_visible_callback, this, std::placeholders::_1));
    target_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      target_distance_topic, rclcpp::QoS(10),
      std::bind(&FollowerSafetyNode::target_distance_callback, this, std::placeholders::_1));
    const auto heartbeat_qos =
      rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
    heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>(
      heartbeat_topic, heartbeat_qos,
      std::bind(&FollowerSafetyNode::heartbeat_callback, this, std::placeholders::_1));
    if (use_scan_safety_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&FollowerSafetyNode::scan_callback, this, std::placeholders::_1));
    }

    safety_timer_ = create_wall_timer(
      50ms, std::bind(&FollowerSafetyNode::safety_timer_callback, this));
  }

private:
  void cmd_vel_raw_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    latest_cmd_ = *msg;
    have_cmd_ = true;
    last_cmd_time_ = now();
  }

  void target_visible_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    target_visible_ = msg->data;
    have_target_visible_ = true;
  }

  void target_distance_callback(const std_msgs::msg::Float32::ConstSharedPtr msg)
  {
    target_distance_ = msg->data;
    have_target_distance_ = std::isfinite(target_distance_) && target_distance_ > 0.0;
    last_target_distance_time_ = now();
  }

  void heartbeat_callback(const std_msgs::msg::Bool::ConstSharedPtr)
  {
    have_heartbeat_ = true;
    last_heartbeat_time_ = now();
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    front_obstacle_ = false;
    if (msg->angle_increment == 0.0F) {
      return;
    }

    constexpr auto pi = 3.14159265358979323846;
    const auto front_angle_rad = front_angle_deg_ * pi / 180.0;
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const auto range = msg->ranges[i];
      if (!std::isfinite(range)) {
        continue;
      }
      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }

      const auto angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
      if (std::abs(static_cast<double>(angle)) <= front_angle_rad &&
        range < front_obstacle_distance_)
      {
        front_obstacle_ = true;
        return;
      }
    }
  }

  void safety_timer_callback()
  {
    std::string state;
    auto cmd = filter_command(now(), state);
    cmd_vel_pub_->publish(cmd);

    std_msgs::msg::String state_msg;
    state_msg.data = state;
    safety_state_pub_->publish(state_msg);
  }

  geometry_msgs::msg::Twist filter_command(const rclcpp::Time & current_time, std::string & state)
  {
    if (use_scan_safety_ && front_obstacle_) {
      state = "FRONT_OBSTACLE";
      return zero_twist();
    }
    if (heartbeat_required_ && heartbeat_timed_out(current_time)) {
      if (!allow_fresh_cmd_without_heartbeat_ || cmd_timed_out(current_time)) {
        state = "HEARTBEAT_TIMEOUT";
        return zero_twist();
      }
    }
    if (cmd_timed_out(current_time)) {
      state = "CMD_TIMEOUT";
      return zero_twist();
    }
    if (use_distance_safety_ && distance_timed_out(current_time)) {
      state = "DISTANCE_TIMEOUT";
      return zero_twist();
    }
    if (!marker_gate_allows_command()) {
      state = "TARGET_LOST";
      return zero_twist();
    }

    auto filtered = latest_cmd_;
    filtered.linear.y = 0.0;
    filtered.linear.z = 0.0;
    filtered.angular.x = 0.0;
    filtered.angular.y = 0.0;

    const auto min_linear_speed = allow_reverse_ ? -max_linear_speed_ : 0.0;
    filtered.linear.x = clamp(filtered.linear.x, min_linear_speed, max_linear_speed_);
    filtered.angular.z = clamp(filtered.angular.z, -max_angular_speed_, max_angular_speed_);

    if (!allow_reverse_ && filtered.linear.x < 0.0) {
      filtered.linear.x = 0.0;
    }

    if (use_distance_safety_ && have_target_distance_) {
      if (target_distance_ <= emergency_stop_distance_) {
        state = "DISTANCE_EMERGENCY_STOP";
        return zero_twist();
      }
      if (target_distance_ < min_distance_ && filtered.linear.x < 0.0) {
        filtered.angular.z = 0.0;
        state = "MIN_DISTANCE_REVERSE";
        return filtered;
      }
      if (target_distance_ <= min_distance_) {
        state = "MIN_DISTANCE_STOP";
        return zero_twist();
      }
    }

    if (std::abs(filtered.linear.x) < 1e-6 && std::abs(filtered.angular.z) < 1e-6) {
      state = "STOPPED";
    } else {
      state = "SAFE";
    }
    return filtered;
  }

  bool marker_gate_allows_command() const
  {
    if (!marker_required_ || (have_target_visible_ && target_visible_)) {
      return true;
    }
    if (!allow_untracked_reverse_or_turn_) {
      return false;
    }

    const auto reversing = latest_cmd_.linear.x < -std::abs(untracked_reverse_threshold_);
    const auto turning =
      std::abs(latest_cmd_.angular.z) > std::abs(untracked_turn_threshold_) &&
      std::abs(latest_cmd_.linear.x) <= std::abs(untracked_turn_linear_deadband_);
    return reversing || turning;
  }

  bool heartbeat_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_heartbeat_) {
      return true;
    }
    return (current_time - last_heartbeat_time_).seconds() > heartbeat_timeout_;
  }

  bool cmd_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_cmd_) {
      return true;
    }
    return (current_time - last_cmd_time_).seconds() > cmd_vel_timeout_;
  }

  bool distance_timed_out(const rclcpp::Time & current_time) const
  {
    if (!have_target_distance_) {
      return true;
    }
    return (current_time - last_target_distance_time_).seconds() > distance_timeout_;
  }

  double front_obstacle_distance_;
  double side_obstacle_distance_;
  double front_angle_deg_;
  bool use_scan_safety_;
  bool marker_required_;
  bool heartbeat_required_;
  bool allow_fresh_cmd_without_heartbeat_;
  bool use_distance_safety_;
  double heartbeat_timeout_;
  double cmd_vel_timeout_;
  double distance_timeout_;
  double min_distance_;
  double emergency_stop_distance_;
  bool allow_reverse_;
  bool allow_untracked_reverse_or_turn_;
  double untracked_reverse_threshold_;
  double untracked_turn_threshold_;
  double untracked_turn_linear_deadband_;
  double max_linear_speed_;
  double max_angular_speed_;

  geometry_msgs::msg::Twist latest_cmd_;
  bool have_cmd_{false};
  bool target_visible_{false};
  bool have_target_visible_{false};
  double target_distance_{-1.0};
  bool have_target_distance_{false};
  bool have_heartbeat_{false};
  bool front_obstacle_{false};

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_distance_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
