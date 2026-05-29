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

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
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

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void rotated_offset(
  double yaw, double offset_x, double offset_y, double & world_x, double & world_y)
{
  const auto cos_yaw = std::cos(yaw);
  const auto sin_yaw = std::sin(yaw);
  world_x = cos_yaw * offset_x - sin_yaw * offset_y;
  world_y = sin_yaw * offset_x + cos_yaw * offset_y;
}
}  // namespace

class LeaderOdomAlignerNode : public rclcpp::Node
{
public:
  LeaderOdomAlignerNode()
  : Node("leader_odom_aligner")
  {
    initial_leader_offset_x_ = declare_parameter<double>("initial_leader_offset_x", 0.30);
    initial_leader_offset_y_ = declare_parameter<double>("initial_leader_offset_y", 0.0);
    leader_imu_offset_x_ = declare_parameter<double>("leader_imu_offset_x", 0.0);
    leader_imu_offset_y_ = declare_parameter<double>("leader_imu_offset_y", 0.0);
    follower_imu_offset_x_ = declare_parameter<double>("follower_imu_offset_x", 0.0);
    follower_imu_offset_y_ = declare_parameter<double>("follower_imu_offset_y", 0.0);

    const auto leader_odom_topic =
      declare_parameter<std::string>("leader_odom_topic", "/leader/odom");
    const auto follower_odom_topic =
      declare_parameter<std::string>("follower_odom_topic", "/odom");
    const auto aligned_leader_odom_topic =
      declare_parameter<std::string>("aligned_leader_odom_topic", "/leader/odom_aligned");

    const auto odom_qos = rclcpp::SensorDataQoS();
    aligned_leader_odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(aligned_leader_odom_topic, odom_qos);
    leader_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_odom_topic, odom_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_leader_odom_ = *msg;
        have_leader_odom_ = true;
        publish_aligned_odom();
      });
    follower_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_odom_topic, odom_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_follower_odom_ = *msg;
        have_follower_odom_ = true;
        publish_aligned_odom();
      });
  }

private:
  void initialize_alignment()
  {
    leader_origin_x_ = latest_leader_odom_.pose.pose.position.x;
    leader_origin_y_ = latest_leader_odom_.pose.pose.position.y;
    leader_origin_yaw_ = yaw_from_quaternion(latest_leader_odom_.pose.pose.orientation);

    follower_origin_x_ = latest_follower_odom_.pose.pose.position.x;
    follower_origin_y_ = latest_follower_odom_.pose.pose.position.y;
    follower_origin_yaw_ = yaw_from_quaternion(latest_follower_odom_.pose.pose.orientation);

    double initial_imu_offset_world_x = 0.0;
    double initial_imu_offset_world_y = 0.0;
    double leader_imu_offset_world_x = 0.0;
    double leader_imu_offset_world_y = 0.0;
    double follower_imu_offset_world_x = 0.0;
    double follower_imu_offset_world_y = 0.0;
    rotated_offset(
      follower_origin_yaw_, initial_leader_offset_x_, initial_leader_offset_y_,
      initial_imu_offset_world_x, initial_imu_offset_world_y);
    rotated_offset(
      leader_origin_yaw_, leader_imu_offset_x_, leader_imu_offset_y_,
      leader_imu_offset_world_x, leader_imu_offset_world_y);
    rotated_offset(
      follower_origin_yaw_, follower_imu_offset_x_, follower_imu_offset_y_,
      follower_imu_offset_world_x, follower_imu_offset_world_y);
    initial_offset_world_x_ =
      initial_imu_offset_world_x + follower_imu_offset_world_x - leader_imu_offset_world_x;
    initial_offset_world_y_ =
      initial_imu_offset_world_y + follower_imu_offset_world_y - leader_imu_offset_world_y;

    initialized_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Leader odom aligned to follower odom: initial leader IMU offset x=%.3f y=%.3f m",
      initial_leader_offset_x_, initial_leader_offset_y_);
  }

  void publish_aligned_odom()
  {
    if (!have_leader_odom_ || !have_follower_odom_) {
      return;
    }
    if (!initialized_) {
      initialize_alignment();
    }

    auto aligned = latest_leader_odom_;
    aligned.header.frame_id = latest_follower_odom_.header.frame_id.empty() ?
      "odom" : latest_follower_odom_.header.frame_id;
    aligned.child_frame_id = "leader_base_footprint_aligned";

    const auto leader_yaw = yaw_from_quaternion(latest_leader_odom_.pose.pose.orientation);
    const auto leader_yaw_delta = normalize_angle(leader_yaw - leader_origin_yaw_);
    aligned.pose.pose.orientation = quaternion_from_yaw(follower_origin_yaw_ + leader_yaw_delta);

    const auto leader_dx = latest_leader_odom_.pose.pose.position.x - leader_origin_x_;
    const auto leader_dy = latest_leader_odom_.pose.pose.position.y - leader_origin_y_;
    const auto odom_frame_yaw_delta = normalize_angle(follower_origin_yaw_ - leader_origin_yaw_);
    double leader_dx_aligned = 0.0;
    double leader_dy_aligned = 0.0;
    rotated_offset(
      odom_frame_yaw_delta, leader_dx, leader_dy, leader_dx_aligned, leader_dy_aligned);
    aligned.pose.pose.position.x =
      follower_origin_x_ + initial_offset_world_x_ + leader_dx_aligned;
    aligned.pose.pose.position.y =
      follower_origin_y_ + initial_offset_world_y_ + leader_dy_aligned;

    aligned_leader_odom_pub_->publish(aligned);
  }

  double initial_leader_offset_x_{0.30};
  double initial_leader_offset_y_{0.0};
  double leader_imu_offset_x_{0.0};
  double leader_imu_offset_y_{0.0};
  double follower_imu_offset_x_{0.0};
  double follower_imu_offset_y_{0.0};
  bool have_leader_odom_{false};
  bool have_follower_odom_{false};
  bool initialized_{false};
  double leader_origin_x_{0.0};
  double leader_origin_y_{0.0};
  double leader_origin_yaw_{0.0};
  double follower_origin_x_{0.0};
  double follower_origin_y_{0.0};
  double follower_origin_yaw_{0.0};
  double initial_offset_world_x_{0.0};
  double initial_offset_world_y_{0.0};
  nav_msgs::msg::Odometry latest_leader_odom_;
  nav_msgs::msg::Odometry latest_follower_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aligned_leader_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderOdomAlignerNode>());
  rclcpp::shutdown();
  return 0;
}
