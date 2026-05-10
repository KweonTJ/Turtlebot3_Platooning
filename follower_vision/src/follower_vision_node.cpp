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
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/version.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace
{
int dictionary_id_from_name(const std::string & name)
{
  if (name == "DICT_4X4_50") {
    return cv::aruco::DICT_4X4_50;
  }
  if (name == "DICT_4X4_100") {
    return cv::aruco::DICT_4X4_100;
  }
  if (name == "DICT_4X4_250") {
    return cv::aruco::DICT_4X4_250;
  }
  if (name == "DICT_4X4_1000") {
    return cv::aruco::DICT_4X4_1000;
  }
  if (name == "DICT_5X5_50") {
    return cv::aruco::DICT_5X5_50;
  }
  if (name == "DICT_5X5_100") {
    return cv::aruco::DICT_5X5_100;
  }
  if (name == "DICT_5X5_250") {
    return cv::aruco::DICT_5X5_250;
  }
  if (name == "DICT_5X5_1000") {
    return cv::aruco::DICT_5X5_1000;
  }
  if (name == "DICT_6X6_50") {
    return cv::aruco::DICT_6X6_50;
  }
  if (name == "DICT_6X6_100") {
    return cv::aruco::DICT_6X6_100;
  }
  if (name == "DICT_6X6_250") {
    return cv::aruco::DICT_6X6_250;
  }
  if (name == "DICT_6X6_1000") {
    return cv::aruco::DICT_6X6_1000;
  }
  return cv::aruco::DICT_4X4_50;
}

float polygon_area(const std::vector<cv::Point2f> & corners)
{
  if (corners.size() < 4) {
    return 0.0F;
  }
  return static_cast<float>(std::abs(cv::contourArea(corners)));
}
}  // namespace

class FollowerVisionNode : public rclcpp::Node
{
public:
  FollowerVisionNode()
  : Node("follower_vision")
  {
    marker_type_ = declare_parameter<std::string>("marker_type", "aruco");
    aruco_dictionary_ = declare_parameter<std::string>("aruco_dictionary", "DICT_4X4_50");
    marker_id_ = declare_parameter<int>("marker_id", 0);
    marker_size_ = declare_parameter<double>("marker_size", 0.10);

    const auto image_topic =
      declare_parameter<std::string>("image_topic", "/follower/camera/image_raw");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/follower/camera/camera_info");
    const auto target_visible_topic =
      declare_parameter<std::string>("target_visible_topic", "/follower/target_visible");
    const auto target_offset_x_topic =
      declare_parameter<std::string>("target_offset_x_topic", "/follower/target_offset_x");
    const auto target_distance_topic =
      declare_parameter<std::string>("target_distance_topic", "/follower/target_distance");
    const auto target_area_topic =
      declare_parameter<std::string>("target_area_topic", "/follower/target_area");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
    const auto debug_image_topic =
      declare_parameter<std::string>("debug_image_topic", "/follower/debug_image");

    if (marker_type_ != "aruco") {
      RCLCPP_WARN(
        get_logger(), "Unsupported marker_type '%s'; using ArUco detection",
        marker_type_.c_str());
    }

    const auto dictionary_id = dictionary_id_from_name(aruco_dictionary_);
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(dictionary_id));
#else
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
#endif

    target_visible_pub_ =
      create_publisher<std_msgs::msg::Bool>(target_visible_topic, rclcpp::QoS(10));
    target_offset_x_pub_ =
      create_publisher<std_msgs::msg::Float32>(target_offset_x_topic, rclcpp::QoS(10));
    target_distance_pub_ =
      create_publisher<std_msgs::msg::Float32>(target_distance_topic, rclcpp::QoS(10));
    target_area_pub_ =
      create_publisher<std_msgs::msg::Float32>(target_area_topic, rclcpp::QoS(10));

    if (publish_debug_image_) {
      debug_image_pub_ =
        create_publisher<sensor_msgs::msg::Image>(debug_image_topic, rclcpp::QoS(10));
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&FollowerVisionNode::image_callback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      std::bind(&FollowerVisionNode::camera_info_callback, this, std::placeholders::_1));
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    cv::Mat camera_matrix(3, 3, CV_64F);
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t col = 0; col < 3; ++col) {
        camera_matrix.at<double>(static_cast<int>(row), static_cast<int>(col)) =
          msg->k[row * 3 + col];
      }
    }

    cv::Mat distortion_coefficients;
    if (msg->d.empty()) {
      distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      distortion_coefficients = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
      for (std::size_t i = 0; i < msg->d.size(); ++i) {
        distortion_coefficients.at<double>(0, static_cast<int>(i)) = msg->d[i];
      }
    }

    camera_matrix_ = camera_matrix;
    distortion_coefficients_ = distortion_coefficients;
    has_camera_info_ = true;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Image conversion failed: %s", ex.what());
      publish_detection(false, 0.0F, -1.0F, 0.0F);
      return;
    }

    if (cv_ptr->image.empty()) {
      publish_detection(false, 0.0F, -1.0F, 0.0F);
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);

    auto target_index = -1;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == marker_id_) {
        target_index = static_cast<int>(i);
        break;
      }
    }

    if (target_index < 0) {
      publish_detection(false, 0.0F, -1.0F, 0.0F);
      publish_debug_image(msg->header, cv_ptr->image, corners, target_index);
      return;
    }

    const auto & target_corners = corners[static_cast<std::size_t>(target_index)];
    cv::Point2f marker_center(0.0F, 0.0F);
    for (const auto & corner : target_corners) {
      marker_center += corner;
    }
    marker_center *= 1.0F / static_cast<float>(target_corners.size());

    const auto image_width = static_cast<float>(cv_ptr->image.cols);
    const auto target_offset_x =
      (marker_center.x - image_width / 2.0F) / std::max(image_width / 2.0F, 1.0F);

    auto target_distance = -1.0F;
    if (has_camera_info_) {
      std::vector<std::vector<cv::Point2f>> selected_corners{target_corners};
      std::vector<cv::Vec3d> rvecs;
      std::vector<cv::Vec3d> tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        selected_corners, marker_size_, camera_matrix_, distortion_coefficients_, rvecs, tvecs);
      if (!tvecs.empty() && std::isfinite(tvecs.front()[2]) && tvecs.front()[2] > 0.0) {
        target_distance = static_cast<float>(tvecs.front()[2]);
      }
    }

    publish_detection(
      true, target_offset_x, target_distance,
      polygon_area(target_corners));
    publish_debug_image(msg->header, cv_ptr->image, corners, target_index);
  }

  void publish_detection(
    bool visible, float target_offset_x, float target_distance,
    float target_area)
  {
    std_msgs::msg::Bool visible_msg;
    visible_msg.data = visible;
    target_visible_pub_->publish(visible_msg);

    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = target_offset_x;
    target_offset_x_pub_->publish(offset_msg);

    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = target_distance;
    target_distance_pub_->publish(distance_msg);

    std_msgs::msg::Float32 area_msg;
    area_msg.data = target_area;
    target_area_pub_->publish(area_msg);
  }

  void publish_debug_image(
    const std_msgs::msg::Header & header, const cv::Mat & image,
    const std::vector<std::vector<cv::Point2f>> & corners, int target_index)
  {
    if (!publish_debug_image_ || !debug_image_pub_) {
      return;
    }

    cv::Mat debug_image = image.clone();
    if (!corners.empty()) {
      cv::aruco::drawDetectedMarkers(debug_image, corners);
    }
    if (target_index >= 0) {
      cv::Point2f marker_center(0.0F, 0.0F);
      const auto & target_corners = corners[static_cast<std::size_t>(target_index)];
      for (const auto & corner : target_corners) {
        marker_center += corner;
      }
      marker_center *= 1.0F / static_cast<float>(target_corners.size());
      cv::circle(debug_image, marker_center, 4, cv::Scalar(0, 255, 0), -1);
    }

    cv_bridge::CvImage debug_msg(header, sensor_msgs::image_encodings::BGR8, debug_image);
    debug_image_pub_->publish(*debug_msg.toImageMsg());
  }

  std::string marker_type_;
  std::string aruco_dictionary_;
  int marker_id_;
  double marker_size_;
  bool publish_debug_image_;
  bool has_camera_info_{false};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_visible_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_offset_x_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_area_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerVisionNode>());
  rclcpp::shutdown();
  return 0;
}
