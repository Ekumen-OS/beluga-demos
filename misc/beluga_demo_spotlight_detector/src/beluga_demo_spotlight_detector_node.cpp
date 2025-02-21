// Copyright 2022-2023 Ekumen, Inc.
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

// project
#include <beluga_april_tag_adapter_msgs/msg/feature_detections.hpp>
#include <beluga_demo_spotlight_detector/beluga_demo_spotlight_detector_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#ifdef cv_bridge_HAS_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

// standard library
#include <functional>
#include <memory>
#include <utility>

namespace beluga_demo_spotlight_detector {

namespace {

auto create_marker(int id, double x, double y, rclcpp::Time timestamp,
                   [[maybe_unused]] std::string frame_id) {
  visualization_msgs::msg::Marker marker;
  // marker.header.frame_id = frame_id;
  marker.header.frame_id = "camera_rgb_frame";
  marker.header.stamp = timestamp;
  marker.id = id;
  marker.ns = "detections";
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.frame_locked = true;

  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;

  const auto scale = 3.0;

  end.x = scale;
  end.y = -x * scale;
  end.z = -y * scale;

  marker.points.push_back(start);
  marker.points.push_back(end);

  return marker;
}

} // namespace

void BelugaDemoSpotlightDetectorNode::publish_markers(
    const std_msgs::msg::Header &header,
    const std::vector<DetectionData> &detections_vector) const {
  visualization_msgs::msg::MarkerArray markers_msg;

  int seq_id = 0;
  static int max_seq_id = 0;

  for (const auto &detection : detections_vector) {
    double x = (detection.u - model_.cx()) / model_.fx();
    double y = (detection.v - model_.cy()) / model_.fy();

    markers_msg.markers.push_back(
        create_marker(seq_id, x, y, now(), header.frame_id));
    seq_id++;
  }

  // delete old markers
  for (int i = seq_id; i < max_seq_id; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.id = i;
    marker.ns = "detections";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers_msg.markers.push_back(marker);
  }

  max_seq_id = seq_id;

  marker_publisher_->publish(markers_msg);
}

void BelugaDemoSpotlightDetectorNode::publish_detections(
    const std_msgs::msg::Header &header,
    const std::vector<DetectionData> &detections_vector) const {

  beluga_april_tag_adapter_msgs::msg::FeatureDetections feature_msg;
  feature_msg.header = header;
  feature_msg.header.frame_id = "camera_rgb_frame";

  for (const auto &detection : detections_vector) {
    double x = (detection.u - model_.cx()) / model_.fx();
    double y = (detection.v - model_.cy()) / model_.fy();

    geometry_msgs::msg::Point point;

    // point.x = x;
    // point.y = y;
    // point.z = 1.0;
    point.x = 1.0;
    point.y = -x;
    point.z = -y;

    feature_msg.positions.push_back(point);
    feature_msg.categories.push_back(0);
  }
  detections_publisher_->publish(feature_msg);
}

void BelugaDemoSpotlightDetectorNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg) {
  model_.fromCameraInfo(info_msg);

  auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
  }

  cv::Mat blurred;
  cv::GaussianBlur(cv_ptr->image, blurred, cv::Size(5, 5), 0);

  cv::Mat hsv;
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

  cv::Mat bands[3];
  cv::split(hsv, bands);

  cv::Mat saturation = bands[1];
  cv::Mat brightness = bands[2];

  cv::Mat thresh1;
  cv::threshold(brightness, thresh1, 253, 255, cv::THRESH_BINARY);

  cv::Mat thresh2;
  cv::threshold(saturation, thresh2, 2, 255, cv::THRESH_BINARY_INV);

  cv::Mat mask;
  thresh1.copyTo(mask, thresh2);

  cv::Mat filtered;
  cv_ptr->image.copyTo(filtered, mask);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(mask, filtered, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(filtered, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  std::vector<DetectionData> detections;
  detections.reserve(contours.size());

  // generate list of generic detections for later processing
  for (const auto &contour : contours) {
    auto area = cv::contourArea(contour);
    auto moments = cv::moments(contour);
    const auto u = static_cast<int>(moments.m10 / moments.m00);
    const auto v = static_cast<int>(moments.m01 / moments.m00);

    if (u >= 0 && u < cv_ptr->image.size().width && v >= 0 &&
        v < cv_ptr->image.size().height) {
      auto r = static_cast<int>(std::sqrt(area / M_PI));
      detections.push_back({u, v, r});
    }
  }

  cv_ptr->image = filtered;
  cv_ptr->encoding = "mono8";
  image_publisher_.publish(*cv_ptr->toImageMsg());

  publish_markers(msg->header, detections);
  publish_detections(msg->header, detections);
}

BelugaDemoSpotlightDetectorNode::BelugaDemoSpotlightDetectorNode(
    const rclcpp::NodeOptions &options)
    : rclcpp::Node{"beluga_demo_spotlight_detector", options} {
  RCLCPP_INFO(get_logger(), "Spotlight detector node started");

  image_subscriber_ = image_transport::create_camera_subscription(
      this, "/image_rect",
      std::bind(&BelugaDemoSpotlightDetectorNode::image_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      image_transport::TransportHints{this}.getTransport());

  image_publisher_ =
      image_transport::create_publisher(this, "spotlight_detections");

  marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "landmark_detection_markers", rclcpp::SystemDefaultsQoS());

  detections_publisher_ = this->create_publisher<
      beluga_april_tag_adapter_msgs::msg::FeatureDetections>(
      "landmark_detections",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

} // namespace beluga_demo_spotlight_detector

RCLCPP_COMPONENTS_REGISTER_NODE(
    beluga_demo_spotlight_detector::BelugaDemoSpotlightDetectorNode)
