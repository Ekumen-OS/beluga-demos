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

// external
#include <beluga_april_tag_adapter_msgs/msg/feature_detections.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// project
#include <beluga_april_tag_adapter/beluga_april_tag_adapter_node.hpp>

// standard library
#include <memory>
#include <utility>

namespace beluga_april_tag_adapter {

namespace {
constexpr auto kMapFileParameter = "map_file";

constexpr auto kMapReloadService = "reload_map";
} // namespace

BelugaAprilTagAdapterNode::BelugaAprilTagAdapterNode(
    const rclcpp::NodeOptions &options)
    : rclcpp::Node{"beluga_april_tag_adapter", options} {
  RCLCPP_INFO(get_logger(), "April Tag detections republisher node started");
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "File to load the feature map from";
    declare_parameter(kMapFileParameter, rclcpp::ParameterValue(""),
                      descriptor);
  }

  tag_subscriber_ =
      this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
          "apriltag_detections", rclcpp::SystemDefaultsQoS(),
          std::bind(&BelugaAprilTagAdapterNode::tagDetectionsCallback, this,
                    std::placeholders::_1));

  detections_republisher_ = this->create_publisher<
      beluga_april_tag_adapter_msgs::msg::FeatureDetections>(
      "landmark_detections",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  marker_republisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "landmark_detection_markers", rclcpp::SystemDefaultsQoS());
}

void BelugaAprilTagAdapterNode::tagDetectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr det) {
  republishAsFeatures(det);
  republishAsMarkers(det);
}

void BelugaAprilTagAdapterNode::republishAsFeatures(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr det) {
  beluga_april_tag_adapter_msgs::msg::FeatureDetections msg;
  msg.header = det->header;

  for (const auto &detection : det->detections) {
    msg.positions.push_back(detection.pose.pose.pose.position);
    msg.orientations.push_back(detection.pose.pose.pose.orientation);
    msg.categories.push_back(detection.id);
  }

  detections_republisher_->publish(msg);
}

void BelugaAprilTagAdapterNode::republishAsMarkers(
    [[maybe_unused]] const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr
        det) {
  constexpr double kSideLength = 0.15;

  int seq_id = 0;
  static int max_seq_id = 0;

  visualization_msgs::msg::MarkerArray msg;
  for (const auto &detection : det->detections) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = det->header.frame_id;
    marker.header.stamp = det->header.stamp;
    marker.id = seq_id++;
    marker.ns = "detections";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = detection.pose.pose.pose;
    marker.scale.x = kSideLength;
    marker.scale.y = kSideLength;
    marker.scale.z = kSideLength;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    msg.markers.push_back(marker);
  }

  // delete old markers
  for (int i = seq_id; i < max_seq_id; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = det->header.frame_id;
    marker.header.stamp = det->header.stamp;
    marker.id = i;
    marker.ns = "detections";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    msg.markers.push_back(marker);
  }

  max_seq_id = seq_id;

  marker_republisher_->publish(msg);
}

} // namespace beluga_april_tag_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(
    beluga_april_tag_adapter::BelugaAprilTagAdapterNode)
