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

#ifndef BELUGA_APRILTAG_ADAPTER_NODE_HPP
#define BELUGA_APRILTAG_ADAPTER_NODE_HPP

// external
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <beluga_april_tag_adapter_msgs/msg/feature_detections.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace beluga_april_tag_adapter {

class BelugaAprilTagAdapterNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Beluga Feature Map Server Node object
   *
   * @param options Node options
   */
  explicit BelugaAprilTagAdapterNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      tag_subscriber_;
  rclcpp::Publisher<beluga_april_tag_adapter_msgs::msg::FeatureDetections>::
      SharedPtr detections_republisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_republisher_;

  void tagDetectionsCallback(
      const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void republishAsFeatures(
      const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void republishAsMarkers(
      const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr det);
};

} // namespace beluga_april_tag_adapter

#endif // BELUGA_APRILTAG_ADAPTER_NODE_HPP
