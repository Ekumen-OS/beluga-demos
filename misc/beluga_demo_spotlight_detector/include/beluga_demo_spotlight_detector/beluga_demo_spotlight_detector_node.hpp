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

#ifndef BELUGA_SPOTLIGHT_DETECTOR_NODE_HPP
#define BELUGA_SPOTLIGHT_DETECTOR_NODE_HPP

#ifdef image_geometry_HAS_HPP
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace beluga_demo_spotlight_detector {

class BelugaDemoSpotlightDetectorNode : public rclcpp::Node {
public:
  explicit BelugaDemoSpotlightDetectorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  struct DetectionData {
    int u;
    int v;
    int r;
  };

  image_geometry::PinholeCameraModel model_{};
  image_transport::CameraSubscriber image_subscriber_;
  image_transport::Publisher image_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;
  rclcpp::Publisher<beluga_april_tag_adapter_msgs::msg::FeatureDetections>::
      SharedPtr detections_publisher_;

  void
  publish_markers(const std_msgs::msg::Header &header,
                  const std::vector<DetectionData> &detections_vector) const;
  void
  publish_detections(const std_msgs::msg::Header &header,
                     const std::vector<DetectionData> &detections_vector) const;

  void
  image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg,
                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
};

} // namespace beluga_demo_spotlight_detector

#endif // BELUGA_SPOTLIGHT_DETECTOR_NODE_HPP
