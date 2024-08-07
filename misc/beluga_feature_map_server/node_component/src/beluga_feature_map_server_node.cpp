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
#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// project
#include <beluga_feature_map_server/beluga_feature_map_server_node.hpp>
#include <beluga_feature_map_server/yaml_feature_map_loader.hpp>

// standard library
#include <memory>
#include <utility>

namespace beluga_feature_map_server {

namespace {
constexpr auto kMapFileParameter = "map_file";

constexpr auto kMapReloadService = "reload_map";
} // namespace

BelugaFeatureMapServerNode::BelugaFeatureMapServerNode(
    const rclcpp::NodeOptions &options)
    : rclcpp::Node{"beluga_feature_map_server", options} {
  RCLCPP_INFO(get_logger(), "Feature map server node starting");
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "File to load the feature map from";
    declare_parameter(kMapFileParameter, rclcpp::ParameterValue(""),
                      descriptor);
  }

  map_publisher_ = this->create_publisher<
      beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>(
      "landmarks_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "landmark_markers",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  load_map_service_ = create_service<std_srvs::srv::Trigger>(
      kMapReloadService,
      std::bind(&BelugaFeatureMapServerNode::reloadMapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));

  map_loader_ = std::make_unique<YamlFeatureMapLoader>(
      get_parameter(kMapFileParameter).as_string());

  RCLCPP_INFO_STREAM(
      get_logger(),
      "Feature map source: " << get_parameter(kMapFileParameter).as_string());

  if (reloadAndPublish()) {
    RCLCPP_INFO(get_logger(), "The feature map was successfully loaded");
  } else {
    RCLCPP_ERROR(get_logger(), "The feature map could not be loaded from the "
                               "source. Please check the file path");
  }
}

void BelugaFeatureMapServerNode::reloadMapServiceCallback(
    [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>
        request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(get_logger(), "Reloading feature map");

  if (reloadAndPublish()) {
    response->success = true;
    response->message = "Feature map reloaded";
  } else {
    response->success = false;
    response->message = "Feature map could not be reloaded";
    RCLCPP_ERROR(get_logger(), "There was an error loading the feature map");
  }
}

bool BelugaFeatureMapServerNode::reloadAndPublish() {
  auto map_opt_ = map_loader_->loadMap();
  if (map_opt_.has_value()) {
    map_publisher_->publish(*map_opt_);
    publishMarkers(*map_opt_);
  } else {
  }
  return map_opt_.has_value();
}

void BelugaFeatureMapServerNode::publishMarkers(
    const beluga_feature_map_server_msgs::msg::DiscreteFeatureMap
        &landmarks_map) {
  constexpr double kSideLength = 0.1;
  int seq_id = 0;
  static int max_seq_id = 0;

  visualization_msgs::msg::MarkerArray msg;
  for (const auto &feature_pose : landmarks_map.positions) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = landmarks_map.header.frame_id;
    marker.header.stamp = landmarks_map.header.stamp;
    marker.id = seq_id++;
    marker.ns = "landmarks_map";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = feature_pose.x;
    marker.pose.position.y = feature_pose.y;
    marker.pose.position.z = feature_pose.z;
    marker.scale.x = kSideLength;
    marker.scale.y = kSideLength;
    marker.scale.z = kSideLength;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }

  // delete old markers
  for (int i = seq_id; i < max_seq_id; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = landmarks_map.header.frame_id;
    marker.header.stamp = landmarks_map.header.stamp;
    marker.id = i;
    marker.ns = "landmarks_map";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    msg.markers.push_back(marker);
  }

  max_seq_id = seq_id;

  marker_publisher_->publish(msg);
}

} // namespace beluga_feature_map_server

RCLCPP_COMPONENTS_REGISTER_NODE(
    beluga_feature_map_server::BelugaFeatureMapServerNode)
