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

#ifndef BELUGA_FEATURE_MAP_SERVER_NODE_HPP
#define BELUGA_FEATURE_MAP_SERVER_NODE_HPP

// external
#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// standard library
#include <memory>

// project
#include <beluga_feature_map_server/feature_map_loader_interface.hpp>

namespace beluga_feature_map_server {

class BelugaFeatureMapServerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Beluga Feature Map Server Node object
   *
   * @param options Node options
   */
  explicit BelugaFeatureMapServerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  std::unique_ptr<FeatureMapLoaderInterface> map_loader_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_service_;
  rclcpp::Publisher<beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>::
      SharedPtr map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;

  void reloadMapServiceCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>
          request,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  [[nodiscard]] bool reloadAndPublish();

  void publishMarkers(
      const beluga_feature_map_server_msgs::msg::DiscreteFeatureMap &map);
};

} // namespace beluga_feature_map_server

#endif // BELUGA_FEATURE_MAP_SERVER_NODE_HPP
