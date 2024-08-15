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

#ifndef BELUGA_FEATURE_MAP_LOADER_INTERFACE_HPP
#define BELUGA_FEATURE_MAP_LOADER_INTERFACE_HPP

// external
#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>

// standard library
#include <memory>
#include <optional>

namespace beluga_feature_map_server {

class FeatureMapLoaderInterface {
public:
  using Ptr = std::unique_ptr<FeatureMapLoaderInterface>;

  /** @brief Construct a new Feature Map Loader Interface object */
  virtual ~FeatureMapLoaderInterface() = default;

  /**
   * @brief Load the feature map
   * @return beluga_feature_map_server_msgs::msg::DiscreteFeatureMap
   */
  virtual std::optional<beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>
  loadMap() = 0;
};

} // namespace beluga_feature_map_server

#endif // BELUGA_FEATURE_MAP_LOADER_INTERFACE_HPP
