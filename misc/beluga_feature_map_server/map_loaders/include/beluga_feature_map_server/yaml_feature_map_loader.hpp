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

#ifndef BELUGA_FEATURE_YAML_MAP_LOADER_HPP

// external
#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>

// standard library
#include <optional>
#include <string>

// project
#include <beluga_feature_map_server/feature_map_loader_interface.hpp>

namespace beluga_feature_map_server {

class YamlFeatureMapLoader : public FeatureMapLoaderInterface {
public:
  /**
   * @brief Construct a new Yaml Feature Map Loader object
   * @param map_file Path to the yaml file that contains the feature map
   * description.
   */
  explicit YamlFeatureMapLoader(const std::string &map_file);

  /**
   * @brief Load the feature map
   * @return beluga_feature_map_server_msgs::msg::DiscreteFeatureMap
   */
  std::optional<beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>
  loadMap() override;

private:
  std::string map_file_;
};

} // namespace beluga_feature_map_server

#endif // BELUGA_FEATURE_MAP_LOADER_INTERFACE_HPP
