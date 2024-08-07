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
#include <yaml-cpp/yaml.h>

#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>

// standard library
#include <iostream>
#include <optional>
#include <string>

// project
#include <beluga_feature_map_server/yaml_conversion_utils.hpp>
#include <beluga_feature_map_server/yaml_feature_map_loader.hpp>

namespace beluga_feature_map_server {

YamlFeatureMapLoader::YamlFeatureMapLoader(const std::string &map_file)
    : map_file_{map_file} {}

std::optional<beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>
YamlFeatureMapLoader::loadMap() {
  auto map = beluga_feature_map_server_msgs::msg::DiscreteFeatureMap{};

  FeatureMap root_data;
  try {
    YAML::Node node = YAML::LoadFile(map_file_);
    root_data = node.as<FeatureMap>();
  } catch (const YAML::BadFile &e) {
    return std::nullopt;
  }

  map.header.frame_id = root_data.frame_id;

  // it may well be that there are no features, and that's not an error
  const auto first_feature = root_data.features.begin();
  if (first_feature == root_data.features.end()) {
    return map;
  }

  const auto convert_position = [](const FeaturePosition &position) {
    auto p = geometry_msgs::msg::Point{};
    p.x = position.x;
    p.y = position.y;
    p.z = position.z;
    return p;
  };

  const auto convert_orientation = [](const FeatureOrientation &orientation) {
    auto r = geometry_msgs::msg::Quaternion{};
    r.x = orientation.x;
    r.y = orientation.y;
    r.z = orientation.z;
    r.w = orientation.w;
    return r;
  };

  // there's at least one feature, so the first one determined which fields are
  // present in the rest of the sequence

  const auto must_have_position = first_feature->position.has_value();
  const auto must_have_orientation = first_feature->orientation.has_value();
  const auto must_have_category = first_feature->category.has_value();

  for (const auto &feature : root_data.features) {
    const auto has_position = feature.position.has_value();
    const auto has_orientation = feature.orientation.has_value();
    const auto has_category = feature.category.has_value();

    if (has_position != must_have_position ||
        has_orientation != must_have_orientation ||
        has_category != must_have_category) {
      return std::nullopt;
    }

    if (has_position) {
      map.positions.push_back(convert_position(feature.position.value()));
    }
    if (has_orientation) {
      map.orientations.push_back(
          convert_orientation(feature.orientation.value()));
    }
    if (has_category) {
      map.categories.push_back(feature.category.value());
    }
  }

  return map;
}

} // namespace beluga_feature_map_server
