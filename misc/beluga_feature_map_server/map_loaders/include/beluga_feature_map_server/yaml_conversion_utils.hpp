// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_FEATURE_YAML_CONVERSION_UTILS_HPP
#define BELUGA_FEATURE_YAML_CONVERSION_UTILS_HPP

// external
#include <yaml-cpp/yaml.h>

// project
#include <beluga_feature_map_server/yaml_conversion_types.hpp>

namespace YAML {
template <> struct convert<beluga_feature_map_server::FeaturePosition> {
  static Node encode(const beluga_feature_map_server::FeaturePosition &rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node &node,
                     beluga_feature_map_server::FeaturePosition &rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    return true;
  }
};

template <> struct convert<beluga_feature_map_server::FeatureOrientation> {
  static Node encode(const beluga_feature_map_server::FeatureOrientation &rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node &node,
                     beluga_feature_map_server::FeatureOrientation &rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    rhs.w = node[3].as<double>();
    return true;
  }
};

template <> struct convert<beluga_feature_map_server::FeatureData> {
  static Node encode(const beluga_feature_map_server::FeatureData &rhs) {
    Node node;

    if (rhs.position.has_value()) {
      node["position"] = rhs.position.value();
    }

    if (rhs.orientation.has_value()) {
      node["orientation"] = rhs.orientation.value();
    }

    if (rhs.category.has_value()) {
      node["category"] = rhs.category.value();
    }

    return node;
  }

  static bool decode(const Node &node,
                     beluga_feature_map_server::FeatureData &rhs) {
    if (!node.IsMap()) {
      return false;
    }

    if (node["position"]) {
      rhs.position =
          node["position"].as<beluga_feature_map_server::FeaturePosition>();
    }

    if (node["orientation"]) {
      rhs.orientation =
          node["orientation"]
              .as<beluga_feature_map_server::FeatureOrientation>();
    }

    if (node["category"]) {
      rhs.category =
          node["category"].as<beluga_feature_map_server::FeatureCategory>();
    }

    return true;
  }
};

template <> struct convert<beluga_feature_map_server::FeatureMap> {
  static Node encode(const beluga_feature_map_server::FeatureMap &rhs) {
    Node node;
    node["frame_id"] = rhs.frame_id;
    node["features"] = rhs.features;
    return node;
  }

  static bool decode(const Node &node,
                     beluga_feature_map_server::FeatureMap &rhs) {
    if (!node.IsMap()) {
      return false;
    }

    if (node["frame_id"]) {
      rhs.frame_id =
          node["frame_id"].as<beluga_feature_map_server::FeatureFrameId>();
    }

    if (node["features"]) {
      rhs.features =
          node["features"].as<beluga_feature_map_server::FeatureVector>();
    }
    return true;
  }
};

} // namespace YAML

#endif // #define BELUGA_FEATURE_YAML_CONVERSION_UTILS_HPP
