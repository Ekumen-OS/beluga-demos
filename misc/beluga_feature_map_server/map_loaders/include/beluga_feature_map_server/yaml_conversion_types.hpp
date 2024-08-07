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

#ifndef BELUGA_YAML_FEATURE_MAP_LOADER_TYPES_HPP
#define BELUGA_YAML_FEATURE_MAP_LOADER_TYPES_HPP

// standard library
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace beluga_feature_map_server {

struct FeaturePosition {
  double x;
  double y;
  double z;
};

struct FeatureOrientation {
  double x;
  double y;
  double z;
  double w;
};

using FeatureCategory = std::uint32_t;

struct FeatureData {
  std::optional<FeaturePosition> position;
  std::optional<FeatureOrientation> orientation;
  std::optional<FeatureCategory> category;
};

using FeatureVector = std::vector<FeatureData>;

using FeatureFrameId = std::string;

struct FeatureMap {
  FeatureFrameId frame_id;
  FeatureVector features;
};

} // namespace beluga_feature_map_server

#endif // BELUGA_YAML_FEATURE_MAP_LOADER_TYPES_HPP
