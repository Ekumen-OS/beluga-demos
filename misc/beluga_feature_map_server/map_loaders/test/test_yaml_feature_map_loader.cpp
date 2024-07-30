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

// external
#include <gtest/gtest.h>

// project
#include <beluga_feature_map_server/yaml_conversion_utils.hpp>
#include <beluga_feature_map_server/yaml_feature_map_loader.hpp>

// standard library
#include <fstream>
#include <iostream>
#include <sstream>

namespace beluga_feature_map_server {

constexpr auto serialized_input_only_category = R"(
frame_id: map
features:
  - category: 99
  - category: 15
  - category: 99
)";

struct YamlFeatureMapLoaderTests : public testing::Test {
  std::string createRandomFilename() {
    std::stringstream ss;
    ss << "/tmp/" << std::rand() << ".yaml";
    return ss.str();
  }

  void setupTestFile(const std::string &filename,
                     const std::string &file_contents) {
    std::ofstream out(filename);
    out << file_contents;
  }
};

TEST_F(YamlFeatureMapLoaderTests, FeatureCategory) {
  const auto filename = createRandomFilename();
  setupTestFile(filename, serialized_input_only_category);

  // check that we can actually load the feature map
  auto uut = YamlFeatureMapLoader{filename};
  auto data = uut.loadMap();

  ASSERT_TRUE(data.has_value());
  ASSERT_EQ(data->header.frame_id, "map");
  ASSERT_EQ(data->positions.size(), 0u);
  ASSERT_EQ(data->orientations.size(), 0u);
  ASSERT_EQ(data->categories.size(), 3u);
}

constexpr auto serialized_input_position_and_orientation = R"(
frame_id: map
features:
  - position: [1, 2, 3]
    orientation: [4, 5, 6, 7]
  - position: [8, 9, 10]
    orientation: [11, 12, 13, 14]
)";

TEST_F(YamlFeatureMapLoaderTests, OnlyPositionAndOrientation) {
  const auto filename = createRandomFilename();
  setupTestFile(filename, serialized_input_position_and_orientation);

  // check that we can actually load the feature map
  auto uut = YamlFeatureMapLoader{filename};
  auto data = uut.loadMap();

  ASSERT_TRUE(data.has_value());
  ASSERT_EQ(data->header.frame_id, "map");
  ASSERT_EQ(data->positions.size(), 2u);
  ASSERT_EQ(data->orientations.size(), 2u);
  ASSERT_EQ(data->categories.size(), 0u);
}

constexpr auto serialized_input_all_three_fields = R"(
frame_id: map
features:
  - position: [1, 2, 3]
    orientation: [4, 5, 6, 7]
    category: 99
  - position: [8, 9, 10]
    orientation: [11, 12, 13, 14]
    category: 15
)";

TEST_F(YamlFeatureMapLoaderTests, AllThreeFields) {
  const auto filename = createRandomFilename();
  setupTestFile(filename, serialized_input_all_three_fields);

  // check that we can actually load the feature map
  auto uut = YamlFeatureMapLoader{filename};
  auto data = uut.loadMap();

  ASSERT_TRUE(data.has_value());
  ASSERT_EQ(data->header.frame_id, "map");
  ASSERT_EQ(data->positions.size(), 2u);
  ASSERT_EQ(data->orientations.size(), 2u);
  ASSERT_EQ(data->categories.size(), 2u);
}

constexpr auto serialized_input_minimal_file = R"(
frame_id: world
features: []
)";

TEST_F(YamlFeatureMapLoaderTests, MinimalFile) {
  const auto filename = createRandomFilename();
  setupTestFile(filename, serialized_input_minimal_file);

  // check that we can actually load the feature map
  auto uut = YamlFeatureMapLoader{filename};
  auto data = uut.loadMap();

  ASSERT_TRUE(data.has_value());
  ASSERT_EQ(data->header.frame_id, "world");
  ASSERT_EQ(data->positions.size(), 0u);
  ASSERT_EQ(data->orientations.size(), 0u);
  ASSERT_EQ(data->categories.size(), 0u);
}

constexpr auto serialized_input_unbalanced_fields_in_different_features = R"(
frame_id: map
features:
  - position: [1, 2, 3]
    orientation: [4, 5, 6, 7]
    category: 99
  - position: [8, 9, 10]
    category: 15
)";

TEST_F(YamlFeatureMapLoaderTests, UnbalancedFields) {
  const auto filename = createRandomFilename();
  setupTestFile(filename,
                serialized_input_unbalanced_fields_in_different_features);

  // check that we can actually load the feature map
  auto uut = YamlFeatureMapLoader{filename};
  auto data = uut.loadMap();

  ASSERT_FALSE(data.has_value());
}

TEST_F(YamlFeatureMapLoaderTests, FileDoesNotExist) {
  const auto filename = createRandomFilename();

  // check that we can actually load the feature map
  auto uut = YamlFeatureMapLoader{filename};
  auto data = uut.loadMap();

  ASSERT_FALSE(data.has_value());
}

} // namespace beluga_feature_map_server
