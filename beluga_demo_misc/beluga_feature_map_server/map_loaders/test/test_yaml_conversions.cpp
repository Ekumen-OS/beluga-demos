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
#include <beluga_feature_map_server/yaml_conversion_types.hpp>
#include <beluga_feature_map_server/yaml_conversion_utils.hpp>

// standard library
#include <sstream>

namespace beluga_feature_map_server {

TEST(DirectConversionTests, FeaturePosition) {
  constexpr auto serialized_input =
      R"(
      [1, 2, 3]
    )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeaturePosition>();

  ASSERT_DOUBLE_EQ(uut.x, 1.0);
  ASSERT_DOUBLE_EQ(uut.y, 2.0);
  ASSERT_DOUBLE_EQ(uut.z, 3.0);
}

TEST(DirectConversionTests, FeatureOrientation) {
  constexpr auto serialized_input =
      R"(
      [1, 2, 3, 4]
    )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureOrientation>();

  ASSERT_DOUBLE_EQ(uut.x, 1.0);
  ASSERT_DOUBLE_EQ(uut.y, 2.0);
  ASSERT_DOUBLE_EQ(uut.z, 3.0);
  ASSERT_DOUBLE_EQ(uut.w, 4.0);
}

TEST(DirectConversionTests, FeatureCategory) {
  constexpr auto serialized_input =
      R"(
        99
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureCategory>();

  ASSERT_EQ(uut, 99u);
}

TEST(DirectConversionTests, FeatureData1) {
  constexpr auto serialized_input =
      R"(
        position: [1, 2, 3]
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureData>();

  ASSERT_DOUBLE_EQ(uut.position->x, 1.0);
  ASSERT_DOUBLE_EQ(uut.position->y, 2.0);
  ASSERT_DOUBLE_EQ(uut.position->z, 3.0);
  ASSERT_FALSE(uut.orientation.has_value());
  ASSERT_FALSE(uut.category.has_value());
}

TEST(DirectConversionTests, FeatureData2) {
  constexpr auto serialized_input =
      R"(
        orientation: [1, 2, 3, 4]
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureData>();

  ASSERT_FALSE(uut.position.has_value());
  ASSERT_DOUBLE_EQ(uut.orientation->x, 1.0);
  ASSERT_DOUBLE_EQ(uut.orientation->y, 2.0);
  ASSERT_DOUBLE_EQ(uut.orientation->z, 3.0);
  ASSERT_DOUBLE_EQ(uut.orientation->w, 4.0);
  ASSERT_FALSE(uut.category.has_value());
}

TEST(DirectConversionTests, FeatureData3) {
  constexpr auto serialized_input =
      R"(
        category: 99
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureData>();

  ASSERT_FALSE(uut.position.has_value());
  ASSERT_FALSE(uut.orientation.has_value());
  ASSERT_EQ(uut.category, 99u);
}

TEST(DirectConversionTests, FeatureData4) {
  constexpr auto serialized_input =
      R"(
        position: [1, 2, 3]
        orientation: [1, 2, 3, 4]
        category: 99
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureData>();

  ASSERT_DOUBLE_EQ(uut.position->x, 1.0);
  ASSERT_DOUBLE_EQ(uut.position->y, 2.0);
  ASSERT_DOUBLE_EQ(uut.position->z, 3.0);
  ASSERT_DOUBLE_EQ(uut.orientation->x, 1.0);
  ASSERT_DOUBLE_EQ(uut.orientation->y, 2.0);
  ASSERT_DOUBLE_EQ(uut.orientation->z, 3.0);
  ASSERT_DOUBLE_EQ(uut.orientation->w, 4.0);
  ASSERT_EQ(uut.category, 99u);
}

TEST(DirectConversionTests, FeatureVector) {
  constexpr auto serialized_input =
      R"(
        - category: 1
        - category: 2
        - category: 3
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureVector>();

  ASSERT_EQ(uut.size(), 3u);
  ASSERT_FALSE(uut[0].position.has_value());
  ASSERT_FALSE(uut[0].orientation.has_value());
  ASSERT_EQ(uut[0].category, 1u);
  ASSERT_FALSE(uut[1].position.has_value());
  ASSERT_FALSE(uut[1].orientation.has_value());
  ASSERT_EQ(uut[1].category, 2u);
  ASSERT_FALSE(uut[2].position.has_value());
  ASSERT_FALSE(uut[2].orientation.has_value());
  ASSERT_EQ(uut[2].category, 3u);
}

TEST(DirectConversionTests, FeatureFrameId) {
  constexpr auto serialized_input =
      R"(
        world
      )";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureFrameId>();

  ASSERT_EQ(uut, "world");
}

TEST(DirectConversionTests, FeatureMap) {
  constexpr auto serialized_input =
      R"(frame_id: world
features:
  - category: 1
  - category: 2
  - category: 3
)";

  auto node = YAML::Load(serialized_input);
  const auto uut = node.as<FeatureMap>();

  ASSERT_EQ(uut.frame_id, "world");
  ASSERT_EQ(uut.features.size(), 3u);
}

TEST(ReverseConversionTests, FeaturePosition) {
  FeaturePosition uut;
  uut.x = 1.0;
  uut.y = 2.0;
  uut.z = 3.0;

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 3u);
  ASSERT_DOUBLE_EQ(node[0].as<double>(), 1.0);
  ASSERT_DOUBLE_EQ(node[1].as<double>(), 2.0);
  ASSERT_DOUBLE_EQ(node[2].as<double>(), 3.0);
}

TEST(ReverseConversionTests, FeatureOrientation) {
  FeatureOrientation uut;
  uut.x = 1.0;
  uut.y = 2.0;
  uut.z = 3.0;
  uut.w = 4.0;

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 4u);
  ASSERT_DOUBLE_EQ(node[0].as<double>(), 1.0);
  ASSERT_DOUBLE_EQ(node[1].as<double>(), 2.0);
  ASSERT_DOUBLE_EQ(node[2].as<double>(), 3.0);
  ASSERT_DOUBLE_EQ(node[3].as<double>(), 4.0);
}

TEST(ReverseConversionTests, FeatureCategoryReverse) {
  FeatureCategory uut = 99u;

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.as<std::size_t>(), 99u);
}

TEST(ReverseConversionTests, FeatureData1) {
  FeatureData uut;
  uut.position = FeaturePosition{1.0, 2.0, 3.0};
  uut.orientation = FeatureOrientation{1.0, 2.0, 3.0, 4.0};
  uut.category = 99u;

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 3u);
  ASSERT_EQ(node["category"].as<std::size_t>(), 99u);
  ASSERT_EQ(node["position"].size(), 3u);
  ASSERT_DOUBLE_EQ(node["position"][0].as<double>(), 1.0);
  ASSERT_DOUBLE_EQ(node["position"][1].as<double>(), 2.0);
  ASSERT_DOUBLE_EQ(node["position"][2].as<double>(), 3.0);
  ASSERT_EQ(node["orientation"].size(), 4u);
  ASSERT_DOUBLE_EQ(node["orientation"][0].as<double>(), 1.0);
  ASSERT_DOUBLE_EQ(node["orientation"][1].as<double>(), 2.0);
  ASSERT_DOUBLE_EQ(node["orientation"][2].as<double>(), 3.0);
  ASSERT_DOUBLE_EQ(node["orientation"][3].as<double>(), 4.0);
}

TEST(ReverseConversionTests, FeatureData2) {
  FeatureData uut;
  uut.position = FeaturePosition{1.0, 2.0, 3.0};

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 1u);
  ASSERT_EQ(node["position"].size(), 3u);
  ASSERT_DOUBLE_EQ(node["position"][0].as<double>(), 1.0);
  ASSERT_DOUBLE_EQ(node["position"][1].as<double>(), 2.0);
  ASSERT_DOUBLE_EQ(node["position"][2].as<double>(), 3.0);
  ASSERT_FALSE(node["orientation"].IsDefined());
  ASSERT_FALSE(node["category"].IsDefined());
}

TEST(ReverseConversionTests, FeatureData3) {
  FeatureData uut;
  uut.orientation = FeatureOrientation{1.0, 2.0, 3.0, 4.0};

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 1u);
  ASSERT_EQ(node["orientation"].size(), 4u);
  ASSERT_DOUBLE_EQ(node["orientation"][0].as<double>(), 1.0);
  ASSERT_DOUBLE_EQ(node["orientation"][1].as<double>(), 2.0);
  ASSERT_DOUBLE_EQ(node["orientation"][2].as<double>(), 3.0);
  ASSERT_DOUBLE_EQ(node["orientation"][3].as<double>(), 4.0);
  ASSERT_FALSE(node["position"].IsDefined());
  ASSERT_FALSE(node["category"].IsDefined());
}

TEST(ReverseConversionTests, FeatureData4) {
  FeatureData uut;
  uut.category = 99u;

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 1u);
  ASSERT_EQ(node["category"].as<std::size_t>(), 99u);
  ASSERT_FALSE(node["position"].IsDefined());
  ASSERT_FALSE(node["orientation"].IsDefined());
}

TEST(ReverseConversionTests, FeatureVector) {
  FeatureVector uut;
  uut.push_back(FeatureData{});
  uut.push_back(FeatureData{});
  uut.push_back(FeatureData{});

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 3u);
  ASSERT_EQ(node[0].size(), 0u);
  ASSERT_EQ(node[1].size(), 0u);
  ASSERT_EQ(node[2].size(), 0u);
}

TEST(ReverseConversionTests, FeatureFrameId) {
  FeatureFrameId uut = "world";

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.as<std::string>(), "world");
}

TEST(ReverseConversionTests, FeatureMap) {
  FeatureMap uut;
  uut.frame_id = "world";
  uut.features.push_back(FeatureData{});
  uut.features.push_back(FeatureData{});
  uut.features.push_back(FeatureData{});

  YAML::Node node;
  node = uut;

  ASSERT_EQ(node.size(), 2u);
  ASSERT_EQ(node["frame_id"].as<std::string>(), "world");
  ASSERT_EQ(node["features"].size(), 3u);
}

} // namespace beluga_feature_map_server
