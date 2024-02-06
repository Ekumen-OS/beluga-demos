# Copyright 2023 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_library(beluga_feature_map_server_component SHARED)
target_include_directories(beluga_feature_map_server_component PUBLIC include)
target_sources(beluga_feature_map_server_component
               PRIVATE src/beluga_feature_map_server_node.cpp)
target_compile_features(beluga_feature_map_server_component PUBLIC cxx_std_17)
target_link_libraries(beluga_feature_map_server_component PUBLIC map_loaders)
ament_target_dependencies(
  beluga_feature_map_server_component
  PUBLIC rclcpp
         rclcpp_components
         beluga_feature_map_server_msgs
         visualization_msgs
         std_srvs)
