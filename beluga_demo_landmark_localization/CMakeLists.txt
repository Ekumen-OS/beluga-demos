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

cmake_minimum_required(VERSION 3.16)
project(beluga_demo_landmark_localization)

find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(beluga_april_tag_adapter_msgs REQUIRED)
find_package(beluga_feature_map_server_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(beluga_lmcl_demo_component SHARED)
target_sources(beluga_lmcl_demo_component PRIVATE src/beluga_lmcl_demo.cpp)
target_compile_features(beluga_lmcl_demo_component PUBLIC cxx_std_17)
target_link_libraries(beluga_lmcl_demo_component PUBLIC beluga::beluga)
ament_target_dependencies(
  beluga_lmcl_demo_component
  PUBLIC beluga_april_tag_adapter_msgs
         beluga_feature_map_server_msgs
         message_filters
         nav2_msgs
         rclcpp
         rclcpp_components
         tf2
         tf2_eigen
         tf2_geometry_msgs
         tf2_ros)
rclcpp_components_register_node(
  beluga_lmcl_demo_component
  PLUGIN "beluga_demo_landmark_localization::LMCLNode"
  EXECUTABLE beluga_lmcl_demo_node)

add_library(beluga_bmcl_demo_component SHARED)
target_sources(beluga_bmcl_demo_component PRIVATE src/beluga_bmcl_demo.cpp)
target_compile_features(beluga_bmcl_demo_component PUBLIC cxx_std_17)
target_link_libraries(beluga_bmcl_demo_component PUBLIC beluga::beluga)
ament_target_dependencies(
  beluga_bmcl_demo_component
  PUBLIC beluga_april_tag_adapter_msgs
         beluga_feature_map_server_msgs
         message_filters
         nav2_msgs
         rclcpp
         rclcpp_components
         tf2
         tf2_eigen
         tf2_geometry_msgs
         tf2_ros)
rclcpp_components_register_node(
  beluga_bmcl_demo_component
  PLUGIN
  "beluga_demo_landmark_localization::BMCLNode"
  EXECUTABLE beluga_bmcl_demo_node)

install(
  TARGETS beluga_lmcl_demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS beluga_lmcl_demo_node DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS beluga_bmcl_demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS beluga_bmcl_demo_component DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()
