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

project(beluga_feature_map_server)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wconversion
    -Wextra
    -Werror
    -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(beluga_feature_map_server_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_subdirectory(map_loaders)
add_subdirectory(node_component)

rclcpp_components_register_node(
  beluga_feature_map_server_component
  PLUGIN "beluga_feature_map_server::BelugaFeatureMapServerNode"
  EXECUTABLE beluga_feature_map_server)

install(
  TARGETS beluga_feature_map_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS beluga_feature_map_server DESTINATION lib/${PROJECT_NAME})

ament_package()
