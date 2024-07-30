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
project(beluga_demo_spotlight_detector)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(beluga_april_tag_adapter_msgs REQUIRED)

add_library(beluga_demo_spotlight_detector_component SHARED)
target_include_directories(
  beluga_demo_spotlight_detector_component
  PUBLIC include)
target_sources(
  beluga_demo_spotlight_detector_component
  PRIVATE src/beluga_demo_spotlight_detector_node.cpp)
target_compile_features(
  beluga_demo_spotlight_detector_component PUBLIC cxx_std_17)
target_link_libraries(
  beluga_demo_spotlight_detector_component PUBLIC)
ament_target_dependencies(
  beluga_demo_spotlight_detector_component
  PUBLIC rclcpp
         rclcpp_components
         cv_bridge
         sensor_msgs
         visualization_msgs
         image_transport
         image_geometry
         beluga_april_tag_adapter_msgs
         OpenCV)

rclcpp_components_register_node(
  beluga_demo_spotlight_detector_component
  PLUGIN "beluga_demo_spotlight_detector::BelugaDemoSpotlightDetectorNode"
  EXECUTABLE beluga_demo_spotlight_detector)

install(
  TARGETS beluga_demo_spotlight_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS beluga_demo_spotlight_detector DESTINATION lib/${PROJECT_NAME})

# install(
#   DIRECTORY bags
#   DESTINATION share/${PROJECT_NAME})

ament_package()
