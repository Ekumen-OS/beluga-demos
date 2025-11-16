# Copyright 2025 Ekumen, Inc.
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
project(beluga_demo_mh_amcl)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(beluga_ros REQUIRED)
find_package(bondcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add the include directory
include_directories(include)

# Executable
add_executable(beluga_mh_amcl_node
  src/beluga_mh_amcl_node.cpp
  src/mh_amcl/mh_amcl.cpp
  src/mh_amcl/map_matcher.cpp
  src/mh_amcl/particles_distribution.cpp
  src/mh_amcl/utils.cpp
)
ament_target_dependencies(beluga_mh_amcl_node
  beluga
  beluga_ros
  bondcpp
  Eigen3
  geometry_msgs
  lifecycle_msgs
  message_filters
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
)
install(TARGETS beluga_mh_amcl_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
 config
 launch
 maps
 rviz
 DESTINATION share/${PROJECT_NAME})

ament_package()
