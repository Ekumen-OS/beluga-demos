# Copyright 2024 Ekumen, Inc.
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

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################

cmake_minimum_required(VERSION 3.5)
project(beluga_demo_amcl3_localization)

set(OPENVDB_CMAKE_MODULE_PATH
  ${OPENVDB_CMAKE_MODULE_PATH}
  CACHE PATH "Path to OpenVDB CMake module")

if(NOT OPENVDB_CMAKE_MODULE_PATH)
  file(GLOB_RECURSE OPENVDB_MODULES /usr/lib/*/FindOpenVDB.cmake
                                    /usr/local/lib/*/FindOpenVDB.cmake)
  list(LENGTH OPENVDB_MODULES NUM_OPENVDB_MODULES)
  if (NUM_OPENVDB_MODULES EQUAL 1)
    list(GET OPENVDB_MODULES 0 OPENVDB_MODULE)
    get_filename_component(OPENVDB_CMAKE_MODULE_PATH ${OPENVDB_MODULE}
                           DIRECTORY)
  endif()
  unset(NUM_OPENVDB_MODULES)
  unset(OPENVDB_MODULES)
endif()
if(OPENVDB_CMAKE_MODULE_PATH)
  list(APPEND CMAKE_MODULE_PATH ${OPENVDB_CMAKE_MODULE_PATH})
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(range-v3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(beluga REQUIRED)
find_package(beluga_ros REQUIRED)
find_package(beluga_vdb REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenVDB REQUIRED)


################################################################################
# Build
################################################################################

add_library(beluga_amcl3_demo_component SHARED)
target_sources(beluga_amcl3_demo_component PRIVATE src/beluga_amcl3_demo.cpp)
target_compile_features(beluga_amcl3_demo_component PUBLIC cxx_std_17)
target_link_libraries(beluga_amcl3_demo_component
  PUBLIC beluga::beluga beluga_ros::beluga_ros beluga_vdb::beluga_vdb)
ament_target_dependencies(
  beluga_amcl3_demo_component
  PUBLIC ament_index_cpp
         geometry_msgs
         visualization_msgs
         std_msgs
         sensor_msgs
         message_filters
         rclcpp
         rclcpp_components
         tf2
         tf2_eigen
         tf2_geometry_msgs
         tf2_ros
         beluga
         beluga_ros
         beluga_vdb)
rclcpp_components_register_node(
  beluga_amcl3_demo_component
  PLUGIN
  "beluga_demo_amcl3_localization::Amcl3Node"
  EXECUTABLE beluga_amcl3_demo_node)


################################################################################
# Install
################################################################################

install(
  TARGETS beluga_amcl3_demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS beluga_amcl3_demo_node DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tf_publisher.py
  scripts/marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
