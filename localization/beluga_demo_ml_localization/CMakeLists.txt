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
project(beluga_demo_ml_localization)

find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(beluga_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(beluga_april_tag_adapter_msgs REQUIRED)
find_package(beluga_feature_map_server_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(beluga_dge_demo_component SHARED)
target_sources(beluga_dge_demo_component PRIVATE src/beluga_dge_demo.cpp)
target_compile_features(beluga_dge_demo_component PUBLIC cxx_std_17)
target_link_libraries(beluga_dge_demo_component
  PUBLIC beluga::beluga beluga_ros::beluga_ros)
ament_target_dependencies(
  beluga_dge_demo_component
  PUBLIC beluga_feature_map_server_msgs
         cv_bridge
         geometry_msgs
         visualization_msgs
         message_filters
         rclcpp
         rclcpp_components
         tf2
         tf2_eigen
         tf2_geometry_msgs
         tf2_ros)
target_link_libraries(beluga_dge_demo_component PUBLIC ${TORCH_LIBRARIES} ${OpenCV_LIBS})
rclcpp_components_register_node(
  beluga_dge_demo_component
  PLUGIN "beluga_demo_ml_localization::DGENode"
  EXECUTABLE beluga_dge_demo_node)

install(
  TARGETS beluga_dge_demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS beluga_dge_demo_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
