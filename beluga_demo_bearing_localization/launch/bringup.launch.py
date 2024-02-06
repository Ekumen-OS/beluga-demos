#!/usr/bin/env python3

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


from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    beluga_feature_map_config_path = PathJoinSubstitution(
        [
            FindPackageShare("beluga_demo_bearing_localization"),
            "config",
            "landmarks_map.yaml",
        ]
    )

    spotlight_detection_cnode = ComposableNode(
        name="spotlight_detector",
        package="beluga_demo_spotlight_detector",
        plugin="beluga_demo_spotlight_detector::BelugaDemoSpotlightDetectorNode",
        parameters=[],
        remappings=[
            ("image_rect", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("landmark_detections", "/landmarks/landmark_detections"),
            ("landmark_detection_markers", "/landmarks/landmark_detection_markers"),
            ("spotlight_detections", "/landmarks/spotlight_detections"),
        ],
    )

    feature_map_server_cnode = ComposableNode(
        package="beluga_feature_map_server",
        plugin="beluga_feature_map_server::BelugaFeatureMapServerNode",
        parameters=[
            {
                "map_file": beluga_feature_map_config_path,
            }
        ],
        remappings=[
            ("landmarks_map", "/landmarks/landmarks_map"),
            ("landmark_markers", "/landmarks/feature_map_markers"),
        ],
    )

    container_pipeline = ComposableNodeContainer(
        package="rclcpp_components",
        name="feature_detection_container",
        namespace="apriltag",
        executable="component_container",
        composable_node_descriptions=[
            feature_map_server_cnode,
            spotlight_detection_cnode,
        ],
        output="screen",
    )

    bmcl_node = Node(
        package="beluga_demo_landmark_localization",
        executable="beluga_bmcl_demo_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        respawn=True,
        parameters=[],
        remappings=[
            ("landmarks_map", "/landmarks/landmarks_map"),
            ("landmark_detections", "/landmarks/landmark_detections"),
        ],
    )

    return LaunchDescription(
        [
            container_pipeline,
            bmcl_node,
        ]
    )
