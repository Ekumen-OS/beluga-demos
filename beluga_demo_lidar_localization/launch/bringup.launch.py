#!/usr/bin/python3

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


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.actions import DeclareLaunchArgument


import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("beluga_demo_lidar_localization")

    maps_install_folder = os.path.join(pkg_dir, "maps")
    available_maps = os.listdir(maps_install_folder)

    param_files_install_folder = os.path.join(pkg_dir, "config")
    available_param_files = os.listdir(param_files_install_folder)

    map_name_conf = LaunchConfiguration("map_name")
    localization_map = PathJoinSubstitution(
        [maps_install_folder, map_name_conf, "map.yaml"]
    )

    amcl_params_file_conf = LaunchConfiguration("amcl_params_file")
    localization_params_file = PathJoinSubstitution(
        [pkg_dir, "config", amcl_params_file_conf]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="localization_package",
                default_value="beluga_amcl",
                description="Localization packages to use in autonomy mode",
                choices=["beluga_amcl", "nav2_amcl"],
            ),
            DeclareLaunchArgument(
                name="map_name",
                description="Name of the map to use for localization",
                choices=available_maps,
                default_value="hq4_office",
            ),
            DeclareLaunchArgument(
                name="amcl_params_file",
                description="Node parameters file for AMCL node",
                choices=available_param_files,
                default_value="likelihood_field_params.yaml",
            ),
            Node(
                package="beluga_amcl",
                executable="amcl_node",
                name="amcl",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                respawn=True,
                parameters=[localization_params_file],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                respawn=True,
                parameters=[
                    {
                        "yaml_filename": localization_map,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                sigterm_timeout="20",
                sigkill_timeout="20",
                parameters=[
                    {"autostart": True},
                    {"node_names": ["map_server", "amcl"]},
                ],
            ),
        ]
    )
