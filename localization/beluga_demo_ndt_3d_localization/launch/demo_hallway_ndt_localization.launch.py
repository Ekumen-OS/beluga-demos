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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("beluga_demo_ndt_3d_localization")

    param_files_install_folder = os.path.join(pkg_dir, "config")
    available_param_files = os.listdir(param_files_install_folder)

    worlds_install_folder = os.path.join(
        get_package_share_directory("beluga_demo_gazebo"),
        "worlds",
    )
    available_worlds = os.listdir(worlds_install_folder)
    world_name_conf = LaunchConfiguration("world_name")
    world_path = PathJoinSubstitution(
        [
            FindPackageShare("beluga_demo_gazebo"),
            "worlds",
            world_name_conf,
        ]
    )

    amcl_ndt_params_file_conf = LaunchConfiguration("amcl_ndt_params_file")
    localization_params_file = PathJoinSubstitution(
        [pkg_dir, "config", amcl_ndt_params_file_conf]
    )

    rviz_file = PathJoinSubstitution([pkg_dir, "rviz", "ndt_amcl_3d.ros2.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="amcl_ndt_params_file",
                description="Node parameters file for AMCL NDT node.",
                choices=available_param_files,
                default_value="ndt_3d_params.yaml",
            ),
            DeclareLaunchArgument(
                name="localization_ndt_map",
                description="Map HDF5 file used by the localization node.",
            ),
            DeclareLaunchArgument(
                name="world_name",
                default_value="magazino_hallway.world",
                description="Name of the world file to load in simulation",
                choices=available_worlds,
            ),
            ###
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("beluga_example"),
                            "launch",
                            "utils",
                            "ndt_3d_localization_launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "localization_params_file": localization_params_file,
                    "localization_ndt_map": LaunchConfiguration("localization_ndt_map"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("beluga_example"),
                            "launch",
                            "utils",
                            "rviz_launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "user_sim_time": "true",
                    "display_config": rviz_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("gazebo_ros"),
                            "launch",
                            "gazebo.launch.py",
                        )
                    ],
                ),
                launch_arguments=[("world", world_path)],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("gonbuki_description"),
                            "launch",
                            "robot_description.launch.py",
                        ]
                    ),
                ),
                launch_arguments={"world_name": "magazino_hallway.world"}.items(),
            ),
            ###
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "gonbuki_robot",
                    "-x",
                    "0.0",
                    "-y",
                    "2.0",
                ],
                output="screen",
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                output="screen",
                prefix="xterm -e",
                remappings=[
                    ("/cmd_vel", "/commands/velocity"),
                ],
            ),
        ]
    )
