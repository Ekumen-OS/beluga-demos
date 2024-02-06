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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    worlds_install_folder = os.path.join(
        get_package_share_directory("beluga_demo_gazebo"),
        "worlds",
    )
    available_worlds = os.listdir(worlds_install_folder)

    world_name_conf = LaunchConfiguration("world_name")

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="hq4_office.world",
        description="Name of the world file to load in simulation",
        choices=available_worlds,
    )

    world_path = PathJoinSubstitution(
        [
            FindPackageShare("beluga_demo_gazebo"),
            "worlds",
            world_name_conf,
        ]
    )

    gazebo_node = IncludeLaunchDescription(
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
    )

    spawn_command = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("turtlebot3_gazebo"),
                    "launch",
                    "spawn_turtlebot3.launch.py",
                )
            ],
        ),
        launch_arguments=[("x_pose", "0"), ("y_pose", "-2")],
    )

    return LaunchDescription(
        [
            world_name_arg,
            gazebo_node,
            spawn_command,
        ]
    )
