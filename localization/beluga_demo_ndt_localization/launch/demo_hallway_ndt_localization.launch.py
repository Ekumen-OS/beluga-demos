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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='localization_ndt_map',
                description='Map HDF5 file used by the localization node.',
            ),
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
                    'localization_ndt_map': LaunchConfiguration('localization_ndt_map')
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
                    'user_sim_time': 'true',
                    # 'display_config': LaunchConfiguration('localization_params_file'),
                }.items(),
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
                launch_arguments={'world_name': 'magazino_hallway.world'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("beluga_demo_gazebo"),
                            "launch",
                            "bringup.launch.py",
                        ]
                    ),
                ),
            ),
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
                remappings=[
                    ('/cmd_vel', '/commands/velocity'),
                ],
            ),
        ]
    )
