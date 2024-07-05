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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    gazebo_model_path_envvar = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        '$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models',
    )

    # Declare the launch arguments
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient',  # noqa: E502
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            os.path.join(nav2_bringup_dir, 'worlds', 'world_only.model'),
        ],
        output='screen',
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', headless])),
        cmd=['gzclient'],
        output='screen',
    )

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity',
            'turtlebot3_waffle',
            '-file',
            os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model'),
            '-x',
            pose['x'],
            '-y',
            pose['y'],
            '-z',
            pose['z'],
            '-R',
            pose['R'],
            '-P',
            pose['P'],
            '-Y',
            pose['Y'],
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(gazebo_model_path_envvar)

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)

    # Add actions to launch gazebo
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    return ld
