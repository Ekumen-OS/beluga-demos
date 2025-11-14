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
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    gamepad_type_conf = LaunchConfiguration('gamepad_type')
    robot_name_conf = LaunchConfiguration('robot_name')

    gamepad_type_arg = DeclareLaunchArgument(
        'gamepad_type',
        default_value='xbox',
        choices=['xbox'],
        description='Gamepad type',
    )

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='tb3',
        description='Robot name',
        choices=['tb3', 'rbkairos'],
    )

    package_dir = get_package_share_directory('beluga_demo_teleop')

    teleop_twist_joy_config_file = [
        package_dir,
        '/gamepads/',
        gamepad_type_conf,
        '.config.yaml',
    ]

    joy_linux_config_file = [
        package_dir,
        '/config/joy_linux.config.yaml',
    ]

    gamepad_driver_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        parameters=[joy_linux_config_file],
        remappings=[
            ('/joy', '/input/joystick'),
        ],
    )

    # Node for rbkairos (stamped + remapped)
    teleop_twist_node_kairos = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_twist_joy_config_file],
        remappings=[
            ('/joy', '/input/joystick'),
            ('/cmd_vel', '/mecanum_drive_controller/reference'),
        ],
        condition=IfCondition(
            PythonExpression(['"', robot_name_conf, '" == "rbkairos"'])
        ),
    )

    # Node for tb3 (no stamped/remaps)
    teleop_twist_node_tb3 = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_twist_joy_config_file],
        remappings={
            ('/joy', '/input/joystick'),
        },
        condition=IfCondition(PythonExpression(['"', robot_name_conf, '" == "tb3"'])),
    )

    return LaunchDescription(
        [
            gamepad_type_arg,
            gamepad_driver_node,
            teleop_twist_node_tb3,
            teleop_twist_node_kairos,
        ]
    )
