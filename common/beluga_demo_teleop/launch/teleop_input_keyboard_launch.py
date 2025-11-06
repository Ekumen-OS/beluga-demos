#!/usr/bin/python3

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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_conf = LaunchConfiguration('robot_name')

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='tb3',
        description='Robot name',
        choices=['tb3', 'rbkairos'],
    )

    # Node for rbkairos (stamped + remapped)
    teleop_node_rbkairos = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard_rbkairos",
        parameters=[{"stamped": True}],
        remappings=[('/cmd_vel', '/mecanum_drive_controller/reference')],
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(
            PythonExpression(['"', robot_name_conf, '" == "rbkairos"'])
        ),
    )

    # Node for tb3 (no stamped/remaps)
    teleop_node_tb3 = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard_tb3",
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(PythonExpression(['"', robot_name_conf, '" == "tb3"'])),
    )

    return LaunchDescription(
        [
            robot_name_arg,
            GroupAction(
                [
                    teleop_node_rbkairos,
                    teleop_node_tb3,
                ]
            ),
        ]
    )
