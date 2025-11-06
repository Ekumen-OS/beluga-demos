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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="tb3", description="Robot to spawn (tb3 or kairos)"
    )

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare('beluga_demo_rviz2'),
            'rviz',
            [
                TextSubstitution(text="model_"),
                LaunchConfiguration("robot_name"),
                TextSubstitution(text=".rviz"),
            ],
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                respawn=True,
                arguments=['-d', rviz_config_path],
            ),
        ]
    )
