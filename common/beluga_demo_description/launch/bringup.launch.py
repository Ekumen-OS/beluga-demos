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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

import os

def generate_launch_description():
    # Launch argument
    robot_name = LaunchConfiguration("robot_name")

    # File paths
    turtlebot_urdf = os.path.join(
        get_package_share_directory("nav2_minimal_tb3_sim"),
        "urdf",
        "turtlebot3_waffle.urdf"
    )
    
    rbkairos_xacro = os.path.join(
    get_package_share_directory("robotnik_description"),
    "robots",
    "rbkairos",
    "rbkairos.urdf.xacro"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="robot_name",
            default_value="tb3",
            description="Robot to spawn"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ",
                    PythonExpression([
                        '"',
                        turtlebot_urdf, '" if "', robot_name, '" != "rbkairos" else "',
                        rbkairos_xacro, '"'
                    ])
                ])
            }],
        ),
    ])
