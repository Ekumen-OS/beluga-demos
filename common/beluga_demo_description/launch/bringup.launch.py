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
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_xacro_file = PathJoinSubstitution(
        [FindPackageShare("gonbuki_description"), "urdf", "gonbuki.urdf.xacro"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Command(["xacro ", robot_description_xacro_file])}
        ],
    )

    robot_joint_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            robot_joint_publisher_node,
        ]
    )
