# Copyright 2025 Ekumen, Inc.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="rbkairos", description="Robot model to spawn"
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Robot namespace"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0.01")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    has_arm_arg = DeclareLaunchArgument("has_arm", default_value="False")

    # Path to the Kairos URDF/Xacro
    description_pkg = get_package_share_directory("robotnik_description")
    urdf_path = os.path.join(description_pkg, "robots", "rbkairos", "rbkairos.urdf.xacro")

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        parameters=[{"use_sim_time": True}],
        arguments=[urdf_path],
        output="screen",
    )

    # Spawn in Gazebo (Ignition/Harmonic)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", LaunchConfiguration("robot"),
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_arg,
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        has_arm_arg,
        robot_state_publisher,
        spawn_entity,
    ])
