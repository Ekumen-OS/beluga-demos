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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    FindExecutable,
    EnvironmentVariable,
)
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Path to the Robotnik Kairos xacro file
    kairos_xacro_file = PathJoinSubstitution(
        [
            get_package_share_directory("robotnik_description"),
            "robots",
            "rbkairos",
            "rbkairos.urdf.xacro",
        ]
    )

    # Path to the Kairos modified (mecanum) controller params
    controller_params_file = PathJoinSubstitution(
        [
            EnvironmentVariable(
                name="BELUGA_DEMO_DESCRIPTION_PATH",
                default_value=get_package_share_directory("beluga_demo_description"),
            ),
            "config",
            "kairos_controller_params.yaml",
        ]
    )

    # Generate robot description:
    # - Expanding Robotnik xacro setting "gazebo_ignition" and "namespace" parameters
    # - Modifying the controller parameters path from the Robotnik's original to the mecanum
    robot_description_content = Command(
        [
            FindExecutable(name="bash"),
            " -c \"",
            "xacro ",
            kairos_xacro_file,
            " gazebo_ignition:=True",
            " namespace:=\\\"''\\\"",
            " | sed 's|<parameters>.*</parameters>|<parameters>",
            controller_params_file,
            "</parameters>|'",
            "\"",
        ]
    )

    # Wrap xacro robot description in ParameterValue so launch knows it’s a string parameter
    robot_description_str = ParameterValue(robot_description_content, value_type=str)

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description_str,
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
