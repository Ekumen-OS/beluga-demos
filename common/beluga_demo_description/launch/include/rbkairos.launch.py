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
# limitations under the License

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.find_executable import FindExecutable

def generate_launch_description():
    # Path to Kairos xacro file
    kairos_xacro_file = PathJoinSubstitution([
        get_package_share_directory("robotnik_description"),
        "robots",
        "rbkairos",
        "rbkairos.urdf.xacro"
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    FindExecutable(name="xacro"),
                    " ",
                    kairos_xacro_file,
                    " gazebo_ignition:=","True",
                    " namespace:=\"''\"",
                ]),
                "publish_frequency": 100.0,
                "use_sim_time": True,
            }],
        ),
    ])