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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


# TODO study if it's necessary to include Rviz with a custom configuration
def generate_launch_description():
    pkg_mh_amcl = get_package_share_directory('beluga_demo_mh_amcl')
    pkg_beluga_demo_bringup = get_package_share_directory("beluga_demo_bringup")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            pkg_beluga_demo_bringup,
                            "launch",
                            "simulation.launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "world_name": ["hq4_office.world"],
                }.items(),
            ),
            Node(
                package='beluga_demo_mh_amcl',
                executable='beluga_mh_amcl_node',
                name='mh_amcl',
                parameters=[
                    PathJoinSubstitution([pkg_mh_amcl, 'config', 'mh_amcl_params.yaml'])
                ],
            ),
        ]
    )
