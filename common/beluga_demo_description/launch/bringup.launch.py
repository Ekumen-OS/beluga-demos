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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    beluga_demo_pkg = get_package_share_directory("beluga_demo_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_name",
                default_value="tb3",
                description="Robot to spawn (tb3 or kairos)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            beluga_demo_pkg,
                            "launch",
                            "include",
                            [
                                LaunchConfiguration("robot_name"),
                                TextSubstitution(text=".launch.py"),
                            ],
                        ]
                    )
                )
            ),
        ]
    )
