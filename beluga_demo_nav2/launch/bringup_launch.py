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

"""
This script is the entry point for executing beluga_amcl integrated with nav2.

Additionally, in this script, you can find the execution of Gazebo and RViz2
for simulation, visualization, and the execution of localization and navigation
commands.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('beluga_demo_nav2')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    headless = LaunchConfiguration('headless')

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'beluga_nav2_params.yaml'
        ),  # noqa: E501
        description='Full path to the ROS2 parameters'
        'file to use for all launched nodes',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient',
    )

    beluga_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'beluga_nav2_launch.py')
        ),  # noqa: E501
        launch_arguments={'params_file': params_file}.items(),
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo_launch.py')
        ),  # noqa: E501
        launch_arguments={'headless': headless}.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')
        )  # noqa: E501
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_simulator_cmd)

    # Add the actions to launch all the programs
    ld.add_action(beluga_nav2_cmd)
    ld.add_action(gazebo_cmd)
    ld.add_action(rviz_cmd)

    return ld
