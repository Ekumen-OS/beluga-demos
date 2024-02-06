#!/usr/bin/python3

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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter


import os


def generate_launch_description():
    return LaunchDescription(
        [
            SetParameter(
                name='use_sim_time',
                value=True,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("beluga_demo_bringup"),
                            "launch",
                        ),
                        "/bringup.launch.py",
                    ]
                ),
                launch_arguments={
                    "world_name": "hq4_office_populated.world",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(
                                "beluga_demo_lidar_localization"
                            ),
                            "launch",
                        ),
                        "/bringup.launch.py",
                    ]
                ),
                launch_arguments={
                    "map_name": "hq4_office",
                    "amcl_params_file": "likelihood_field_params.yaml",
                }.items(),
            ),
        ]
    )
