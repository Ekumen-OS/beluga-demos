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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    world_paths = [
        os.path.join(get_package_share_directory("gz_ekumen_worlds"), "worlds"),
        os.path.join(
            get_package_share_directory("beluga_demo_gazebo"),
            "worlds",
        ),
    ]

    available_worlds = []
    for path in world_paths:
        available_worlds.extend(os.listdir(path))

    world_name_conf = LaunchConfiguration("world_name")

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty_ekumen_hq4.world",
        description="Name of the world file to load in simulation",
        choices=available_worlds,
    )

    robot_name_conf = LaunchConfiguration("robot_name")

    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="tb3",
        description="Robot to spawn"
    )

    append_tb3_gz_sim_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join(
            [
                os.path.dirname(get_package_share_directory("nav2_minimal_tb3_sim")),
                os.path.join(
                    get_package_share_directory("nav2_minimal_tb3_sim"), "models"
                ),
            ]
        ),
    )

    append_gz_worlds = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.pathsep.join(world_paths)
    )

    gz_sim_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ],
        ),
        launch_arguments=[("gz_args", ["-r ", world_name_conf])],
    )
    # Spawn TurtleBot3 if robot_name == "tb3"
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_minimal_tb3_sim"),
                    "launch",
                    "spawn_tb3.launch.py",
                )
            ],
        ),
        launch_arguments=[("x_pose", "0"), ("y_pose", "-2"), ("z_pose", "0.01")],
        condition=IfCondition(PythonExpression(["'", robot_name_conf, "' == 'tb3'"]))
    )
 
    spawn_kairos = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("beluga_demo_gazebo"),
                "launch",
                "spawn_kairos.launch.py",
            )
        ),
        launch_arguments={
            "robot_name": "rbkairos",
            "x_pose": "0",
            "y_pose": "-1.8",
            "z_pose": "0.01",
        }.items(),
        condition=IfCondition(PythonExpression(["'", robot_name_conf, "' == 'rbkairos'"]))
    )

    return LaunchDescription(
        [
            world_name_arg,
            robot_name_arg,
            append_gz_worlds,
            append_tb3_gz_sim_resources,
            gz_sim_nodes,
            spawn_tb3,
            spawn_kairos,
        ]
    )
