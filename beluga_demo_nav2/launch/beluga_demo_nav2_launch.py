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
This is all-in-one launch script intended for use by nav2 developers.

Specifically this one is adapted and simplified to simulate turtlebot3
using beluga_amcl.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('beluga_demo_nav2')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    # TODO(alon): The path of this environment variable is used only by
    # this docker. In general the real path should be:
    # $GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
    gazebo_model_path_envvar = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        '$GAZEBO_MODEL_PATH:/home/developer/ws/src/external-deps/'
        'turtlebot3_simulations/turtlebot3_gazebo/models',
    )

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters'
        'file to use for all launched nodes',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient',  # noqa: E502
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            os.path.join(bringup_dir, 'worlds', 'world_only.model'),
        ],
        cwd=[launch_dir],
        output='screen',
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir],
        output='screen',
    )

    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True, 'robot_description': robot_description}
        ],  # noqa: E502
    )

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity',
            'turtlebot3_waffle',
            '-file',
            os.path.join(bringup_dir, 'worlds', 'waffle.model'),
            '-x',
            pose['x'],
            '-y',
            pose['y'],
            '-z',
            pose['z'],
            '-R',
            pose['R'],
            '-P',
            pose['P'],
            '-Y',
            pose['Y'],
        ],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')
        )  # noqa: E501
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')
        ),  # noqa: E501
        launch_arguments={'params_file': params_file}.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(gazebo_model_path_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_simulator_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
