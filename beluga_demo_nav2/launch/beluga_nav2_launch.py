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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('beluga_demo_nav2')

    # Create the complete parameter file
    params_file = LaunchConfiguration('params_file')

    param_substitutions = {
        'use_sim_time': 'True',
        'yaml_filename': os.path.join(
            nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'
        ),  # noqa: E501
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'beluga_nav2_params.yaml'
        ),  # noqa: E501
        description='Full path to the ROS2 parameters'
        'file to use for all launched nodes',
    )

    # Robot state publisher
    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
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

    # Container for the nav2 nodes and beluga_amcl
    start_nav2_container_cmd = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    # Localization nodes
    localization_lifecycle_nodes = ['map_server', 'amcl']

    load_composable_localization_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
            ),
            ComposableNode(
                package='beluga_amcl',
                plugin='beluga_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[
                    {
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': localization_lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Navigation nodes
    navigation_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    load_composable_navigation_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[
                    {
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': navigation_lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the navigation and localization nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_nav2_container_cmd)
    ld.add_action(load_composable_localization_nodes)
    ld.add_action(load_composable_navigation_nodes)

    return ld
