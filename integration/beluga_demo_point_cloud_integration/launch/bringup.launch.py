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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory("beluga_demo_point_cloud_integration")

    maps_install_folder = os.path.join(pkg_dir, "maps")
    available_maps = os.listdir(maps_install_folder)

    param_files_install_folder = os.path.join(pkg_dir, "params")
    available_param_files = os.listdir(param_files_install_folder)

    # Create the complete parameter file
    params_file = LaunchConfiguration('params_file')

    configured_params = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare('beluga_demo_point_cloud_integration'), "params", params_file]
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='beluga_point_cloud_params.yaml',
        description='Parameter filename to use to configure all nodes',
        choices=available_param_files,
    )

    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='hq4_office',
        description='Map basename to serve and localize',
        choices=available_maps,
    )

    # Container for the nav2 nodes and beluga_amcl
    # IT IS CALLED NAV2_CONTAINER...SHOULD I CHANGE TO BELUGA_AMCL CONTAINER?
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
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the navigation and localization nodes
    ld.add_action(start_nav2_container_cmd)
    ld.add_action(load_composable_localization_nodes)
    ld.add_action(load_composable_navigation_nodes)

    return ld
