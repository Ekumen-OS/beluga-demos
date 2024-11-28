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
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ################################################################################
    # CONFIGURATION FILES
    ################################################################################
    # Packages names
    amcl3_package_name = "beluga_demo_amcl3_localization"

    # Rviz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(amcl3_package_name),
        "rviz",
        "rviz.rviz"
    ])


    ################################################################################
    # NODES DEFINITION
    ################################################################################
    beluga_amcl3 = Node(
        package="beluga_demo_amcl3_localization",
        executable="beluga_amcl3_demo_node",
        name="beluga_amcl3_demo",
        parameters = [{'use_sim_time': True}],
        output="screen",
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_in_xsense',
        arguments = ['0.0584867781527745', '0.00840419966766332', '0.168915521980526', '0.0078031', '0.0015042', '-0.0252884', 'base_link', 'velodyne'],
        parameters = [{'use_sim_time': True}],
    )

    tf = Node(
        package='beluga_demo_amcl3_localization',
        executable='tf_publisher.py',
        name='odom_to_base_link',
        parameters = [{'use_sim_time': True}],
    )

    markers = Node(
        package='beluga_demo_amcl3_localization',
        executable='marker_publisher.py',
        name='pose_marker_node',
        parameters = [{'use_sim_time': True}],
    )

    voxel_filter = Node(
        package="pcl_ros",
        executable="filter_voxel_grid_node",
        name="filter_voxel_grid_node",
        parameters = [{'use_sim_time': True, 
                        'leaf_size': 0.1}],
        remappings=[
            ('/input', '/velodyne_points'),
            #('/output', '/pointcloud'),
            ('/output', '/output_voxel_filter'),
        ],
            output="screen",
    )

    crop_box_filter = Node(
        package="pcl_ros",
        executable="filter_crop_box_node",
        name="filter_crop_box_node",
        parameters = [{'use_sim_time': True, 
                        'min_x': -20.0,
                        'max_x': 20.0,
                        'min_y': -20.0,
                        'max_y': 20.0,
                        'min_z': -20.0,
                        'max_z': 20.0}],
        remappings=[
            ('/input', '/output_voxel_filter'),
            ('/output', '/pointcloud'),
        ],
            output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=["-d", rviz_config_file],
    )


    ################################################################################
    # LAUNCH NODES
    ################################################################################
    nodes = [
        rviz,
        static_tf,
        tf,
        markers,
        voxel_filter,
        crop_box_filter,
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rviz,
                                                          on_start=[beluga_amcl3])
        )
    ]

    return LaunchDescription(nodes)

    