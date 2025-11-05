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

import os
import tempfile
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import tempfile
import yaml

def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name")

    pose = {
        "x": LaunchConfiguration("x_pose", default="0.0"),
        "y": LaunchConfiguration("y_pose", default="0.0"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="rbkairos", description="Name of the robot"
    )

    # Spawn Kairos robot in Gazebo
    spawn_model = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", robot_name,
            "-topic", "/robot_description",
            "-x", pose["x"], "-y", pose["y"], "-z", pose["z"],
            "-R", pose["R"], "-P", pose["P"], "-Y", pose["Y"],
        ],
    )

    # Bridge config
    bridge_config = [
        {
            "ros_topic_name": "/clock",
            "gz_topic_name": "/clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": "/front_laser/scan",
            "gz_topic_name": "/front_laser/scan",
            "ros_type_name": "sensor_msgs/msg/LaserScan",
            "gz_type_name": "gz.msgs.LaserScan",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": "/rear_laser/scan",
            "gz_topic_name": "/rear_laser/scan",
            "ros_type_name": "sensor_msgs/msg/LaserScan",
            "gz_type_name": "gz.msgs.LaserScan",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": "/camera/camera_info",
            "gz_topic_name": "/front_camera_color/color/camera_info",
            "ros_type_name": "sensor_msgs/msg/CameraInfo",
            "gz_type_name": "gz.msgs.CameraInfo",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": "/camera/image_raw",
            "gz_topic_name": "/front_camera_color/color/image_raw",
            "ros_type_name": "sensor_msgs/msg/Image",
            "gz_type_name": "gz.msgs.Image",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": "/imu/data",
            "gz_topic_name": "/imu/data",
            "ros_type_name": "sensor_msgs/msg/Imu",
            "gz_type_name": "gz.msgs.IMU",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": "/gps/data",
            "gz_topic_name": "/gps/data",
            "ros_type_name": "sensor_msgs/msg/NavSatFix",
            "gz_type_name": "gz.msgs.NavSat",
            "direction": "GZ_TO_ROS",
        },
    ]
    with tempfile.NamedTemporaryFile(mode="w", delete=False) as tmp:
        yaml.dump(bridge_config, tmp)
        bridge_yaml = tmp.name

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    # Spawn robot controllers in Gazebo
    mecanum_controller_params = os.path.join(
        get_package_share_directory("beluga_demo_gazebo"), "config", "mecanum_controller_params.yaml",
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--switch-timeout', '10',
            '--controller-manager-timeout', '10',
        ],
        output='screen',
    )

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '--switch-timeout', '10',
            '--controller-manager-timeout', '10',
            '--param-file', mecanum_controller_params,
            '--controller-ros-args', '-r ~/odometry:=/odom -r ~/tf_odometry:=/tf',
        ],
        output='screen',
    )

    # Env var for Gazebo resources
    set_env_root = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(get_package_share_directory("robotnik_gazebo_ignition"))).parent.resolve()))

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(declare_robot_name)
    ld.add_action(set_env_root)
    ld.add_action(spawn_model)
    ld.add_action(bridge)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(mecanum_drive_controller_spawner)

    return ld
