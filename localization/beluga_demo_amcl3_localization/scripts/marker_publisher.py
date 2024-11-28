#!/usr/bin/env python3

'''
 Copyright 2024 Ekumen, Inc.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_marker_node')
        self.gt_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/gt_poses',
            self.gt_pose_callback,
            10
        )

        self.amcl3_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl3_poses',
            self.amcl3_pose_callback,
            10
        )

        self.gt_marker_publisher = self.create_publisher(MarkerArray, '/gt_pose_markers', 10)
        self.amcl3_marker_publisher = self.create_publisher(MarkerArray, '/amcl3_pose_markers', 10)
        self.gt_points = []
        self.amcl3_points = []


    def amcl3_pose_callback(self, msg: PoseWithCovarianceStamped):
        position = msg.pose.pose.position
        # Create a point for visualization
        point = Point()
        point.x = position.x
        point.y = position.y
        point.z = position.z
        self.amcl3_points.append(point)

        # Create MarkerArray to hold the Marker
        marker_array = MarkerArray()

        # Create the LineStrip marker
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = 'map' 
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.id = 0
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.1  
        line_strip_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        # Add points to the LineStrip marker
        line_strip_marker.points = self.amcl3_points
        
        # Add the marker to the MarkerArray
        marker_array.markers.append(line_strip_marker)
        
        # Publish the MarkerArray
        self.amcl3_marker_publisher.publish(marker_array)

    
    def gt_pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        # Create a point for visualization
        point = Point()
        point.x = position.x
        point.y = position.y
        point.z = position.z
        self.gt_points.append(point)

        # Create MarkerArray to hold the Marker
        marker_array = MarkerArray()

        # Create the LineStrip marker
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = 'map' 
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.id = 0
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.1  
        line_strip_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # Add points to the LineStrip marker
        line_strip_marker.points = self.gt_points
        
        # Add the marker to the MarkerArray
        marker_array.markers.append(line_strip_marker)
        
        # Publish the MarkerArray
        self.gt_marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    pose_visualizer = PoseVisualizer()
    rclpy.spin(pose_visualizer)
    pose_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    