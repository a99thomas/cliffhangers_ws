#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster
import tf2_ros
import numpy as np
from tf_transformations import quaternion_from_euler

class ArucoMarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker_publisher')
        
        # Create static transform broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)
        
        # Create marker publisher
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'blue_object_centroids',
            10
        )
        
        # Set up timer for periodic publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Publish static transform
        self.publish_static_transform()
        
        # Initialize markers
        self.markers = self.create_markers()

    def publish_static_transform(self):
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'  # Changed from 'map' to 'world'
        transform.child_frame_id = 'aruco_link'
        
        # Set position
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.5
        transform.transform.translation.z = 0.5
        
        # Set orientation (upside down - 180 degrees around X axis)
        q = quaternion_from_euler(np.pi, 0, 0)  # Roll=180 degrees, Pitch=0, Yaw=0
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        # Publish transform
        self.broadcaster.sendTransform(transform)

    def create_markers(self):
        markers = MarkerArray()
        
        # Create 4 points in a line perpendicular to aruco_link
        base_position = [1.5, 0.0, 0.0]  # Starting point relative to aruco_link
        spacing = 0.1  # Distance between points
        
        for i in range(4):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.ns = "blue_centroids"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position (moving along Y axis)
            if i == 1 or i == 3:

                marker.pose.position.x = base_position[0] + (i * spacing)
                marker.pose.position.y = base_position[1] + 0.15
                marker.pose.position.z = base_position[2]
            else:
                marker.pose.position.x = base_position[0] + (i * spacing)
                marker.pose.position.y = base_position[1] - 0.15
                marker.pose.position.z = base_position[2]
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Set color (blue)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            markers.markers.append(marker)
        
        return markers

    def timer_callback(self):
        # Update timestamp
        current_time = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = current_time
        
        # Publish markers
        self.marker_publisher.publish(self.markers)

def main():
    rclpy.init()
    node = ArucoMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()