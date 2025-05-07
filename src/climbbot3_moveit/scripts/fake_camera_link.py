#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
import numpy as np

class WorldToArucoPublisher(Node):
    def __init__(self):
        super().__init__('world_to_aruco_publisher')
        
        # Create static transform broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)
        
        # Publish the static transform
        self.publish_static_transform()
    
    def publish_static_transform(self):
        # Create TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'aruco_link'
        
        # Set translation
        transform.transform.translation.x = 0.1
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # Set rotation (180 degrees around X-axis)
        q = quaternion_from_euler(np.pi, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        # Broadcast the transform
        self.broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = WorldToArucoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
