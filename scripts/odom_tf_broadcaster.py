#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.get_logger().info('Odometry to TF broadcaster initialized')
    
    def odom_callback(self, msg):
        # Create a new transform
        transform = TransformStamped()
        
        # Set header information
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'world'  # Parent frame
        transform.child_frame_id = 'base_footprint'  # Child frame
        
        # Set translation from odometry
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation from odometry
        transform.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()