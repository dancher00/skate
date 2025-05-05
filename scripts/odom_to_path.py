#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class OdomToPath(Node):
    """
    Node that subscribes to odometry and publishes a path representing the robot's trajectory
    """
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Parameters
        self.declare_parameter('path_length', 1000)  # Maximum number of poses to keep in the path
        self.max_path_length = self.get_parameter('path_length').value
        
        # Initialize empty path
        self.path = Path()
        self.path.header.frame_id = 'world'  # Set the path to be in the world frame
        
        # Create subscribers and publishers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # From gazebo_ros_p3d plugin
            self.odom_callback,
            10)
        
        self.path_pub = self.create_publisher(
            Path,
            '/odom_path',  # The actual trajectory
            10)
        
        # Timer for publishing the path at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_path)  # 10 Hz
        
        self.get_logger().info('Odometry to Path converter initialized')
    
    def odom_callback(self, msg):
        """Process odometry data to update the path"""
        # Create a PoseStamped message from the odometry
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = 'world'  # Ensure it's in the world frame
        pose_stamped.pose = msg.pose.pose
        
        # Add to the path
        self.path.poses.append(pose_stamped)
        
        # Trim the path if it gets too long
        if len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        
        # Update the path header
        self.path.header.stamp = self.get_clock().now().to_msg()
    
    def publish_path(self):
        """Publish the current path"""
        if self.path.poses:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    odom_to_path = OdomToPath()
    
    try:
        rclpy.spin(odom_to_path)
    except KeyboardInterrupt:
        odom_to_path.get_logger().info('Shutting down Odometry to Path Converter')
    finally:
        odom_to_path.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()