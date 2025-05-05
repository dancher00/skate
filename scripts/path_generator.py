#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        
        # Parameters
        self.declare_parameter('path_type', 'figure_eight')  # Options: figure_eight, circle, straight
        self.declare_parameter('path_scale', 3.0)  # Scale of the path
        self.declare_parameter('num_points', 100)  # Number of points in the path
        self.declare_parameter('publish_frequency', 1.0)  # How often to publish the path (Hz)
        
        # Get parameters
        self.path_type = self.get_parameter('path_type').value
        self.path_scale = self.get_parameter('path_scale').value
        self.num_points = self.get_parameter('num_points').value
        
        # Create publisher
        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10)
        
        # Create timer for publishing path
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_frequency').value,
            self.publish_path)
        
        self.get_logger().info(f'Path Generator initialized with {self.path_type} path')
    
    def generate_path(self):
        """Generate a path based on the specified type"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'world'  # Assumes world frame is used
        
        if self.path_type == 'figure_eight':
            # Generate a figure-eight path
            t = np.linspace(0, 2 * math.pi, self.num_points)
            scale_x = self.path_scale
            scale_y = self.path_scale / 2
            
            for i in range(len(t)):
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                
                # Figure eight equation: x = scale_x * sin(t), y = scale_y * sin(t) * cos(t)
                pose.pose.position.x = scale_x * math.sin(t[i])
                pose.pose.position.y = scale_y * math.sin(2 * t[i])
                pose.pose.position.z = 0.0
                
                # Calculate heading (tangent to the path)
                dx = scale_x * math.cos(t[i])
                dy = scale_y * 2 * math.cos(2 * t[i])
                heading = math.atan2(dy, dx)
                
                # Convert heading to quaternion (assuming planar motion)
                qz = math.sin(heading / 2.0)
                qw = math.cos(heading / 2.0)
                
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                path.poses.append(pose)
        
        elif self.path_type == 'circle':
            # Generate a circular path
            t = np.linspace(0, 2 * math.pi, self.num_points)
            radius = self.path_scale
            
            for i in range(len(t)):
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                
                # Circle equation: x = radius * cos(t), y = radius * sin(t)
                pose.pose.position.x = radius * math.cos(t[i])
                pose.pose.position.y = radius * math.sin(t[i])
                pose.pose.position.z = 0.0
                
                # Heading is tangent to the circle
                heading = t[i] + math.pi / 2.0  # 90 degrees ahead of radial direction
                
                # Convert heading to quaternion
                qz = math.sin(heading / 2.0)
                qw = math.cos(heading / 2.0)
                
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                path.poses.append(pose)
        
        elif self.path_type == 'straight':
            # Generate a straight line path
            length = self.path_scale
            x_vals = np.linspace(0, length, self.num_points)
            
            for i in range(len(x_vals)):
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                
                pose.pose.position.x = x_vals[i]
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.0
                
                # Heading is along the x-axis
                qz = 0.0  # sin(0/2)
                qw = 1.0  # cos(0/2)
                
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                path.poses.append(pose)
        
        else:
            self.get_logger().error(f'Unknown path type: {self.path_type}')
            return None
        
        return path
    
    def publish_path(self):
        """Generate and publish the path"""
        path = self.generate_path()
        if path:
            self.path_pub.publish(path)
            self.get_logger().debug(f'Published {self.path_type} path with {len(path.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    path_generator = PathGenerator()
    
    try:
        rclpy.spin(path_generator)
    except KeyboardInterrupt:
        path_generator.get_logger().info('Shutting down Path Generator')
    finally:
        path_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()