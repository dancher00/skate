#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        
        # Parameters - MODIFIED VALUES
        self.declare_parameter('k_gain', 0.5)  # Reduced from 10.5 for less aggressive control
        self.declare_parameter('k_soft', 0.0)  # Increased from 1.0 for more smoothing
        self.declare_parameter('max_steering', 0.227799)  # Max steering angle (from URDF limits)
        self.declare_parameter('control_frequency', 60.0)  # Increased from 10.0 for smoother control
        self.declare_parameter('base_wheel_speed', 30.0)  # Reduced from 20.0 for less aggressive motion
        
        # Get parameters
        self.k_gain = self.get_parameter('k_gain').value
        self.k_soft = self.get_parameter('k_soft').value
        self.max_steering = self.get_parameter('max_steering').value
        self.base_wheel_speed = self.get_parameter('base_wheel_speed').value
        
        # Path tracking variables
        self.path = None
        self.current_waypoint_idx = 0
        
        # Previous steering angles for smoothing and rate limiting
        self.prev_steering_angle = None
        self.prev_smoothed_angle = None
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # From gazebo_ros_p3d plugin
            self.odom_callback,
            10)
        
        self.path_sub = self.create_subscription(
            Path,
            '/path',  # Expected to receive a path message
            self.path_callback,
            10)
        
        # Create publishers for ROS2 controllers
        # New format for JointGroupPositionController
        self.front_steering_pub = self.create_publisher(
            Float64MultiArray,
            '/trj0_controller/commands',
            10)
        
        self.rear_steering_pub = self.create_publisher(
            Float64MultiArray,
            '/trj1_controller/commands',
            10)
        
        # For wheel velocity control - using a Float64MultiArray for all wheels
        self.wheel_velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10)
        
        # Individual wheel publishers (alternative approach)
        self.wheel0_pub = self.create_publisher(
            Float64,
            '/wheel_controller/commands/whj0',
            10)
        
        self.wheel1_pub = self.create_publisher(
            Float64,
            '/wheel_controller/commands/whj1',
            10)
        
        self.wheel2_pub = self.create_publisher(
            Float64,
            '/wheel_controller/commands/whj2',
            10)
        
        self.wheel3_pub = self.create_publisher(
            Float64,
            '/wheel_controller/commands/whj3',
            10)
        
        # Velocity publisher for cmd_vel if needed
        self.velocity_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_frequency').value,
            self.control_loop)
        
        # Initialize TF listener for coordinate transformations if needed
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Current state
        self.current_pose = None
        self.current_velocity = 0.0
        
        self.get_logger().info('Stanley Controller initialized')
    
    def odom_callback(self, msg):
        """Process odometry data to extract position, orientation, and velocity"""
        self.current_pose = msg.pose.pose
        
        # Calculate current speed from linear velocity
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x ** 2 + 
            msg.twist.twist.linear.y ** 2)
        
    def path_callback(self, msg):
        """Store the received path"""
        if not msg.poses:
            self.get_logger().warn('Received empty path')
            return
        
        self.path = msg
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Received new path with {len(self.path.poses)} waypoints')
    


    def get_cross_track_error(self):
        """Calculate the cross-track error (lateral error from path)"""
        if not self.path or not self.current_pose:
            return 0.0, 0.0, 0.0
        
        # Find the nearest point on the path
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Start from the current waypoint index to avoid searching the entire path
        min_dist = float('inf')
        nearest_idx = self.current_waypoint_idx
        
        # Search for the closest point on the path
        for i in range(self.current_waypoint_idx, len(self.path.poses)):
            waypoint = self.path.poses[i].pose
            dist = math.sqrt((waypoint.position.x - robot_x)**2 + 
                            (waypoint.position.y - robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
            elif dist > min_dist * 2:  # If distance starts increasing significantly
                break
            
        self.current_waypoint_idx = nearest_idx
        
        # Get the nearest waypoint
        nearest_waypoint = self.path.poses[nearest_idx].pose
        
        # Calculate the heading of the path at this point
        path_heading = 0.0
        if nearest_idx < len(self.path.poses) - 1:
            next_waypoint = self.path.poses[nearest_idx + 1].pose
            path_heading = math.atan2(
                next_waypoint.position.y - nearest_waypoint.position.y,
                next_waypoint.position.x - nearest_waypoint.position.x)
        elif nearest_idx > 0:
            prev_waypoint = self.path.poses[nearest_idx - 1].pose
            path_heading = math.atan2(
                nearest_waypoint.position.y - prev_waypoint.position.y,
                nearest_waypoint.position.x - prev_waypoint.position.x)
        
        # Get the robot's heading from quaternion
        qx = self.current_pose.orientation.x
        qy = self.current_pose.orientation.y
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        
        # Convert quaternion to yaw (assuming robot moves in the XY plane)
        robot_heading = math.atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Calculate the vector from robot to nearest point
        dx = nearest_waypoint.position.x - robot_x
        dy = nearest_waypoint.position.y - robot_y
        
        # Get the direction vector of the path
        path_direction_x = math.cos(path_heading)
        path_direction_y = math.sin(path_heading)
        
        # Calculate the signed perpendicular distance (cross-track error)
        # Using the cross product to get the signed distance
        cross_track_error = -(dx * path_direction_y - dy * path_direction_x)
        
        # Calculate heading error (difference between path heading and robot heading)
        heading_error = self.normalize_angle(path_heading - robot_heading)
        
        self.get_logger().debug(f'Cross-track error: {cross_track_error}, Heading error: {heading_error}')
        
        return cross_track_error, heading_error, path_heading, robot_x, robot_y



    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    # MODIFIED: Added rate limiting to the steering angle calculation
    def stanley_control(self, cross_track_error, heading_error):
        """Implement the Stanley controller to calculate steering angle"""
        # Avoid division by zero at very low speeds
        adjusted_velocity = self.current_velocity + self.k_soft
        
        # Calculate steering angle using Stanley controller formula
        # δ = ψ + arctan(k * e / v)
        steering_angle = -heading_error + math.atan2(-self.k_gain * cross_track_error, adjusted_velocity)
        # steering_angle = math.atan2(-self.k_gain * cross_track_error, adjusted_velocity)
        # steering_angle = heading_error
        # Limit steering angle to max_steering
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        
        # Add steering rate limiting (NEW CODE)
        if hasattr(self, 'prev_steering_angle') and self.prev_steering_angle is not None:
            max_steering_change = 0.05  # Max change in radians per control cycle
            steering_delta = steering_angle - self.prev_steering_angle
            if abs(steering_delta) > max_steering_change:
                steering_angle = self.prev_steering_angle + math.copysign(max_steering_change, steering_delta)
        
        # Save for next iteration
        self.prev_steering_angle = steering_angle
        
        return steering_angle
    
    # NEW METHOD: Add exponential smoothing for steering angle
    def smooth_angle(self, angle, prev_angle, alpha=0.6):
        """Apply exponential smoothing to angle"""
        # If no previous angle, just return current angle
        if prev_angle is None:
            return angle
        
        # Apply exponential smoothing
        # alpha=0 would be no change, alpha=1 would be no smoothing
        smoothed_angle = alpha * angle + (1.0 - alpha) * prev_angle
        return smoothed_angle
    
    def control_wheel_velocities(self, forward_speed):
        """Set all wheels to rotate at the same velocity"""
        # Create a Float64MultiArray message for all wheels
        wheel_velocities = Float64MultiArray()
        
        # All wheels rotate at the same speed
        wheel_velocities.data = [self.base_wheel_speed] * 4
        
        # Publish to the wheel controller
        self.wheel_velocity_pub.publish(wheel_velocities)
        
        # Alternative: publish to individual wheel controllers
        wheel_speed_msg = Float64()
        wheel_speed_msg.data = self.base_wheel_speed
        
        self.wheel0_pub.publish(wheel_speed_msg)
        self.wheel1_pub.publish(wheel_speed_msg)
        self.wheel2_pub.publish(wheel_speed_msg)
        self.wheel3_pub.publish(wheel_speed_msg)
        
        self.get_logger().debug(f'Set all wheels to velocity: {self.base_wheel_speed}')
    
    # MODIFIED: Updated control loop to include smoothing
    def control_loop(self):
        """Main control loop that runs at the specified frequency"""
        if not self.path or not self.current_pose:
            self.get_logger().debug('Waiting for path and odometry data...')
            return
        
        # Calculate errors
        cross_track_error, heading_error, path_heading, robot_x, robot_y = self.get_cross_track_error()

        self.get_logger().info(f'Cross-track error: {cross_track_error:.3f}, head: {heading_error}, rob_x: {robot_x}, rob_y: {robot_y}')  #####
        
        # Get steering command from Stanley controller
        raw_steering_angle = self.stanley_control(cross_track_error, heading_error)
        
        # Apply additional smoothing (NEW CODE)
        if not hasattr(self, 'prev_smoothed_angle') or self.prev_smoothed_angle is None:
            self.prev_smoothed_angle = raw_steering_angle
        
        steering_angle = self.smooth_angle(raw_steering_angle, self.prev_smoothed_angle)
        self.prev_smoothed_angle = steering_angle
        
        # Publish steering commands
        front_steering_msg = Float64MultiArray()
        front_steering_msg.data = [steering_angle]
        self.front_steering_pub.publish(front_steering_msg)
        
        # For a tighter turning radius
        rear_steering_msg = Float64MultiArray()
        rear_steering_msg.data = [-steering_angle]  # Opposite angle for rear steering
        self.rear_steering_pub.publish(rear_steering_msg)
        
        # Control wheel velocities to be synchronized
        self.control_wheel_velocities(self.base_wheel_speed)
        
        # Also publish cmd_vel for overall robot motion if needed
        target_velocity = 0.5  # Base velocity
        velocity_factor = 1.0 - (abs(steering_angle) / self.max_steering) * 0.5
        
        vel_msg = Twist()
        vel_msg.linear.x = target_velocity * velocity_factor
        self.velocity_pub.publish(vel_msg)
        
        # Log information
        self.get_logger().debug(
            f'Cross-track error: {cross_track_error:.3f}, '
            f'Heading error: {heading_error:.3f}, '
            f'Raw steering: {raw_steering_angle:.3f}, '
            f'Smoothed steering: {steering_angle:.3f}, '
            f'Wheel velocity: {self.base_wheel_speed:.3f}'
        )
        
        # Check if we've reached the end of the path
        if self.current_waypoint_idx >= len(self.path.poses) - 1:
            self.get_logger().info('Reached end of path!')

def main(args=None):
    rclpy.init(args=args)
    stanley_controller = StanleyController()
    
    try:
        rclpy.spin(stanley_controller)
    except KeyboardInterrupt:
        stanley_controller.get_logger().info('Shutting down Stanley Controller')
    finally:
        stanley_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()