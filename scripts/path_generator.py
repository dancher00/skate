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
        self.declare_parameter('path_type', 'figure_eight')
        self.declare_parameter('path_scale', 20.0)  # Уменьшил с 3.0 для более плавной восьмерки
        self.declare_parameter('num_points', 500)  # Увеличил с 100 для более гладкой траектории
        self.declare_parameter('publish_frequency', 2.0)
        
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
        path.header.frame_id = 'world'
        
        if self.path_type == 'figure_eight':
            # Улучшенный генератор восьмерки с плавными переходами
            # Используем больше точек и модифицированную формулу
            t = np.linspace(0, 2 * math.pi, self.num_points)
            scale_x = self.path_scale
            scale_y = self.path_scale / 2
            
            # Вычисляем кривизну в каждой точке для адаптивного распределения
            curvatures = []
            for i in range(len(t)):
                # Первые и вторые производные
                dx = scale_x * math.cos(t[i])
                dy = scale_y * 2 * math.cos(2 * t[i])
                ddx = -scale_x * math.sin(t[i])
                ddy = -scale_y * 4 * math.sin(2 * t[i])
                
                # Формула кривизны
                curvature = abs(dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-10)
                curvatures.append(curvature)
            
            # Добавляем больше точек в областях с высокой кривизной
            max_curvature = max(curvatures)
            min_spacing = 0.01  # Минимальное расстояние между точками
            
            last_pose = None
            for i in range(len(t)):
                # Создаем точку пути
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                
                # Используем более плавную версию уравнения восьмерки
                pose.pose.position.x = scale_x * math.sin(t[i])
                pose.pose.position.y = scale_y * math.sin(2 * t[i])
                pose.pose.position.z = 0.0
                
                # Вычисляем направление (тангенс к пути)
                dx = scale_x * math.cos(t[i])
                dy = scale_y * 2 * math.cos(2 * t[i])
                heading = math.atan2(dy, dx)
                
                # Преобразуем направление в кватернион
                qz = math.sin(heading / 2.0)
                qw = math.cos(heading / 2.0)
                
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                # Добавляем эту точку в путь
                # Проверяем, достаточно ли далеко от предыдущей
                if last_pose is None:
                    path.poses.append(pose)
                    last_pose = pose
                else:
                    dx = pose.pose.position.x - last_pose.pose.position.x
                    dy = pose.pose.position.y - last_pose.pose.position.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    # Нормализованная кривизна (0-1)
                    norm_curvature = curvatures[i] / max_curvature if max_curvature > 0 else 0
                    # Адаптивное расстояние: меньше на поворотах, больше на прямых
                    adaptive_spacing = min_spacing + (1.0 - norm_curvature) * 0.05
                    
                    if dist >= adaptive_spacing:
                        path.poses.append(pose)
                        last_pose = pose
            
            # Сглаживаем финальный путь с помощью скользящего среднего
            if len(path.poses) > 5:
                smoothed_poses = []
                window_size = 5  # Размер окна сглаживания
                
                # Копируем первые (window_size//2) точек
                for i in range(window_size//2):
                    smoothed_poses.append(path.poses[i])
                
                # Сглаживаем средние точки
                for i in range(window_size//2, len(path.poses) - window_size//2):
                    smooth_pose = PoseStamped()
                    smooth_pose.header = path.poses[i].header
                    
                    # Среднее положение
                    x_sum = 0
                    y_sum = 0
                    for j in range(i - window_size//2, i + window_size//2 + 1):
                        x_sum += path.poses[j].pose.position.x
                        y_sum += path.poses[j].pose.position.y
                    
                    smooth_pose.pose.position.x = x_sum / window_size
                    smooth_pose.pose.position.y = y_sum / window_size
                    smooth_pose.pose.position.z = 0.0
                    
                    # Ориентация остается от исходной точки
                    smooth_pose.pose.orientation = path.poses[i].pose.orientation
                    
                    smoothed_poses.append(smooth_pose)
                
                # Копируем последние (window_size//2) точек
                for i in range(len(path.poses) - window_size//2, len(path.poses)):
                    smoothed_poses.append(path.poses[i])
                
                # Заменяем путь сглаженным
                path.poses = smoothed_poses
        
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