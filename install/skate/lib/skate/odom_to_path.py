#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor

class OdomToPath(Node):
    """
    Улучшенный узел, который конвертирует данные одометрии в путь для отображения
    фактической траектории робота
    """
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Улучшенная инициализация параметров с описаниями
        self.declare_parameter('path_length', 1000, 
                              ParameterDescriptor(description='Максимальное количество точек в пути'))
        self.declare_parameter('min_distance', 0.05, 
                              ParameterDescriptor(description='Минимальное расстояние между точками пути'))
        self.declare_parameter('path_frame_id', 'world', 
                              ParameterDescriptor(description='Фрейм для публикации пути'))
        
        # Получение параметров
        self.max_path_length = self.get_parameter('path_length').value
        self.min_distance = self.get_parameter('min_distance').value
        self.path_frame_id = self.get_parameter('path_frame_id').value
        
        # Инициализация пустого пути
        self.path = Path()
        self.path.header.frame_id = self.path_frame_id
        
        # Создание подписок и издателей
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # От плагина gazebo_ros_p3d
            self.odom_callback,
            10)
        
        self.path_pub = self.create_publisher(
            Path,
            '/odom_path',  # Фактическая траектория
            10)
        
        # Таймер для публикации пути с заданной частотой
        self.timer = self.create_timer(0.1, self.publish_path)  # 10 Гц
        
        # Для оптимизации - хранение последней позиции
        self.last_pose = None
        
        # Счетчик для диагностики
        self.points_added = 0
        self.points_skipped = 0
        
        # Обработчик изменения параметров
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info('Odometry to Path converter initialized')
    
    def parameters_callback(self, params):
        """Обработчик изменения параметров"""
        for param in params:
            if param.name == 'path_length' and param.value != self.max_path_length:
                self.max_path_length = param.value
                self.get_logger().info(f'Path length changed to {self.max_path_length}')
                
                # Если новая длина меньше текущей - обрезаем путь
                if len(self.path.poses) > self.max_path_length:
                    self.path.poses = self.path.poses[-self.max_path_length:]
            
            elif param.name == 'min_distance' and param.value != self.min_distance:
                self.min_distance = param.value
                self.get_logger().info(f'Minimum distance changed to {self.min_distance}')
            
            elif param.name == 'path_frame_id' and param.value != self.path_frame_id:
                self.path_frame_id = param.value
                self.path.header.frame_id = self.path_frame_id
                self.get_logger().info(f'Path frame ID changed to {self.path_frame_id}')
        
        return True
    
    def odom_callback(self, msg):
        """Улучшенная обработка данных одометрии для обновления пути"""
        # Создание сообщения PoseStamped из данных одометрии
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = self.path_frame_id
        pose_stamped.pose = msg.pose.pose
        
        # Добавление в путь с фильтрацией по расстоянию
        if self.last_pose is None:
            self.path.poses.append(pose_stamped)
            self.last_pose = pose_stamped
            self.points_added += 1
        else:
            # Расчет расстояния до последней добавленной точки
            dx = pose_stamped.pose.position.x - self.last_pose.pose.position.x
            dy = pose_stamped.pose.position.y - self.last_pose.pose.position.y
            dist = (dx*dx + dy*dy)**0.5
            
            # Добавляем только если расстояние больше минимального
            if dist >= self.min_distance:
                self.path.poses.append(pose_stamped)
                self.last_pose = pose_stamped
                self.points_added += 1
            else:
                self.points_skipped += 1
        
        # Обрезаем путь, если он стал слишком длинным
        if len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        
        # Обновляем заголовок пути
        self.path.header.stamp = self.get_clock().now().to_msg()
        
        # Периодическая диагностика
        total_points = self.points_added + self.points_skipped
        if total_points > 0 and total_points % 1000 == 0:
            efficiency = (self.points_added / total_points) * 100
            self.get_logger().info(
                f'Path efficiency: {efficiency:.1f}% ({self.points_added} added, {self.points_skipped} skipped)')
    
    def publish_path(self):
        """Публикация текущего пути"""
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