#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import os
import math
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import heapq  # Для приоритетной очереди в A*
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planning')
        
        # Объявляем параметры
        self.declare_parameter('map_yaml_file', '', 
                             ParameterDescriptor(description='Путь к YAML-файлу карты'))
        self.declare_parameter('start_x', -8.0, 
                             ParameterDescriptor(description='Начальная X-координата'))
        self.declare_parameter('start_y', 8.0,
                             ParameterDescriptor(description='Начальная Y-координата'))
        self.declare_parameter('goal_x', 8.0, 
                             ParameterDescriptor(description='X-координата цели'))
        self.declare_parameter('goal_y', 8.0, 
                             ParameterDescriptor(description='Y-координата цели'))
        self.declare_parameter('inflation_radius', 15,
                             ParameterDescriptor(description='Радиус расширения препятствий (в ячейках)'))
        
        # Получаем параметры
        self.map_yaml_file = self.get_parameter('map_yaml_file').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        
        # Инициализируем данные карты
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.occupancy_grid = None
        self.inflated_grid = None
        
        # Создаём издателей
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Создаем широковещатель трансформации для карты
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Таймеры
        self.map_timer = self.create_timer(1.0, self.publish_occupancy_grid)
        self.path_timer = self.create_timer(1.0, self.publish_path)
        
        # Загружаем карту и планируем путь
        self.load_map()
        
        # Планируем путь после небольшой задержки
        self.create_timer(2.0, self.plan_path_once)
        
        self.get_logger().info(f'Планировщик пути инициализирован. Старт: ({self.start_x}, {self.start_y}), Финиш: ({self.goal_x}, {self.goal_y})')
    
    def publish_map_transform(self):
        """Публикуем статическую трансформацию между world и map"""
        if self.map_origin is None:
            self.get_logger().error('Не могу опубликовать трансформацию: map_origin не инициализирован')
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        
        # Устанавливаем смещение согласно origin карты из YAML
        t.transform.translation.x = self.map_origin[0]
        t.transform.translation.y = self.map_origin[1]
        t.transform.translation.z = self.map_origin[2]
        
        # Идентичная ориентация (без поворота)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Опубликована трансформация world -> map с origin {self.map_origin}')
    
    def plan_path_once(self):
        """Запускаем планирование пути один раз и отменяем таймер"""
        self.plan_path()
        return True  # Отменяет таймер
    
    def load_map(self):
        """Загрузка карты из YAML-файла и соответствующего PGM-файла"""
        try:
            # Если указан относительный путь, превратим его в абсолютный
            if not os.path.isabs(self.map_yaml_file):
                pkg_path = get_package_share_directory('skate')
                self.map_yaml_file = os.path.join(pkg_path, self.map_yaml_file)
            
            self.get_logger().info(f'Загрузка карты из: {self.map_yaml_file}')
            
            # Проверяем, существует ли файл
            if not os.path.exists(self.map_yaml_file):
                self.get_logger().error(f'YAML-файл карты не найден: {self.map_yaml_file}')
                return
            
            # Читаем YAML-файл
            with open(self.map_yaml_file, 'r') as yaml_file:
                map_data = yaml.safe_load(yaml_file)
            
            # Получаем параметры карты
            map_image_file = map_data['image']
            self.map_resolution = map_data['resolution']
            self.map_origin = map_data['origin']
            occupied_thresh = map_data['occupied_thresh']
            free_thresh = map_data['free_thresh']
            
            self.get_logger().info(f'Параметры карты: разрешение={self.map_resolution}, origin={self.map_origin}')
            
            # Создаем абсолютный путь к PGM-файлу
            map_dir = os.path.dirname(self.map_yaml_file)
            map_image_path = os.path.join(map_dir, map_image_file)
            
            self.get_logger().info(f'Загрузка изображения карты из: {map_image_path}')
            
            # Проверяем существование файла изображения
            if not os.path.exists(map_image_path):
                self.get_logger().error(f'Файл изображения карты не найден: {map_image_path}')
                return
            
            # Читаем PGM-файл с помощью OpenCV
            self.map_data = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
            
            if self.map_data is None:
                self.get_logger().error(f'Не удалось загрузить изображение карты из {map_image_path}')
                return
            
            self.get_logger().info(f'Размер изображения карты: {self.map_data.shape}')
            
            # Конвертируем в формат occupancy grid (0-100)
            # В occupancy grid: 0 - свободно, 100 - занято, -1 - неизвестно
            # В изображении оттенков серого: 0 - черный (занято), 255 - белый (свободно)
            self.occupancy_grid = np.zeros_like(self.map_data, dtype=np.int8)
            
            # Используем порог из YAML для определения препятствий
            threshold = int(occupied_thresh * 255)
            self.get_logger().info(f'Используемый порог для препятствий: {threshold}')
            
            # Инвертируем логику: белое (высокие значения) - свободно, черное (низкие значения) - занято
            self.occupancy_grid[self.map_data > threshold] = 0  # Свободное пространство
            self.occupancy_grid[self.map_data <= threshold] = 100  # Занятое пространство
            
            # Создаем расширенную сетку для безопасной навигации
            self.inflate_obstacles(self.inflation_radius)
            
            # Отображение для отладки
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                debug_path = os.path.join(os.path.expanduser('~'), 'debug_map.png')
                cv2.imwrite(debug_path, self.inflated_grid)
                self.get_logger().debug(f'Сохранена отладочная карта: {debug_path}')
            
            self.get_logger().info(f'Карта успешно загружена и обработана. Размер карты: {self.map_data.shape}, разрешение: {self.map_resolution}')
            
            # Публикуем статическую трансформацию ПОСЛЕ загрузки карты
            self.publish_map_transform()
            
            # Публикуем occupancy grid для визуализации
            self.publish_occupancy_grid()
            
        except Exception as e:
            self.get_logger().error(f'Ошибка загрузки карты: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def inflate_obstacles(self, radius):
        """Расширяем препятствия для безопасного планирования"""
        if self.occupancy_grid is None:
            return
        
        # Создаем копию для расширения
        self.inflated_grid = self.occupancy_grid.copy()
        
        # Используем морфологические операции для расширения препятствий
        kernel = np.ones((2 * radius + 1, 2 * radius + 1), np.uint8)
        obstacle_mask = (self.occupancy_grid == 100).astype(np.uint8) * 255
        
        # Расширяем препятствия
        dilated_obstacles = cv2.dilate(obstacle_mask, kernel, iterations=1)
        
        # Применяем расширенные препятствия к сетке
        self.inflated_grid[dilated_obstacles > 0] = 100
        
        self.get_logger().info(f'Препятствия расширены с радиусом {radius} ячеек')
    
    def publish_occupancy_grid(self):
        """Публикуем occupancy grid для визуализации"""
        if self.inflated_grid is None:
            self.get_logger().warn('Нет данных сетки занятости для публикации')
            return
        
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'  # Используем frame_id 'map'
        
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.inflated_grid.shape[1]
        grid_msg.info.height = self.inflated_grid.shape[0]
        
        # Устанавливаем origin в (0,0,0) для фрейма map
        # Так как смещение учитывается в transform между world и map
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = 0.0
        grid_msg.info.origin.position.z = 0.0
        
        # Ориентация карты без поворота относительно map
        grid_msg.info.origin.orientation.w = 1.0
        
        # Переворачиваем карту если нужно
        flattened_grid = np.flip(self.inflated_grid, axis=0).flatten().astype(np.int8).tolist()
        grid_msg.data = flattened_grid
        
        self.map_pub.publish(grid_msg)
        self.get_logger().debug(f'Опубликована occupancy grid ({len(grid_msg.data)} ячеек)')

    
    def world_to_grid(self, x, y):
        """Конвертируем мировые координаты в координаты сетки"""
        if self.map_data is None:
            return None, None
        
        # Вычисляем координаты сетки
        grid_x = int((x - self.map_origin[0]) / self.map_resolution)
        
        # В формате сетки, ось Y направлена вниз, а карта перевернута,
        # поэтому нужно инвертировать Y-координату
        grid_y = self.map_data.shape[0] - int((y - self.map_origin[1]) / self.map_resolution) - 1
        
        # Проверяем, что координаты находятся в пределах сетки
        if (0 <= grid_x < self.map_data.shape[1] and 
            0 <= grid_y < self.map_data.shape[0]):
            return grid_x, grid_y
        else:
            self.get_logger().warn(f'Мировые координаты ({x}, {y}) выходят за пределы карты')
            return None, None
    
    def grid_to_world(self, grid_x, grid_y):
        """Конвертируем координаты сетки в мировые координаты"""
        if self.map_data is None:
            return None, None
        
        # Исправлено для учета перевернутой карты
        world_x = grid_x * self.map_resolution + self.map_origin[0]
        
        # Инвертируем Y-координату для соответствия мировым координатам
        world_y = (self.map_data.shape[0] - grid_y - 1) * self.map_resolution + self.map_origin[1]
        
        return world_x, world_y
    
    def is_valid(self, x, y):
        """Проверка, является ли ячейка сетки допустимой (в пределах границ и не препятствие)"""
        # Проверяем, что координаты в пределах сетки
        if (0 <= x < self.inflated_grid.shape[1] and 
            0 <= y < self.inflated_grid.shape[0]):
            # Проверяем, что ячейка свободна (не препятствие)
            return self.inflated_grid[y, x] != 100
        return False
    
    def get_neighbors(self, x, y):
        """Получаем допустимых соседей для алгоритма A*"""
        neighbors = []
        
        # Проверяем 8-связных соседей
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                # Пропускаем текущую ячейку
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                
                # Проверяем, что сосед допустим
                if self.is_valid(nx, ny):
                    # Для диагонального движения проверяем, что оба кардинальных движения допустимы
                    if dx != 0 and dy != 0:
                        if not (self.is_valid(x + dx, y) and self.is_valid(x, y + dy)):
                            continue
                    
                    # Стоимость выше для диагонального движения
                    cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                    neighbors.append((nx, ny, cost))
        
        return neighbors
    
    def heuristic(self, x1, y1, x2, y2):
        """Эвклидово расстояние в качестве эвристики для A*"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def a_star_search(self, start_x, start_y, goal_x, goal_y):
        """Алгоритм A* для поиска пути от старта до цели"""
        if self.inflated_grid is None:
            self.get_logger().error('Карта не загружена')
            return None
        
        # Конвертируем мировые координаты в координаты сетки
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.world_to_grid(goal_x, goal_y)
        
        self.get_logger().info(f'Конвертированные координаты: Старт: ({start_grid_x}, {start_grid_y}), Цель: ({goal_grid_x}, {goal_grid_y})')
        
        if (start_grid_x is None or start_grid_y is None or 
            goal_grid_x is None or goal_grid_y is None):
            self.get_logger().error('Начальная или целевая позиция за пределами карты')
            return None
        
        # Проверяем, что начало или цель не на препятствии
        if not self.is_valid(start_grid_x, start_grid_y):
            self.get_logger().error(f'Начальная позиция находится на препятствии: ({start_grid_x}, {start_grid_y})')
            # Попробуем найти ближайшую свободную ячейку
            for r in range(1, 20):  # Пробуем в радиусе до 20 ячеек
                found = False
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = start_grid_x + dx, start_grid_y + dy
                        if self.is_valid(nx, ny):
                            start_grid_x, start_grid_y = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    self.get_logger().info(f'Найдена альтернативная начальная позиция: ({start_grid_x}, {start_grid_y})')
                    break
            else:
                return None
        
        if not self.is_valid(goal_grid_x, goal_grid_y):
            self.get_logger().error(f'Целевая позиция находится на препятствии: ({goal_grid_x}, {goal_grid_y})')
            # Попробуем найти ближайшую свободную ячейку
            for r in range(1, 20):  # Пробуем в радиусе до 20 ячеек
                found = False
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = goal_grid_x + dx, goal_grid_y + dy
                        if self.is_valid(nx, ny):
                            goal_grid_x, goal_grid_y = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    self.get_logger().info(f'Найдена альтернативная целевая позиция: ({goal_grid_x}, {goal_grid_y})')
                    break
            else:
                return None
        
        # Инициализируем структуры данных для A*
        open_set = []  # Приоритетная очередь
        closed_set = set()  # Множество посещенных узлов
        g_score = {}  # Стоимость от старта до текущего узла
        came_from = {}  # Родительские указатели для реконструкции пути
        
        # Добавляем начальный узел в открытое множество
        start_node = (start_grid_x, start_grid_y)
        goal_node = (goal_grid_x, goal_grid_y)
        h_start = self.heuristic(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y)
        heapq.heappush(open_set, (h_start, 0, start_node))  # (f_score, tiebreaker, node)
        g_score[start_node] = 0
        
        self.get_logger().info(f'Начинаем поиск A* из ({start_grid_x}, {start_grid_y}) к ({goal_grid_x}, {goal_grid_y})')
        
        counter = 0  # Счетчик для тай-брейкера и отладки
        
        while open_set:
            # Получаем узел с наименьшим f_score (приоритет)
            _, _, current = heapq.heappop(open_set)
            current_x, current_y = current
            
            counter += 1
            if counter % 1000 == 0:
                self.get_logger().info(f'Обработано {counter} узлов, текущий: ({current_x}, {current_y})')
            
            # Проверяем, достигли ли мы цели
            if current == goal_node:
                self.get_logger().info(f'Путь найден за {counter} шагов!')
                
                # Реконструируем путь
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                path.reverse()
                
                # Конвертируем путь в сетке в мировые координаты
                world_path = []
                for grid_x, grid_y in path:
                    world_x, world_y = self.grid_to_world(grid_x, grid_y)
                    world_path.append((world_x, world_y))
                
                return world_path
            
            # Добавляем текущий узел в закрытое множество
            closed_set.add(current)
            
            # Исследуем соседей
            for neighbor_x, neighbor_y, move_cost in self.get_neighbors(current_x, current_y):
                neighbor = (neighbor_x, neighbor_y)
                
                # Пропускаем, если этот сосед уже оценен
                if neighbor in closed_set:
                    continue
                
                # Вычисляем предварительный g score
                tentative_g = g_score[current] + move_cost
                
                # Проверяем, является ли это лучшим путем к соседу
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # Этот путь лучше, записываем его
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor_x, neighbor_y, goal_grid_x, goal_grid_y)
                    
                    # Добавляем в открытое множество с приоритетом f_score
                    heapq.heappush(open_set, (f_score, counter, neighbor))
        
        self.get_logger().error(f'Путь не найден после проверки {counter} узлов!')
        return None
    
    def smooth_path(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001):
        """Применяем сглаживание пути для уменьшения резкости"""
        if not path or len(path) <= 2:
            return path
        
        # Создаем копию пути
        smoothed_path = [list(point) for point in path]
        
        # Применяем сглаживание
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(smoothed_path) - 1):
                for j in range(2):  # x и y координаты
                    aux = smoothed_path[i][j]
                    
                    # Формула обновления: weight_data тянет к исходному пути
                    # weight_smooth тянет к соседним точкам
                    smoothed_path[i][j] += (
                        weight_data * (path[i][j] - smoothed_path[i][j]) +
                        weight_smooth * (smoothed_path[i - 1][j] + smoothed_path[i + 1][j] - 
                                        2.0 * smoothed_path[i][j])
                    )
                    
                    change += abs(aux - smoothed_path[i][j])
        
        return [(x, y) for x, y in smoothed_path]
    
    def plan_path(self):
        """Находим путь от старта до цели и сохраняем его"""
        # Выполняем поиск A*
        self.path_coords = self.a_star_search(
            self.start_x, self.start_y, self.goal_x, self.goal_y)
        
        if self.path_coords:
            self.get_logger().info(f'Путь спланирован с {len(self.path_coords)} точками')
            
            # Сглаживаем путь, чтобы сделать его более естественным
            self.path_coords = self.smooth_path(self.path_coords)
            self.get_logger().info(f'Сглаженный путь имеет {len(self.path_coords)} точек')
            
            # Создаем сообщение Path
            self.path_msg = Path()
            self.path_msg.header.frame_id = 'world'
            
            # Конвертируем координаты пути в сообщения PoseStamped
            for i, (x, y) in enumerate(self.path_coords):
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                
                # Вычисляем ориентацию, чтобы смотреть на следующую точку
                if i < len(self.path_coords) - 1:
                    next_x, next_y = self.path_coords[i + 1]
                    yaw = math.atan2(next_y - y, next_x - x)
                else:
                    # Для последней точки используем ориентацию предыдущего сегмента
                    if i > 0:
                        prev_x, prev_y = self.path_coords[i - 1]
                        yaw = math.atan2(y - prev_y, x - prev_x)
                    else:
                        yaw = 0.0
                
                # Конвертируем угол рыскания в кватернион
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                self.path_msg.poses.append(pose)
            
            self.publish_path()
        else:
            self.get_logger().error('Не удалось спланировать путь')
    
    def publish_path(self):
        """Публикуем спланированный путь"""
        if hasattr(self, 'path_msg'):
            # Обновляем временную метку
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Обновляем все временные метки поз
            for pose in self.path_msg.poses:
                pose.header.stamp = self.get_clock().now().to_msg()
            
            # Публикуем путь
            self.path_pub.publish(self.path_msg)
            self.get_logger().debug(f'Опубликован путь с {len(self.path_msg.poses)} позами')

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        path_planner.get_logger().info('Завершение работы планировщика пути')
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()