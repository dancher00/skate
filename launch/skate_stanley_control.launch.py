import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Package paths
    pkg_skate = get_package_share_directory('skate')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    path_type = LaunchConfiguration('path_type')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_path_type = DeclareLaunchArgument(
        'path_type',
        default_value='figure_eight',
        description='Type of path to generate (figure_eight, circle, straight)'
    )
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_skate, 'description', 'board', 'urdf', 'board.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    
    # Create robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': use_sim_time}]
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'board'],
        output='screen'
    )
    
    # Load controller parameters
    controller_params_file = os.path.join(pkg_skate, 'config', 'controller_config.yaml')
    
    # Controller nodes
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_raw},
            controller_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    trj0_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['trj0_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    trj1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['trj1_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Path generator node
    path_generator_node = Node(
        package='skate',
        executable='path_generator.py',
        name='path_generator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path_type': path_type,
            'path_scale': 3.0,
            'num_points': 100,
            'publish_frequency': 1.0
        }]
    )
    
    # Stanley controller node
    stanley_controller_node = Node(
        package='skate',
        executable='stanley_controller.py',
        name='stanley_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'k_gain': 0.5,
            'k_soft': 1.0,
            'max_steering': 0.227799,
            'wheelbase': 0.5,
            'control_frequency': 10.0
        }]
    )
    
    # Launch RViz with a configuration
    rviz_config = os.path.join(pkg_skate, 'config', 'view_skate.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Define launch sequence with event handlers
    # First spawn the robot in Gazebo
    spawn_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[spawn_entity]
        )
    )
    
    # Start controller manager after gazebo has started
    controller_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[controller_manager]
        )
    )
    
    # Start joint state broadcaster after controller manager
    broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )
    
    # Start the controllers after the joint state broadcaster
    controller_event_1 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[trj0_controller_spawner]
        )
    )
    
    controller_event_2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=trj0_controller_spawner,
            on_start=[trj1_controller_spawner]
        )
    )
    
    controller_event_3 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=trj1_controller_spawner,
            on_start=[wheel_controller_spawner]
        )
    )
    
    # Start the path generator and Stanley controller after all controllers are up
    app_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=wheel_controller_spawner,
            on_start=[path_generator_node, stanley_controller_node]
        )
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_path_type,
        
        # Start Gazebo
        gazebo,
        
        # Start robot state publisher
        robot_state_publisher,
        
        # Register event sequence
        spawn_event,
        controller_event,
        broadcaster_event,
        controller_event_1,
        controller_event_2,
        controller_event_3,
        app_event,
        
        # Start RViz
        rviz_node
    ])