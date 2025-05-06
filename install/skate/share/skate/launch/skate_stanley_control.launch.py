import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Package paths
    pkg_skate = get_package_share_directory('skate')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    path_type = LaunchConfiguration('path_type')
    
    # Declare launch arguments
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
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': use_sim_time}]
    )
    
    # Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'board'],
        output='screen'
    )
    
    # Create controllers
    controller_params_file = os.path.join(pkg_skate, 'config', 'controller_config.yaml')
    
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
    
    # App nodes
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
    
    # RViz
    rviz_config = os.path.join(pkg_skate, 'config', 'view_skate.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Use TimerAction to sequence the launch
    spawn_timer = TimerAction(
        period=3.0,  # Wait for 3 seconds after Gazebo starts
        actions=[spawn_entity]
    )
    
    controller_manager_timer = TimerAction(
        period=5.0,  # Wait for 5 seconds after spawn
        actions=[controller_manager]
    )
    
    joint_state_broadcaster_timer = TimerAction(
        period=7.0,  # Wait for 7 seconds 
        actions=[joint_state_broadcaster_spawner]
    )
    
    controllers_timer = TimerAction(
        period=9.0,  # Wait for 9 seconds
        actions=[trj0_controller_spawner, trj1_controller_spawner, wheel_controller_spawner]
    )
    
    stanley_timer = TimerAction(
        period=15.0,  # Wait for 15 seconds
        actions=[path_generator_node, stanley_controller_node]
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_path_type,
        
        # Start robot state publisher right away
        robot_state_publisher,
        
        # Start Gazebo
        gazebo,
        
        # Sequence other components with timers
        spawn_timer,
        controller_manager_timer,
        joint_state_broadcaster_timer,
        controllers_timer,
        stanley_timer,
        
        # Start RViz
        rviz_node
    ])