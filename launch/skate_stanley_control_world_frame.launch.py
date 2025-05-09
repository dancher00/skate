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
        default_value='circle',
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
    
    # Static transform publishers for coordinate frames
    static_transform_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        name='static_tf_world_to_map',
        output='screen'
    )
    
    # Launch Gazebo simulation with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': os.path.join(pkg_skate, 'worlds', 'obstacle_world.world'),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn entity in Gazebo at a specific corner position (-8, -8)
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'board',
            '-x', '-8.0',
            '-y', '-8.0',
            '-z', '0.1',
            '-Y', '0.0'  # Yaw orientation
        ],
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
    # path_generator_node = Node(
    #     package='skate',
    #     executable='path_generator.py',
    #     name='path_generator',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'path_type': path_type,
    #         'path_scale': 6.0,  # Increased scale for the larger world
    #         'num_points': 500   # More points for smoother paths
    #     }]
    # )
    

    path_planning_node = Node(
        package='skate',
        executable='path_planning.py',
        name='path_planning',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_yaml_file': os.path.join(pkg_skate, 'maps', 'obstacle_world.yaml'),
            'start_x': -8.0,
            'start_y': -8.0,
            'goal_x': 8.0,
            'goal_y': 8.0
        }]
    )



    stanley_controller_node = Node(
        package='skate',
        executable='stanley_controller.py',
        name='stanley_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'k_gain': 1.0,        # Adjusted for better tracking
            'k_soft': 0.5,        # Smoothing factor
            'max_steering': 0.227799,
            'wheelbase': 0.5,
            'control_frequency': 60.0,
            'base_wheel_speed': 25.0  # Adjusted for the obstacle course
        }]
    )
    
    # Odometry to Path converter node
    odom_to_path_node = Node(
        package='skate',
        executable='odom_to_path.py',
        name='odom_to_path',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path_length': 2000,  # Increased to track longer paths
            'min_distance': 0.02  # Smaller distance between points for better visualization
        }]
    )

    # Robot localization node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'publish_tf': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'world',
            'odom0': '/odom',
            'odom0_config': [True, True, False, False, False, False, False, False, False, False, False, True, False, False, False]
        }]
    )

    # TF broadcaster for odometry
    odom_tf_broadcaster_node = Node(
        package='skate',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen'
    )
    
    # RViz with updated configuration
    rviz_config = os.path.join(pkg_skate, 'config', 'view_skate_world_frame.rviz')
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
        actions=[path_planning_node, stanley_controller_node, odom_to_path_node, robot_localization_node]
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_path_type,
        
        # Start transformation publishers right away
        robot_state_publisher,
        static_transform_world_to_map,
        odom_tf_broadcaster_node,
        
        # Start Gazebo with custom world
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