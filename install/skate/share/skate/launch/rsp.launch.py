import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('skate'))
    xacro_file = os.path.join(pkg_path,'description', 'board', 'urdf', 'board.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Save robot description to a temporary file
    robot_description_content = robot_description_config.toxml()
    with open('/tmp/robot_description', 'w') as f:
        f.write(robot_description_content)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])