from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    k_gain = LaunchConfiguration('k_gain')
    k_soft = LaunchConfiguration('k_soft')
    max_steering = LaunchConfiguration('max_steering')
    wheelbase = LaunchConfiguration('wheelbase')
    control_frequency = LaunchConfiguration('control_frequency')

    # Declare launch arguments
    declare_k_gain = DeclareLaunchArgument(
        'k_gain',
        default_value='0.5',
        description='Stanley controller gain parameter'
    )
    
    declare_k_soft = DeclareLaunchArgument(
        'k_soft',
        default_value='1.0',
        description='Softening factor for low speeds'
    )
    
    declare_max_steering = DeclareLaunchArgument(
        'max_steering',
        default_value='0.227799',
        description='Maximum steering angle in radians'
    )
    
    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.5',
        description='Wheelbase of the robot (distance between front and rear axles)'
    )
    
    declare_control_frequency = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Control loop frequency in Hz'
    )

    # Stanley controller node
    stanley_controller_node = Node(
        package='skate',
        executable='stanley_controller.py',
        name='stanley_controller',
        output='screen',
        parameters=[{
            'k_gain': k_gain,
            'k_soft': k_soft,
            'max_steering': max_steering,
            'wheelbase': wheelbase,
            'control_frequency': control_frequency,
        }]
    )

    return LaunchDescription([
        declare_k_gain,
        declare_k_soft,
        declare_max_steering,
        declare_wheelbase,
        declare_control_frequency,
        stanley_controller_node,
    ])