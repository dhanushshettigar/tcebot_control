import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments with default values
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='Serial port for communication with Arduino'
    )

    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter', default_value='0.065',
        description='Diameter of the wheels in meters'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.17',
        description='Distance between the two wheels in meters'
    )

    # Node configuration with parameters
    differential_drive_node = Node(
        package='tcebot_control',
        executable='differential_drive',
        name='differential_drive',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'wheel_diameter': LaunchConfiguration('wheel_diameter'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
        }]
    )

    return LaunchDescription([
        port_arg,
        wheel_diameter_arg,
        wheel_separation_arg,
        differential_drive_node
    ])
