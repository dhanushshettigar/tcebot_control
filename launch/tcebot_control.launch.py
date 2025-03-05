import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Motor Control Node
        Node(
            package='tcebot_control',
            executable='differential_drive',
            name='motor_controller',
            output='screen'
        )
    ])

