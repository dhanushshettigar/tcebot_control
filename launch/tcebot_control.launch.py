import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    controller_params_file = os.path.join(get_package_share_directory("tcebot_control"),'config','tcebot_controllers.yaml')

    # We need the robot description to be passed to the controller_manager
    # So it can check the ros2_control parameters.
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                    controller_params_file],
        remappings=[
            ('/diff_controller/cmd_vel', '/cmd_vel'), # Used if use_stamped_vel param is true
            ('/diff_controller/cmd_vel_unstamped', '/cmd_vel'), # Used if use_stamped_vel param is false
            ('/diff_controller/cmd_vel_out', '/cmd_vel_out'), # Used if publish_limited_velocity param is true
            ('/diff_controller/odom', '/odom'),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of diff_drive_controller_spawner after `joint_state_broadcaster`
    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

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
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
        port_arg,
        wheel_diameter_arg,
        wheel_separation_arg,
        differential_drive_node
    ])
