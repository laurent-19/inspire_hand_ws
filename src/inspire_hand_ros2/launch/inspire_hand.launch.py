"""
Launch file for Inspire Hand ROS2 node.

Usage:
    ros2 launch inspire_hand_ros2 inspire_hand.launch.py
    ros2 launch inspire_hand_ros2 inspire_hand.launch.py hand_ip:=192.168.11.211
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('inspire_hand_ros2')

    # Declare launch arguments
    hand_ip_arg = DeclareLaunchArgument(
        'hand_ip',
        default_value='192.168.123.211',
        description='IP address of the Inspire Hand'
    )

    hand_port_arg = DeclareLaunchArgument(
        'hand_port',
        default_value='6000',
        description='Modbus TCP port'
    )

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='1',
        description='Hand device ID (1-254)'
    )

    state_rate_arg = DeclareLaunchArgument(
        'state_rate',
        default_value='100.0',
        description='State publishing rate in Hz'
    )

    touch_rate_arg = DeclareLaunchArgument(
        'touch_rate',
        default_value='50.0',
        description='Tactile publishing rate in Hz'
    )

    enable_tactile_arg = DeclareLaunchArgument(
        'enable_tactile',
        default_value='true',
        description='Enable tactile sensor reading'
    )

    default_force_arg = DeclareLaunchArgument(
        'default_force',
        default_value='500',
        description='Default grasp force in grams'
    )

    default_speed_arg = DeclareLaunchArgument(
        'default_speed',
        default_value='500',
        description='Default movement speed (0-1000)'
    )

    slip_compensation_arg = DeclareLaunchArgument(
        'slip_compensation',
        default_value='true',
        description='Enable slip detection and force compensation'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='inspire_hand',
        description='Node namespace'
    )

    # Create node
    inspire_hand_node = Node(
        package='inspire_hand_ros2',
        executable='inspire_hand_node.py',
        name='inspire_hand_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'hand_ip': LaunchConfiguration('hand_ip'),
            'hand_port': LaunchConfiguration('hand_port'),
            'device_id': LaunchConfiguration('device_id'),
            'state_rate': LaunchConfiguration('state_rate'),
            'touch_rate': LaunchConfiguration('touch_rate'),
            'enable_tactile': LaunchConfiguration('enable_tactile'),
            'default_force': LaunchConfiguration('default_force'),
            'default_speed': LaunchConfiguration('default_speed'),
            'slip_compensation': LaunchConfiguration('slip_compensation'),
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        hand_ip_arg,
        hand_port_arg,
        device_id_arg,
        state_rate_arg,
        touch_rate_arg,
        enable_tactile_arg,
        default_force_arg,
        default_speed_arg,
        slip_compensation_arg,
        namespace_arg,
        inspire_hand_node
    ])
