"""
Launch file for Inspire Hand ROS2 node.

Usage:
    # Right hand (default, IP: 192.168.123.211)
    ros2 launch inspire_hand_ros2 inspire_hand.launch.py

    # Left hand (IP: 192.168.123.210)
    ros2 launch inspire_hand_ros2 inspire_hand.launch.py hand_side:=left namespace:=inspire_hand_left
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('inspire_hand_ros2')

    # Declare launch arguments
    hand_side_arg = DeclareLaunchArgument(
        'hand_side',
        default_value='right',
        description='Hand side: right (192.168.123.211) or left (192.168.123.210)'
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

    auto_calibrate_arg = DeclareLaunchArgument(
        'auto_calibrate',
        default_value='false',
        description='Run force sensor calibration on startup (hand must be fully open)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='inspire_hand',
        description='Node namespace (override hand_side if set)'
    )

    # Compute hand_ip from hand_side
    hand_ip_value = PythonExpression([
        "'192.168.123.210' if '",
        LaunchConfiguration('hand_side'),
        "' == 'left' else '192.168.123.211'"
    ])

    # Create node
    inspire_hand_node = Node(
        package='inspire_hand_ros2',
        executable='inspire_hand_node.py',
        name='inspire_hand_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'hand_ip': hand_ip_value,
            'hand_port': LaunchConfiguration('hand_port'),
            'device_id': LaunchConfiguration('device_id'),
            'state_rate': LaunchConfiguration('state_rate'),
            'touch_rate': LaunchConfiguration('touch_rate'),
            'enable_tactile': LaunchConfiguration('enable_tactile'),
            'default_force': LaunchConfiguration('default_force'),
            'default_speed': LaunchConfiguration('default_speed'),
            'slip_compensation': LaunchConfiguration('slip_compensation'),
            'auto_calibrate': LaunchConfiguration('auto_calibrate'),
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        hand_side_arg,
        hand_port_arg,
        device_id_arg,
        state_rate_arg,
        touch_rate_arg,
        enable_tactile_arg,
        default_force_arg,
        default_speed_arg,
        slip_compensation_arg,
        auto_calibrate_arg,
        namespace_arg,
        inspire_hand_node
    ])
