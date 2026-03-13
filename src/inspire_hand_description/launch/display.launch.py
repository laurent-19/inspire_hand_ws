#!/usr/bin/env python3
"""
Launch file for visualizing RH56E2 Hand URDF in RViz

Usage:
    ros2 launch inspire_hand_description display.launch.py
    ros2 launch inspire_hand_description display.launch.py hand:=left
    ros2 launch inspire_hand_description display.launch.py gui:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get launch configuration values
    hand = LaunchConfiguration('hand').perform(context)

    # Package share directory
    pkg_share = get_package_share_directory('inspire_hand_description')

    # Build file paths
    urdf_file = os.path.join(pkg_share, 'urdf', f'rh56e2_{hand}.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', f'{hand}.rviz')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
        }]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]


def generate_launch_description():
    # Launch arguments
    hand_arg = DeclareLaunchArgument(
        'hand',
        default_value='right',
        description='Which hand to display: right or left'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show joint_state_publisher_gui'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    return LaunchDescription([
        hand_arg,
        gui_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
