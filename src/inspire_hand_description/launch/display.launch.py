#!/usr/bin/env python3
"""
Launch file for visualizing Inspire Hand URDF in RViz

Usage:
    ros2 launch inspire_hand_description display.launch.py
    ros2 launch inspire_hand_description display.launch.py hand:=left
    ros2 launch inspire_hand_description display.launch.py gui:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('inspire_hand_description')

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

    # URDF file path
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        ['inspire_hand_', LaunchConfiguration('hand'), '.urdf']
    ])

    # RViz config file
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'display.rviz'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_file])
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

    return LaunchDescription([
        hand_arg,
        gui_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
