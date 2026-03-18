#!/usr/bin/env python3
"""Launch file for complete tactile visualization pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for complete tactile visualization."""

    pkg_dir = get_package_share_directory('inspire_hand_ros2')

    # Launch tactile pointcloud
    tactile_pc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'tactile_pointcloud.launch.py')
        )
    )

    # Launch cylinder projection
    cylinder_proj_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'cylinder_projection.launch.py')
        )
    )

    # Launch cylinder unwrap
    cylinder_unwrap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'cylinder_unwrap.launch.py')
        )
    )

    return LaunchDescription([
        tactile_pc_launch,
        cylinder_proj_launch,
        cylinder_unwrap_launch,
    ])
