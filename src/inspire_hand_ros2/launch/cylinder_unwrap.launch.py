#!/usr/bin/env python3
"""Launch file for cylinder unwrap node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for cylinder unwrap visualization."""

    # Declare arguments
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='360',
        description='Width of unwrapped image in pixels (1 pixel per degree)'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='200',
        description='Height of unwrapped image in pixels'
    )

    height_min_arg = DeclareLaunchArgument(
        'height_min',
        default_value='-0.075',
        description='Minimum height along cylinder axis (meters)'
    )

    height_max_arg = DeclareLaunchArgument(
        'height_max',
        default_value='0.075',
        description='Maximum height along cylinder axis (meters)'
    )

    # Cylinder unwrap node
    unwrap_node = Node(
        package='inspire_hand_ros2',
        executable='cylinder_unwrap_node.py',
        name='cylinder_unwrap_node',
        output='screen',
        parameters=[{
            'input_topic': '/cylinder_projection/projected_pointcloud',
            'output_topic': '/cylinder_projection/unwrapped_image',
            'output_colormap_topic': '/cylinder_projection/unwrapped_colormap',
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'cylinder_frame': 'right_palm_force_sensor',
            'cylinder_radius': 0.02,  # Must match cylinder_projection radius
            'height_min': LaunchConfiguration('height_min'),
            'height_max': LaunchConfiguration('height_max'),
            'colormap': 2,  # COLORMAP_JET (blue->cyan->green->yellow->red)
            'intensity_min': 0.0,
            'intensity_max': 4095.0,
        }]
    )

    return LaunchDescription([
        image_width_arg,
        image_height_arg,
        height_min_arg,
        height_max_arg,
        unwrap_node,
    ])
