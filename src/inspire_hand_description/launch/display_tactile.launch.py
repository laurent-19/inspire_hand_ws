"""
Launch file for tactile point cloud visualization with joint_state_publisher.

This can be launched standalone from the description package to visualize
the hand with simulated joint states and tactile data.

Usage:
    ros2 launch inspire_hand_description display_tactile.launch.py
    ros2 launch inspire_hand_description display_tactile.launch.py use_gui:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    description_pkg = get_package_share_directory('inspire_hand_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'rh56e2_right.urdf')

    # Declare arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui (true) or regular joint_state_publisher (false)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str),
            'use_sim_time': False,
        }]
    )

    # Joint state publisher (with or without GUI)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
        parameters=[{'use_gui': False}]
    )

    # Tactile point cloud node (listening to /joint_states)
    tactile_pointcloud_node = Node(
        package='inspire_hand_ros2',
        executable='tactile_pointcloud_node.py',
        name='tactile_pointcloud_node',
        output='screen',
        parameters=[{
            'urdf_path': urdf_file,
            'use_hand_state': False,  # Use standard JointState
            'joint_state_topic': '/joint_states',
            'tactile_topic': '/inspire_hand/inspire_hand_node/touch',
            'publish_rate': 30.0,
            'frame_id': 'base_footprint',
        }]
    )

    # RViz
    rviz_config_file = os.path.join(description_pkg, 'rviz', 'tactile_pointcloud.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        use_gui_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        tactile_pointcloud_node,
        rviz_node,
    ])
