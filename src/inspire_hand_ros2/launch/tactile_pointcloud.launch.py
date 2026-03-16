"""
Launch file for tactile point cloud visualization with real hand.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    description_pkg = get_package_share_directory('inspire_hand_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'rh56e2_right.urdf')

    # Get URDF path from inspire_hand_description package
    urdf_path = PathJoinSubstitution([
        FindPackageShare('inspire_hand_description'),
        'urdf',
        'rh56e2_right.urdf'
    ])

    package_path = FindPackageShare('inspire_hand_description')

    # Declare arguments
    use_hand_state_arg = DeclareLaunchArgument(
        'use_hand_state',
        default_value='true',
        description='Use InspireHandState (true) or standard JointState (false)'
    )

    tactile_topic_arg = DeclareLaunchArgument(
        'tactile_topic',
        default_value='/inspire_hand/inspire_hand_node/touch',
        description='Topic for tactile data (default for real hardware)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publishing rate in Hz'
    )

    points_per_taxel_arg = DeclareLaunchArgument(
        'points_per_taxel',
        default_value='16',
        description='Points to sample per tactile sensor'
    )

    # Configuration file path
    config_file = PathJoinSubstitution([
        FindPackageShare('inspire_hand_ros2'),
        'config',
        'tactile_pointcloud.yaml'
    ])

    # Robot state publisher - publishes /robot_description and TF transforms
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

    # Tactile point cloud node
    tactile_pointcloud_node = Node(
        package='inspire_hand_ros2',
        executable='tactile_pointcloud_node.py',
        name='tactile_pointcloud_node',
        output='screen',
        parameters=[
            config_file,
            {
                'urdf_path': urdf_path,
                'package_path': package_path,
                'use_hand_state': LaunchConfiguration('use_hand_state'),
                'tactile_topic': LaunchConfiguration('tactile_topic'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'points_per_taxel': LaunchConfiguration('points_per_taxel'),
            }
        ],
    )

    # RViz config path
    rviz_config = PathJoinSubstitution([
        FindPackageShare('inspire_hand_description'),
        'rviz',
        'tactile_pointcloud.rviz'
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        use_hand_state_arg,
        tactile_topic_arg,
        publish_rate_arg,
        points_per_taxel_arg,
        robot_state_publisher,
        tactile_pointcloud_node,
        rviz_node,
    ])
