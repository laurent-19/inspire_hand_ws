"""
Launch file for tactile point cloud visualization with real hand.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    # Configuration file path
    config_file = PathJoinSubstitution([
        FindPackageShare('inspire_hand_ros2'),
        'config',
        'tactile_pointcloud.yaml'
    ])

    # Tactile point cloud node
    tactile_pointcloud_node = Node(
        package='inspire_hand_ros2',
        executable='tactile_pointcloud_node.py',
        name='tactile_pointcloud_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_hand_state': LaunchConfiguration('use_hand_state'),
                'tactile_topic': LaunchConfiguration('tactile_topic'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
    )

    return LaunchDescription([
        use_hand_state_arg,
        tactile_topic_arg,
        publish_rate_arg,
        tactile_pointcloud_node,
    ])
