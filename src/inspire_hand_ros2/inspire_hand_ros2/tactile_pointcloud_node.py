#!/usr/bin/env python3
"""
Tactile Point Cloud Node for Inspire Hand.

Publishes a colored point cloud representing the hand's 3D geometry
with tactile sensor values mapped to colors.

Supports:
- Real hand joint states (InspireHandState) or standard JointState
- Configurable tactile topic
- Launch from description package
"""

import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

# Try to import custom messages (may not be built yet)
try:
    from inspire_hand_ros2.msg import InspireHandState, InspireHandTouch
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: inspire_hand_ros2 messages not available. Will only support standard JointState.")

# Import our modules
try:
    from . import mesh_sampler, kinematics_solver, tactile_mapper
except ImportError:
    # Fallback for direct execution
    import mesh_sampler
    import kinematics_solver
    import tactile_mapper


class TactilePointCloudNode(Node):
    """ROS2 node for tactile-mapped point cloud visualization."""

    def __init__(self):
        super().__init__('tactile_pointcloud_node')

        # Declare parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('package_path', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('colormap_min', 0.0)
        self.declare_parameter('colormap_max', 4095.0)  # 12-bit ADC range
        self.declare_parameter('points_per_taxel', 16)  # 16 points per tactile sensor
        self.declare_parameter('tactile_topic', '/inspire_hand/inspire_hand_node/touch')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('hand_state_topic', '/inspire_hand/inspire_hand_node/state')
        self.declare_parameter('use_hand_state', True)  # If False, use standard JointState
        self.declare_parameter('frame_id', 'base_footprint')

        # Get parameters
        urdf_path = self.get_parameter('urdf_path').value
        package_path = self.get_parameter('package_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        colormap_min = self.get_parameter('colormap_min').value
        colormap_max = self.get_parameter('colormap_max').value
        points_per_taxel = self.get_parameter('points_per_taxel').value
        tactile_topic = self.get_parameter('tactile_topic').value
        joint_state_topic = self.get_parameter('joint_state_topic').value
        hand_state_topic = self.get_parameter('hand_state_topic').value
        self.use_hand_state = self.get_parameter('use_hand_state').value
        self.frame_id = self.get_parameter('frame_id').value

        # Auto-detect URDF path if not provided
        if not urdf_path:
            urdf_path = self._find_urdf_path()

        self.get_logger().info(f"Using URDF: {urdf_path}")

        # Initialize components
        self.get_logger().info(f"Loading and sampling meshes ({points_per_taxel} points/taxel)...")
        self.mesh_sampler = mesh_sampler.MeshSampler(
            urdf_path, package_path, points_per_taxel=points_per_taxel
        )
        self.mesh_cache = self.mesh_sampler.load_and_sample_meshes()

        self.get_logger().info("Initializing kinematics solver...")
        self.kinematics_solver = kinematics_solver.KinematicsSolver(urdf_path)

        self.get_logger().info("Initializing tactile mapper...")
        self.tactile_mapper = tactile_mapper.TactileMapper(
            self.mesh_cache,
            colormap_min=colormap_min,
            colormap_max=colormap_max
        )

        # State variables
        self.lock = threading.Lock()
        self.latest_joint_angles = np.zeros(6)  # 6 DOF
        self.latest_tactile_data = {}
        self.last_update_time = self.get_clock().now()

        # Subscribers
        if self.use_hand_state and CUSTOM_MSGS_AVAILABLE:
            self.get_logger().info(f"Subscribing to InspireHandState: {hand_state_topic}")
            self.state_sub = self.create_subscription(
                InspireHandState,
                hand_state_topic,
                self.hand_state_callback,
                10
            )
            self.get_logger().info(f"Subscribing to InspireHandTouch: {tactile_topic}")
            self.touch_sub = self.create_subscription(
                InspireHandTouch,
                tactile_topic,
                self.touch_callback,
                10
            )
        else:
            self.get_logger().info(f"Subscribing to standard JointState: {joint_state_topic}")
            self.joint_state_sub = self.create_subscription(
                JointState,
                joint_state_topic,
                self.joint_state_callback,
                10
            )
            # For standard mode, tactile topic should publish generic sensor data
            self.get_logger().info(f"Subscribing to InspireHandTouch: {tactile_topic}")
            self.touch_sub = self.create_subscription(
                InspireHandTouch,
                tactile_topic,
                self.touch_callback,
                10
            )

        # Publishers
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/inspire_hand/tactile_pointcloud',
            10
        )
        self.js_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self.get_logger().info(f"Tactile point cloud node initialized. Publishing at {self.publish_rate} Hz")

    def _find_urdf_path(self):
        """Auto-detect URDF path from package structure."""
        # Try to find inspire_hand_description package
        possible_paths = [
            '/home/analog/rh56dfx-and-rh56e2/inspire_hand_ws/src/inspire_hand_description/urdf/rh56e2_right.urdf',
            os.path.expanduser('~/rh56dfx-and-rh56e2/inspire_hand_ws/src/inspire_hand_description/urdf/rh56e2_right.urdf'),
        ]

        for path in possible_paths:
            if os.path.exists(path):
                return path

        raise FileNotFoundError("Could not find URDF file. Please specify urdf_path parameter.")

    def hand_state_callback(self, msg):
        """Callback for InspireHandState message."""
        with self.lock:
            self.latest_joint_angles = np.array(msg.angle_actual[:6], dtype=np.float32)
            self.last_update_time = self.get_clock().now()

    def touch_callback(self, msg):
        """Callback for InspireHandTouch message."""
        with self.lock:
            self.latest_tactile_data = self.tactile_mapper.parse_tactile_message(msg)

    def joint_state_callback(self, msg):
        """Callback for standard JointState message."""
        with self.lock:
            # Map joint names to DOF indices
            joint_angles = {}
            for i, name in enumerate(msg.name):
                if name in self.kinematics_solver.DOF_TO_JOINT.values():
                    joint_angles[name] = msg.position[i]

            # Convert to angle_actual format (radians to raw value)
            # This is inverse of kinematics_solver.angle_actual_to_radians
            # Uses per-joint limits and zero offsets
            for dof_idx, joint_name in self.kinematics_solver.DOF_TO_JOINT.items():
                if joint_name in joint_angles:
                    radians = joint_angles[joint_name]
                    upper_limit = self.kinematics_solver.DOF_JOINT_LIMITS.get(dof_idx, np.pi / 2.0)
                    zero_offset = self.kinematics_solver.DOF_ZERO_OFFSET.get(dof_idx, 1000)
                    angle_actual = int(zero_offset - (radians / upper_limit) * zero_offset)
                    self.latest_joint_angles[dof_idx] = angle_actual

            self.last_update_time = self.get_clock().now()

    def publish_callback(self):
        """Timer callback to publish point cloud and joint states."""
        with self.lock:
            joint_angles = self.latest_joint_angles.copy()
            tactile_data = self.latest_tactile_data.copy()

        # Compute forward kinematics
        link_transforms = self.kinematics_solver.compute_all_transforms(
            angle_actual=joint_angles
        )

        # Apply tactile colors - now returns tuple
        link_colors, link_intensities = self.tactile_mapper.apply_tactile_colors(tactile_data)

        # Build point cloud
        points = []
        colors = []
        intensities = []

        for link_name, mesh_points in self.mesh_cache.items():
            if link_name not in link_transforms:
                continue

            # Transform points to world frame
            transform = link_transforms[link_name]
            points_local = mesh_points.points_local
            points_homogeneous = np.hstack([points_local, np.ones((len(points_local), 1))])
            points_world = (transform @ points_homogeneous.T).T[:, :3]

            points.append(points_world)

            # Get colors
            if link_name in link_colors:
                colors.append(link_colors[link_name])
            else:
                # Default gray
                colors.append(np.full(len(points_local), 0x808080, dtype=np.uint32))

            # Get intensities
            if link_name in link_intensities:
                intensities.append(link_intensities[link_name])
            else:
                intensities.append(np.full(len(points_local), np.nan, dtype=np.float32))

        # Concatenate all points and colors
        if len(points) == 0:
            self.get_logger().warn("No points to publish")
            return

        all_points = np.vstack(points).astype(np.float32)
        all_colors = np.concatenate(colors).astype(np.uint32)
        all_intensities = np.concatenate(intensities).astype(np.float32)

        # Create PointCloud2 message
        pc_msg = self.create_pointcloud2(all_points, all_colors, all_intensities)
        self.pc_pub.publish(pc_msg)

        # Publish joint states for RViz robot model
        self.publish_joint_states(joint_angles)

    def create_pointcloud2(self, points: np.ndarray, colors: np.ndarray,
                           intensities: np.ndarray) -> PointCloud2:
        """Create PointCloud2 message with RGB colors and intensity."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        # Define fields - add intensity
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack data
        num_points = len(points)
        point_step = 20  # 4 bytes * 5 fields
        row_step = point_step * num_points

        # Create structured array - add intensity field
        data = np.zeros(num_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32),
            ('intensity', np.float32),
        ])

        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        data['rgb'] = colors
        data['intensity'] = intensities

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = num_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = row_step
        msg.data = data.tobytes()
        msg.is_dense = True

        return msg

    def publish_joint_states(self, angle_actual: np.ndarray):
        """Publish joint states for RViz visualization."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Convert angle_actual to radians using per-joint limits
        joint_positions = {}
        for dof_idx, joint_name in self.kinematics_solver.DOF_TO_JOINT.items():
            if dof_idx < len(angle_actual):
                joint_positions[joint_name] = self.kinematics_solver.angle_actual_to_radians(
                    angle_actual[dof_idx], dof_idx
                )

        # Add mimic joints
        joint_positions = self.kinematics_solver._resolve_mimic_joints(joint_positions)

        # Populate message
        msg.name = list(joint_positions.keys())
        msg.position = [float(v) for v in joint_positions.values()]  # Convert numpy floats to Python floats

        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TactilePointCloudNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
