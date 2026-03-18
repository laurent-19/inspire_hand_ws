#!/usr/bin/env python3
"""
Cylinder Projection Node for Inspire Hand Tactile Visualization.

Projects tactile point cloud onto a virtual cylinder surface. The cylinder center is
offset by one radius from the palm force sensor origin, so the cylinder edge touches
the palm sensor. The cylinder axis aligns with the palm's X-axis (finger direction),
and points are projected radially in the YZ plane.

This enables grasp pattern visualization for cylindrical objects.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import tf2_ros


class CylinderProjectionNode(Node):
    """ROS2 node for projecting tactile point cloud onto cylinder surface."""

    def __init__(self):
        super().__init__('cylinder_projection_node')

        # Declare parameters
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cylinder_frame', 'right_palm_force_sensor')
        self.declare_parameter('cylinder_radius', 0.02)  # meters
        self.declare_parameter('cylinder_height', 0.15)  # meters
        self.declare_parameter('cylinder_offset', 0.0)  # meters (additional offset beyond radius)
        self.declare_parameter('max_distance', 0.10)  # filter distant points
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('publish_reference', True)
        self.declare_parameter('input_topic', '/inspire_hand/tactile_pointcloud')
        self.declare_parameter('output_topic', '/cylinder_projection/projected_pointcloud')
        self.declare_parameter('reference_topic', '/cylinder_projection/reference_cylinder')
        self.declare_parameter('reference_points', 1000)

        # Get parameters
        self.base_frame = self.get_parameter('base_frame').value
        self.cylinder_frame = self.get_parameter('cylinder_frame').value
        self.cylinder_radius = self.get_parameter('cylinder_radius').value
        self.cylinder_height = self.get_parameter('cylinder_height').value
        self.cylinder_offset = self.get_parameter('cylinder_offset').value
        self.max_distance = self.get_parameter('max_distance').value
        self.publish_rate_value = self.get_parameter('publish_rate').value
        self.publish_reference_flag = self.get_parameter('publish_reference').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.reference_topic = self.get_parameter('reference_topic').value
        self.reference_points_count = self.get_parameter('reference_points').value

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.proj_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )

        if self.publish_reference_flag:
            self.ref_pub = self.create_publisher(
                PointCloud2,
                self.reference_topic,
                10
            )
            # Generate reference cylinder once
            self.reference_cylinder_points, self.reference_cylinder_colors = self.generate_reference_cylinder()

            # Timer for publishing reference cylinder
            ref_timer_period = 1.0  # Publish reference at 1 Hz
            self.ref_timer = self.create_timer(ref_timer_period, self.publish_reference_cylinder)

        # Subscriber to tactile point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        self.get_logger().info(
            f"Cylinder projection node initialized:\n"
            f"  Cylinder: radius={self.cylinder_radius}m, height={self.cylinder_height}m\n"
            f"  Offset: {self.cylinder_offset}m (edge at palm sensor when offset=0)\n"
            f"  Frame: {self.cylinder_frame} (axis along X, positioned in +Z)\n"
            f"  Input: {self.input_topic}\n"
            f"  Output: {self.output_topic}"
        )

    def generate_reference_cylinder(self):
        """
        Generate point cloud representing the cylinder surface.

        Returns:
            tuple: (points, colors) as numpy arrays
        """
        num_points = self.reference_points_count

        # Parametric sampling: uniform in theta and x
        # Calculate samples for roughly uniform density
        num_circle = int(np.sqrt(num_points / self.cylinder_height * 2 * np.pi * self.cylinder_radius))
        num_height = max(num_points // num_circle, 2)

        # Generate parametric coordinates
        theta = np.linspace(0, 2*np.pi, num_circle, endpoint=False)
        x = np.linspace(-self.cylinder_height/2, self.cylinder_height/2, num_height)

        # Create mesh grid
        theta_grid, x_grid = np.meshgrid(theta, x)
        theta_flat = theta_grid.flatten()
        x_flat = x_grid.flatten()

        # Cylinder surface (axis along X)
        # In palm_force_sensor frame: X = along fingers, Y = left-right, Z = up-down
        # Position cylinder so edge is at origin when offset=0
        # Total offset = radius + additional offset
        y = self.cylinder_radius * np.cos(theta_flat)
        z = self.cylinder_radius * np.sin(theta_flat) + self.cylinder_radius + self.cylinder_offset

        points = np.column_stack([x_flat, y, z]).astype(np.float32)

        # Gray color (RGB: 128, 128, 128)
        gray = 0x808080
        colors = np.full(len(points), gray, dtype=np.uint32)

        self.get_logger().info(f"Generated reference cylinder: {len(points)} points")

        return points, colors

    def publish_reference_cylinder(self):
        """Publish the reference cylinder point cloud in cylinder frame."""
        if not self.publish_reference_flag:
            return

        # Create PointCloud2 message in cylinder frame
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.cylinder_frame

        # Reference cylinder has no tactile data - use NaN intensities
        intensities = np.full(len(self.reference_cylinder_points), np.nan, dtype=np.float32)

        msg = self.create_pointcloud2_msg(
            self.reference_cylinder_points,
            self.reference_cylinder_colors,
            intensities,
            header
        )

        self.ref_pub.publish(msg)

    def get_transform(self):
        """
        Get transform from base_frame to cylinder_frame.

        Returns:
            np.ndarray: 4x4 transformation matrix, or None if lookup fails
        """
        try:
            # Lookup transform
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.cylinder_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Convert to 4x4 matrix
            t = transform_stamped.transform.translation
            r = transform_stamped.transform.rotation

            # Rotation quaternion to matrix
            # Using formula: R = I + 2*[q_v]_x^2 + 2*w*[q_v]_x
            # where q = [w, x, y, z] and [q_v]_x is skew-symmetric matrix
            qw, qx, qy, qz = r.w, r.x, r.y, r.z

            # Rotation matrix from quaternion
            rot_matrix = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
            ])

            # Build 4x4 homogeneous transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rot_matrix
            transform_matrix[:3, 3] = [t.x, t.y, t.z]

            return transform_matrix

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=1.0)
            return None

    def parse_pointcloud2(self, msg):
        """
        Parse PointCloud2 message into points, colors, and intensities.

        Args:
            msg: PointCloud2 message

        Returns:
            tuple: (points, colors, intensities) as numpy arrays, or (None, None, None) if parsing fails
        """
        # Check for required fields
        field_names = [field.name for field in msg.fields]
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            self.get_logger().error("PointCloud2 missing xyz fields")
            return None, None, None

        has_rgb = 'rgb' in field_names
        has_intensity = 'intensity' in field_names

        # Parse using numpy
        # Create dtype for structured array
        dtype_list = []
        for field in msg.fields:
            if field.name in ['x', 'y', 'z']:
                dtype_list.append((field.name, np.float32))
            elif field.name == 'rgb':
                dtype_list.append((field.name, np.uint32))
            elif field.name == 'intensity':
                dtype_list.append((field.name, np.float32))

        # Convert bytes to structured array
        data_array = np.frombuffer(msg.data, dtype=dtype_list)

        # Extract points
        points = np.zeros((len(data_array), 3), dtype=np.float32)
        points[:, 0] = data_array['x']
        points[:, 1] = data_array['y']
        points[:, 2] = data_array['z']

        # Extract colors
        if has_rgb:
            colors = data_array['rgb'].copy()
        else:
            # Default gray if no color
            colors = np.full(len(data_array), 0x808080, dtype=np.uint32)

        # Extract intensities
        if has_intensity:
            intensities = data_array['intensity'].copy()
        else:
            # Default NaN if no intensity
            intensities = np.full(len(data_array), np.nan, dtype=np.float32)

        return points, colors, intensities

    def project_pointcloud(self, points, transform_matrix):
        """
        Project points onto cylinder surface.

        Cylinder axis: X-axis of cylinder_frame (palm forward direction)
        Radial projection: In YZ plane

        Args:
            points: Nx3 array of points in base_frame
            transform_matrix: 4x4 transform from base to cylinder frame

        Returns:
            tuple: (projected_points, valid_mask) where projected_points is Nx3 array
                   and valid_mask is boolean array indicating which input points were kept
        """
        if len(points) == 0:
            return points, np.ones(0, dtype=bool)

        N = len(points)

        # Inverse transform to go from base to cylinder frame
        T_palm_to_base = np.linalg.inv(transform_matrix)

        # Transform points to cylinder frame
        points_homo = np.hstack([points, np.ones((N, 1))])
        points_palm = (T_palm_to_base @ points_homo.T).T[:, :3]

        # Extract coordinates
        x = points_palm[:, 0]  # Along cylinder axis (preserve)
        y = points_palm[:, 1]  # Radial component
        z = points_palm[:, 2]  # Radial component

        # Adjust z coordinate for cylinder offset
        # Cylinder center is at z = radius + offset (so edge is at origin when offset=0)
        z_centered = z - (self.cylinder_radius + self.cylinder_offset)

        # Cylindrical coordinates in YZ plane (relative to cylinder center)
        r = np.sqrt(y**2 + z_centered**2)

        # Handle points on axis (r ≈ 0)
        theta = np.arctan2(z_centered, y)
        # For points very close to axis, theta is undefined but we'll use default
        # The projection will map them to arbitrary point on cylinder

        # Filter points based on radial distance and axial position
        valid_mask = np.ones(N, dtype=bool)

        # Filter by radial distance from axis
        if self.max_distance > 0:
            valid_mask &= (r <= self.max_distance)

        # Filter by axial position (X coordinate must be within cylinder height)
        half_height = self.cylinder_height / 2.0
        valid_mask &= (x >= -half_height) & (x <= half_height)

        if not np.any(valid_mask):
            # All points filtered out
            return np.zeros((0, 3), dtype=np.float32), valid_mask

        # Project to cylinder surface (radial projection in YZ plane)
        x_proj = x[valid_mask]
        y_proj = self.cylinder_radius * np.cos(theta[valid_mask])
        z_proj = self.cylinder_radius * np.sin(theta[valid_mask]) + self.cylinder_radius + self.cylinder_offset

        # Build projected points in cylinder frame
        points_proj_palm = np.column_stack([x_proj, y_proj, z_proj, np.ones(np.sum(valid_mask))])

        # Transform back to base frame
        points_proj_base = (transform_matrix @ points_proj_palm.T).T[:, :3]

        return points_proj_base.astype(np.float32), valid_mask

    def create_pointcloud2_msg(self, points, colors, intensities, header):
        """
        Create PointCloud2 message with RGB colors and intensity.

        Args:
            points: Nx3 numpy array of points
            colors: N numpy array of uint32 RGB colors
            intensities: N numpy array of float32 intensity values
            header: Header message

        Returns:
            PointCloud2 message
        """
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

        # Create message
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

    def pointcloud_callback(self, msg):
        """
        Callback for input tactile point cloud.

        Projects points onto cylinder and publishes result.
        """
        # Parse input point cloud
        points, colors, intensities = self.parse_pointcloud2(msg)

        if points is None or len(points) == 0:
            return

        # Get transform
        transform = self.get_transform()
        if transform is None:
            # TF not available, skip this frame
            return

        # Project points onto cylinder (returns projected points and filter mask)
        projected_points, valid_mask = self.project_pointcloud(points, transform)

        if len(projected_points) == 0:
            # All points filtered out
            return

        # Filter colors and intensities using the same mask
        colors_filtered = colors[valid_mask]
        intensities_filtered = intensities[valid_mask]

        # Create output message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame

        output_msg = self.create_pointcloud2_msg(projected_points, colors_filtered,
                                                  intensities_filtered, header)

        # Publish
        self.proj_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderProjectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
