#!/usr/bin/env python3
"""
Cylinder Unwrap Node for Inspire Hand Tactile Visualization.

Creates a 2D "unwrapped" image from cylinder-projected tactile point cloud.
The cylinder is cut at 180° (opposite palm sensor) and laid flat.

Image axes:
- X-axis (columns): Angle around cylinder (0° at palm sensor, wraps at 180°/-180°)
- Y-axis (rows): Height along cylinder axis
- Pixel value: Tactile intensity (0-4095) or NaN for no contact
"""

import numpy as np
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import tf2_ros

# Define colormaps without cv2 dependency
COLORMAP_JET = 2
COLORMAP_HOT = 11
COLORMAP_VIRIDIS = 20


def apply_colormap_jet(normalized):
    """
    Apply JET colormap to normalized values (0-255).

    JET: Blue (0) -> Cyan -> Green -> Yellow -> Red (255)

    Args:
        normalized: numpy array with values 0-255 (uint8)

    Returns:
        RGB image (height, width, 3) uint8
    """
    height, width = normalized.shape
    rgb = np.zeros((height, width, 3), dtype=np.uint8)

    # Normalize to 0-1
    norm = normalized.astype(np.float32) / 255.0

    # JET colormap (same as tactile_mapper)
    for i in range(height):
        for j in range(width):
            val = norm[i, j]

            if val < 0.25:
                # Blue to Cyan
                t = val / 0.25
                r, g, b = 0, int(255 * t), 255
            elif val < 0.5:
                # Cyan to Green
                t = (val - 0.25) / 0.25
                r, g, b = 0, 255, int(255 * (1 - t))
            elif val < 0.75:
                # Green to Yellow
                t = (val - 0.5) / 0.25
                r, g, b = int(255 * t), 255, 0
            else:
                # Yellow to Red
                t = (val - 0.75) / 0.25
                r, g, b = 255, int(255 * (1 - t)), 0

            rgb[i, j] = [r, g, b]

    return rgb


def numpy_to_imgmsg(img, encoding):
    """
    Convert numpy array to ROS Image message.

    Args:
        img: numpy array
        encoding: image encoding ('32FC1', 'rgb8', etc.)

    Returns:
        sensor_msgs/Image message
    """
    msg = Image()

    if encoding == '32FC1':
        # Single channel float32
        msg.height, msg.width = img.shape
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width * 4  # 4 bytes per float32
        msg.data = img.astype(np.float32).tobytes()
    elif encoding == 'rgb8':
        # 3 channel uint8
        msg.height, msg.width = img.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width * 3  # 3 bytes per pixel
        msg.data = img.astype(np.uint8).tobytes()
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    return msg


class CylinderUnwrapNode(Node):
    """ROS2 node for unwrapping cylinder projection to 2D image."""

    def __init__(self):
        super().__init__('cylinder_unwrap_node')

        # Declare parameters
        self.declare_parameter('input_topic', '/cylinder_projection/projected_pointcloud')
        self.declare_parameter('output_topic', '/cylinder_projection/unwrapped_image')
        self.declare_parameter('output_colormap_topic', '/cylinder_projection/unwrapped_colormap')
        self.declare_parameter('image_width', 360)  # pixels (1 pixel per degree)
        self.declare_parameter('image_height', 200)  # pixels
        self.declare_parameter('cylinder_frame', 'right_palm_force_sensor')
        self.declare_parameter('cylinder_radius', 0.02)  # meters (must match cylinder_projection)
        self.declare_parameter('height_min', -0.075)  # meters (symmetric around palm)
        self.declare_parameter('height_max', 0.075)   # meters
        self.declare_parameter('colormap', COLORMAP_JET)  # Colormap selection
        self.declare_parameter('intensity_min', 0.0)
        self.declare_parameter('intensity_max', 4095.0)

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_colormap_topic = self.get_parameter('output_colormap_topic').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.cylinder_frame = self.get_parameter('cylinder_frame').value
        self.cylinder_radius = self.get_parameter('cylinder_radius').value
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value
        self.colormap = self.get_parameter('colormap').value
        self.intensity_min = self.get_parameter('intensity_min').value
        self.intensity_max = self.get_parameter('intensity_max').value

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        self.pc_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.img_pub = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        self.colormap_pub = self.create_publisher(
            Image,
            self.output_colormap_topic,
            10
        )

        self.get_logger().info(
            f"Cylinder unwrap node initialized:\n"
            f"  Input: {input_topic}\n"
            f"  Output (raw): {self.output_topic}\n"
            f"  Output (colormap): {self.output_colormap_topic}\n"
            f"  Image size: {self.image_width}x{self.image_height}\n"
            f"  Height range: [{self.height_min:.3f}, {self.height_max:.3f}] m\n"
            f"  Intensity range: [{self.intensity_min:.1f}, {self.intensity_max:.1f}]"
        )

    def parse_pointcloud2(self, msg):
        """
        Parse PointCloud2 message into points and intensities.

        Args:
            msg: PointCloud2 message

        Returns:
            tuple: (points, intensities) as numpy arrays, or (None, None) if parsing fails
        """
        # Check for required fields
        field_names = [field.name for field in msg.fields]
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            self.get_logger().error("PointCloud2 missing xyz fields")
            return None, None

        has_intensity = 'intensity' in field_names

        # Parse using numpy structured array
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

        # Extract intensities
        if has_intensity:
            intensities = data_array['intensity'].copy()
        else:
            # Default NaN if no intensity
            intensities = np.full(len(data_array), np.nan, dtype=np.float32)

        return points, intensities

    def get_transform_matrix(self, source_frame):
        """Get transform matrix from source frame to cylinder frame."""
        try:
            # Lookup transform
            transform = self.tf_buffer.lookup_transform(
                self.cylinder_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Convert to 4x4 matrix
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Build rotation matrix from quaternion
            x, y, z, w = rot.x, rot.y, rot.z, rot.w
            R = np.array([
                [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
                [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
                [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
            ])

            # Build 4x4 homogeneous transform
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [trans.x, trans.y, trans.z]

            return T

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def pointcloud_callback(self, msg):
        """
        Callback for projected point cloud.

        Creates unwrapped 2D image from cylinder coordinates.
        """
        # Parse point cloud
        points, intensities = self.parse_pointcloud2(msg)

        if points is None or len(points) == 0:
            return

        # Get transform from message frame to cylinder frame
        transform_matrix = self.get_transform_matrix(msg.header.frame_id)
        if transform_matrix is None:
            return

        # Transform points to cylinder frame
        points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
        points_cylinder = (transform_matrix @ points_homogeneous.T).T[:, :3]

        # Extract cylindrical coordinates in cylinder frame
        # Cylinder axis: X-axis (height direction)
        # Cylinder center: offset by radius in +Z direction
        # Palm contact edge: at Z=0
        height = points_cylinder[:, 0]  # X-coordinate is height along cylinder
        y = points_cylinder[:, 1]
        z = points_cylinder[:, 2]

        # Calculate angle in YZ plane
        # Cylinder center is at (any_x, 0, radius) in cylinder frame
        # Radial vector from center: (y, z - radius)
        # Angle: atan2(z - radius, y)
        #   - Palm contact (y=0, z=0): atan2(-radius, 0) = -90°
        #   - Top (y=0, z=2*radius): atan2(radius, 0) = 90°
        #   - Right (y=radius, z=radius): atan2(0, radius) = 0°
        #   - Left (y=-radius, z=radius): atan2(0, -radius) = 180°
        #
        # We want 0° at palm contact and 180° at cut line (opposite side)
        # So add 90° to shift: palm contact becomes 0°, top becomes 180°
        angles = np.arctan2(z - self.cylinder_radius, y)  # Range: [-π, π]
        angles_shifted = angles + np.pi / 2  # Shift by +90°

        # Convert to 0-360° range
        angles_deg = np.degrees(angles_shifted) % 360.0  # Range: [0, 360]

        # Shift angles so that:
        # - 0° (palm contact) → middle of image
        # - 180° (cut line) → edges (index 0 and width-1)
        # Mapping: θ_unwrapped = (θ + 180) % 360
        #   180° → 0° → index 0
        #   270° → 90° → index width/4
        #   0° → 180° → index width/2 (middle)
        #   90° → 270° → index 3*width/4
        #   180° → 360° → index width-1
        angles_unwrapped = (angles_deg + 180.0) % 360.0  # Range: [0, 360]

        # Create 2D image
        # Rows = height bins, Columns = angle bins
        image = np.full((self.image_height, self.image_width), np.nan, dtype=np.float32)
        count = np.zeros((self.image_height, self.image_width), dtype=np.int32)

        # Map points to image pixels
        # Height bins
        height_range = self.height_max - self.height_min
        row_indices = ((height - self.height_min) / height_range * self.image_height).astype(int)
        row_indices = np.clip(row_indices, 0, self.image_height - 1)

        # Angle bins
        col_indices = (angles_unwrapped / 360.0 * self.image_width).astype(int)
        col_indices = np.clip(col_indices, 0, self.image_width - 1)

        # Accumulate intensity values (average if multiple points map to same pixel)
        for i in range(len(points)):
            if not np.isnan(intensities[i]):
                r, c = row_indices[i], col_indices[i]
                if np.isnan(image[r, c]):
                    image[r, c] = intensities[i]
                    count[r, c] = 1
                else:
                    image[r, c] += intensities[i]
                    count[r, c] += 1

        # Average accumulated values
        mask = count > 0
        image[mask] /= count[mask]

        # Flip vertically so that top of image = top of cylinder (positive height)
        image = np.flipud(image)

        # Publish raw intensity image
        self.publish_intensity_image(image, msg.header.stamp)

        # Publish colormap image
        self.publish_colormap_image(image, msg.header.stamp)

    def publish_intensity_image(self, image, timestamp):
        """Publish raw intensity image (32FC1)."""
        # Convert to ROS Image message (32-bit float, single channel)
        msg = numpy_to_imgmsg(image, encoding='32FC1')
        msg.header.stamp = timestamp
        msg.header.frame_id = self.cylinder_frame

        self.img_pub.publish(msg)

    def publish_colormap_image(self, image, timestamp):
        """Publish colormap visualization (8UC3 RGB)."""
        # Normalize to 0-255 range
        # Handle NaN values
        valid_mask = ~np.isnan(image)

        if not np.any(valid_mask):
            # No valid data - publish blank image
            colormap_img_rgb = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        else:
            # Normalize valid values
            normalized = np.zeros_like(image, dtype=np.uint8)
            valid_values = image[valid_mask]

            # Clip to intensity range
            valid_values = np.clip(valid_values, self.intensity_min, self.intensity_max)

            # Normalize to 0-255
            normalized_values = ((valid_values - self.intensity_min) /
                                 (self.intensity_max - self.intensity_min) * 255).astype(np.uint8)

            normalized[valid_mask] = normalized_values

            # Apply colormap (JET)
            colormap_img_rgb = apply_colormap_jet(normalized)

            # Set NaN pixels to black
            colormap_img_rgb[~valid_mask] = [0, 0, 0]

        # Publish
        msg = numpy_to_imgmsg(colormap_img_rgb, encoding='rgb8')
        msg.header.stamp = timestamp
        msg.header.frame_id = self.cylinder_frame

        self.colormap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderUnwrapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
