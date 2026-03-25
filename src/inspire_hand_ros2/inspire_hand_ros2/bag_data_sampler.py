#!/usr/bin/env python3
"""
Synchronized multi-modal data sampler for ROS2 bags.
Saves camera images, tactile heatmaps, point clouds, and hand/joint states.

Uses a caching strategy to handle topics with misaligned timestamps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, JointState, CameraInfo
from sensor_msgs_py import point_cloud2
from inspire_hand_ros2.msg import InspireHandState
import numpy as np
import cv2
import json
import os
import open3d as o3d
from cv_bridge import CvBridge


class BagDataSampler(Node):
    def __init__(self):
        super().__init__('bag_data_sampler')

        # Declare parameters
        self.declare_parameter('output_dir', 'training_data')
        self.declare_parameter('category', 'non_deformable')
        self.declare_parameter('bag_name', 'unnamed')
        self.declare_parameter('sample_interval', 0.5)

        self.output_dir = self.get_parameter('output_dir').value
        self.category = self.get_parameter('category').value
        self.bag_name = self.get_parameter('bag_name').value
        self.sample_interval = self.get_parameter('sample_interval').value

        # Create output directory
        self.sample_dir = os.path.join(self.output_dir, self.category, self.bag_name)
        os.makedirs(self.sample_dir, exist_ok=True)

        self.sample_count = 0
        self.last_sample_stamp = None

        # Cache for latest messages
        self.cache = {
            'camera': None,
            'tactile_colormap': None,
            'hand_state': None,
            'tactile_pcd': None,
            'joint_state': None,
            'depth_image': None,
        }

        # Camera intrinsics (will be set from CameraInfo topic)
        self.camera_intrinsic = None

        # GPU device for CUDA-accelerated point cloud processing (same as bag_to_pcd.py)
        self.bridge = CvBridge()
        self.device = o3d.core.Device("CUDA:0")
        self.voxel_size = 0.01  # 1cm voxels for GPU reduction

        # QoS for bag playback compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # More permissive QoS for depth (may have different publisher settings)
        qos_depth = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_ALL,
            depth=100
        )

        # Create independent subscribers
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.camera_callback, qos_profile)
        self.create_subscription(
            Image, '/cylinder_projection/unwrapped_colormap',
            self.tactile_colormap_callback, qos_profile)
        self.create_subscription(
            InspireHandState, '/inspire_hand/inspire_hand_node/state',
            self.hand_state_callback, qos_profile)
        self.create_subscription(
            PointCloud2, '/inspire_hand/tactile_pointcloud',
            self.tactile_pcd_callback, qos_profile)
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, qos_profile)
        # Use unaligned depth topics (aligned has only 0-2 messages in bags)
        # With more permissive QoS to catch all depth messages
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_callback, qos_depth)
        self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info',
            self.camera_info_callback, qos_depth)

        self.get_logger().info(
            f'BagDataSampler initialized: output={self.sample_dir}, interval={self.sample_interval}s')

    def camera_callback(self, msg):
        self.cache['camera'] = msg

    def tactile_colormap_callback(self, msg):
        self.cache['tactile_colormap'] = msg

    def hand_state_callback(self, msg):
        self.cache['hand_state'] = msg

    def tactile_pcd_callback(self, msg):
        self.cache['tactile_pcd'] = msg

    def depth_callback(self, msg):
        """Cache depth image for point cloud generation."""
        self.cache['depth_image'] = msg
        # Try to save a sample with the newly arrived depth image
        # This ensures we capture samples with camera PCD when depth data arrives
        if self.cache['joint_state'] is not None:
            self.try_save_sample(self.cache['joint_state'].header.stamp, force=True)

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo message."""
        if self.camera_intrinsic is None:
            # K matrix as 3x3 Tensor on GPU: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            self.camera_intrinsic = o3d.core.Tensor(
                [[msg.k[0], 0, msg.k[2]], [0, msg.k[4], msg.k[5]], [0, 0, 1]],
                dtype=o3d.core.Dtype.Float32, device=self.device
            )
            self.depth_image_size = (msg.width, msg.height)
            self.get_logger().info(f'Camera intrinsics loaded: {msg.width}x{msg.height}')

    def joint_state_callback(self, msg):
        """Use joint_state as trigger - save sample when all data is available."""
        self.cache['joint_state'] = msg
        self.try_save_sample(msg.header.stamp)

    def try_save_sample(self, stamp, force=False):
        """Try to save a sample if all data is available and enough time has passed.

        Args:
            stamp: Timestamp for rate limiting
            force: If True, bypass interval check (used when depth image arrives)
        """
        # Check if required data is available (projected_pcd is optional)
        required = ['camera', 'tactile_colormap', 'hand_state', 'tactile_pcd', 'joint_state']
        if any(self.cache[k] is None for k in required):
            return

        # Rate limit using joint_state timestamps (consistent with other topics)
        current_stamp_sec = stamp.sec + stamp.nanosec / 1e9
        if not force and self.last_sample_stamp is not None:
            elapsed = current_stamp_sec - self.last_sample_stamp
            if elapsed < self.sample_interval:
                return

        self.last_sample_stamp = current_stamp_sec

        # Create sample subdirectory
        sample_subdir = os.path.join(self.sample_dir, f'sample_{self.sample_count:04d}')
        os.makedirs(sample_subdir, exist_ok=True)

        # Save all modalities
        self.save_image(self.cache['camera'], os.path.join(sample_subdir, 'camera_rgb.png'))
        self.save_image(self.cache['tactile_colormap'], os.path.join(sample_subdir, 'tactile_colormap.png'))
        self.save_hand_state(self.cache['hand_state'], os.path.join(sample_subdir, 'hand_state.json'))
        self.save_joint_state(self.cache['joint_state'], os.path.join(sample_subdir, 'joint_state.json'))
        self.save_pointcloud(self.cache['tactile_pcd'], os.path.join(sample_subdir, 'tactile_pointcloud.pcd'), filter_grey=True)
        self.save_pointcloud(self.cache['tactile_pcd'], os.path.join(sample_subdir, 'tactile_pointcloud_raw.pcd'), filter_grey=False)
        if self.cache['depth_image'] is not None:
            self.save_depth_pointcloud(self.cache['depth_image'], os.path.join(sample_subdir, 'camera_pointcloud.pcd'))

        self.get_logger().info(f'Saved sample {self.sample_count}')
        self.sample_count += 1

    def ros_image_to_numpy(self, msg: Image) -> np.ndarray | None:
        """Convert a ROS Image message to a BGR numpy array (OpenCV convention)."""
        enc = msg.encoding.lower()
        raw = np.frombuffer(msg.data, dtype=np.uint8)

        # mono / depth
        if enc in ('mono8', '8uc1'):
            return raw.reshape((msg.height, msg.width))

        if enc in ('mono16', '16uc1'):
            img16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            img8 = cv2.normalize(img16, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            return img8

        # colour (3-channel)
        if enc in ('rgb8', '8uc3'):
            img = raw.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        if enc in ('bgr8',):
            return raw.reshape((msg.height, msg.width, 3))

        # colour (4-channel)
        if enc in ('rgba8',):
            img = raw.reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)

        if enc in ('bgra8',):
            return raw.reshape((msg.height, msg.width, 4))

        # 32-bit float (depth maps, etc.)
        if enc in ('32fc1',):
            img32 = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            img32 = np.nan_to_num(img32, nan=0.0, posinf=0.0, neginf=0.0)
            img8 = cv2.normalize(img32, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            return img8

        # Bayer patterns
        bayer_map = {
            'bayer_rggb8': cv2.COLOR_BAYER_BG2BGR,
            'bayer_bggr8': cv2.COLOR_BAYER_RG2BGR,
            'bayer_gbrg8': cv2.COLOR_BAYER_GR2BGR,
            'bayer_grbg8': cv2.COLOR_BAYER_GB2BGR,
        }
        if enc in bayer_map:
            img = raw.reshape((msg.height, msg.width))
            return cv2.cvtColor(img, bayer_map[enc])

        return None

    def save_image(self, msg: Image, filepath: str):
        """Save ROS Image message to PNG file."""
        img = self.ros_image_to_numpy(msg)
        if img is None:
            self.get_logger().warn(f'Unsupported encoding "{msg.encoding}", skipping {filepath}')
            return
        cv2.imwrite(filepath, img)

    def save_hand_state(self, msg: InspireHandState, filepath: str):
        """Save InspireHandState message to JSON file."""
        data = {
            'timestamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec)
            },
            'position_actual': [int(x) for x in msg.position_actual],
            'angle_actual': [int(x) for x in msg.angle_actual],
            'force_actual': [int(x) for x in msg.force_actual],
            'current': [int(x) for x in msg.current],
            'status': [int(x) for x in msg.status],
            'error': [int(x) for x in msg.error],
            'temperature': [int(x) for x in msg.temperature]
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def save_joint_state(self, msg: JointState, filepath: str):
        """Save JointState message to JSON file."""
        data = {
            'timestamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec)
            },
            'name': list(msg.name),
            'position': [float(x) for x in msg.position],
            'velocity': [float(x) for x in msg.velocity],
            'effort': [float(x) for x in msg.effort]
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def save_pointcloud(self, msg: PointCloud2, filepath: str, filter_grey: bool = False):
        """Save PointCloud2 message to PCD file with x,y,z,rgb,intensity.

        Args:
            msg: PointCloud2 message
            filepath: Output file path
            filter_grey: If True, filter out grey points (where R≈G≈B)
        """
        field_names = [f.name for f in msg.fields]
        want = [f for f in ['x', 'y', 'z', 'rgb', 'intensity'] if f in field_names]

        gen = point_cloud2.read_points(msg, field_names=want, skip_nans=True)
        cloud = np.array(list(gen))

        if cloud.size == 0:
            self.get_logger().warn(f'Empty cloud for {filepath}')
            return

        xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(np.float32)

        # RGB
        colors = None
        r_vals = g_vals = b_vals = None
        if 'rgb' in want:
            packed = cloud['rgb'].view(np.uint32)
            r_vals = ((packed >> 16) & 0xFF).astype(np.float64)
            g_vals = ((packed >> 8) & 0xFF).astype(np.float64)
            b_vals = (packed & 0xFF).astype(np.float64)
            colors = np.column_stack([r_vals / 255.0, g_vals / 255.0, b_vals / 255.0])

        # Intensity
        intensity = None
        if 'intensity' in want:
            intensity = cloud['intensity'].astype(np.float32)
            intensity = np.where(np.isfinite(intensity), intensity, -1.0)

        # Filter grey points: where R, G, B are all similar (grey/background)
        if filter_grey and r_vals is not None:
            # Grey threshold: max difference between R,G,B channels < 30
            # and all channels are in grey range (100-200 typically)
            max_diff = np.maximum(np.maximum(np.abs(r_vals - g_vals), np.abs(g_vals - b_vals)), np.abs(r_vals - b_vals))
            avg_val = (r_vals + g_vals + b_vals) / 3.0
            # Keep points that are NOT grey (high color variance OR very dark/bright)
            is_grey = (max_diff < 30) & (avg_val > 80) & (avg_val < 200)
            keep_mask = ~is_grey

            xyz = xyz[keep_mask]
            colors = colors[keep_mask]
            if intensity is not None:
                intensity = intensity[keep_mask]

            self.get_logger().debug(f'Filtered {np.sum(is_grey)} grey points, kept {np.sum(keep_mask)}')

        if len(xyz) == 0:
            self.get_logger().warn(f'All points filtered for {filepath}')
            return

        self.write_pcd(filepath, xyz, colors, intensity)

    def write_pcd(self, filename: str, xyz: np.ndarray,
                  colors: np.ndarray | None, intensity: np.ndarray | None):
        """Write a binary PCD file with x,y,z + optional rgb + optional intensity."""
        n = len(xyz)

        # Build the field layout
        fields = ['x', 'y', 'z']
        sizes = [4, 4, 4]
        types = ['F', 'F', 'F']
        counts = [1, 1, 1]

        if colors is not None:
            fields.append('rgb')
            sizes.append(4)
            types.append('F')
            counts.append(1)

        if intensity is not None:
            fields.append('intensity')
            sizes.append(4)
            types.append('F')
            counts.append(1)

        # Pack binary data
        dtype_fields = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
        ]
        if colors is not None:
            dtype_fields.append(('rgb', np.float32))
        if intensity is not None:
            dtype_fields.append(('intensity', np.float32))

        buf = np.zeros(n, dtype=dtype_fields)
        buf['x'] = xyz[:, 0]
        buf['y'] = xyz[:, 1]
        buf['z'] = xyz[:, 2]

        if colors is not None:
            r = (colors[:, 0] * 255).astype(np.uint32)
            g = (colors[:, 1] * 255).astype(np.uint32)
            b = (colors[:, 2] * 255).astype(np.uint32)
            packed = (r << 16) | (g << 8) | b
            buf['rgb'] = packed.view(np.float32)

        if intensity is not None:
            buf['intensity'] = intensity

        # Write file
        header = (
            f"# .PCD v0.7 - Point Cloud Data\n"
            f"VERSION 0.7\n"
            f"FIELDS {' '.join(fields)}\n"
            f"SIZE {' '.join(str(s) for s in sizes)}\n"
            f"TYPE {' '.join(types)}\n"
            f"COUNT {' '.join(str(c) for c in counts)}\n"
            f"WIDTH {n}\n"
            f"HEIGHT 1\n"
            f"VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\n"
            f"DATA binary\n"
        )

        with open(filename, 'wb') as f:
            f.write(header.encode('ascii'))
            f.write(buf.tobytes())

    def save_depth_pointcloud(self, msg: Image, filepath: str):
        """Convert depth image to point cloud and save as PCD.

        Uses exact algorithm from bag_to_pcd.py with CUDA acceleration.
        """
        # Need camera intrinsics
        if self.camera_intrinsic is None:
            # Use default RealSense D435 intrinsics as Tensor on GPU
            self.camera_intrinsic = o3d.core.Tensor(
                [[615.0, 0, msg.width / 2.0], [0, 615.0, msg.height / 2.0], [0, 0, 1]],
                dtype=o3d.core.Dtype.Float32, device=self.device
            )
            self.get_logger().info(f'Using default camera intrinsics ({msg.width}x{msg.height})')

        # Convert ROS Image to Open3D Tensor Image on GPU (cv_bridge)
        depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_data_float = depth_data.astype(np.float32)
        depth_tensor = o3d.core.Tensor(depth_data_float, device=self.device)

        # Create identity 4x4 extrinsic matrix on GPU
        extrinsic = o3d.core.Tensor(np.eye(4), dtype=o3d.core.Dtype.Float32, device=self.device)

        # CUDA Accelerated Projection
        pcd = o3d.t.geometry.PointCloud.create_from_depth_image(
            o3d.t.geometry.Image(depth_tensor),
            self.camera_intrinsic,
            extrinsic,
            depth_scale=1000.0,
            depth_max=0.8
        )

        # Fast Voxel Downsampling on GPU
        if self.voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Convert back to CPU for saving
        points = pcd.point.positions.to(o3d.core.Device("CPU:0")).numpy()

        if len(points) == 0:
            self.get_logger().warn(f'Empty depth pointcloud for {filepath}')
            return

        # Write PCD (xyz only, no color/intensity)
        self.write_pcd(filepath, points.astype(np.float32), colors=None, intensity=None)


def main(args=None):
    rclpy.init(args=args)
    node = BagDataSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.get_logger().info(f'Sampler finished. Total samples: {node.sample_count}')
        node.destroy_node()


if __name__ == '__main__':
    main()
