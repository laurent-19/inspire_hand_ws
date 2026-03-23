#!/usr/bin/env python3
"""
Synchronized multi-modal data sampler for ROS2 bags.
Saves camera images, tactile heatmaps, point clouds, and hand/joint states.

Uses a caching strategy to handle topics with misaligned timestamps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from inspire_hand_ros2.msg import InspireHandState
import numpy as np
import cv2
import json
import os


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
        }

        # QoS for bag playback compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
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

    def joint_state_callback(self, msg):
        """Use joint_state as trigger - save sample when all data is available."""
        self.cache['joint_state'] = msg
        self.try_save_sample(msg.header.stamp)

    def try_save_sample(self, stamp):
        """Try to save a sample if all data is available and enough time has passed."""
        # Check if all data is available
        if any(v is None for v in self.cache.values()):
            return

        # Rate limit using joint_state timestamps (consistent with other topics)
        current_stamp_sec = stamp.sec + stamp.nanosec / 1e9
        if self.last_sample_stamp is not None:
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
        self.save_pointcloud(self.cache['tactile_pcd'], os.path.join(sample_subdir, 'tactile_pointcloud.pcd'))

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

    def save_pointcloud(self, msg: PointCloud2, filepath: str):
        """Save PointCloud2 message to PCD file with x,y,z,rgb,intensity."""
        field_names = [f.name for f in msg.fields]
        want = [f for f in ['x', 'y', 'z', 'rgb', 'intensity'] if f in field_names]

        gen = point_cloud2.read_points(msg, field_names=want, skip_nans=True)
        cloud = np.array(list(gen))

        if cloud.size == 0:
            self.get_logger().warn(f'Empty cloud for {filepath}')
            return

        n = len(cloud)
        xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(np.float32)

        # RGB
        colors = None
        if 'rgb' in want:
            packed = cloud['rgb'].view(np.uint32)
            r = ((packed >> 16) & 0xFF).astype(np.float64) / 255.0
            g = ((packed >> 8) & 0xFF).astype(np.float64) / 255.0
            b = (packed & 0xFF).astype(np.float64) / 255.0
            colors = np.column_stack([r, g, b])

        # Intensity
        intensity = None
        if 'intensity' in want:
            intensity = cloud['intensity'].astype(np.float32)
            intensity = np.where(np.isfinite(intensity), intensity, -1.0)

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
