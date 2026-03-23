#!/usr/bin/env python3
"""
Visualize a training sample: PNG images and PCD point clouds.

Usage:
    ./visualize_sample.py training_data/non_deformable/record_250_1/sample_0000
    ./visualize_sample.py training_data/deformable/record_250_empty_1/sample_0005
"""

import sys
import os
from contextlib import contextmanager

@contextmanager
def suppress_qt_warnings():
    """Suppress Qt font warnings by redirecting stderr at fd level."""
    stderr_fd = sys.stderr.fileno()
    stderr_backup = os.dup(stderr_fd)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, stderr_fd)
    try:
        yield
    finally:
        os.dup2(stderr_backup, stderr_fd)
        os.close(devnull)
        os.close(stderr_backup)

import json
import numpy as np
import cv2
import open3d as o3d


def load_pcd_with_data(filepath):
    """Load PCD file and return Open3D point cloud plus raw data for toggling."""
    if not os.path.exists(filepath):
        return None, None, None

    pcd = o3d.io.read_point_cloud(filepath)

    # Store original RGB colors
    rgb_colors = np.asarray(pcd.colors).copy() if pcd.has_colors() else None

    # Parse intensity and compute intensity colors
    intensity = parse_intensity_from_pcd(filepath)
    intensity_colors = None

    if intensity is not None and len(intensity) == len(pcd.points):
        # Normalize intensity to [0, 1]
        i_min, i_max = intensity.min(), intensity.max()
        if i_max > i_min:
            intensity_norm = (intensity - i_min) / (i_max - i_min)
        else:
            intensity_norm = np.zeros_like(intensity)

        # Apply colormap (jet: blue=low, red=high)
        intensity_colors = np.zeros((len(intensity), 3))
        intensity_colors[:, 0] = np.clip(1.5 - np.abs(4 * intensity_norm - 3), 0, 1)  # R
        intensity_colors[:, 1] = np.clip(1.5 - np.abs(4 * intensity_norm - 2), 0, 1)  # G
        intensity_colors[:, 2] = np.clip(1.5 - np.abs(4 * intensity_norm - 1), 0, 1)  # B

    return pcd, rgb_colors, intensity_colors


def parse_intensity_from_pcd(filepath):
    """Parse intensity values from binary PCD file."""
    with open(filepath, 'rb') as f:
        # Read header
        fields = []
        sizes = []
        types = []

        while True:
            line = f.readline().decode('ascii').strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('TYPE'):
                types = line.split()[1:]
            elif line.startswith('DATA'):
                break

        if 'intensity' not in fields:
            return None

        # Build dtype
        dtype_map = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype_list = []
        for name, size, typ in zip(fields, sizes, types):
            dtype_list.append((name, f'{dtype_map.get(typ, "f")}{size}'))

        # Read binary data
        data = np.frombuffer(f.read(), dtype=dtype_list)
        return data['intensity'].astype(np.float32)


def visualize_sample(sample_dir):
    """Visualize all data in a sample directory."""

    # Check directory exists
    if not os.path.isdir(sample_dir):
        print(f"Error: Directory not found: {sample_dir}")
        sys.exit(1)

    print(f"Visualizing: {sample_dir}")

    # Load images
    camera_path = os.path.join(sample_dir, 'camera_rgb.png')
    tactile_path = os.path.join(sample_dir, 'tactile_colormap.png')

    camera_img = cv2.imread(camera_path) if os.path.exists(camera_path) else None
    tactile_img = cv2.imread(tactile_path) if os.path.exists(tactile_path) else None

    # Load JSON data
    hand_state_path = os.path.join(sample_dir, 'hand_state.json')
    joint_state_path = os.path.join(sample_dir, 'joint_state.json')

    if os.path.exists(hand_state_path):
        with open(hand_state_path) as f:
            hand_state = json.load(f)
        print("\n=== Hand State ===")
        print(f"  Position: {hand_state['position_actual']}")
        print(f"  Angle:    {hand_state['angle_actual']}")
        print(f"  Force:    {hand_state['force_actual']}")

    if os.path.exists(joint_state_path):
        with open(joint_state_path) as f:
            joint_state = json.load(f)
        print("\n=== Joint State ===")
        for name, pos in zip(joint_state['name'], joint_state['position']):
            print(f"  {name}: {pos:.4f}")

    # Load point cloud with both color modes
    pcd_path = os.path.join(sample_dir, 'tactile_pointcloud.pcd')
    pcd, rgb_colors, intensity_colors = load_pcd_with_data(pcd_path)
    if pcd:
        print(f"\n=== Point Cloud ===")
        print(f"  Points: {len(pcd.points)}")

    # Display images one at a time
    if camera_img is not None:
        print("\n[1/3] Camera RGB - press any key to continue")
        with suppress_qt_warnings():
            cv2.imshow('Camera RGB', camera_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    if tactile_img is not None:
        print("[2/3] Tactile Colormap - press any key to continue")
        with suppress_qt_warnings():
            cv2.imshow('Tactile Colormap', tactile_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # Display point cloud with toggle
    if pcd and len(pcd.points) > 0:
        print("[3/3] Point Cloud - press A to toggle RGB/Intensity, Q to quit")

        # Add coordinate frame for reference
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

        # State for toggling
        state = {'use_intensity': False}

        def toggle_color(vis):
            state['use_intensity'] = not state['use_intensity']
            if state['use_intensity'] and intensity_colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(intensity_colors)
                print("  -> Intensity mode")
            elif rgb_colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(rgb_colors)
                print("  -> RGB mode")
            vis.update_geometry(pcd)
            return False

        # Key callback: A (ord 65) to toggle
        key_callbacks = {ord('A'): toggle_color}

        o3d.visualization.draw_geometries_with_key_callbacks(
            [pcd, coord_frame],
            key_callbacks,
            window_name=f'Tactile Point Cloud - {os.path.basename(sample_dir)}',
            width=1024,
            height=768
        )


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    sample_dir = sys.argv[1]
    visualize_sample(sample_dir)


if __name__ == '__main__':
    main()
