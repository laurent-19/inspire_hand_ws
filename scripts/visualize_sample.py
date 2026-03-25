#!/usr/bin/env python3
"""
Visualize a training sample: PNG images and PCD point clouds.

Displays:
1. Camera RGB image
2. Tactile colormap
3. Tactile point cloud (with RGB/Intensity toggle)
4. Camera point cloud (if available)

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

    # Load tactile point cloud with both color modes (filtered)
    pcd_path = os.path.join(sample_dir, 'tactile_pointcloud.pcd')
    pcd, rgb_colors, intensity_colors = load_pcd_with_data(pcd_path)
    if pcd:
        print(f"\n=== Tactile Point Cloud (filtered) ===")
        print(f"  Points: {len(pcd.points)}")

    # Load raw tactile point cloud (unfiltered)
    pcd_raw_path = os.path.join(sample_dir, 'tactile_pointcloud_raw.pcd')
    pcd_raw, rgb_colors_raw, intensity_colors_raw = load_pcd_with_data(pcd_raw_path)
    if pcd_raw:
        print(f"\n=== Tactile Point Cloud (raw) ===")
        print(f"  Points: {len(pcd_raw.points)}")

    # Load camera point cloud (optional)
    camera_pcd_path = os.path.join(sample_dir, 'camera_pointcloud.pcd')
    camera_pcd = None
    if os.path.exists(camera_pcd_path):
        camera_pcd = o3d.io.read_point_cloud(camera_pcd_path)
        if camera_pcd and len(camera_pcd.points) > 0:
            # Color camera points grey for distinction
            camera_pcd.paint_uniform_color([0.5, 0.5, 0.5])
            print(f"\n=== Camera Point Cloud ===")
            print(f"  Points: {len(camera_pcd.points)}")

    # Count total steps
    total_steps = sum([
        camera_img is not None,
        tactile_img is not None,
        pcd is not None and len(pcd.points) > 0,
        camera_pcd is not None and len(camera_pcd.points) > 0
    ])
    step = 0

    # Display images one at a time
    if camera_img is not None:
        step += 1
        print(f"\n[{step}/{total_steps}] Camera RGB - press any key to continue")
        with suppress_qt_warnings():
            cv2.imshow('Camera RGB', camera_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    if tactile_img is not None:
        step += 1
        print(f"[{step}/{total_steps}] Tactile Colormap - press any key to continue")
        with suppress_qt_warnings():
            cv2.imshow('Tactile Colormap', tactile_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # Display tactile point cloud with toggle
    if pcd and len(pcd.points) > 0:
        step += 1
        has_raw = pcd_raw is not None and len(pcd_raw.points) > 0
        controls = "A=RGB/Intensity"
        if has_raw:
            controls += ", R=Filtered/Raw"
        print(f"[{step}/{total_steps}] Tactile Point Cloud - {controls}, Q to quit")

        # Add coordinate frame for reference
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

        # State for toggling
        state = {
            'use_intensity': False,
            'use_raw': False,
            'current_pcd': pcd,
            'current_rgb': rgb_colors,
            'current_intensity': intensity_colors
        }

        def toggle_color(vis):
            state['use_intensity'] = not state['use_intensity']
            cur_pcd = state['current_pcd']
            cur_intensity = state['current_intensity']
            cur_rgb = state['current_rgb']
            if state['use_intensity'] and cur_intensity is not None:
                cur_pcd.colors = o3d.utility.Vector3dVector(cur_intensity)
                print("  -> Intensity mode")
            elif cur_rgb is not None:
                cur_pcd.colors = o3d.utility.Vector3dVector(cur_rgb)
                print("  -> RGB mode")
            vis.update_geometry(cur_pcd)
            return False

        def toggle_raw(vis):
            if not has_raw:
                return False
            state['use_raw'] = not state['use_raw']
            vis.remove_geometry(state['current_pcd'], reset_bounding_box=False)
            if state['use_raw']:
                state['current_pcd'] = pcd_raw
                state['current_rgb'] = rgb_colors_raw
                state['current_intensity'] = intensity_colors_raw
                print("  -> Raw (unfiltered)")
            else:
                state['current_pcd'] = pcd
                state['current_rgb'] = rgb_colors
                state['current_intensity'] = intensity_colors
                print("  -> Filtered")
            # Apply current color mode
            if state['use_intensity'] and state['current_intensity'] is not None:
                state['current_pcd'].colors = o3d.utility.Vector3dVector(state['current_intensity'])
            elif state['current_rgb'] is not None:
                state['current_pcd'].colors = o3d.utility.Vector3dVector(state['current_rgb'])
            vis.add_geometry(state['current_pcd'], reset_bounding_box=False)
            return False

        # Key callbacks: A to toggle color, R to toggle raw
        key_callbacks = {ord('A'): toggle_color, ord('R'): toggle_raw}

        o3d.visualization.draw_geometries_with_key_callbacks(
            [pcd, coord_frame],
            key_callbacks,
            window_name=f'Tactile Point Cloud - {os.path.basename(sample_dir)}',
            width=1024,
            height=768
        )

    # Display camera point cloud
    if camera_pcd and len(camera_pcd.points) > 0:
        step += 1
        print(f"[{step}/{total_steps}] Camera Point Cloud - press Q to quit")

        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

        o3d.visualization.draw_geometries(
            [camera_pcd, coord_frame],
            window_name=f'Camera Point Cloud - {os.path.basename(sample_dir)}',
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
