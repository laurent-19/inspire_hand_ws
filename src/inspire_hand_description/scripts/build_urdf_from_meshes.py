#!/usr/bin/env python3
"""
Build URDF from Mesh Geometry - All Fixed Joints

Creates a static URDF where all links are connected with fixed joints.
This ensures correct assembly position. Joints can be converted to
revolute later once positions are verified.
"""

import sys
from pathlib import Path
from typing import Dict
import numpy as np

try:
    from stl import mesh as stl_mesh
except ImportError:
    print("ERROR: pip install numpy-stl")
    sys.exit(1)


def get_mesh_bounds(mesh_path: Path) -> Dict:
    """Get bounding box and centroid of a mesh."""
    m = stl_mesh.Mesh.from_file(str(mesh_path))
    vertices = m.vectors.reshape(-1, 3)

    return {
        'min': vertices.min(axis=0),
        'max': vertices.max(axis=0),
        'centroid': vertices.mean(axis=0),
        'size': vertices.max(axis=0) - vertices.min(axis=0)
    }


def analyze_meshes(mesh_dir: Path) -> Dict:
    """Analyze all link meshes."""
    results = {}
    for stl_file in sorted(mesh_dir.glob('*.stl')):
        link_name = stl_file.stem
        bounds = get_mesh_bounds(stl_file)
        results[link_name] = bounds
        print(f"{link_name}: Y[{bounds['min'][1]:.4f}, {bounds['max'][1]:.4f}]")
    return results


def generate_static_urdf(mesh_bounds: Dict, side: str = 'right') -> str:
    """
    Generate URDF with all fixed joints.

    All meshes are in world/assembly coordinates.
    Using fixed joints with zero origin means each link's mesh
    renders at its original CAD position.
    """
    mesh_dir = f"file:///home/analog/develop/inspire_hand_ws/src/inspire_hand_description/meshes/{side}/links"

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<robot name="inspire_hand_{side}">',
        '',
        '  <material name="hand_material">',
        '    <color rgba="0.85 0.85 0.85 1.0"/>',
        '  </material>',
        '',
    ]

    # Base link - root of the tree, no transform needed
    lines.append('  <!-- Base link (palm) -->')
    lines.append(f'''  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_dir}/base_link.stl"/>
      </geometry>
      <material name="hand_material"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_dir}/base_link.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.07" rpy="0 0 0"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0002"/>
    </inertial>
  </link>
''')

    # All other links - connected to base with fixed joints at origin
    # Since meshes are in world coords, fixed joint at origin keeps them in place

    # Define the kinematic tree structure
    # parent -> children
    tree = {
        'base_link': ['little_1_link', 'ring_1_link', 'middle_1_link', 'index_1_link', 'thumb_1_link'],
        'little_1_link': ['little_2_link'],
        'ring_1_link': ['ring_2_link'],
        'middle_1_link': ['middle_2_link'],
        'index_1_link': ['index_2_link'],
        'thumb_1_link': ['thumb_2_link'],
        'thumb_2_link': ['thumb_3_link'],
        'thumb_3_link': ['thumb_4_link'],
    }

    def add_link_and_children(parent_name: str, indent: str = ''):
        if parent_name not in tree:
            return

        for child_name in tree[parent_name]:
            if child_name not in mesh_bounds:
                print(f"  Skipping {child_name} - no mesh")
                continue

            # Add link
            lines.append(f'{indent}  <!-- {child_name} -->')
            lines.append(f'''{indent}  <link name="{child_name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_dir}/{child_name}.stl"/>
      </geometry>
      <material name="hand_material"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_dir}/{child_name}.stl"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
  </link>
''')

            # Add fixed joint - origin at 0,0,0 since meshes are in world coords
            joint_name = child_name.replace('_link', '_joint')
            lines.append(f'''{indent}  <joint name="{joint_name}" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="{parent_name}"/>
    <child link="{child_name}"/>
  </joint>
''')

            # Recursively add children
            add_link_and_children(child_name, indent)

    add_link_and_children('base_link')

    lines.append('</robot>')
    return '\n'.join(lines)


def main():
    pkg_dir = Path.home() / 'develop' / 'inspire_hand_ws' / 'src' / 'inspire_hand_description'

    for side in ['right']:
        mesh_dir = pkg_dir / 'meshes' / side / 'links'

        if not mesh_dir.exists():
            print(f"No meshes for {side} hand")
            continue

        print(f"\n{'='*60}")
        print(f"Building STATIC URDF for {side.upper()} hand")
        print(f"{'='*60}\n")

        bounds = analyze_meshes(mesh_dir)
        urdf_content = generate_static_urdf(bounds, side)

        urdf_path = pkg_dir / 'urdf' / f'inspire_hand_{side}.urdf'
        urdf_path.write_text(urdf_content)
        print(f"\nSaved: {urdf_path}")

        # Validate
        print("\nValidating URDF...")
        import subprocess
        result = subprocess.run(['check_urdf', str(urdf_path)],
                              capture_output=True, text=True)
        print(result.stdout)
        if result.returncode != 0:
            print(result.stderr)


if __name__ == '__main__':
    main()
