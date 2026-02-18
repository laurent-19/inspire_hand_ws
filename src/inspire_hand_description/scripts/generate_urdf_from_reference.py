#!/usr/bin/env python3
"""
Generate URDF using reference joint positions from inspire_hand_485.

The reference URDF has joints exported from SolidWorks with proper orientations.
We use those joint positions/orientations and calculate visual origins for our meshes.
"""

import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom
import math

# Joint structure combining reference URDF (mimic, limits) with our coordinate system (axis)
# Our meshes: fingers extend in +Y, palm at Z~0.11
# Axis (-1, 0, 0): rotation around -X makes fingers curl toward palm (decreasing Z)
JOINT_STRUCTURE = {
    # Thumb - simplified to 1 joint (our mesh has single phalanx)
    # Thumb extends in -Y direction, so rotation around -X curls it
    'thumb_1': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),  # Rotation around -X
        'parent': 'base_link',
        'limits': (0, 1.246),
    },
    # Fingers - axis (-1, 0, 0) for proper curling, mimic from reference
    'index_1': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'base_link',
        'limits': (0, 1.333),
    },
    'index_2': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'index_1_link',
        'mimic': ('index_1_joint', 1.1169),
        'limits': (0, 1.527),
    },
    'middle_1': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'base_link',
        'limits': (0, 1.333),
    },
    'middle_2': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'middle_1_link',
        'mimic': ('middle_1_joint', 1.1169),
        'limits': (0, 1.527),
    },
    'ring_1': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'base_link',
        'limits': (0, 1.333),
    },
    'ring_2': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'ring_1_link',
        'mimic': ('ring_1_joint', 1.1169),
        'limits': (0, 1.527),
    },
    'little_1': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'base_link',
        'limits': (0, 1.333),
    },
    'little_2': {
        'rpy': (0, 0, 0),
        'axis': (-1, 0, 0),
        'parent': 'little_1_link',
        'mimic': ('little_1_joint', 1.1169),
        'limits': (0, 1.527),
    },
}

def compute_joint_positions(parts):
    """Compute joint positions from mesh data."""
    positions = {}

    # Thumb - at proximal end of part_054
    p054 = parts.get('part_054', {})
    if p054:
        positions['thumb_1'] = (
            p054['centroid'][0],
            p054['bbox_max'][1],  # Proximal end (toward palm)
            p054['centroid'][2]
        )

    # Finger joints - computed from mesh connections
    finger_structure = {
        'little': {
            'connector': ['part_034'],
            'proximal': ['part_038', 'part_047'],
            'distal': ['part_042'],
        },
        'ring': {
            'connector': ['part_036'],
            'proximal': ['part_040', 'part_048'],
            'distal': ['part_043'],
        },
        'middle': {
            'connector': ['part_033'],
            'proximal': ['part_037', 'part_049'],
            'distal': ['part_045'],
        },
        'index': {
            'connector': ['part_035'],
            'proximal': ['part_039', 'part_046'],
            'distal': ['part_044'],
        },
    }

    for finger, groups in finger_structure.items():
        # Joint 1: between connector and proximal
        conn_y_max = max(parts[p]['bbox_max'][1] for p in groups['connector'] if p in parts)
        prox_y_min = min(parts[p]['bbox_min'][1] for p in groups['proximal'] if p in parts)
        prox_x = sum(parts[p]['centroid'][0] for p in groups['proximal'] if p in parts) / len(groups['proximal'])
        prox_z = sum(parts[p]['centroid'][2] for p in groups['proximal'] if p in parts) / len(groups['proximal'])

        positions[f'{finger}_1'] = (prox_x, (conn_y_max + prox_y_min) / 2, prox_z)

        # Joint 2: between proximal and distal (relative to joint 1)
        prox_y_max = max(parts[p]['bbox_max'][1] for p in groups['proximal'] if p in parts)
        dist_y_min = min(parts[p]['bbox_min'][1] for p in groups['distal'] if p in parts)
        dist_x = parts[groups['distal'][0]]['centroid'][0]
        dist_z = parts[groups['distal'][0]]['centroid'][2]

        # Store as relative position from joint 1
        j1 = positions[f'{finger}_1']
        positions[f'{finger}_2'] = (
            dist_x - j1[0],
            ((prox_y_max + dist_y_min) / 2) - j1[1],
            dist_z - j1[2]
        )

    return positions

# Part assignments - maps link names to mesh part files
PART_ASSIGNMENTS = {
    'right': {
        'base_link': [
            'part_032', 'part_050',  # Main palm
            'part_000', 'part_002', 'part_003', 'part_005', 'part_006', 'part_008',
            'part_009', 'part_011', 'part_023', 'part_024', 'part_025',
            'part_026', 'part_027', 'part_028', 'part_029', 'part_030', 'part_031',
            'part_001', 'part_004', 'part_007', 'part_010',
            'part_012', 'part_013', 'part_014', 'part_015', 'part_017',
            'part_016', 'part_019', 'part_020', 'part_021',
            'part_018', 'part_022', 'part_041', 'part_051', 'part_052', 'part_053',
            'part_033', 'part_034', 'part_035', 'part_036',  # Finger connectors
        ],
        'thumb_1_link': ['part_054', 'part_055'],
        'index_1_link': ['part_039', 'part_046'],
        'index_2_link': ['part_044'],
        'middle_1_link': ['part_037', 'part_049'],
        'middle_2_link': ['part_045'],
        'ring_1_link': ['part_040', 'part_048'],
        'ring_2_link': ['part_043'],
        'little_1_link': ['part_038', 'part_047'],
        'little_2_link': ['part_042'],
    }
}

def load_mesh_data(side):
    """Load mesh analysis data."""
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    json_path = workspace / 'src' / 'inspire_hand_description' / 'config' / f'mesh_analysis_{side}.json'

    with open(json_path) as f:
        return json.load(f)


def build_reference_joints(parts):
    """
    Build complete joint definitions by combining:
    - JOINT_STRUCTURE: reference RPY, axis, mimic from inspire_hand_485 URDF
    - Computed xyz positions from our mesh data
    """
    # Compute xyz positions from mesh bounding boxes
    positions = compute_joint_positions(parts)

    reference_joints = {}
    for joint_name, structure in JOINT_STRUCTURE.items():
        if joint_name not in positions:
            print(f"Warning: No position for {joint_name}")
            continue

        xyz = positions[joint_name]
        reference_joints[joint_name] = {
            'xyz': xyz,
            'rpy': structure['rpy'],
            'axis': structure['axis'],
            'parent': structure['parent'],
            'limits': structure['limits'],
        }
        if 'mimic' in structure:
            reference_joints[joint_name]['mimic'] = structure['mimic']

    return reference_joints


def compute_visual_offset(link_name, reference_joints):
    """
    Compute visual offset for a link.

    Meshes are at world coordinates, but link frames are at joint positions.
    We need to offset visuals by the negative of accumulated joint positions.
    """
    if link_name == 'base_link':
        return (0, 0, 0)

    # Map link to joint
    joint_name = link_name.replace('_link', '')

    # Build parent chain
    parent_chain = []
    current = joint_name
    while current in reference_joints:
        parent_chain.append(current)
        parent_link = reference_joints[current]['parent']
        if parent_link == 'base_link':
            break
        current = parent_link.replace('_link', '')

    # Accumulate joint positions (in reverse order, from base to this link)
    offset = [0.0, 0.0, 0.0]
    for jname in reversed(parent_chain):
        xyz = reference_joints[jname]['xyz']
        offset[0] += xyz[0]
        offset[1] += xyz[1]
        offset[2] += xyz[2]

    # Return negative offset to move world-coordinate meshes to link frame
    return (-offset[0], -offset[1], -offset[2])


def prettify(xml_string):
    """Pretty print XML."""
    dom = minidom.parseString(xml_string)
    return dom.toprettyxml(indent='  ')


def create_urdf(side):
    """Create URDF with reference joint positions."""
    robot = ET.Element('robot', name=f'inspire_hand_{side}')

    mesh_data = load_mesh_data(side)
    parts = mesh_data['parts']
    assignments = PART_ASSIGNMENTS.get(side, {})

    # Build complete joint definitions
    reference_joints = build_reference_joints(parts)

    # Create links
    for link_name, part_list in assignments.items():
        link = ET.SubElement(robot, 'link', name=link_name)

        # Compute visual offset for this link
        visual_offset = compute_visual_offset(link_name, reference_joints)
        offset_str = f'{visual_offset[0]:.6f} {visual_offset[1]:.6f} {visual_offset[2]:.6f}'

        for part in part_list:
            if part not in parts:
                continue

            # Visual - meshes at world coordinates, offset to link frame
            visual = ET.SubElement(link, 'visual')
            ET.SubElement(visual, 'origin', xyz=offset_str, rpy='0 0 0')

            geometry = ET.SubElement(visual, 'geometry')
            mesh_path = f'package://inspire_hand_description/meshes/{side}/{part}.stl'
            ET.SubElement(geometry, 'mesh', filename=mesh_path)

            material = ET.SubElement(visual, 'material', name='')
            ET.SubElement(material, 'color', rgba='0.9 0.9 0.9 1')

        # Collision
        if part_list:
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(collision, 'origin', xyz=offset_str, rpy='0 0 0')
            coll_geom = ET.SubElement(collision, 'geometry')
            mesh_path = f'package://inspire_hand_description/meshes/{side}/{part_list[0]}.stl'
            ET.SubElement(coll_geom, 'mesh', filename=mesh_path)

        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        mass = 0.05 if 'base' in link_name else 0.01
        ET.SubElement(inertial, 'mass', value=str(mass))
        ET.SubElement(inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(inertial, 'inertia',
                      ixx='1e-06', ixy='0', ixz='0',
                      iyy='1e-06', iyz='0', izz='1e-06')

    # Create joints using reference positions
    for joint_base, jdata in reference_joints.items():
        joint_name = f'{joint_base}_joint'
        link_name = f'{joint_base}_link'

        # Skip if link has no parts
        if link_name not in assignments or not assignments.get(link_name, []):
            continue

        joint = ET.SubElement(robot, 'joint', name=joint_name, type='revolute')

        xyz = jdata['xyz']
        rpy = jdata.get('rpy', (0, 0, 0))
        ET.SubElement(joint, 'origin',
                      xyz=f'{xyz[0]} {xyz[1]} {xyz[2]}',
                      rpy=f'{rpy[0]} {rpy[1]} {rpy[2]}')

        ET.SubElement(joint, 'parent', link=jdata['parent'])
        ET.SubElement(joint, 'child', link=link_name)

        axis = jdata['axis']
        ET.SubElement(joint, 'axis', xyz=f'{axis[0]} {axis[1]} {axis[2]}')

        limits = jdata.get('limits', (0, 1.5))
        ET.SubElement(joint, 'limit', lower=str(limits[0]), upper=str(limits[1]),
                      effort='50', velocity='1')

        if 'mimic' in jdata:
            mimic_joint, multiplier = jdata['mimic']
            ET.SubElement(joint, 'mimic', joint=mimic_joint,
                          multiplier=str(multiplier), offset='0')

    return prettify(ET.tostring(robot, encoding='unicode'))


def main():
    parser = argparse.ArgumentParser(description='Generate URDF from reference')
    parser.add_argument('--side', choices=['right', 'left'], default='right')
    parser.add_argument('--output', type=str, default=None)
    args = parser.parse_args()

    urdf = create_urdf(args.side)

    if args.output:
        output_path = Path(args.output)
    else:
        workspace = Path.home() / 'develop' / 'inspire_hand_ws'
        output_path = workspace / 'src' / 'inspire_hand_description' / 'urdf' / f'inspire_hand_{args.side}_ref.urdf'

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(urdf)

    print(f'Generated: {output_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
