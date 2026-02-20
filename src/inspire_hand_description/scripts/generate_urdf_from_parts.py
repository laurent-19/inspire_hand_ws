#!/usr/bin/env python3
"""
Generate URDF from individual part meshes.

The STL parts are already positioned in world coordinates.
This script creates a URDF that references each part with appropriate
joint positions derived from where parts connect.

Usage:
    python3 generate_urdf_from_parts.py --side right
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Tuple
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Part assignments based on mesh analysis
# Parts that move together are grouped into links
PART_ASSIGNMENTS = {
    'right': {
        # Base link: palm shell, internal mechanism, screws, and finger connectors
        'base_link': [
            # Main palm shells (largest parts)
            'part_032', 'part_050',
            # Palm mechanism/internals (Y ~ 0 to -25mm)
            'part_000', 'part_002', 'part_003', 'part_005', 'part_006', 'part_008',
            'part_009', 'part_011', 'part_023', 'part_024', 'part_025',
            # Palm base/mount (029-031 moved to thumb links)
            'part_026', 'part_027', 'part_028',
            # Screws/fasteners (tiny parts at high Z)
            'part_001', 'part_004', 'part_007', 'part_010',
            'part_012', 'part_013', 'part_014', 'part_015', 'part_017',
            # Internal mechanism parts
            'part_016', 'part_019', 'part_020', 'part_021',
            # Wrist-level and thumb mechanism parts (internal, don't rotate visibly)
            'part_018', 'part_022', 'part_041', 'part_051', 'part_052', 'part_053',
            # Finger connectors (fixed to palm)
            'part_033', 'part_034', 'part_035', 'part_036',
        ],

        # Thumb - parts 029-031 per SolidWorks verification
        # 029=tip, 030=middle, 031=bottom (base)
        'thumb_1_link': ['part_031'],  # Bottom/base of thumb (rotation)
        'thumb_2_link': ['part_030', 'part_029'],  # Middle and tip (bend)

        # Little finger
        'little_1_link': ['part_038', 'part_047'],
        'little_2_link': ['part_042'],

        # Ring finger
        'ring_1_link': ['part_040', 'part_048'],
        'ring_2_link': ['part_043'],

        # Middle finger
        'middle_1_link': ['part_037', 'part_049'],
        'middle_2_link': ['part_045'],

        # Index finger
        'index_1_link': ['part_039', 'part_046'],
        'index_2_link': ['part_044'],
    }
}

# Joint positions derived from mesh bounding box overlaps (in meters)
# These are actual connection points from our CAD meshes
JOINT_POSITIONS = {
    'right': {
        # Thumb rotation joint (DOF5 - opposition/abduction)
        # part_031 centroid: (-0.041, -0.025, 0.071), connects to palm at top (Z~0.090)
        # Joint at the pivot point where thumb connects to palm
        'thumb_1_joint': {
            'xyz': (-0.041, -0.020, 0.090),  # Pivot at top of part_031
            'rpy': (0, 0, 0),
            'axis': (0, 0, -1),  # Rotate around -Z
            'type': 'revolute',
            'limits': (0, 1.246),
        },
        # Thumb bend joint (DOF4 - flexion)
        # Between part_031 and part_030
        # part_030 centroid: (-0.049, -0.004, 0.046)
        # Relative offset from thumb_1: (-0.008, 0.016, -0.044)
        'thumb_2_joint': {
            'xyz': (-0.008, 0.016, -0.044),  # Offset toward part_030
            'rpy': (0, 0, 0),
            'axis': (0, 1, 0),  # Bend around Y axis
            'type': 'revolute',
            'limits': (0, 0.48),
        },

        # Index finger joints (from mesh analysis)
        # joint_1 at world: (0.0252, 0.0332, 0.1091)
        # joint_2 at world: (0.0296, 0.0720, 0.1094)
        'index_1_joint': {
            'xyz': (0.0252, 0.0332, 0.1091),  # World coords (relative to base)
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),  # Negative X for fingers to close with positive angle
            'type': 'revolute',
            'limits': (0, 1.333),
        },
        'index_2_joint': {
            'xyz': (0.0044, 0.0388, 0.0003),  # Relative to index_1
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.527),
            'mimic': ('index_1_joint', 1.1169),
        },

        # Middle finger joints (from mesh analysis)
        # joint_1 at world: (0.0034, 0.0369, 0.1091)
        # joint_2 at world: (0.0057, 0.0756, 0.1084)
        'middle_1_joint': {
            'xyz': (0.0034, 0.0369, 0.1091),
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.333),
        },
        'middle_2_joint': {
            'xyz': (0.0023, 0.0387, -0.0007),  # Relative to middle_1
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.527),
            'mimic': ('middle_1_joint', 1.1169),
        },

        # Ring finger joints (from mesh analysis)
        # joint_1 at world: (-0.0188, 0.0376, 0.1091)
        # joint_2 at world: (-0.0188, 0.0760, 0.1077)
        'ring_1_joint': {
            'xyz': (-0.0188, 0.0376, 0.1091),
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.333),
        },
        'ring_2_joint': {
            'xyz': (0.0, 0.0384, -0.0014),  # Relative to ring_1
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.527),
            'mimic': ('ring_1_joint', 1.1169),
        },

        # Little finger joints (from mesh analysis)
        # joint_1 at world: (-0.0408, 0.0371, 0.1091)
        # joint_2 at world: (-0.0422, 0.0757, 0.1084)
        'little_1_joint': {
            'xyz': (-0.0408, 0.0371, 0.1091),
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.333),
        },
        'little_2_joint': {
            'xyz': (-0.0014, 0.0386, -0.0007),  # Relative to little_1
            'rpy': (0, 0, 0),
            'axis': (-1, 0, 0),
            'type': 'revolute',
            'limits': (0, 1.527),
            'mimic': ('little_1_joint', 1.1169),
        },
    }
}

# Kinematic chain definition
KINEMATIC_CHAIN = {
    'right': [
        # Thumb (2 links: rotation + bend)
        ('base_link', 'thumb_1_link', 'thumb_1_joint'),  # Rotation
        ('thumb_1_link', 'thumb_2_link', 'thumb_2_joint'),  # Bend

        # Fingers (2 links each: proximal, distal)
        ('base_link', 'index_1_link', 'index_1_joint'),
        ('index_1_link', 'index_2_link', 'index_2_joint'),

        ('base_link', 'middle_1_link', 'middle_1_joint'),
        ('middle_1_link', 'middle_2_link', 'middle_2_joint'),

        ('base_link', 'ring_1_link', 'ring_1_joint'),
        ('ring_1_link', 'ring_2_link', 'ring_2_joint'),

        ('base_link', 'little_1_link', 'little_1_joint'),
        ('little_1_link', 'little_2_link', 'little_2_joint'),
    ]
}


def get_link_world_origin(link_name: str, side: str) -> Tuple[float, float, float]:
    """
    Get the world-coordinate origin of a link (sum of joint positions in chain).
    This is needed to offset meshes which are in world coordinates.
    Accounts for joint rotations (rpy) when computing positions.
    """
    import math
    chain = KINEMATIC_CHAIN.get(side, [])
    joints = JOINT_POSITIONS.get(side, {})

    # Build parent map
    parent_map = {}
    joint_map = {}
    for parent, child, joint_name in chain:
        parent_map[child] = parent
        joint_map[child] = joint_name

    # Build chain from link to base
    chain_to_base = []
    current = link_name
    while current in parent_map:
        chain_to_base.append(current)
        current = parent_map[current]

    # Accumulate joint positions from base to this link
    origin = [0.0, 0.0, 0.0]
    accumulated_yaw = 0.0  # Track Z rotation

    # Process from base to link (reverse order)
    for link in reversed(chain_to_base):
        joint_name = joint_map[link]
        if joint_name in joints:
            xyz = joints[joint_name]['xyz']
            rpy = joints[joint_name].get('rpy', (0, 0, 0))

            # Apply current accumulated rotation to xyz offset
            cos_yaw = math.cos(accumulated_yaw)
            sin_yaw = math.sin(accumulated_yaw)
            rotated_x = xyz[0] * cos_yaw - xyz[1] * sin_yaw
            rotated_y = xyz[0] * sin_yaw + xyz[1] * cos_yaw

            origin[0] += rotated_x
            origin[1] += rotated_y
            origin[2] += xyz[2]

            # Accumulate yaw rotation for next iteration
            accumulated_yaw += rpy[2]

    return tuple(origin)


def create_link_element(robot: ET.Element, link_name: str, parts: List[str],
                        side: str, pkg_name: str = 'inspire_hand_description') -> ET.Element:
    """Create a link element with visual meshes for all parts."""
    link = ET.SubElement(robot, 'link', name=link_name)

    # Calculate visual origin offset
    # Meshes are in world coords, link frame is at joint position
    # For base_link: no offset needed (link is at origin)
    # For other links: offset = -(link world position) so mesh stays at world coords
    if link_name == 'base_link':
        visual_origin = (0, 0, 0)
    else:
        # For thumb, the mesh (part_054) is at world position, we need to
        # offset it so it appears at the correct location relative to the link frame
        world_origin = get_link_world_origin(link_name, side)
        # The visual origin shifts the mesh; we want mesh to stay at its world position
        # mesh_world = link_world + visual_origin + mesh_local
        # Since mesh_local IS mesh_world (STL in world coords), we need:
        # mesh_world = link_world + visual_origin + mesh_world
        # 0 = link_world + visual_origin
        # visual_origin = -link_world
        visual_origin = (-world_origin[0], -world_origin[1], -world_origin[2])

    origin_str = f'{visual_origin[0]:.6f} {visual_origin[1]:.6f} {visual_origin[2]:.6f}'

    # Determine color based on link name
    if 'thumb' in link_name:
        color_rgba = '0.9 0.2 0.2 1'  # Red for thumb
    elif 'base' in link_name:
        color_rgba = '0.3 0.3 0.9 1'  # Blue for base
    else:
        color_rgba = '0.9 0.9 0.9 1'  # Gray for fingers

    # Add visual for each part
    for part in parts:
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', xyz=origin_str, rpy='0 0 0')
        geometry = ET.SubElement(visual, 'geometry')
        mesh_path = f'package://{pkg_name}/meshes/{side}/{part}.stl'
        ET.SubElement(geometry, 'mesh', filename=mesh_path)
        material = ET.SubElement(visual, 'material', name='')
        ET.SubElement(material, 'color', rgba=color_rgba)

    # Add simple collision (only if we have parts)
    if parts:
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz=origin_str, rpy='0 0 0')
        coll_geom = ET.SubElement(collision, 'geometry')
        mesh_path = f'package://{pkg_name}/meshes/{side}/{parts[0]}.stl'
        ET.SubElement(coll_geom, 'mesh', filename=mesh_path)

    # Add inertial (simplified)
    inertial = ET.SubElement(link, 'inertial')
    mass = 0.05 if 'base' in link_name else 0.01
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
    ixx = 1e-6
    ET.SubElement(inertial, 'inertia',
                  ixx=str(ixx), ixy='0', ixz='0',
                  iyy=str(ixx), iyz='0', izz=str(ixx))

    return link


def create_joint_element(robot: ET.Element, joint_name: str, joint_data: Dict,
                         parent_link: str, child_link: str) -> ET.Element:
    """Create a joint element."""
    joint = ET.SubElement(robot, 'joint', name=joint_name, type=joint_data['type'])

    xyz = joint_data['xyz']
    rpy = joint_data.get('rpy', (0, 0, 0))
    ET.SubElement(joint, 'origin',
                  xyz=f'{xyz[0]} {xyz[1]} {xyz[2]}',
                  rpy=f'{rpy[0]} {rpy[1]} {rpy[2]}')

    ET.SubElement(joint, 'parent', link=parent_link)
    ET.SubElement(joint, 'child', link=child_link)

    if joint_data['type'] != 'fixed':
        axis = joint_data.get('axis', (0, 0, 1))
        ET.SubElement(joint, 'axis', xyz=f'{axis[0]} {axis[1]} {axis[2]}')

        limits = joint_data.get('limits', (0, 1))
        ET.SubElement(joint, 'limit',
                      lower=str(limits[0]), upper=str(limits[1]),
                      effort='50', velocity='1')

    # Mimic joint if specified
    if 'mimic' in joint_data:
        mimic_joint, multiplier = joint_data['mimic']
        ET.SubElement(joint, 'mimic', joint=mimic_joint,
                      multiplier=str(multiplier), offset='0')

    return joint


def prettify(xml_string: str) -> str:
    """Pretty print XML."""
    dom = minidom.parseString(xml_string)
    return dom.toprettyxml(indent='  ')


def generate_urdf(side: str) -> str:
    """Generate complete URDF for the hand."""
    robot = ET.Element('robot', name=f'inspire_hand_{side}')

    parts = PART_ASSIGNMENTS.get(side, {})
    joints = JOINT_POSITIONS.get(side, {})
    chain = KINEMATIC_CHAIN.get(side, [])

    # Create all links
    for link_name, link_parts in parts.items():
        create_link_element(robot, link_name, link_parts, side)

    # Create all joints
    for parent, child, joint_name in chain:
        if joint_name in joints:
            create_joint_element(robot, joint_name, joints[joint_name], parent, child)

    return prettify(ET.tostring(robot, encoding='unicode'))


def main():
    parser = argparse.ArgumentParser(description='Generate URDF from part meshes')
    parser.add_argument('--side', choices=['right', 'left'], default='right')
    parser.add_argument('--output', type=str, default=None)
    args = parser.parse_args()

    urdf = generate_urdf(args.side)

    if args.output:
        output_path = Path(args.output)
    else:
        workspace = Path.home() / 'develop' / 'inspire_hand_ws'
        output_path = workspace / 'src' / 'inspire_hand_description' / 'urdf' / f'inspire_hand_{args.side}_parts.urdf'

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(urdf)

    print(f'Generated URDF: {output_path}')
    print(f'Links: {len(PART_ASSIGNMENTS.get(args.side, {}))}')
    print(f'Joints: {len(KINEMATIC_CHAIN.get(args.side, []))}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
