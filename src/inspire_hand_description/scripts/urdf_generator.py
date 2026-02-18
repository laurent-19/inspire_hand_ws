#!/usr/bin/env python3
"""
URDF Generator for Inspire Hand

Generates URDF files from extracted STEP data with proper kinematics.
"""

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import xml.etree.ElementTree as ET
from xml.dom import minidom


@dataclass
class JointKinematics:
    """Joint kinematic parameters."""
    name: str
    parent_link: str
    child_link: str
    joint_type: str  # revolute, prismatic, fixed
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    lower_limit: float
    upper_limit: float
    effort: float
    velocity: float
    mimic_joint: Optional[str] = None
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0


@dataclass
class LinkData:
    """Link data for URDF."""
    name: str
    mesh_path: str
    origin_xyz: Tuple[float, float, float] = (0, 0, 0)
    origin_rpy: Tuple[float, float, float] = (0, 0, 0)
    mass: float = 0.01  # Default mass in kg
    inertia_scale: float = 1e-6  # Default inertia scale


class InspireHandURDFGenerator:
    """Generate URDF for Inspire Hand RH56DFTP."""

    # Known kinematics from driver
    # DOF order: [little, ring, middle, index, thumb_bend, thumb_rotate]
    # Angle range in driver: 0-1000 (0=closed, 1000=open)

    # Angle limits in radians (from plan)
    FINGER_ANGLE_MAX = 1.333  # ~76 degrees for fingers
    THUMB_BEND_MAX = 0.48    # ~27.5 degrees for thumb bend
    THUMB_ROTATE_MAX = 1.246  # ~71.4 degrees for thumb rotation

    # Mimic ratio for coupled finger joints
    MIMIC_MULTIPLIER = 1.1169

    # Use simple visual mode (single mesh on base, boxes for links)
    USE_SIMPLE_VISUALS = True

    # Approximate link lengths (in meters, from visual inspection of CAD)
    # These will be refined when we can extract from STEP transforms
    FINGER_LINK_LENGTHS = {
        'little': [0.025, 0.020, 0.015],   # proximal, middle, distal
        'ring': [0.030, 0.022, 0.017],
        'middle': [0.035, 0.025, 0.020],
        'index': [0.030, 0.022, 0.017],
        'thumb': [0.025, 0.020, 0.015],
    }

    # Finger base positions relative to palm center (approximate)
    FINGER_BASE_XYZ = {
        'little': (0.08, -0.02, 0.01),
        'ring': (0.085, 0.0, 0.01),
        'middle': (0.085, 0.02, 0.01),
        'index': (0.08, 0.04, 0.01),
        'thumb': (0.03, -0.04, 0.015),
    }

    def __init__(self, hand_side: str = "right", mesh_dir: str = "meshes"):
        """
        Initialize URDF generator.

        Args:
            hand_side: "right" or "left"
            mesh_dir: Relative path to mesh directory
        """
        self.hand_side = hand_side
        self.mesh_dir = mesh_dir
        self.links: List[LinkData] = []
        self.joints: List[JointKinematics] = []

    def generate(self, mesh_paths: Optional[Dict[str, Path]] = None) -> str:
        """
        Generate complete URDF string.

        Args:
            mesh_paths: Optional dict mapping link names to mesh file paths

        Returns:
            URDF XML string
        """
        self.links = []
        self.joints = []

        # Create root element
        robot = ET.Element("robot", name=f"inspire_hand_{self.hand_side}")

        # Add base link
        self._add_base_link()

        # Add finger chains
        for finger in ['little', 'ring', 'middle', 'index']:
            self._add_finger_chain(finger)

        # Add thumb (special kinematics)
        self._add_thumb_chain()

        # Generate XML
        for link in self.links:
            self._add_link_element(robot, link, mesh_paths)

        for joint in self.joints:
            self._add_joint_element(robot, joint)

        # Pretty print
        xml_str = ET.tostring(robot, encoding='unicode')
        return self._prettify(xml_str)

    def _add_base_link(self):
        """Add palm/base link."""
        self.links.append(LinkData(
            name="base_link",
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/base.stl",
            mass=0.2  # Palm is heavier
        ))

    def _add_finger_chain(self, finger: str):
        """Add a 3-link finger chain with 2 joints (link 2 mimics link 1)."""
        prefix = finger
        base_xyz = self.FINGER_BASE_XYZ[finger]
        link_lengths = self.FINGER_LINK_LENGTHS[finger]

        # Mirror X for left hand
        if self.hand_side == "left":
            base_xyz = (-base_xyz[0], base_xyz[1], base_xyz[2])

        # Link 1 (proximal phalanx)
        link1_name = f"{prefix}_1_link"
        self.links.append(LinkData(
            name=link1_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_1.stl"
        ))

        # Joint 1 (base to link 1) - active joint
        joint1_name = f"{prefix}_1_joint"
        self.joints.append(JointKinematics(
            name=joint1_name,
            parent_link="base_link",
            child_link=link1_name,
            joint_type="revolute",
            origin_xyz=base_xyz,
            origin_rpy=(0, 0, 0),
            axis=(0, 1, 0),  # Flex around Y axis
            lower_limit=0.0,
            upper_limit=self.FINGER_ANGLE_MAX,
            effort=1.0,
            velocity=1.0
        ))

        # Link 2 (middle phalanx)
        link2_name = f"{prefix}_2_link"
        self.links.append(LinkData(
            name=link2_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_2.stl"
        ))

        # Joint 2 (link 1 to link 2) - mimic joint
        joint2_name = f"{prefix}_2_joint"
        self.joints.append(JointKinematics(
            name=joint2_name,
            parent_link=link1_name,
            child_link=link2_name,
            joint_type="revolute",
            origin_xyz=(link_lengths[0], 0, 0),
            origin_rpy=(0, 0, 0),
            axis=(0, 1, 0),
            lower_limit=0.0,
            upper_limit=self.FINGER_ANGLE_MAX * self.MIMIC_MULTIPLIER,
            effort=1.0,
            velocity=1.0,
            mimic_joint=joint1_name,
            mimic_multiplier=self.MIMIC_MULTIPLIER
        ))

        # Link 3 (distal phalanx) - fixed to link 2 for simplicity
        # or could be another mimic joint
        link3_name = f"{prefix}_3_link"
        self.links.append(LinkData(
            name=link3_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_3.stl"
        ))

        # Joint 3 (link 2 to link 3) - fixed or mimic
        joint3_name = f"{prefix}_3_joint"
        self.joints.append(JointKinematics(
            name=joint3_name,
            parent_link=link2_name,
            child_link=link3_name,
            joint_type="fixed",  # Simplified - could be mimic
            origin_xyz=(link_lengths[1], 0, 0),
            origin_rpy=(0, 0, 0),
            axis=(0, 1, 0),
            lower_limit=0.0,
            upper_limit=0.0,
            effort=1.0,
            velocity=1.0
        ))

    def _add_thumb_chain(self):
        """Add thumb with rotation + bend joints."""
        prefix = "thumb"
        base_xyz = self.FINGER_BASE_XYZ[prefix]
        link_lengths = self.FINGER_LINK_LENGTHS[prefix]

        # Mirror X for left hand
        if self.hand_side == "left":
            base_xyz = (-base_xyz[0], base_xyz[1], base_xyz[2])

        # Link 1 (metacarpal with rotation)
        link1_name = f"{prefix}_1_link"
        self.links.append(LinkData(
            name=link1_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_1.stl"
        ))

        # Joint 1 (rotation joint - DOF 5)
        joint1_name = f"{prefix}_1_joint"
        self.joints.append(JointKinematics(
            name=joint1_name,
            parent_link="base_link",
            child_link=link1_name,
            joint_type="revolute",
            origin_xyz=base_xyz,
            origin_rpy=(0, 0, math.pi/4),  # Thumb is angled
            axis=(0, 0, 1),  # Rotate around Z axis
            lower_limit=0.0,
            upper_limit=self.THUMB_ROTATE_MAX,
            effort=1.0,
            velocity=1.0
        ))

        # Link 2 (proximal phalanx)
        link2_name = f"{prefix}_2_link"
        self.links.append(LinkData(
            name=link2_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_2.stl"
        ))

        # Joint 2 (bend joint - DOF 4)
        joint2_name = f"{prefix}_2_joint"
        self.joints.append(JointKinematics(
            name=joint2_name,
            parent_link=link1_name,
            child_link=link2_name,
            joint_type="revolute",
            origin_xyz=(link_lengths[0], 0, 0),
            origin_rpy=(0, 0, 0),
            axis=(0, 1, 0),  # Flex around Y
            lower_limit=0.0,
            upper_limit=self.THUMB_BEND_MAX,
            effort=1.0,
            velocity=1.0
        ))

        # Link 3 (distal phalanx)
        link3_name = f"{prefix}_3_link"
        self.links.append(LinkData(
            name=link3_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_3.stl"
        ))

        # Joint 3 (mimic thumb_2)
        joint3_name = f"{prefix}_3_joint"
        self.joints.append(JointKinematics(
            name=joint3_name,
            parent_link=link2_name,
            child_link=link3_name,
            joint_type="revolute",
            origin_xyz=(link_lengths[1], 0, 0),
            origin_rpy=(0, 0, 0),
            axis=(0, 1, 0),
            lower_limit=0.0,
            upper_limit=self.THUMB_BEND_MAX * self.MIMIC_MULTIPLIER,
            effort=1.0,
            velocity=1.0,
            mimic_joint=joint2_name,
            mimic_multiplier=self.MIMIC_MULTIPLIER
        ))

        # Link 4 (fingertip)
        link4_name = f"{prefix}_4_link"
        self.links.append(LinkData(
            name=link4_name,
            mesh_path=f"{self.mesh_dir}/{self.hand_side}/{prefix}_4.stl"
        ))

        # Joint 4 (fixed to link 3)
        joint4_name = f"{prefix}_4_joint"
        self.joints.append(JointKinematics(
            name=joint4_name,
            parent_link=link3_name,
            child_link=link4_name,
            joint_type="fixed",
            origin_xyz=(link_lengths[2], 0, 0),
            origin_rpy=(0, 0, 0),
            axis=(0, 1, 0),
            lower_limit=0.0,
            upper_limit=0.0,
            effort=1.0,
            velocity=1.0
        ))

    def _add_link_element(self, parent: ET.Element, link: LinkData,
                          mesh_paths: Optional[Dict[str, Path]] = None):
        """Add a link element to the URDF."""
        link_elem = ET.SubElement(parent, "link", name=link.name)

        # Determine mesh path or use simple geometry
        use_mesh = True
        mesh_file = f"package://inspire_hand_description/{link.mesh_path}"

        # Check if mesh file exists or use simple visuals
        if self.USE_SIMPLE_VISUALS and link.name != "base_link":
            use_mesh = False

        # For base_link, use full_hand.stl if in simple mode
        if self.USE_SIMPLE_VISUALS and link.name == "base_link":
            mesh_file = f"package://inspire_hand_description/meshes/{self.hand_side}/full_hand.stl"

        # Visual
        visual = ET.SubElement(link_elem, "visual")
        visual_origin = ET.SubElement(visual, "origin",
                                      xyz=f"{link.origin_xyz[0]} {link.origin_xyz[1]} {link.origin_xyz[2]}",
                                      rpy=f"{link.origin_rpy[0]} {link.origin_rpy[1]} {link.origin_rpy[2]}")
        visual_geom = ET.SubElement(visual, "geometry")

        if use_mesh:
            ET.SubElement(visual_geom, "mesh", filename=mesh_file)
        else:
            # Use small box for finger links
            size = "0.01 0.008 0.006"  # 10x8x6 mm
            if "thumb" in link.name:
                size = "0.012 0.01 0.008"
            ET.SubElement(visual_geom, "box", size=size)

            # Add color for visibility
            material = ET.SubElement(visual, "material", name="finger_color")
            ET.SubElement(material, "color", rgba="0.8 0.6 0.5 1.0")

        # Collision (use boxes for all links - more efficient)
        collision = ET.SubElement(link_elem, "collision")
        coll_origin = ET.SubElement(collision, "origin",
                                    xyz=f"{link.origin_xyz[0]} {link.origin_xyz[1]} {link.origin_xyz[2]}",
                                    rpy=f"{link.origin_rpy[0]} {link.origin_rpy[1]} {link.origin_rpy[2]}")
        coll_geom = ET.SubElement(collision, "geometry")

        if link.name == "base_link":
            # Approximate palm as box
            ET.SubElement(coll_geom, "box", size="0.1 0.08 0.03")
        else:
            # Finger links as small boxes
            size = "0.01 0.008 0.006"
            if "thumb" in link.name:
                size = "0.012 0.01 0.008"
            ET.SubElement(coll_geom, "box", size=size)

        # Inertial
        inertial = ET.SubElement(link_elem, "inertial")
        ET.SubElement(inertial, "mass", value=str(link.mass))
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        # Simplified inertia (box approximation)
        ixx = link.mass * link.inertia_scale
        ET.SubElement(inertial, "inertia",
                      ixx=str(ixx), ixy="0", ixz="0",
                      iyy=str(ixx), iyz="0", izz=str(ixx))

    def _add_joint_element(self, parent: ET.Element, joint: JointKinematics):
        """Add a joint element to the URDF."""
        joint_elem = ET.SubElement(parent, "joint",
                                   name=joint.name,
                                   type=joint.joint_type)

        ET.SubElement(joint_elem, "parent", link=joint.parent_link)
        ET.SubElement(joint_elem, "child", link=joint.child_link)

        ET.SubElement(joint_elem, "origin",
                      xyz=f"{joint.origin_xyz[0]} {joint.origin_xyz[1]} {joint.origin_xyz[2]}",
                      rpy=f"{joint.origin_rpy[0]} {joint.origin_rpy[1]} {joint.origin_rpy[2]}")

        if joint.joint_type != "fixed":
            ET.SubElement(joint_elem, "axis",
                          xyz=f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}")

            ET.SubElement(joint_elem, "limit",
                          lower=str(joint.lower_limit),
                          upper=str(joint.upper_limit),
                          effort=str(joint.effort),
                          velocity=str(joint.velocity))

        # Add mimic if specified
        if joint.mimic_joint:
            ET.SubElement(joint_elem, "mimic",
                          joint=joint.mimic_joint,
                          multiplier=str(joint.mimic_multiplier),
                          offset=str(joint.mimic_offset))

    def _prettify(self, xml_string: str) -> str:
        """Pretty print XML with indentation."""
        dom = minidom.parseString(xml_string)
        return dom.toprettyxml(indent="  ")


def main():
    """Generate URDF files."""
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    urdf_dir = workspace / 'src' / 'inspire_hand_description' / 'urdf'
    urdf_dir.mkdir(parents=True, exist_ok=True)

    # Generate right hand
    print("Generating right hand URDF...")
    generator_right = InspireHandURDFGenerator(hand_side="right")
    urdf_right = generator_right.generate()

    urdf_right_path = urdf_dir / "inspire_hand_right.urdf"
    with open(urdf_right_path, 'w') as f:
        f.write(urdf_right)
    print(f"  Saved: {urdf_right_path}")

    # Generate left hand
    print("Generating left hand URDF...")
    generator_left = InspireHandURDFGenerator(hand_side="left")
    urdf_left = generator_left.generate()

    urdf_left_path = urdf_dir / "inspire_hand_left.urdf"
    with open(urdf_left_path, 'w') as f:
        f.write(urdf_left)
    print(f"  Saved: {urdf_left_path}")

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
