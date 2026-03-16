"""
Forward kinematics solver for Inspire Hand.

Computes transform matrices for all links based on joint states.
"""

import xml.etree.ElementTree as ET
from typing import Dict, Tuple, Optional
import numpy as np


class KinematicsSolver:
    """Forward kinematics solver using URDF transform tree."""

    # Mapping from InspireHandState DOF indices to URDF joint names
    DOF_TO_JOINT = {
        0: 'right_little_1_joint',   # Little finger
        1: 'right_ring_1_joint',     # Ring finger
        2: 'right_middle_1_joint',   # Middle finger
        3: 'right_index_1_joint',    # Index finger
        4: 'right_thumb_2_joint',    # Thumb bend
        5: 'right_thumb_1_joint',    # Thumb rotation
    }

    # Mimic joint definitions (child_joint: (parent_joint, multiplier))
    MIMIC_JOINTS = {
        'right_thumb_3_joint': ('right_thumb_2_joint', 0.8392),
        'right_thumb_4_joint': ('right_thumb_3_joint', 0.891),
        'right_index_2_joint': ('right_index_1_joint', 1.0843),
        'right_middle_2_joint': ('right_middle_1_joint', 1.0843),
        'right_ring_2_joint': ('right_ring_1_joint', 1.0843),
        'right_little_2_joint': ('right_little_1_joint', 1.0843),
    }

    def __init__(self, urdf_path: str):
        """
        Initialize kinematics solver.

        Args:
            urdf_path: Path to URDF file
        """
        self.urdf_path = urdf_path
        self.joints = {}  # joint_name -> (parent_link, child_link, origin, axis, type)
        self.links = {}   # link_name -> parent_joint
        self._parse_urdf()

    def _parse_urdf(self):
        """Parse URDF to extract joint and link information."""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        # Extract all joints
        for joint_elem in root.findall('joint'):
            joint_name = joint_elem.get('name')
            joint_type = joint_elem.get('type')

            parent = joint_elem.find('parent').get('link')
            child = joint_elem.find('child').get('link')

            # Parse origin (xyz, rpy)
            origin_elem = joint_elem.find('origin')
            if origin_elem is not None:
                xyz = [float(x) for x in origin_elem.get('xyz', '0 0 0').split()]
                rpy = [float(x) for x in origin_elem.get('rpy', '0 0 0').split()]
            else:
                xyz = [0, 0, 0]
                rpy = [0, 0, 0]

            # Parse axis
            axis_elem = joint_elem.find('axis')
            if axis_elem is not None:
                axis = [float(x) for x in axis_elem.get('xyz', '1 0 0').split()]
            else:
                axis = [1, 0, 0]

            self.joints[joint_name] = {
                'parent': parent,
                'child': child,
                'type': joint_type,
                'xyz': np.array(xyz),
                'rpy': np.array(rpy),
                'axis': np.array(axis)
            }

        print(f"Parsed {len(self.joints)} joints from URDF")

    @staticmethod
    def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert roll-pitch-yaw to rotation matrix."""
        # Rotation around X (roll)
        cr, sr = np.cos(roll), np.sin(roll)
        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr, cr]])

        # Rotation around Y (pitch)
        cp, sp = np.cos(pitch), np.sin(pitch)
        R_y = np.array([[cp, 0, sp],
                        [0, 1, 0],
                        [-sp, 0, cp]])

        # Rotation around Z (yaw)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R_z = np.array([[cy, -sy, 0],
                        [sy, cy, 0],
                        [0, 0, 1]])

        return R_z @ R_y @ R_x

    @staticmethod
    def _transform_matrix(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        """Create 4x4 homogeneous transform matrix."""
        T = np.eye(4)
        T[:3, :3] = KinematicsSolver._rpy_to_matrix(rpy[0], rpy[1], rpy[2])
        T[:3, 3] = xyz
        return T

    @staticmethod
    def _rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
        """Rodrigues' rotation formula."""
        axis = axis / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def _compute_joint_transform(self, joint_name: str, joint_angle: float) -> np.ndarray:
        """Compute transform for a single joint given its angle."""
        joint = self.joints[joint_name]

        # Base transform from URDF origin
        T_base = self._transform_matrix(joint['xyz'], joint['rpy'])

        # Additional rotation from joint angle (if revolute)
        if joint['type'] == 'revolute':
            R_joint = self._rotation_matrix_from_axis_angle(joint['axis'], joint_angle)
            T_joint = np.eye(4)
            T_joint[:3, :3] = R_joint
            return T_base @ T_joint
        else:
            return T_base

    def _resolve_mimic_joints(self, joint_positions: Dict[str, float]) -> Dict[str, float]:
        """Add mimic joint values based on their parent joints."""
        resolved = joint_positions.copy()

        # Iteratively resolve mimic joints (in case of chained mimics like thumb)
        max_iterations = 10
        for _ in range(max_iterations):
            updated = False
            for mimic_joint, (parent_joint, multiplier) in self.MIMIC_JOINTS.items():
                if parent_joint in resolved and mimic_joint not in resolved:
                    resolved[mimic_joint] = resolved[parent_joint] * multiplier
                    updated = True

            if not updated:
                break

        return resolved

    def angle_actual_to_radians(self, angle_actual: int) -> float:
        """
        Convert angle_actual (0-1000) to radians.

        This is an empirical mapping that may need calibration.
        Actual mapping: 1000 = fully open (0 rad) and 0 = fully closed (~1.57 rad)
        """
        # Inverted linear mapping: 1000 units (open) → 0 rad, 0 units (closed) → π/2 radians
        return ((1000.0 - angle_actual) / 1000.0) * (np.pi / 2.0)

    def compute_all_transforms(self,
                               joint_positions: Optional[Dict[str, float]] = None,
                               angle_actual: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """
        Compute transform matrix for all links.

        Args:
            joint_positions: Dictionary of joint_name -> angle in radians
                           OR
            angle_actual: Array of 6 DOF values from InspireHandState (0-1000)

        Returns:
            Dictionary mapping link_name to 4x4 transform matrix (from base_footprint)
        """
        # Convert angle_actual to joint positions if provided
        if angle_actual is not None:
            joint_positions = {}
            for dof_idx, joint_name in self.DOF_TO_JOINT.items():
                if dof_idx < len(angle_actual):
                    joint_positions[joint_name] = self.angle_actual_to_radians(angle_actual[dof_idx])

        # Default to zero positions if not provided
        if joint_positions is None:
            joint_positions = {name: 0.0 for name in self.DOF_TO_JOINT.values()}

        # Resolve mimic joints
        joint_positions = self._resolve_mimic_joints(joint_positions)

        # Recursively compute transforms using DFS from base_footprint
        link_transforms = {}
        self._compute_link_transform_recursive('base_footprint', np.eye(4),
                                               joint_positions, link_transforms)

        return link_transforms

    def _compute_link_transform_recursive(self,
                                         link_name: str,
                                         parent_transform: np.ndarray,
                                         joint_positions: Dict[str, float],
                                         link_transforms: Dict[str, np.ndarray]):
        """Recursively compute transforms for all links in the tree."""
        # Store this link's transform
        link_transforms[link_name] = parent_transform

        # Find all child joints
        for joint_name, joint_info in self.joints.items():
            if joint_info['parent'] == link_name:
                # Get joint angle (default to 0 if not specified)
                joint_angle = joint_positions.get(joint_name, 0.0)

                # Compute joint transform
                joint_transform = self._compute_joint_transform(joint_name, joint_angle)

                # Combine with parent transform
                child_transform = parent_transform @ joint_transform

                # Recurse to child link
                child_link = joint_info['child']
                self._compute_link_transform_recursive(child_link, child_transform,
                                                      joint_positions, link_transforms)


