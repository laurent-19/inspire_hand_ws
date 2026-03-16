"""
Mesh sampling module for tactile point cloud visualization.

Loads URDF, extracts mesh paths, and samples points from STL files.
"""

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
import numpy as np

try:
    import trimesh
except ImportError:
    raise ImportError("trimesh is required. Install with: pip3 install trimesh")


@dataclass
class MeshPoints:
    """Sampled points from a mesh with metadata."""
    link_name: str                    # e.g., "right_thumb_4"
    points_local: np.ndarray          # Shape (N, 3) - points in link frame
    normals_local: np.ndarray         # Shape (N, 3) - surface normals
    tactile_region: Optional[str]     # e.g., "thumb_tip", "index_pad"


class MeshSampler:
    """Load and sample points from URDF mesh files."""

    # Default points per taxel for proportional sampling
    # 16 points per taxel = 16 bits (2 bytes) per taxel in hardware
    DEFAULT_POINTS_PER_TAXEL = 16

    # Define which links have tactile sensors, their regions, and taxel counts
    # Tactile sensors are on the force sensor pads
    # Format: link_name -> (region_name, num_taxels)
    TACTILE_LINKS = {
        # Thumb (4 regions, 210 taxels total) - per hardware doc
        # sensor_1=pad(96), sensor_2=middle(9), sensor_3=nail(96), sensor_4=tip(9)
        'right_thumb_force_sensor_4': ('thumb_tip', 9),       # 3×3 grid
        'right_thumb_force_sensor_3': ('thumb_nail', 96),     # 12×8 grid
        'right_thumb_force_sensor_2': ('thumb_middle', 9),    # 3×3 grid
        'right_thumb_force_sensor_1': ('thumb_pad', 96),      # 12×8 grid

        # Index finger (3 regions, 185 taxels)
        'right_index_force_sensor_3': ('index_tip', 9),       # 3×3 grid
        'right_index_force_sensor_2': ('index_nail', 96),     # 12×8 grid
        'right_index_force_sensor_1': ('index_pad', 80),      # 10×8 grid

        # Middle finger (3 regions, 185 taxels)
        'right_middle_force_sensor_3': ('middle_tip', 9),
        'right_middle_force_sensor_2': ('middle_nail', 96),
        'right_middle_force_sensor_1': ('middle_pad', 80),

        # Ring finger (3 regions, 185 taxels)
        'right_ring_force_sensor_3': ('ring_tip', 9),
        'right_ring_force_sensor_2': ('ring_nail', 96),
        'right_ring_force_sensor_1': ('ring_pad', 80),

        # Little finger (3 regions, 185 taxels)
        'right_little_force_sensor_3': ('little_tip', 9),
        'right_little_force_sensor_2': ('little_nail', 96),
        'right_little_force_sensor_1': ('little_pad', 80),

        # Palm (112 taxels, 8×14 grid)
        'right_palm_force_sensor': ('palm', 112),
    }

    # Point density for non-tactile links
    NON_TACTILE_DENSITY = 200

    def __init__(self, urdf_path: str, package_path: Optional[str] = None,
                 points_per_taxel: int = None):
        """
        Initialize mesh sampler.

        Args:
            urdf_path: Path to URDF file
            package_path: Root path for resolving package:// URIs
            points_per_taxel: Points to sample per tactile sensor (default: 16)
        """
        self.urdf_path = urdf_path
        self.package_path = package_path or self._find_package_path(urdf_path)
        self.points_per_taxel = points_per_taxel or self.DEFAULT_POINTS_PER_TAXEL
        self.mesh_cache: Dict[str, MeshPoints] = {}

    def _find_package_path(self, urdf_path: str) -> str:
        """Find package root from URDF path."""
        # Assume urdf_path is like .../inspire_hand_description/urdf/rh56e2_right.urdf
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        package_dir = os.path.dirname(urdf_dir)  # Go up one level
        return package_dir

    def _resolve_package_uri(self, uri: str) -> str:
        """Resolve package:// URI to absolute path."""
        if uri.startswith('package://'):
            # Extract relative path after package://package_name/
            parts = uri.replace('package://', '').split('/', 1)
            if len(parts) == 2:
                package_name, rel_path = parts
                # Construct absolute path
                return os.path.join(self.package_path, rel_path)
        return uri

    def _get_sample_count(self, link_name: str) -> int:
        """
        Determine number of points to sample based on link type.

        For tactile links: points = num_taxels × points_per_taxel
        This gives equal visual weight to each tactile sensor.
        """
        if link_name in self.TACTILE_LINKS:
            region_name, num_taxels = self.TACTILE_LINKS[link_name]
            return num_taxels * self.points_per_taxel
        else:
            return self.NON_TACTILE_DENSITY

    def load_and_sample_meshes(self) -> Dict[str, MeshPoints]:
        """
        Load URDF and sample points from all visual meshes.

        Returns:
            Dictionary mapping link names to sampled mesh points
        """
        # Parse URDF
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        # Extract all links with visual meshes
        for link in root.findall('link'):
            link_name = link.get('name')

            # Find visual/geometry/mesh element
            visual = link.find('visual')
            if visual is None:
                continue

            geometry = visual.find('geometry')
            if geometry is None:
                continue

            mesh_elem = geometry.find('mesh')
            if mesh_elem is None:
                continue

            # Get mesh filename
            mesh_uri = mesh_elem.get('filename')
            if mesh_uri is None:
                continue

            # Resolve to absolute path
            mesh_path = self._resolve_package_uri(mesh_uri)

            if not os.path.exists(mesh_path):
                print(f"Warning: Mesh file not found: {mesh_path}")
                continue

            # Load and sample mesh
            try:
                mesh_points = self._sample_mesh(link_name, mesh_path)
                self.mesh_cache[link_name] = mesh_points
                print(f"Sampled {len(mesh_points.points_local)} points from {link_name}")
            except Exception as e:
                print(f"Error sampling mesh {link_name}: {e}")

        print(f"\nLoaded {len(self.mesh_cache)} meshes with "
              f"{sum(len(m.points_local) for m in self.mesh_cache.values())} total points")

        return self.mesh_cache

    def _sample_mesh(self, link_name: str, mesh_path: str) -> MeshPoints:
        """
        Sample points from a single mesh file.

        Args:
            link_name: Name of the link
            mesh_path: Path to STL file

        Returns:
            MeshPoints with sampled data
        """
        # Load mesh
        mesh = trimesh.load(mesh_path)

        # Determine sample count
        num_points = self._get_sample_count(link_name)

        # Sample points uniformly on surface
        points, face_indices = trimesh.sample.sample_surface_even(mesh, num_points)

        # Get normals at sampled points
        normals = mesh.face_normals[face_indices]

        # Determine tactile region
        tactile_info = self.TACTILE_LINKS.get(link_name, None)
        tactile_region = tactile_info[0] if tactile_info else None

        return MeshPoints(
            link_name=link_name,
            points_local=points.astype(np.float32),
            normals_local=normals.astype(np.float32),
            tactile_region=tactile_region
        )

    def get_tactile_links(self) -> Dict[str, MeshPoints]:
        """Get only meshes with tactile sensors."""
        return {
            name: mesh for name, mesh in self.mesh_cache.items()
            if name in self.TACTILE_LINKS
        }

    def get_all_points(self) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
        """
        Get all sampled points concatenated.

        Returns:
            Tuple of (all_points, link_indices) where:
            - all_points: (N, 3) array of all points
            - link_indices: dict mapping link_name to indices in all_points
        """
        all_points = []
        link_indices = {}
        current_idx = 0

        for link_name, mesh_points in self.mesh_cache.items():
            num_points = len(mesh_points.points_local)
            all_points.append(mesh_points.points_local)
            link_indices[link_name] = np.arange(current_idx, current_idx + num_points)
            current_idx += num_points

        return np.vstack(all_points), link_indices


