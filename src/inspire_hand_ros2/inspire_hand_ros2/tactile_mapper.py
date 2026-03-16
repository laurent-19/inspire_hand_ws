"""
Tactile sensor mapping to mesh points with color coding.

Maps 1062 tactile sensor values to mesh point colors.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class TactileRegion:
    """Definition of a tactile sensor region."""
    name: str           # e.g., "thumb_tip"
    link_name: str      # e.g., "right_thumb_4"
    grid_shape: Tuple[int, int]  # e.g., (3, 3) for tip
    num_taxels: int     # Total number of sensors


class TactileMapper:
    """Maps tactile sensor values to mesh points and generates colors."""

    # Tactile region definitions matching viz_data.py
    # Map to force sensor meshes where the actual tactile pads are located
    TACTILE_REGIONS = [
        # Little finger (185 taxels) - on force sensors
        TactileRegion("little_tip", "right_little_force_sensor_3", (3, 3), 9),
        TactileRegion("little_nail", "right_little_force_sensor_2", (12, 8), 96),
        TactileRegion("little_pad", "right_little_force_sensor_1", (10, 8), 80),

        # Ring finger (185 taxels) - on force sensors
        TactileRegion("ring_tip", "right_ring_force_sensor_3", (3, 3), 9),
        TactileRegion("ring_nail", "right_ring_force_sensor_2", (12, 8), 96),
        TactileRegion("ring_pad", "right_ring_force_sensor_1", (10, 8), 80),

        # Middle finger (185 taxels) - on force sensors
        TactileRegion("middle_tip", "right_middle_force_sensor_3", (3, 3), 9),
        TactileRegion("middle_nail", "right_middle_force_sensor_2", (12, 8), 96),
        TactileRegion("middle_pad", "right_middle_force_sensor_1", (10, 8), 80),

        # Index finger (185 taxels) - on force sensors
        TactileRegion("index_tip", "right_index_force_sensor_3", (3, 3), 9),
        TactileRegion("index_nail", "right_index_force_sensor_2", (12, 8), 96),
        TactileRegion("index_pad", "right_index_force_sensor_1", (10, 8), 80),

        # Thumb (210 taxels) - on force sensors
        TactileRegion("thumb_tip", "right_thumb_force_sensor_4", (3, 3), 9),
        TactileRegion("thumb_nail", "right_thumb_force_sensor_1", (12, 8), 96),
        TactileRegion("thumb_middle", "right_thumb_force_sensor_3", (3, 3), 9),
        TactileRegion("thumb_pad", "right_thumb_force_sensor_2", (12, 8), 96),

        # Palm (112 taxels) - on palm force sensor
        TactileRegion("palm", "right_palm_force_sensor", (14, 8), 112),
    ]

    def __init__(self, mesh_points: Dict, colormap_min: float = 0.0, colormap_max: float = 4095.0):
        """
        Initialize tactile mapper.

        Args:
            mesh_points: Dictionary from mesh_sampler (link_name -> MeshPoints)
            colormap_min: Minimum tactile value for colormap (default: 0)
            colormap_max: Maximum tactile value for colormap (default: 4095, 12-bit ADC)
        """
        self.mesh_points = mesh_points
        self.colormap_min = colormap_min
        self.colormap_max = colormap_max

        # Build mapping from mesh points to tactile sensors
        self.point_to_taxel = {}  # link_name -> array of taxel indices per point
        self._build_point_mappings()

    def _build_point_mappings(self):
        """Build mapping from mesh points to tactile sensor indices."""
        for region in self.TACTILE_REGIONS:
            if region.link_name not in self.mesh_points:
                continue

            mesh = self.mesh_points[region.link_name]
            num_points = len(mesh.points_local)

            # Divide mesh points into grid zones
            taxel_indices = self._assign_points_to_grid(
                mesh.points_local,
                mesh.normals_local,
                region.grid_shape,
                region.name
            )

            self.point_to_taxel[region.link_name] = {
                'region': region,
                'taxel_indices': taxel_indices  # Shape: (num_points,) - taxel index for each point
            }

    def _assign_points_to_grid(self,
                               points: np.ndarray,
                               normals: np.ndarray,
                               grid_shape: Tuple[int, int],
                               region_name: str) -> np.ndarray:
        """
        Assign each mesh point to a taxel in the grid.

        Args:
            points: (N, 3) array of points in local frame
            normals: (N, 3) array of surface normals
            grid_shape: (rows, cols) of taxel grid
            region_name: Name of tactile region for determining projection

        Returns:
            (N,) array of taxel indices (0 to rows*cols-1)
        """
        num_points = len(points)
        rows, cols = grid_shape

        # Different projection strategies for different regions
        if 'tip' in region_name or 'middle' in region_name:
            # For tips: use XY projection (looking down from above)
            coords = points[:, :2]  # X, Y
        elif 'nail' in region_name:
            # For nails: use cylindrical coordinates around Z axis
            # Angle around Z + Z height
            angles = np.arctan2(points[:, 1], points[:, 0])
            z_vals = points[:, 2]
            coords = np.column_stack([angles, z_vals])
        elif 'pad' in region_name:
            # For pads: use Y-Z projection (side view)
            coords = points[:, 1:3]  # Y, Z
        elif 'palm' in region_name:
            # For palm: use X-Y projection
            coords = points[:, :2]  # X, Y
        else:
            # Default: use first two coordinates
            coords = points[:, :2]

        # Normalize coordinates to [0, 1] range
        coord_min = coords.min(axis=0)
        coord_max = coords.max(axis=0)
        coord_range = coord_max - coord_min
        coord_range[coord_range < 1e-6] = 1.0  # Avoid division by zero

        coords_normalized = (coords - coord_min) / coord_range

        # Map to grid indices
        row_indices = np.clip((coords_normalized[:, 1] * rows).astype(int), 0, rows - 1)
        col_indices = np.clip((coords_normalized[:, 0] * cols).astype(int), 0, cols - 1)

        # Convert to linear taxel index
        taxel_indices = row_indices * cols + col_indices

        return taxel_indices

    def tactile_to_rgb(self, value: float) -> Tuple[int, int, int]:
        """
        Convert tactile value to RGB color using heatmap.

        Colormap: Blue (low) -> Cyan -> Green -> Yellow -> Red (high)

        Args:
            value: Tactile sensor value

        Returns:
            (r, g, b) tuple with values 0-255
        """
        # Normalize to [0, 1]
        norm = np.clip((value - self.colormap_min) / (self.colormap_max - self.colormap_min), 0, 1)

        # Multi-stage colormap (similar to matplotlib 'jet' or 'turbo')
        if norm < 0.25:
            # Blue to Cyan
            t = norm / 0.25
            r = 0
            g = int(255 * t)
            b = 255
        elif norm < 0.5:
            # Cyan to Green
            t = (norm - 0.25) / 0.25
            r = 0
            g = 255
            b = int(255 * (1 - t))
        elif norm < 0.75:
            # Green to Yellow
            t = (norm - 0.5) / 0.25
            r = int(255 * t)
            g = 255
            b = 0
        else:
            # Yellow to Red
            t = (norm - 0.75) / 0.25
            r = 255
            g = int(255 * (1 - t))
            b = 0

        return (r, g, b)

    def pack_rgb(self, r: int, g: int, b: int) -> int:
        """Pack RGB values into a single 32-bit integer."""
        return (r << 16) | (g << 8) | b

    def apply_tactile_colors(self, tactile_data: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Apply tactile sensor colors to mesh points.

        Args:
            tactile_data: Dictionary mapping region names to tactile arrays
                         Example: {'little_tip': array[9], 'little_nail': array[96], ...}

        Returns:
            Dictionary mapping link names to RGB color arrays (packed uint32)
        """
        link_colors = {}

        for link_name, mapping in self.point_to_taxel.items():
            region = mapping['region']
            taxel_indices = mapping['taxel_indices']
            num_points = len(taxel_indices)

            # Get tactile values for this region
            if region.name not in tactile_data:
                # No data - use default blue color
                colors = np.full(num_points, self.pack_rgb(0, 0, 255), dtype=np.uint32)
            else:
                tactile_values = tactile_data[region.name]

                # Map each point to its corresponding tactile value
                point_values = tactile_values[taxel_indices]

                # Convert to RGB
                colors = np.array([
                    self.pack_rgb(*self.tactile_to_rgb(val))
                    for val in point_values
                ], dtype=np.uint32)

            link_colors[link_name] = colors

        # Add default colors for non-tactile links
        for link_name, mesh in self.mesh_points.items():
            if link_name not in link_colors:
                # Gray color for non-tactile parts
                num_points = len(mesh.points_local)
                link_colors[link_name] = np.full(num_points, self.pack_rgb(128, 128, 128), dtype=np.uint32)

        return link_colors

    def parse_tactile_message(self, touch_msg) -> Dict[str, np.ndarray]:
        """
        Parse InspireHandTouch ROS message into tactile data dict.

        Args:
            touch_msg: InspireHandTouch message

        Returns:
            Dictionary mapping region names to numpy arrays
        """
        tactile_data = {}

        # Helper to extract finger data
        def extract_finger_data(finger_msg, prefix):
            if hasattr(finger_msg, 'tip') and len(finger_msg.tip) > 0:
                tactile_data[f"{prefix}_tip"] = np.array(finger_msg.tip, dtype=np.float32)
            if hasattr(finger_msg, 'nail') and len(finger_msg.nail) > 0:
                tactile_data[f"{prefix}_nail"] = np.array(finger_msg.nail, dtype=np.float32)
            if hasattr(finger_msg, 'pad') and len(finger_msg.pad) > 0:
                tactile_data[f"{prefix}_pad"] = np.array(finger_msg.pad, dtype=np.float32)
            if hasattr(finger_msg, 'middle_section') and len(finger_msg.middle_section) > 0:
                tactile_data[f"{prefix}_middle"] = np.array(finger_msg.middle_section, dtype=np.float32)

        # Extract data for each finger
        if hasattr(touch_msg, 'little'):
            extract_finger_data(touch_msg.little, 'little')
        if hasattr(touch_msg, 'ring'):
            extract_finger_data(touch_msg.ring, 'ring')
        if hasattr(touch_msg, 'middle'):
            extract_finger_data(touch_msg.middle, 'middle')
        if hasattr(touch_msg, 'index'):
            extract_finger_data(touch_msg.index, 'index')
        if hasattr(touch_msg, 'thumb'):
            extract_finger_data(touch_msg.thumb, 'thumb')

        # Palm data
        if hasattr(touch_msg, 'palm') and len(touch_msg.palm) > 0:
            tactile_data['palm'] = np.array(touch_msg.palm, dtype=np.float32)

        return tactile_data


