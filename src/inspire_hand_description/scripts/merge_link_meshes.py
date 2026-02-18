#!/usr/bin/env python3
"""
Merge Link Meshes for Inspire Hand

Reads mesh mapping YAML files and merges part STLs into per-link meshes.
Applies coordinate transformations to match reference URDF coordinate system.

Coordinate Systems:
  CAD (our meshes): X across fingers, Y toward fingertips, Z along palm
  Reference URDF:   Y along finger, joint at origin for finger links

Transformations:
  - Base link: Swap X<->Y and shift Z to match reference frame
  - Finger links: Translate so joint is at origin, rotate to match reference
  - Thumb links: Complex transform (handled separately)
"""

import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

try:
    import numpy as np
    from stl import mesh as stl_mesh
except ImportError as e:
    print(f"ERROR: Missing numpy-stl: {e}")
    print("Install with: pip install numpy-stl")
    sys.exit(1)


def load_mapping(yaml_path: Path) -> Dict:
    """Load mesh mapping from YAML file."""
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


def get_all_vertices(mesh: stl_mesh.Mesh) -> np.ndarray:
    """Extract all vertices from mesh as Nx3 array."""
    # mesh.vectors has shape (n_triangles, 3, 3) - 3 vertices per triangle, 3 coords per vertex
    return mesh.vectors.reshape(-1, 3)


def set_all_vertices(mesh: stl_mesh.Mesh, vertices: np.ndarray) -> None:
    """Update mesh vertices from Nx3 array."""
    mesh.vectors = vertices.reshape(-1, 3, 3)


def transform_base_link(vertices: np.ndarray) -> np.ndarray:
    """
    Transform base link mesh from CAD to reference frame.

    CAD:       X across fingers, Y toward fingertips, Z along palm (0.02-0.15)
    Reference: X out of palm, Y across fingers, Z along palm (0-0.14)

    Transform:
      - Swap X and Y axes (with sign flip for right hand: little at -Y)
      - Shift Z down by ~0.02m
    """
    # Get bounds for analysis
    x_min, x_max = vertices[:, 0].min(), vertices[:, 0].max()
    y_min, y_max = vertices[:, 1].min(), vertices[:, 1].max()
    z_min, z_max = vertices[:, 2].min(), vertices[:, 2].max()

    print(f"    CAD bounds: X[{x_min:.4f}, {x_max:.4f}] Y[{y_min:.4f}, {y_max:.4f}] Z[{z_min:.4f}, {z_max:.4f}]")

    transformed = np.zeros_like(vertices)
    # Reference has: little at -Y, index at +Y  (Y across fingers)
    # CAD has: little at -X, index at +X  (X across fingers)
    # So: ref_Y = cad_X

    # Reference has: X out of palm (depth)
    # CAD has: Y toward fingertips (but at palm, Y is small)
    # So: ref_X = cad_Y

    # Reference has: Z along palm (0 to 0.14)
    # CAD has: Z along palm (0.02 to 0.15)
    # So: ref_Z = cad_Z - z_min (shift to start at 0)

    transformed[:, 0] = vertices[:, 1]  # ref_X = cad_Y
    transformed[:, 1] = vertices[:, 0]  # ref_Y = cad_X
    transformed[:, 2] = vertices[:, 2] - z_min  # ref_Z = cad_Z shifted

    new_x_min, new_x_max = transformed[:, 0].min(), transformed[:, 0].max()
    new_y_min, new_y_max = transformed[:, 1].min(), transformed[:, 1].max()
    new_z_min, new_z_max = transformed[:, 2].min(), transformed[:, 2].max()
    print(f"    Ref bounds: X[{new_x_min:.4f}, {new_x_max:.4f}] Y[{new_y_min:.4f}, {new_y_max:.4f}] Z[{new_z_min:.4f}, {new_z_max:.4f}]")

    return transformed


def transform_finger_link(vertices: np.ndarray, joint_position: List[float]) -> np.ndarray:
    """
    Transform finger link mesh from CAD to reference frame.

    CAD:       X across fingers, Y toward fingertips, Z along palm
    Reference: Y along finger, joint at origin, Z perpendicular to finger face

    Transform:
      1. Translate to put joint at origin
      2. Rotate to match reference orientation:
         - ref_X = cad_Z (perpendicular to palm -> perpendicular to finger)
         - ref_Y = cad_Y (along finger stays)
         - ref_Z = -cad_X (across fingers -> across finger width)
    """
    jx, jy, jz = joint_position

    # Get original bounds
    x_min, x_max = vertices[:, 0].min(), vertices[:, 0].max()
    y_min, y_max = vertices[:, 1].min(), vertices[:, 1].max()
    z_min, z_max = vertices[:, 2].min(), vertices[:, 2].max()
    print(f"    CAD bounds: X[{x_min:.4f}, {x_max:.4f}] Y[{y_min:.4f}, {y_max:.4f}] Z[{z_min:.4f}, {z_max:.4f}]")
    print(f"    Joint pos:  ({jx:.4f}, {jy:.4f}, {jz:.4f})")

    # Step 1: Translate to put joint at origin
    translated = vertices - np.array([jx, jy, jz])

    # Step 2: Rotate coordinate system
    # CAD has Y along finger (good), but X is across fingers and Z is up palm
    # Reference has Y along finger, X out of finger face, Z across finger width
    transformed = np.zeros_like(translated)
    transformed[:, 0] = translated[:, 2]   # ref_X = cad_Z (depth out of finger)
    transformed[:, 1] = translated[:, 1]   # ref_Y = cad_Y (along finger)
    transformed[:, 2] = -translated[:, 0]  # ref_Z = -cad_X (across finger width)

    new_x_min, new_x_max = transformed[:, 0].min(), transformed[:, 0].max()
    new_y_min, new_y_max = transformed[:, 1].min(), transformed[:, 1].max()
    new_z_min, new_z_max = transformed[:, 2].min(), transformed[:, 2].max()
    print(f"    Ref bounds: X[{new_x_min:.4f}, {new_x_max:.4f}] Y[{new_y_min:.4f}, {new_y_max:.4f}] Z[{new_z_min:.4f}, {new_z_max:.4f}]")

    return transformed


def transform_thumb_link(vertices: np.ndarray, link_name: str) -> np.ndarray:
    """
    Transform thumb link mesh from CAD to reference frame.

    Thumb has complex orientation due to opposition. For now, apply minimal transform.
    The thumb's orientation in the reference URDF uses significant rpy rotations in joints.
    """
    # For thumb, just center the mesh around its centroid for now
    # This is a placeholder - proper thumb transform needs more analysis
    centroid = vertices.mean(axis=0)

    x_min, x_max = vertices[:, 0].min(), vertices[:, 0].max()
    y_min, y_max = vertices[:, 1].min(), vertices[:, 1].max()
    z_min, z_max = vertices[:, 2].min(), vertices[:, 2].max()
    print(f"    CAD bounds: X[{x_min:.4f}, {x_max:.4f}] Y[{y_min:.4f}, {y_max:.4f}] Z[{z_min:.4f}, {z_max:.4f}]")
    print(f"    Centroid:   ({centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f})")

    # Translate to center at origin (joint roughly at centroid for initial guess)
    transformed = vertices - centroid

    # Apply rotation similar to base link but accounting for thumb orientation
    # Thumb in CAD also has X across, Y toward tip, Z up
    # But thumb points in a different direction than fingers
    rotated = np.zeros_like(transformed)
    rotated[:, 0] = transformed[:, 1]   # ref_X = cad_Y
    rotated[:, 1] = transformed[:, 0]   # ref_Y = cad_X
    rotated[:, 2] = transformed[:, 2]   # ref_Z = cad_Z

    new_x_min, new_x_max = rotated[:, 0].min(), rotated[:, 0].max()
    new_y_min, new_y_max = rotated[:, 1].min(), rotated[:, 1].max()
    new_z_min, new_z_max = rotated[:, 2].min(), rotated[:, 2].max()
    print(f"    Ref bounds: X[{new_x_min:.4f}, {new_x_max:.4f}] Y[{new_y_min:.4f}, {new_y_max:.4f}] Z[{new_z_min:.4f}, {new_z_max:.4f}]")

    return rotated


def merge_and_transform_stl_files(
    stl_paths: List[Path],
    transform_type: str,
    joint_position: Optional[List[float]] = None,
    link_name: str = ""
) -> Optional[stl_mesh.Mesh]:
    """
    Load, transform, and merge multiple STL files into a single mesh.

    Args:
        stl_paths: List of paths to STL files
        transform_type: One of 'base', 'finger', 'thumb', or None
        joint_position: Joint position in CAD coords (for finger links)
        link_name: Name of the link (for logging)

    Returns:
        Merged and transformed mesh
    """
    if not stl_paths:
        return None

    meshes = []
    for path in stl_paths:
        try:
            m = stl_mesh.Mesh.from_file(str(path))
            meshes.append(m)
        except Exception as e:
            print(f"  Warning: Failed to load {path.name}: {e}")

    if not meshes:
        return None

    # First merge all meshes (they're already in correct positions relative to each other)
    if len(meshes) == 1:
        merged = meshes[0]
    else:
        combined_data = np.concatenate([m.data for m in meshes])
        merged = stl_mesh.Mesh(combined_data)

    # Get all vertices for transformation
    vertices = get_all_vertices(merged)

    # Apply coordinate transformation based on link type
    if transform_type == 'base':
        transformed_vertices = transform_base_link(vertices)
    elif transform_type == 'finger' and joint_position is not None:
        transformed_vertices = transform_finger_link(vertices, joint_position)
    elif transform_type == 'thumb':
        transformed_vertices = transform_thumb_link(vertices, link_name)
    else:
        # No transformation
        print(f"    No transform applied")
        transformed_vertices = vertices

    # Update mesh with transformed vertices
    set_all_vertices(merged, transformed_vertices)

    # Recalculate normals after transformation
    merged.update_normals()

    return merged


def main():
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    pkg_dir = workspace / 'src' / 'inspire_hand_description'
    config_dir = pkg_dir / 'config'

    for side in ['right', 'left']:
        mapping_path = config_dir / f'mesh_mapping_{side}.yaml'

        if not mapping_path.exists():
            print(f"Skipping {side} (no mapping file)")
            continue

        print(f"\n{'='*70}")
        print(f"Processing {side.upper()} hand")
        print(f"{'='*70}")

        # Load mapping
        mapping = load_mapping(mapping_path)

        # Source mesh directory
        src_mesh_dir = pkg_dir / 'meshes' / side

        # Output directory for link meshes
        output_dir = pkg_dir / 'meshes' / side / 'links'
        output_dir.mkdir(parents=True, exist_ok=True)

        # Process each link
        links_config = mapping.get('links', {})
        for link_name, link_data in links_config.items():
            mesh_files = link_data.get('meshes', [])
            transform_type = link_data.get('transform', None)
            joint_position = link_data.get('joint_position', None)
            output_name = link_data.get('output_name', f'{link_name}.stl')

            if not mesh_files:
                print(f"  {link_name}: no meshes assigned, skipping")
                continue

            # Build full paths
            stl_paths = []
            for mesh_file in mesh_files:
                path = src_mesh_dir / mesh_file
                if path.exists():
                    stl_paths.append(path)
                else:
                    print(f"  Warning: {mesh_file} not found")

            if not stl_paths:
                print(f"  {link_name}: no valid meshes found")
                continue

            print(f"  {link_name}: merging {len(stl_paths)} meshes (transform={transform_type})")

            # Merge and transform meshes
            merged = merge_and_transform_stl_files(
                stl_paths,
                transform_type,
                joint_position,
                link_name
            )

            if merged is None:
                print(f"    ERROR: Failed to merge meshes")
                continue

            # Save merged mesh
            output_path = output_dir / output_name
            merged.save(str(output_path))

            size_kb = output_path.stat().st_size / 1024
            print(f"    Saved: {output_path.name} ({size_kb:.1f} KB)")

    print(f"\n{'='*70}")
    print("Done!")
    print("Next steps:")
    print("  1. Rebuild: colcon build --packages-select inspire_hand_description")
    print("  2. Launch:  ros2 launch inspire_hand_description display.launch.py")
    print(f"{'='*70}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
