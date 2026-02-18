#!/usr/bin/env python3
"""
Mesh Analyzer for Inspire Hand

Extracts centroid and bounding box data from all part STL files
and outputs to JSON for part-to-link mapping.
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import numpy as np
    from stl import mesh as stl_mesh
except ImportError as e:
    print(f"ERROR: Missing numpy-stl: {e}")
    print("Install with: pip install numpy-stl")
    sys.exit(1)


def analyze_stl(stl_path: Path) -> Dict:
    """
    Analyze an STL file and return geometry info.

    Returns dict with:
        - centroid: [x, y, z] in meters
        - bbox_min: [x, y, z]
        - bbox_max: [x, y, z]
        - size: [dx, dy, dz]
        - volume_approx: approximate volume
    """
    try:
        m = stl_mesh.Mesh.from_file(str(stl_path))
    except Exception as e:
        return {"error": str(e)}

    # Get all vertices
    vertices = m.vectors.reshape(-1, 3)

    # Bounding box
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    size = bbox_max - bbox_min

    # Centroid (center of bounding box)
    centroid = (bbox_min + bbox_max) / 2

    # Approximate volume (bounding box volume)
    volume_approx = float(np.prod(size))

    return {
        "centroid": centroid.tolist(),
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
        "size": size.tolist(),
        "volume_approx": volume_approx,
    }


def classify_by_position(centroid: List[float], joint_positions: Dict) -> Tuple[str, float]:
    """
    Classify a part by its proximity to known joint positions.

    Returns (link_name, distance) for the closest match.
    """
    cx, cy, cz = centroid

    best_link = "base_link"
    best_dist = float('inf')

    for link_name, (jx, jy, jz) in joint_positions.items():
        dist = np.sqrt((cx - jx)**2 + (cy - jy)**2 + (cz - jz)**2)
        if dist < best_dist:
            best_dist = dist
            best_link = link_name

    return best_link, float(best_dist)


def main():
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    pkg_dir = workspace / 'src' / 'inspire_hand_description'
    output_dir = pkg_dir / 'config'
    output_dir.mkdir(parents=True, exist_ok=True)

    # Known joint positions from URDF (in meters)
    # These are the positions where finger chains connect to base_link
    joint_positions = {
        # Finger base joints
        'little_1_joint': (0.080, -0.020, 0.010),
        'ring_1_joint': (0.085, 0.000, 0.010),
        'middle_1_joint': (0.085, 0.020, 0.010),
        'index_1_joint': (0.080, 0.040, 0.010),
        'thumb_1_joint': (0.030, -0.040, 0.015),
    }

    # Extended joint positions for finger links
    # Approximate positions based on link lengths
    finger_link_lengths = {
        'little': [0.025, 0.020, 0.015],
        'ring': [0.030, 0.022, 0.017],
        'middle': [0.035, 0.025, 0.020],
        'index': [0.030, 0.022, 0.017],
        'thumb': [0.025, 0.020, 0.015, 0.010],  # 4 links for thumb
    }

    # Generate approximate positions for all finger links
    all_link_positions = {}

    for finger in ['little', 'ring', 'middle', 'index']:
        base_pos = joint_positions[f'{finger}_1_joint']
        lengths = finger_link_lengths[finger]

        # Link 1 center (proximal phalanx)
        l1_center = (base_pos[0] + lengths[0]/2, base_pos[1], base_pos[2])
        all_link_positions[f'{finger}_1_link'] = l1_center

        # Link 2 center (middle phalanx)
        l2_start_x = base_pos[0] + lengths[0]
        l2_center = (l2_start_x + lengths[1]/2, base_pos[1], base_pos[2])
        all_link_positions[f'{finger}_2_link'] = l2_center

        # Link 3 center (distal phalanx)
        l3_start_x = l2_start_x + lengths[1]
        l3_center = (l3_start_x + lengths[2]/2, base_pos[1], base_pos[2])
        all_link_positions[f'{finger}_3_link'] = l3_center

    # Thumb (different kinematics)
    thumb_base = joint_positions['thumb_1_joint']
    thumb_lengths = finger_link_lengths['thumb']

    all_link_positions['thumb_1_link'] = (thumb_base[0] + thumb_lengths[0]/2, thumb_base[1], thumb_base[2])
    all_link_positions['thumb_2_link'] = (thumb_base[0] + thumb_lengths[0] + thumb_lengths[1]/2, thumb_base[1], thumb_base[2])
    all_link_positions['thumb_3_link'] = (thumb_base[0] + thumb_lengths[0] + thumb_lengths[1] + thumb_lengths[2]/2, thumb_base[1], thumb_base[2])
    all_link_positions['thumb_4_link'] = (thumb_base[0] + thumb_lengths[0] + thumb_lengths[1] + thumb_lengths[2] + thumb_lengths[3]/2, thumb_base[1], thumb_base[2])

    # Palm center
    all_link_positions['base_link'] = (0.045, 0.0, 0.010)  # Center of palm

    for side in ['right', 'left']:
        mesh_dir = pkg_dir / 'meshes' / side

        if not mesh_dir.exists():
            print(f"Skipping {side} (directory not found)")
            continue

        print(f"\n{'='*70}")
        print(f"Analyzing {side.upper()} hand meshes")
        print(f"{'='*70}")

        # Find all part files
        part_files = sorted(mesh_dir.glob("part_*.stl"))
        print(f"Found {len(part_files)} part files")

        parts_data = {}
        link_assignments = {link: [] for link in all_link_positions.keys()}

        for stl_path in part_files:
            part_name = stl_path.stem  # e.g., "part_000"
            info = analyze_stl(stl_path)

            if "error" in info:
                print(f"  {part_name}: ERROR - {info['error']}")
                continue

            centroid = info['centroid']
            size = info['size']

            # Classify by position
            assigned_link, dist = classify_by_position(centroid, all_link_positions)

            # Store info
            info['assigned_link'] = assigned_link
            info['distance_to_link'] = dist
            info['file'] = stl_path.name
            parts_data[part_name] = info

            # Add to link assignments
            link_assignments[assigned_link].append({
                'part': part_name,
                'file': stl_path.name,
                'distance': dist,
                'centroid': centroid,
                'size': size,
            })

            print(f"  {part_name}: centroid=({centroid[0]*1000:.1f}, {centroid[1]*1000:.1f}, {centroid[2]*1000:.1f})mm "
                  f"size=({size[0]*1000:.1f}x{size[1]*1000:.1f}x{size[2]*1000:.1f})mm -> {assigned_link}")

        # Sort link assignments by distance
        for link in link_assignments:
            link_assignments[link].sort(key=lambda x: x['distance'])

        # Write detailed analysis JSON
        analysis_output = {
            'hand_side': side,
            'num_parts': len(parts_data),
            'link_positions': {k: list(v) for k, v in all_link_positions.items()},
            'parts': parts_data,
            'link_assignments': link_assignments,
        }

        analysis_path = output_dir / f'mesh_analysis_{side}.json'
        with open(analysis_path, 'w') as f:
            json.dump(analysis_output, f, indent=2)
        print(f"\nSaved analysis to: {analysis_path}")

        # Print summary by link
        print(f"\n{'='*70}")
        print(f"Link Assignment Summary ({side})")
        print(f"{'='*70}")

        for link, parts in sorted(link_assignments.items()):
            part_names = [p['part'] for p in parts]
            print(f"  {link}: {len(parts)} parts")
            for p in parts[:5]:  # Show top 5 closest
                print(f"    - {p['part']} (dist={p['distance']*1000:.1f}mm)")
            if len(parts) > 5:
                print(f"    ... and {len(parts)-5} more")

    print(f"\n{'='*70}")
    print("Done! Next steps:")
    print("  1. Review mesh_analysis_*.json files")
    print("  2. Manually verify/adjust assignments if needed")
    print("  3. Create mesh_mapping_*.yaml config files")
    print("  4. Run merge_link_meshes.py")
    print(f"{'='*70}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
