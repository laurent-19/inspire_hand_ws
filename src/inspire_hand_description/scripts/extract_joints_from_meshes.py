#!/usr/bin/env python3
"""
Extract Joint Positions from Mesh Connectivity

Analyzes STL mesh files to find where parts connect and extracts
joint positions for URDF generation.

Approach:
1. Load all part STL files and compute bounding boxes
2. Find adjacent parts (bounding boxes that touch/overlap)
3. Compute connection points between adjacent parts
4. Build connectivity graph
5. Output joint positions for URDF

Usage:
    python3 extract_joints_from_meshes.py [--side right|left] [--visualize]
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Set
from dataclasses import dataclass, field
from collections import defaultdict

try:
    import numpy as np
    from stl import mesh as stl_mesh
except ImportError as e:
    print(f"ERROR: Missing numpy-stl: {e}")
    print("Install with: pip install numpy-stl")
    sys.exit(1)


@dataclass
class PartInfo:
    """Information about a single mesh part."""
    name: str
    file_path: Path
    mesh: Optional[stl_mesh.Mesh] = None
    vertices: Optional[np.ndarray] = None
    centroid: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bbox_min: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bbox_max: np.ndarray = field(default_factory=lambda: np.zeros(3))
    size: np.ndarray = field(default_factory=lambda: np.zeros(3))
    volume: float = 0.0


@dataclass
class Connection:
    """Connection between two parts."""
    part_a: str
    part_b: str
    contact_point: np.ndarray  # Estimated joint position
    contact_normal: np.ndarray  # Estimated joint axis
    overlap_volume: float
    distance: float  # Min distance between parts (0 if touching)


def load_part(stl_path: Path) -> Optional[PartInfo]:
    """Load an STL file and extract geometry info."""
    try:
        m = stl_mesh.Mesh.from_file(str(stl_path))
    except Exception as e:
        print(f"  Warning: Failed to load {stl_path.name}: {e}")
        return None

    vertices = m.vectors.reshape(-1, 3)
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    size = bbox_max - bbox_min
    centroid = (bbox_min + bbox_max) / 2
    volume = float(np.prod(size))

    return PartInfo(
        name=stl_path.stem,
        file_path=stl_path,
        mesh=m,
        vertices=vertices,
        centroid=centroid,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        size=size,
        volume=volume,
    )


def bbox_overlap(a: PartInfo, b: PartInfo, tolerance: float = 0.001) -> Tuple[bool, float]:
    """
    Check if two bounding boxes overlap or are within tolerance.

    Returns:
        (overlaps, overlap_volume or -distance)
    """
    # Check overlap in each dimension
    overlap = np.zeros(3)
    for i in range(3):
        overlap[i] = min(a.bbox_max[i], b.bbox_max[i]) - max(a.bbox_min[i], b.bbox_min[i])

    # If all overlaps are positive, boxes overlap
    if np.all(overlap > 0):
        return True, float(np.prod(np.maximum(overlap, 0)))

    # Check if within tolerance (nearly touching)
    gap = -np.min(overlap)
    if gap <= tolerance:
        return True, -gap

    return False, -gap


def find_contact_point(a: PartInfo, b: PartInfo) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find the contact point and normal between two adjacent parts.

    Returns:
        (contact_point, contact_normal)
    """
    # Find the overlapping region of bounding boxes
    overlap_min = np.maximum(a.bbox_min, b.bbox_min)
    overlap_max = np.minimum(a.bbox_max, b.bbox_max)

    # Contact point is center of overlap region
    # If no overlap, use midpoint between closest faces
    if np.all(overlap_max > overlap_min):
        contact_point = (overlap_min + overlap_max) / 2
    else:
        # Find which dimension has the gap
        contact_point = (a.centroid + b.centroid) / 2

    # Contact normal points from a to b
    direction = b.centroid - a.centroid
    distance = np.linalg.norm(direction)
    if distance > 1e-6:
        contact_normal = direction / distance
    else:
        contact_normal = np.array([0, 1, 0])  # Default to Y axis

    return contact_point, contact_normal


def find_adjacent_parts(parts: Dict[str, PartInfo],
                        tolerance: float = 0.002) -> List[Connection]:
    """
    Find all pairs of adjacent (touching/overlapping) parts.

    Args:
        parts: Dictionary of part name -> PartInfo
        tolerance: Distance tolerance for considering parts adjacent (meters)

    Returns:
        List of Connection objects
    """
    connections = []
    part_names = list(parts.keys())

    for i, name_a in enumerate(part_names):
        for name_b in part_names[i+1:]:
            a, b = parts[name_a], parts[name_b]

            overlaps, value = bbox_overlap(a, b, tolerance)
            if overlaps:
                contact_point, contact_normal = find_contact_point(a, b)
                connections.append(Connection(
                    part_a=name_a,
                    part_b=name_b,
                    contact_point=contact_point,
                    contact_normal=contact_normal,
                    overlap_volume=max(0, value),
                    distance=max(0, -value),
                ))

    return connections


def build_connectivity_graph(connections: List[Connection]) -> Dict[str, Set[str]]:
    """Build adjacency graph from connections."""
    graph = defaultdict(set)
    for conn in connections:
        graph[conn.part_a].add(conn.part_b)
        graph[conn.part_b].add(conn.part_a)
    return dict(graph)


def find_connected_components(graph: Dict[str, Set[str]],
                              all_parts: Set[str]) -> List[Set[str]]:
    """Find connected components in the graph."""
    visited = set()
    components = []

    for start in all_parts:
        if start in visited:
            continue

        # BFS to find component
        component = set()
        queue = [start]
        while queue:
            node = queue.pop(0)
            if node in visited:
                continue
            visited.add(node)
            component.add(node)
            for neighbor in graph.get(node, []):
                if neighbor not in visited:
                    queue.append(neighbor)

        if component:
            components.append(component)

    return components


def identify_base_and_fingers(parts: Dict[str, PartInfo],
                              connections: List[Connection]) -> Dict:
    """
    Identify which parts belong to base (palm) vs fingers.

    Strategy:
    - Parts with largest volume/surface area are likely base
    - Parts extending in +Y direction (toward fingertips) are fingers
    - Parts clustered together that move as a unit form links
    """
    # Sort parts by volume (largest first)
    sorted_by_volume = sorted(parts.values(), key=lambda p: p.volume, reverse=True)

    # Find Y extent of all parts to identify finger direction
    all_y_max = max(p.bbox_max[1] for p in parts.values())
    all_y_min = min(p.bbox_min[1] for p in parts.values())
    y_range = all_y_max - all_y_min

    # Parts in the lower Y region are likely base
    y_threshold = all_y_min + y_range * 0.3

    base_candidates = []
    finger_candidates = []

    for part in parts.values():
        if part.centroid[1] < y_threshold:
            base_candidates.append(part.name)
        else:
            finger_candidates.append(part.name)

    # Large parts are definitely base
    large_parts = [p.name for p in sorted_by_volume[:5] if p.volume > 1e-5]

    return {
        'base_candidates': list(set(base_candidates + large_parts)),
        'finger_candidates': finger_candidates,
        'sorted_by_volume': [p.name for p in sorted_by_volume],
        'y_threshold': y_threshold,
    }


def cluster_parts_by_position(parts: Dict[str, PartInfo]) -> Dict[str, List[str]]:
    """
    Cluster parts by position to identify finger columns.

    Coordinate system (from mesh analysis):
    - Y: finger direction (+Y = fingertips, -Y = wrist/thumb base)
    - X: across fingers (-X = little/thumb side, +X = index side)
    - Z: palm depth

    For right hand:
    - Thumb: low Y (negative), separate from main finger region
    - Little: most negative X in positive Y region
    - Ring: second most negative X
    - Middle: near X=0
    - Index: most positive X
    """
    clusters = defaultdict(list)

    # Separate thumb region (low Y, below wrist)
    thumb_y_threshold = 0.0  # Parts with Y < 0 are likely thumb/palm mechanism

    # Finger region Y threshold (parts extending above palm)
    finger_y_threshold = 0.03  # 30mm above origin

    # Get X range for finger parts only
    finger_parts_x = [
        (name, part.centroid[0])
        for name, part in parts.items()
        if part.centroid[1] > finger_y_threshold
    ]

    if finger_parts_x:
        finger_x_min = min(p[1] for p in finger_parts_x)
        finger_x_max = max(p[1] for p in finger_parts_x)
        finger_x_range = finger_x_max - finger_x_min

        # Define X boundaries for each finger (4 fingers in positive Y region)
        # From negative X to positive X: little, ring, middle, index
        finger_boundaries = [
            ('little', finger_x_min - 0.01, finger_x_min + finger_x_range * 0.25),
            ('ring', finger_x_min + finger_x_range * 0.25, finger_x_min + finger_x_range * 0.50),
            ('middle', finger_x_min + finger_x_range * 0.50, finger_x_min + finger_x_range * 0.75),
            ('index', finger_x_min + finger_x_range * 0.75, finger_x_max + 0.01),
        ]
    else:
        finger_boundaries = []

    for name, part in parts.items():
        x, y, z = part.centroid

        # Thumb region: low Y (negative Y direction)
        if y < thumb_y_threshold:
            clusters['thumb'].append(name)
            continue

        # Finger region: high Y
        if y > finger_y_threshold:
            assigned = False
            for finger_name, x_lo, x_hi in finger_boundaries:
                if x_lo <= x < x_hi:
                    clusters[finger_name].append(name)
                    assigned = True
                    break
            if assigned:
                continue

        # Everything else is base/palm
        clusters['base'].append(name)

    return dict(clusters)


def extract_finger_joints(parts: Dict[str, PartInfo],
                          finger_parts: List[str],
                          finger_name: str) -> List[Dict]:
    """
    Extract joint positions for a single finger.

    Finger structure (Y positions in mm):
    - Connector parts: ~31-35mm (attached to palm)
    - Proximal phalanx: ~53-60mm (two overlapping parts)
    - Distal phalanx: ~90-105mm (one part)

    Joints:
    - MCP joint (joint_1): between connector and proximal (~Y=40-45mm)
    - PIP joint (joint_2): between proximal and distal (~Y=75-85mm)
    """
    if not finger_parts:
        return [], []

    # Sort parts by Y position (base to tip)
    sorted_parts = sorted(finger_parts, key=lambda n: parts[n].centroid[1])

    joints = []
    link_groups = []  # Groups of parts that move together
    current_group = []

    # Group parts by Y position clusters
    # Use centroid Y difference to detect gaps
    y_gap_threshold = 0.015  # 15mm gap indicates different link

    for i, name in enumerate(sorted_parts):
        part = parts[name]

        if i == 0:
            current_group = [name]
            continue

        prev_part = parts[sorted_parts[i-1]]
        y_gap = part.centroid[1] - prev_part.centroid[1]

        if y_gap > y_gap_threshold:
            # Start new group
            if current_group:
                link_groups.append(current_group)
            current_group = [name]
        else:
            current_group.append(name)

    # Don't forget last group
    if current_group:
        link_groups.append(current_group)

    # Create joints between consecutive link groups
    for i in range(len(link_groups) - 1):
        parent_group = link_groups[i]
        child_group = link_groups[i + 1]

        # Joint position: between max Y of parent and min Y of child
        parent_y_max = max(parts[n].bbox_max[1] for n in parent_group)
        child_y_min = min(parts[n].bbox_min[1] for n in child_group)

        # X and Z from average of child parts (joint rotates child)
        child_x = np.mean([parts[n].centroid[0] for n in child_group])
        child_z = np.mean([parts[n].centroid[2] for n in child_group])

        # Joint Y at the midpoint of the gap, or at child's base
        joint_y = (parent_y_max + child_y_min) / 2

        joint_index = i + 1
        joint_type = 'revolute'

        # Mimic joints for second joint
        mimic_joint = None
        if joint_index == 2:
            mimic_joint = f'{finger_name}_1_joint'

        joints.append({
            'name': f'{finger_name}_{joint_index}_joint',
            'position': [float(child_x), float(joint_y), float(child_z)],
            'axis': [1, 0, 0],  # Rotation around X (flex/extend)
            'parent_parts': parent_group,
            'child_parts': child_group,
            'type': joint_type,
            'mimic': mimic_joint,
        })

    # Also record link groups for URDF generation
    link_info = []
    for i, group in enumerate(link_groups):
        link_name = f'{finger_name}_{i+1}_link' if i > 0 else f'{finger_name}_base_link'
        link_info.append({
            'name': link_name,
            'parts': group,
            'centroid': np.mean([parts[n].centroid for n in group], axis=0).tolist(),
        })

    return joints, link_groups


def to_json_serializable(obj):
    """Convert numpy types to JSON-serializable Python types."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, (np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, dict):
        return {k: to_json_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [to_json_serializable(v) for v in obj]
    return obj


def analyze_hand(mesh_dir: Path, tolerance: float = 0.002) -> Dict:
    """
    Full analysis of hand mesh parts.

    Returns dictionary with:
    - parts: info about each part
    - connections: adjacency information
    - clusters: parts grouped by finger
    - joints: extracted joint positions
    """
    print(f"Loading meshes from: {mesh_dir}")

    # Load all parts
    parts = {}
    for stl_path in sorted(mesh_dir.glob("part_*.stl")):
        part = load_part(stl_path)
        if part:
            parts[part.name] = part

    print(f"Loaded {len(parts)} parts")

    # Find adjacent parts
    print("Finding adjacent parts...")
    connections = find_adjacent_parts(parts, tolerance)
    print(f"Found {len(connections)} connections")

    # Build connectivity graph
    graph = build_connectivity_graph(connections)

    # Find connected components
    components = find_connected_components(graph, set(parts.keys()))
    print(f"Found {len(components)} connected components")

    # Cluster parts by position
    print("Clustering parts by position...")
    clusters = cluster_parts_by_position(parts)
    for region, part_list in clusters.items():
        print(f"  {region}: {len(part_list)} parts")

    # Identify base and finger parts
    classification = identify_base_and_fingers(parts, connections)

    # Extract joints for each finger
    print("Extracting joint positions...")
    all_joints = []
    all_link_groups = {}
    for finger_name in ['thumb', 'index', 'middle', 'ring', 'little']:
        finger_parts = clusters.get(finger_name, [])
        result = extract_finger_joints(parts, finger_parts, finger_name)
        if result:
            finger_joints, link_groups = result
            all_joints.extend(finger_joints)
            all_link_groups[finger_name] = link_groups
            if finger_joints:
                print(f"  {finger_name}: {len(finger_joints)} joints, {len(link_groups)} link groups")

    # Compile results
    result = {
        'num_parts': len(parts),
        'num_connections': len(connections),
        'num_components': len(components),
        'parts': {
            name: {
                'centroid': p.centroid.tolist(),
                'bbox_min': p.bbox_min.tolist(),
                'bbox_max': p.bbox_max.tolist(),
                'size': p.size.tolist(),
                'volume': p.volume,
            }
            for name, p in parts.items()
        },
        'connections': [
            {
                'part_a': c.part_a,
                'part_b': c.part_b,
                'contact_point': c.contact_point.tolist(),
                'contact_normal': c.contact_normal.tolist(),
                'overlap_volume': c.overlap_volume,
                'distance': c.distance,
            }
            for c in connections
        ],
        'clusters': clusters,
        'classification': classification,
        'joints': all_joints,
        'link_groups': all_link_groups,
        'components': [list(c) for c in components],
    }

    return result


def main():
    parser = argparse.ArgumentParser(
        description="Extract joint positions from mesh connectivity"
    )
    parser.add_argument('--side', choices=['right', 'left'], default='right',
                        help='Which hand to analyze')
    parser.add_argument('--tolerance', type=float, default=0.002,
                        help='Distance tolerance for adjacency (meters)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output JSON file path')

    args = parser.parse_args()

    # Paths
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    pkg_dir = workspace / 'src' / 'inspire_hand_description'
    mesh_dir = pkg_dir / 'meshes' / args.side

    if not mesh_dir.exists():
        print(f"ERROR: Mesh directory not found: {mesh_dir}")
        return 1

    # Analyze
    print("=" * 70)
    print(f"Analyzing {args.side.upper()} hand meshes")
    print("=" * 70)

    result = analyze_hand(mesh_dir, args.tolerance)
    result['hand_side'] = args.side

    # Output
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = pkg_dir / 'config' / f'joint_extraction_{args.side}.json'

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(to_json_serializable(result), f, indent=2)

    print(f"\nSaved analysis to: {output_path}")

    # Print summary
    print("\n" + "=" * 70)
    print("Joint Extraction Summary")
    print("=" * 70)

    print(f"\nExtracted {len(result['joints'])} joints:")
    for joint in result['joints']:
        pos = joint['position']
        print(f"  {joint['name']}: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")

    print("\nCluster assignment:")
    for region, parts in result['clusters'].items():
        print(f"  {region}: {len(parts)} parts")

    return 0


if __name__ == '__main__':
    sys.exit(main())
