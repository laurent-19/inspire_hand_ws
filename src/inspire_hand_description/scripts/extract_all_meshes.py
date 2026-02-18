#!/usr/bin/env python3
"""
Simple mesh extractor for Inspire Hand STEP files.

Extracts all solid parts from STEP file as individual STL files.
Naming is sequential; user can rename/reorganize as needed.
"""

import sys
from pathlib import Path
from typing import List, Tuple

try:
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopoDS import TopoDS
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
except ImportError as e:
    print(f"ERROR: Missing cadquery-ocp: {e}")
    print("Install with: pip install cadquery-ocp")
    sys.exit(1)


def load_step(step_path: Path):
    """Load STEP file and return the shape."""
    print(f"Loading: {step_path.name}")
    print(f"  Size: {step_path.stat().st_size / 1e6:.1f} MB")

    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))

    if status != 1:  # IFSelect_RetDone
        raise RuntimeError(f"Failed to read STEP: status={status}")

    reader.TransferRoots()
    return reader.OneShape()


def extract_solids(shape) -> List:
    """Extract all solid bodies from shape."""
    solids = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)

    while explorer.More():
        solid = TopoDS.Solid_s(explorer.Current())
        solids.append(solid)
        explorer.Next()

    return solids


def get_bounding_box(shape) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Get bounding box of shape (min, max) in mm."""
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return ((xmin, ymin, zmin), (xmax, ymax, zmax))


def get_centroid(shape) -> Tuple[float, float, float]:
    """Get centroid of bounding box in mm."""
    (xmin, ymin, zmin), (xmax, ymax, zmax) = get_bounding_box(shape)
    return (
        (xmin + xmax) / 2,
        (ymin + ymax) / 2,
        (zmin + zmax) / 2
    )


def export_stl(shape, output_path: Path, scale: float = 0.001,
               linear_deflection: float = 0.1) -> bool:
    """Export shape to STL with optional scaling (mm to m)."""
    # Apply scale
    if scale != 1.0:
        trsf = gp_Trsf()
        trsf.SetScaleFactor(scale)
        transform = BRepBuilderAPI_Transform(shape, trsf, True)
        shape = transform.Shape()

    # Mesh
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection * scale)
    mesh.Perform()

    if not mesh.IsDone():
        return False

    # Write
    writer = StlAPI_Writer()
    writer.ASCIIMode = False  # Binary

    return writer.Write(shape, str(output_path))


def classify_part(bbox_min, bbox_max, centroid, all_centroids) -> str:
    """
    Attempt to classify a part based on geometry.

    Returns a suggested name like 'palm', 'finger_1', etc.
    """
    size_x = bbox_max[0] - bbox_min[0]
    size_y = bbox_max[1] - bbox_min[1]
    size_z = bbox_max[2] - bbox_min[2]
    volume_approx = size_x * size_y * size_z

    # Large volume is likely the palm
    if volume_approx > 50000:  # > 50cm^3 in mm units
        return "base_palm"

    # Use centroid position for finger classification
    # Fingers extend in +X direction typically
    # Palm center at origin, fingers at +X, thumb at -Y

    x, y, z = centroid

    # Simple heuristics - these will need tuning
    if y < -30:  # Likely thumb region
        return "thumb"
    elif y > 30:  # Likely index
        return "index"
    elif y > 10:  # Likely middle
        return "middle"
    elif y > -10:  # Likely ring
        return "ring"
    else:
        return "little"


def main():
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    pkg_dir = workspace / 'src' / 'inspire_hand_description'

    step_files = {
        'right': pkg_dir / 'inspire_R1' / 'RH56DFTP-0R   数模文件.STEP',
        'left': pkg_dir / 'inspire_R1' / 'RH56DFTP-0L    数模文件.STEP',
    }

    for side, step_path in step_files.items():
        if not step_path.exists():
            print(f"Skipping {side} (file not found)")
            continue

        output_dir = pkg_dir / 'meshes' / side
        output_dir.mkdir(parents=True, exist_ok=True)

        print(f"\n{'='*70}")
        print(f"Processing {side.upper()} hand")
        print(f"{'='*70}")

        # Load STEP
        try:
            shape = load_step(step_path)
        except Exception as e:
            print(f"ERROR loading STEP: {e}")
            continue

        # Extract solids
        solids = extract_solids(shape)
        print(f"\nFound {len(solids)} solid parts")

        if not solids:
            print("ERROR: No solids found")
            continue

        # Analyze parts
        part_info = []
        for i, solid in enumerate(solids):
            try:
                bbox_min, bbox_max = get_bounding_box(solid)
                centroid = get_centroid(solid)
                size = tuple(bbox_max[j] - bbox_min[j] for j in range(3))
                part_info.append({
                    'index': i,
                    'solid': solid,
                    'bbox_min': bbox_min,
                    'bbox_max': bbox_max,
                    'centroid': centroid,
                    'size': size,
                })
            except Exception:
                pass

        # Sort by centroid Y (to group fingers)
        part_info.sort(key=lambda p: p['centroid'][1])

        # Export all parts
        print(f"\nExporting meshes to: {output_dir}/")
        exported = 0

        for info in part_info:
            i = info['index']
            solid = info['solid']
            centroid = info['centroid']
            size = info['size']

            filename = f"part_{i:03d}.stl"
            output_path = output_dir / filename

            if export_stl(solid, output_path):
                size_kb = output_path.stat().st_size / 1024
                print(f"  {filename}: {size_kb:6.1f} KB  "
                      f"centroid=({centroid[0]:.1f}, {centroid[1]:.1f}, {centroid[2]:.1f}) "
                      f"size=({size[0]:.1f}x{size[1]:.1f}x{size[2]:.1f})")
                exported += 1
            else:
                print(f"  {filename}: FAILED")

        print(f"\nExported {exported}/{len(solids)} parts")

        # Also export full hand as single mesh
        full_path = output_dir / "full_hand.stl"
        print(f"\nExporting full hand mesh: {full_path.name}")
        if export_stl(shape, full_path):
            size_mb = full_path.stat().st_size / 1e6
            print(f"  Size: {size_mb:.1f} MB")
        else:
            print("  FAILED")

    print("\n" + "="*70)
    print("Done!")
    print("="*70)
    print("\nNext steps:")
    print("  1. View meshes in a 3D viewer (e.g., MeshLab, Blender)")
    print("  2. Identify which parts correspond to which links")
    print("  3. Rename parts to match URDF link names:")
    print("     - base.stl, index_1.stl, index_2.stl, index_3.stl, etc.")
    print("  4. Run: colcon build --packages-select inspire_hand_description")

    return 0


if __name__ == '__main__':
    sys.exit(main())
