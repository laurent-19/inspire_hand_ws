#!/usr/bin/env python3
"""
STEP Mesh Extractor

Exports individual parts from STEP assembly as STL meshes for URDF.
"""

import sys
from pathlib import Path
from typing import List, Dict, Optional, Tuple

try:
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopoDS import TopoDS_Shape
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
except ImportError as e:
    print(f"ERROR: Missing cadquery-ocp: {e}")
    print("Install with: pip install cadquery-ocp")
    sys.exit(1)

from step_assembly_parser import AssemblyComponent


class StepMeshExtractor:
    """Export STEP components as STL meshes."""

    def __init__(self, linear_deflection: float = 0.1, angular_deflection: float = 0.5):
        """
        Initialize mesh extractor.

        Args:
            linear_deflection: Mesh accuracy in mm (smaller = finer mesh)
            angular_deflection: Angular accuracy in radians
        """
        self.linear_deflection = linear_deflection
        self.angular_deflection = angular_deflection

    def export_component(self, component: AssemblyComponent, output_path: Path,
                        apply_transform: bool = True, scale: float = 0.001) -> bool:
        """
        Export a single component as STL.

        Args:
            component: AssemblyComponent to export
            output_path: Output STL file path
            apply_transform: Apply component's transform
            scale: Scale factor (0.001 to convert mm to m for URDF)

        Returns:
            True if export successful
        """
        if component.shape is None:
            print(f"  Warning: No shape for {component.name}")
            return False

        shape = component.shape

        # Apply transform if requested
        if apply_transform and component.transform is not None:
            transform = BRepBuilderAPI_Transform(shape, component.transform, True)
            shape = transform.Shape()

        # Apply scale (mm to m)
        if scale != 1.0:
            scale_trsf = gp_Trsf()
            scale_trsf.SetScaleFactor(scale)
            transform = BRepBuilderAPI_Transform(shape, scale_trsf, True)
            shape = transform.Shape()

        # Create mesh
        mesh = BRepMesh_IncrementalMesh(
            shape,
            self.linear_deflection,
            False,  # isRelative
            self.angular_deflection,
            True    # inParallel
        )
        mesh.Perform()

        if not mesh.IsDone():
            print(f"  Warning: Meshing failed for {component.name}")
            return False

        # Create output directory
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Write STL
        writer = StlAPI_Writer()
        writer.ASCIIMode = False  # Binary for smaller files

        success = writer.Write(shape, str(output_path))

        if success:
            size_kb = output_path.stat().st_size / 1024
            print(f"  Exported: {output_path.name} ({size_kb:.1f} KB)")
        else:
            print(f"  Failed to write: {output_path.name}")

        return success

    def export_all_parts(self, components: List[AssemblyComponent],
                        output_dir: Path, prefix: str = "") -> Dict[str, Path]:
        """
        Export all components as STL files.

        Args:
            components: List of components to export
            output_dir: Output directory for STL files
            prefix: Prefix for output filenames

        Returns:
            Dictionary mapping component names to output paths
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        exported = {}

        for component in components:
            if component.shape is None:
                continue

            # Generate filename
            safe_name = self._safe_filename(component.name)
            if prefix:
                filename = f"{prefix}_{safe_name}.stl"
            else:
                filename = f"{safe_name}.stl"

            output_path = output_dir / filename

            if self.export_component(component, output_path):
                exported[component.name] = output_path

        return exported

    def export_for_urdf(self, components: List[AssemblyComponent],
                       output_dir: Path,
                       hand_side: str = "right") -> Dict[str, Path]:
        """
        Export components organized for URDF structure.

        Creates meshes with URDF-compatible naming:
        - base.stl
        - index_1.stl, index_2.stl, index_3.stl
        - middle_1.stl, etc.

        Args:
            components: List of AssemblyComponent
            output_dir: Output directory
            hand_side: "right" or "left"

        Returns:
            Dictionary mapping URDF link names to mesh paths
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        exported = {}

        # Separate by type
        base_components = [c for c in components if c.is_base]
        finger_components = {
            'little': [],
            'ring': [],
            'middle': [],
            'index': [],
            'thumb': [],
        }

        for c in components:
            if c.finger_name and c.finger_name in finger_components:
                finger_components[c.finger_name].append(c)

        # Export base (combine all base parts into one mesh)
        if base_components:
            # For now export first base component
            # TODO: Merge multiple base components
            output_path = output_dir / "base.stl"
            if self.export_component(base_components[0], output_path):
                exported["base"] = output_path

        # Export fingers
        for finger_name, parts in finger_components.items():
            # Sort by link index
            sorted_parts = sorted(parts, key=lambda x: x.link_index or 99)

            for part in sorted_parts:
                if part.link_index:
                    link_name = f"{finger_name}_{part.link_index}"
                    output_path = output_dir / f"{link_name}.stl"

                    if self.export_component(part, output_path):
                        exported[link_name] = output_path

        return exported

    def _safe_filename(self, name: str) -> str:
        """Convert component name to safe filename."""
        # Replace unsafe characters
        safe = name.replace(" ", "_")
        safe = safe.replace("/", "_")
        safe = safe.replace("\\", "_")
        safe = safe.replace(":", "_")
        # Keep only alphanumeric, underscore, hyphen
        safe = "".join(c for c in safe if c.isalnum() or c in "_-")
        # Limit length
        return safe[:50] if len(safe) > 50 else safe


def main():
    """Test mesh extraction."""
    from step_assembly_parser import StepAssemblyParser

    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    step_file = workspace / 'src' / 'inspire_hand_description' / 'inspire_R1' / 'RH56DFTP-0R   数模文件.STEP'
    output_dir = workspace / 'src' / 'inspire_hand_description' / 'meshes' / 'right'

    if not step_file.exists():
        print(f"ERROR: STEP file not found: {step_file}")
        return 1

    # Parse assembly
    parser = StepAssemblyParser()
    components = parser.parse(step_file)

    # Get all components flat
    all_components = parser.get_components_flat()

    print(f"\nExporting {len(all_components)} components to: {output_dir}")

    # Export
    extractor = StepMeshExtractor()
    exported = extractor.export_for_urdf(all_components, output_dir, "right")

    print(f"\nExported {len(exported)} meshes:")
    for name, path in exported.items():
        print(f"  {name}: {path.name}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
