#!/usr/bin/env python3
"""
STEP to URDF Pipeline for Inspire Hand RH56DFTP

Automated pipeline that:
1. Parses STEP assembly structure
2. Extracts component meshes as STL
3. Generates URDF with proper kinematics
4. Creates ROS2 package structure

Usage:
    python3 step_to_urdf_pipeline.py [--right-only] [--left-only] [--skip-meshes]
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, Optional

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from step_assembly_parser import StepAssemblyParser
from step_mesh_extractor import StepMeshExtractor
from urdf_generator import InspireHandURDFGenerator


class StepToURDFPipeline:
    """Orchestrates STEP to URDF conversion for Inspire Hand."""

    def __init__(self, workspace: Optional[Path] = None):
        """
        Initialize pipeline.

        Args:
            workspace: ROS2 workspace path (defaults to ~/develop/inspire_hand_ws)
        """
        if workspace is None:
            workspace = Path.home() / 'develop' / 'inspire_hand_ws'

        self.workspace = workspace
        self.package_dir = workspace / 'src' / 'inspire_hand_description'
        self.inspire_r1_dir = self.package_dir / 'inspire_R1'

        # STEP file paths (with Chinese characters in filenames)
        self.step_files = {
            'right': self.inspire_r1_dir / 'RH56DFTP-0R   数模文件.STEP',
            'left': self.inspire_r1_dir / 'RH56DFTP-0L    数模文件.STEP',
        }

        # Output directories
        self.meshes_dir = self.package_dir / 'meshes'
        self.urdf_dir = self.package_dir / 'urdf'

    def check_dependencies(self) -> bool:
        """Check that required dependencies are installed."""
        print("Checking dependencies...")

        # Check OCP
        try:
            from OCP.STEPCAFControl import STEPCAFControl_Reader
            print("  cadquery-ocp: OK")
        except ImportError:
            print("  cadquery-ocp: MISSING")
            print("    Install with: pip install cadquery-ocp")
            return False

        return True

    def check_step_files(self) -> Dict[str, bool]:
        """Check which STEP files exist."""
        print("\nChecking STEP files...")
        status = {}

        for side, path in self.step_files.items():
            exists = path.exists()
            status[side] = exists
            size_str = f"({path.stat().st_size / 1e6:.1f} MB)" if exists else "(not found)"
            mark = "OK" if exists else "MISSING"
            print(f"  {side}: {mark} {size_str}")
            if exists:
                print(f"    {path}")

        return status

    def process_hand(self, side: str, skip_meshes: bool = False) -> bool:
        """
        Process a single hand (right or left).

        Args:
            side: "right" or "left"
            skip_meshes: Skip mesh extraction (use existing meshes)

        Returns:
            True if successful
        """
        print(f"\n{'='*70}")
        print(f"Processing {side.upper()} hand")
        print(f"{'='*70}")

        step_path = self.step_files[side]
        mesh_dir = self.meshes_dir / side
        urdf_path = self.urdf_dir / f"inspire_hand_{side}.urdf"

        if not step_path.exists():
            print(f"ERROR: STEP file not found: {step_path}")
            return False

        mesh_paths = {}

        if not skip_meshes:
            # Step 1: Parse assembly
            print("\n[1/3] Parsing STEP assembly...")
            try:
                parser = StepAssemblyParser()
                components = parser.parse(step_path)
                parser.print_summary()
            except Exception as e:
                print(f"ERROR parsing assembly: {e}")
                import traceback
                traceback.print_exc()

                # Continue with URDF generation using default meshes
                print("\nContinuing with URDF generation (no meshes)...")
                components = []

            # Step 2: Extract meshes
            if components:
                print("\n[2/3] Extracting meshes...")
                try:
                    all_components = parser.get_components_flat()
                    extractor = StepMeshExtractor()
                    mesh_paths = extractor.export_for_urdf(all_components, mesh_dir, side)
                    print(f"\nExported {len(mesh_paths)} meshes to {mesh_dir}")
                except Exception as e:
                    print(f"WARNING: Mesh extraction failed: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                print("\n[2/3] Skipping mesh extraction (no components found)")
        else:
            print("\n[1/3] Skipping STEP parsing (--skip-meshes)")
            print("[2/3] Skipping mesh extraction (--skip-meshes)")

            # Check for existing meshes
            if mesh_dir.exists():
                existing_meshes = list(mesh_dir.glob("*.stl"))
                print(f"  Found {len(existing_meshes)} existing meshes")

        # Step 3: Generate URDF
        print("\n[3/3] Generating URDF...")
        try:
            generator = InspireHandURDFGenerator(hand_side=side)
            urdf_content = generator.generate(mesh_paths if mesh_paths else None)

            # Save URDF
            self.urdf_dir.mkdir(parents=True, exist_ok=True)
            with open(urdf_path, 'w') as f:
                f.write(urdf_content)
            print(f"  Saved: {urdf_path}")

        except Exception as e:
            print(f"ERROR generating URDF: {e}")
            import traceback
            traceback.print_exc()
            return False

        return True

    def run(self, right_only: bool = False, left_only: bool = False,
            skip_meshes: bool = False) -> bool:
        """
        Run the full pipeline.

        Args:
            right_only: Only process right hand
            left_only: Only process left hand
            skip_meshes: Skip mesh extraction

        Returns:
            True if all processing succeeded
        """
        print("=" * 70)
        print("STEP to URDF Pipeline for Inspire Hand RH56DFTP")
        print("=" * 70)

        # Check dependencies
        if not self.check_dependencies():
            return False

        # Check STEP files
        step_status = self.check_step_files()

        # Determine which hands to process
        hands_to_process = []
        if not left_only and step_status.get('right', False):
            hands_to_process.append('right')
        if not right_only and step_status.get('left', False):
            hands_to_process.append('left')

        if not hands_to_process:
            print("\nERROR: No STEP files available to process")
            return False

        # Process each hand
        success = True
        for side in hands_to_process:
            if not self.process_hand(side, skip_meshes):
                success = False

        # Summary
        print("\n" + "=" * 70)
        print("Pipeline Complete")
        print("=" * 70)

        print("\nGenerated files:")
        for side in hands_to_process:
            urdf_path = self.urdf_dir / f"inspire_hand_{side}.urdf"
            if urdf_path.exists():
                print(f"  URDF: {urdf_path}")

            mesh_dir = self.meshes_dir / side
            if mesh_dir.exists():
                mesh_count = len(list(mesh_dir.glob("*.stl")))
                print(f"  Meshes: {mesh_dir}/ ({mesh_count} files)")

        print("\nNext steps:")
        print("  1. Validate URDF: check_urdf urdf/inspire_hand_right.urdf")
        print("  2. Build package: colcon build --packages-select inspire_hand_description")
        print("  3. Visualize: ros2 launch inspire_hand_description display.launch.py")

        return success


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert Inspire Hand STEP files to URDF"
    )
    parser.add_argument('--right-only', action='store_true',
                        help='Only process right hand')
    parser.add_argument('--left-only', action='store_true',
                        help='Only process left hand')
    parser.add_argument('--skip-meshes', action='store_true',
                        help='Skip mesh extraction (use existing meshes)')
    parser.add_argument('--workspace', type=str, default=None,
                        help='ROS2 workspace path')

    args = parser.parse_args()

    workspace = Path(args.workspace) if args.workspace else None
    pipeline = StepToURDFPipeline(workspace)

    success = pipeline.run(
        right_only=args.right_only,
        left_only=args.left_only,
        skip_meshes=args.skip_meshes
    )

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
