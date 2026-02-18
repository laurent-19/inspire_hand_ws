#!/usr/bin/env python3
"""
STEP Assembly Parser using STEPCAFControl_Reader

Parses STEP files with assembly structure, extracting component names,
transforms, and hierarchy for URDF generation.
"""

import sys
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import re

try:
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_LabelSequence, TDF_Label
    from OCP.gp import gp_Trsf, gp_XYZ
    from OCP.TopoDS import TopoDS_Shape
except ImportError as e:
    print(f"ERROR: Missing cadquery-ocp: {e}")
    print("Install with: pip install cadquery-ocp")
    sys.exit(1)


@dataclass
class AssemblyComponent:
    """Represents a component in the assembly hierarchy."""
    name: str
    label: TDF_Label
    shape: Optional[TopoDS_Shape] = None
    transform: Optional[gp_Trsf] = None
    parent: Optional['AssemblyComponent'] = None
    children: List['AssemblyComponent'] = field(default_factory=list)
    is_assembly: bool = False

    # Parsed finger info
    finger_name: Optional[str] = None  # index, middle, ring, little, thumb
    link_index: Optional[int] = None   # 1, 2, 3 for finger links
    is_base: bool = False

    def get_translation(self) -> Tuple[float, float, float]:
        """Get translation from transform (in meters)."""
        if self.transform is None:
            return (0.0, 0.0, 0.0)
        # STEP uses mm, convert to meters
        return (
            self.transform.TranslationPart().X() / 1000.0,
            self.transform.TranslationPart().Y() / 1000.0,
            self.transform.TranslationPart().Z() / 1000.0
        )

    def get_rotation_matrix(self) -> List[List[float]]:
        """Get 3x3 rotation matrix from transform."""
        if self.transform is None:
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        trsf = self.transform
        return [
            [trsf.Value(1, 1), trsf.Value(1, 2), trsf.Value(1, 3)],
            [trsf.Value(2, 1), trsf.Value(2, 2), trsf.Value(2, 3)],
            [trsf.Value(3, 1), trsf.Value(3, 2), trsf.Value(3, 3)],
        ]

    def __repr__(self):
        child_count = len(self.children)
        return f"AssemblyComponent(name='{self.name}', children={child_count}, is_assembly={self.is_assembly})"


class StepAssemblyParser:
    """Parse STEP assembly structure using STEPCAFControl_Reader."""

    # Pattern matching for Inspire Hand component names
    FINGER_PATTERNS = {
        'little': [r'小指', r'little', r'pinky', r'finger.*1', r'one'],
        'ring': [r'无名指', r'ring', r'finger.*2', r'two'],
        'middle': [r'中指', r'middle', r'finger.*3', r'three'],
        'index': [r'食指', r'index', r'finger.*4', r'four'],
        'thumb': [r'拇指', r'thumb', r'finger.*5', r'five'],
    }

    LINK_PATTERNS = {
        1: [r'近节', r'proximal', r'link.*1', r'first'],
        2: [r'中节', r'middle', r'link.*2', r'second'],
        3: [r'远节', r'distal', r'link.*3', r'third', r'tip'],
    }

    BASE_PATTERNS = [r'手掌', r'palm', r'base', r'手背']

    def __init__(self):
        self.document = None
        self.shape_tool = None
        self.components: Dict[str, AssemblyComponent] = {}
        self.root_components: List[AssemblyComponent] = []

    def parse(self, step_path: Path) -> List[AssemblyComponent]:
        """
        Parse STEP file and extract assembly structure.

        Args:
            step_path: Path to STEP file

        Returns:
            List of root AssemblyComponent objects
        """
        print(f"Parsing assembly: {step_path.name}")

        # Create XDE document
        self.document = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))

        # Read STEP with CAF (to get assembly structure)
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)  # Read component names
        reader.SetColorMode(True)
        reader.SetLayerMode(True)

        status = reader.ReadFile(str(step_path))
        if status != 1:  # IFSelect_RetDone
            raise RuntimeError(f"Failed to read STEP file: status={status}")

        # Transfer to document
        if not reader.Transfer(self.document):
            raise RuntimeError("Failed to transfer STEP data to document")

        # Get shape tool
        self.shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(self.document.Main())

        # Get free shapes (top-level)
        free_shapes = TDF_LabelSequence()
        self.shape_tool.GetFreeShapes(free_shapes)

        print(f"Found {free_shapes.Length()} top-level shapes")

        self.root_components = []
        for i in range(1, free_shapes.Length() + 1):
            label = free_shapes.Value(i)
            component = self._parse_label(label, None, 0)
            if component:
                self.root_components.append(component)

        # Identify hand parts
        self._identify_hand_parts()

        return self.root_components

    def _parse_label(self, label: TDF_Label, parent: Optional[AssemblyComponent],
                     depth: int) -> Optional[AssemblyComponent]:
        """Recursively parse a label and its children."""
        # Get name
        name = self._get_label_name(label)
        if not name:
            name = f"Component_{depth}_{label.Tag()}"

        # Get shape (static method)
        shape = XCAFDoc_ShapeTool.GetShape_s(label)

        # Get transform (static method returns Location directly)
        transform = None
        try:
            loc = XCAFDoc_ShapeTool.GetLocation_s(label)
            if not loc.IsIdentity():
                transform = loc.Transformation()
        except Exception:
            pass  # No location defined

        # Check if assembly (static method)
        is_assembly = XCAFDoc_ShapeTool.IsAssembly_s(label)

        component = AssemblyComponent(
            name=name,
            label=label,
            shape=shape,
            transform=transform,
            parent=parent,
            is_assembly=is_assembly
        )

        # Store in dictionary
        self.components[name] = component

        # Print hierarchy
        indent = "  " * depth
        shape_type = "assembly" if is_assembly else "part"
        print(f"{indent}- {name} ({shape_type})")

        # Parse children
        if is_assembly:
            children = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(label, children)

            for i in range(1, children.Length() + 1):
                child_label = children.Value(i)

                # For references, get the referred shape (use static method)
                if XCAFDoc_ShapeTool.IsReference_s(child_label):
                    # GetReferredShape_s returns bool and modifies ref_label
                    ref_label = TDF_Label()
                    if XCAFDoc_ShapeTool.GetReferredShape_s(child_label, ref_label):
                        child_component = self._parse_label(ref_label, component, depth + 1)
                    else:
                        child_component = self._parse_label(child_label, component, depth + 1)
                else:
                    child_component = self._parse_label(child_label, component, depth + 1)

                if child_component:
                    component.children.append(child_component)

        return component

    def _get_label_name(self, label: TDF_Label) -> Optional[str]:
        """Get the name attribute from a label."""
        name_attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), name_attr):
            return name_attr.Get().ToExtString()
        return None

    def _identify_hand_parts(self):
        """Identify finger links and base from component names."""
        for name, component in self.components.items():
            name_lower = name.lower()

            # Check for base/palm
            for pattern in self.BASE_PATTERNS:
                if re.search(pattern, name_lower, re.IGNORECASE):
                    component.is_base = True
                    print(f"  Identified base: {name}")
                    break

            # Check for fingers
            for finger_name, patterns in self.FINGER_PATTERNS.items():
                for pattern in patterns:
                    if re.search(pattern, name_lower, re.IGNORECASE):
                        component.finger_name = finger_name

                        # Check link index
                        for link_idx, link_patterns in self.LINK_PATTERNS.items():
                            for lp in link_patterns:
                                if re.search(lp, name_lower, re.IGNORECASE):
                                    component.link_index = link_idx
                                    break
                            if component.link_index:
                                break

                        print(f"  Identified finger: {name} -> {finger_name} link {component.link_index}")
                        break
                if component.finger_name:
                    break

    def get_components_flat(self) -> List[AssemblyComponent]:
        """Get flat list of all components."""
        return list(self.components.values())

    def get_finger_components(self) -> Dict[str, List[AssemblyComponent]]:
        """Get components grouped by finger."""
        fingers = {
            'little': [],
            'ring': [],
            'middle': [],
            'index': [],
            'thumb': [],
        }

        for component in self.components.values():
            if component.finger_name:
                fingers[component.finger_name].append(component)

        return fingers

    def get_base_components(self) -> List[AssemblyComponent]:
        """Get base/palm components."""
        return [c for c in self.components.values() if c.is_base]

    def print_summary(self):
        """Print summary of identified parts."""
        print("\n" + "=" * 60)
        print("Assembly Summary")
        print("=" * 60)

        base_parts = self.get_base_components()
        print(f"\nBase/Palm parts: {len(base_parts)}")
        for p in base_parts:
            print(f"  - {p.name}")

        finger_parts = self.get_finger_components()
        for finger_name, parts in finger_parts.items():
            print(f"\n{finger_name.capitalize()} finger parts: {len(parts)}")
            for p in sorted(parts, key=lambda x: x.link_index or 0):
                link_str = f"link {p.link_index}" if p.link_index else "unidentified link"
                print(f"  - {p.name} ({link_str})")

        print("=" * 60)


def main():
    """Test the parser with Inspire Hand STEP files."""
    workspace = Path.home() / 'develop' / 'inspire_hand_ws'
    step_files = [
        workspace / 'src' / 'inspire_hand_description' / 'inspire_R1' / 'RH56DFTP-0R   数模文件.STEP',
        workspace / 'src' / 'inspire_hand_description' / 'inspire_R1' / 'RH56DFTP-0L    数模文件.STEP',
    ]

    parser = StepAssemblyParser()

    for step_file in step_files:
        if step_file.exists():
            print(f"\n{'=' * 70}")
            print(f"Processing: {step_file.name}")
            print(f"{'=' * 70}")

            try:
                components = parser.parse(step_file)
                parser.print_summary()
            except Exception as e:
                print(f"ERROR: {e}")
                import traceback
                traceback.print_exc()

            break  # Just process first available file for testing


if __name__ == '__main__':
    main()
