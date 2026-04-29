"""Base dataset class for grasp radius prediction."""

import os
import re
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any

from torch.utils.data import Dataset

from ..config import (
    OBJECT_TO_RADIUS,
    ALL_OBJECTS,
    NON_DEFORMABLE_DIR,
    normalize_radius,
)


def extract_object_name(record_name: str) -> Optional[str]:
    """
    Extract object name from record directory name.

    Examples:
        'record_250_1' -> '250'
        'record_330_fat_5' -> '330_fat'
        'record_small_bottle_3' -> 'small_bottle'
        'record_250_empty_1' -> '250'  (for deformable)
    """
    # Remove 'record_' prefix
    name = record_name.replace("record_", "")

    # Remove '_empty' suffix (for deformable objects)
    name = re.sub(r"_empty", "", name)

    # Remove trailing number (e.g., '_1', '_10')
    name = re.sub(r"_\d+$", "", name)

    # Check if it matches a known object
    if name in OBJECT_TO_RADIUS:
        return name

    return None


def extract_radius(record_name: str) -> Optional[float]:
    """Extract radius from record directory name."""
    obj_name = extract_object_name(record_name)
    if obj_name is not None:
        return OBJECT_TO_RADIUS[obj_name]
    return None


class GraspRadiusDataset(Dataset):
    """
    Base dataset for grasp radius prediction.

    Handles:
    - Scanning non_deformable directory for samples
    - Extracting radius labels from record names
    - Leave-one-object-out splitting

    Args:
        root_dir: Path to training_data directory
        val_object: Object name to hold out for validation (leave-one-out)
        split: 'train' or 'val'
        normalize_target: Whether to normalize radius to [0, 1]
    """

    def __init__(
        self,
        root_dir: Optional[Path] = None,
        val_object: str = "mid_bottle",
        split: str = "train",
        normalize_target: bool = True,
    ):
        self.root_dir = Path(root_dir) if root_dir else NON_DEFORMABLE_DIR
        self.val_object = val_object
        self.split = split
        self.normalize_target = normalize_target

        # Build sample list
        self.samples: List[Dict[str, Any]] = []
        self._scan_samples()
        self._apply_split()

    def _scan_samples(self):
        """Scan directory and build sample list."""
        if not self.root_dir.exists():
            raise FileNotFoundError(f"Data directory not found: {self.root_dir}")

        # Iterate over record directories
        for record_dir in sorted(self.root_dir.iterdir()):
            if not record_dir.is_dir():
                continue

            record_name = record_dir.name
            obj_name = extract_object_name(record_name)
            radius = extract_radius(record_name)

            if obj_name is None or radius is None:
                continue

            # Iterate over sample directories
            for sample_dir in sorted(record_dir.iterdir()):
                if not sample_dir.is_dir():
                    continue

                self.samples.append({
                    "path": sample_dir,
                    "record_name": record_name,
                    "object_name": obj_name,
                    "radius": radius,
                })

    def _apply_split(self):
        """Apply leave-one-object-out split."""
        if self.split == "train":
            # Keep all samples except validation object
            self.samples = [
                s for s in self.samples
                if s["object_name"] != self.val_object
            ]
        elif self.split == "val":
            # Keep only validation object
            self.samples = [
                s for s in self.samples
                if s["object_name"] == self.val_object
            ]
        else:
            raise ValueError(f"Unknown split: {self.split}")

    def get_target(self, idx: int) -> float:
        """Get target radius for sample."""
        radius = self.samples[idx]["radius"]
        if self.normalize_target:
            return normalize_radius(radius)
        return radius

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        """Override in subclass to load specific modality."""
        raise NotImplementedError("Subclass must implement __getitem__")

    def get_sample_info(self, idx: int) -> Dict[str, Any]:
        """Get metadata for a sample."""
        return self.samples[idx]

    def get_stats(self) -> Dict[str, int]:
        """Get sample counts per object."""
        stats = {}
        for sample in self.samples:
            obj = sample["object_name"]
            stats[obj] = stats.get(obj, 0) + 1
        return stats

    @staticmethod
    def get_split_objects(val_object: str) -> Tuple[List[str], List[str]]:
        """Get train and val object lists for a given val_object."""
        train_objects = [o for o in ALL_OBJECTS if o != val_object]
        val_objects = [val_object]
        return train_objects, val_objects
