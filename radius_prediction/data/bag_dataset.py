"""Bag-level split dataset for tactile point cloud with surface normals."""

import json
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset

from .pointcloud_dataset import load_pcd_binary, normalize_point_cloud, farthest_point_sample
from .pointcloud_dataset_normals import compute_normals_knn_pca
from ..config import NON_DEFORMABLE_DIR, OBJECT_TO_RADIUS, normalize_radius


class BagSplitDataset(Dataset):
    """
    Dataset with bag-level train/val split for PointNet++.

    Loads split from JSON file, includes surface normals, and applies
    simple augmentation (rotation, scale only).

    Args:
        split_file: Path to split JSON file
        split: 'train' or 'val'
        root_dir: Path to non_deformable directory
        num_points: Number of points to sample
        use_intensity: Whether to include intensity channel
        use_normals: Whether to compute surface normals
        k_neighbors: Number of neighbors for normal computation
        augment: Whether to apply data augmentation
        normalize_target: Whether to normalize radius to [0, 1]
    """

    def __init__(
        self,
        split_file: Path,
        split: str = "train",
        root_dir: Optional[Path] = None,
        num_points: int = 1024,
        use_intensity: bool = True,
        use_normals: bool = True,
        k_neighbors: int = 30,
        augment: bool = True,
        normalize_target: bool = True,
    ):
        self.root_dir = Path(root_dir) if root_dir else NON_DEFORMABLE_DIR
        self.split = split
        self.num_points = num_points
        self.use_intensity = use_intensity
        self.use_normals = use_normals
        self.k_neighbors = k_neighbors
        self.augment = augment and (split == "train")
        self.normalize_target = normalize_target

        # Load split
        with open(split_file) as f:
            split_data = json.load(f)

        self.bags = split_data[split]

        # Build sample list
        self.samples = []
        self._scan_samples()

    def _extract_object_name(self, record_name: str) -> Optional[str]:
        """Extract object name from record directory name."""
        import re
        name = record_name.replace("record_", "")
        name = re.sub(r"_\d+$", "", name)
        return name if name in OBJECT_TO_RADIUS else None

    def _scan_samples(self):
        """Scan bags and build sample list."""
        for bag_name in self.bags:
            bag_dir = self.root_dir / bag_name
            if not bag_dir.exists():
                continue

            obj_name = self._extract_object_name(bag_name)
            if obj_name is None:
                continue

            radius = OBJECT_TO_RADIUS[obj_name]

            # Iterate over sample directories
            for sample_dir in sorted(bag_dir.iterdir()):
                if not sample_dir.is_dir():
                    continue

                self.samples.append({
                    "path": sample_dir,
                    "bag_name": bag_name,
                    "object_name": obj_name,
                    "radius": radius,
                })

    def _load_pointcloud(self, sample_path: Path) -> np.ndarray:
        """Load tactile point cloud from PCD file."""
        pcd_path = sample_path / "tactile_pointcloud.pcd"
        return load_pcd_binary(pcd_path, use_intensity=self.use_intensity)

    def _augment_simple(self, points: np.ndarray) -> np.ndarray:
        """
        Simple augmentation for cylinder radius regression.

        Only rotation and small scale - no translation, dropout, or noise.
        These preserve surface geometry which is critical for radius estimation.

        Args:
            points: (N, C) array with xyz + optional intensity + optional normals
        """
        # Determine channel layout
        has_normals = self.use_normals
        has_intensity = self.use_intensity

        if has_intensity and has_normals:
            xyz = points[:, :3]
            intensity = points[:, 3:4]
            normals = points[:, 4:7]
        elif has_normals:
            xyz = points[:, :3]
            intensity = None
            normals = points[:, 3:6]
        elif has_intensity:
            xyz = points[:, :3]
            intensity = points[:, 3:4]
            normals = None
        else:
            xyz = points[:, :3]
            intensity = None
            normals = None

        # Random Z-axis rotation (cylinders are rotationally symmetric)
        theta = np.random.uniform(0, 2 * np.pi)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        rotation = np.array([
            [cos_t, -sin_t, 0],
            [sin_t, cos_t, 0],
            [0, 0, 1],
        ], dtype=np.float32)

        xyz = xyz @ rotation.T
        if normals is not None:
            normals = normals @ rotation.T

        # Small random scale (0.9 to 1.1) - geometry matters for radius
        scale = np.random.uniform(0.9, 1.1)
        xyz = xyz * scale

        # Reassemble points
        components = [xyz]
        if intensity is not None:
            components.append(intensity)
        if normals is not None:
            components.append(normals)

        return np.hstack(components)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Get a sample.

        Returns:
            points: (C, N) tensor
            target: scalar radius
        """
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load point cloud
        points = self._load_pointcloud(sample_path)

        # Farthest point sampling
        points = farthest_point_sample(points, self.num_points)

        # Compute surface normals after sampling
        if self.use_normals:
            xyz = points[:, :3]
            normals = compute_normals_knn_pca(xyz, self.k_neighbors)

            if self.use_intensity:
                points = np.hstack([points, normals])
            else:
                points = np.hstack([points[:, :3], normals])

        # Normalize xyz
        points = normalize_point_cloud(points)

        # Augmentation
        if self.augment:
            points = self._augment_simple(points)

        # Convert to (C, N) format
        points = torch.tensor(points, dtype=torch.float32).T

        # Get target
        radius = sample_info["radius"]
        if self.normalize_target:
            radius = normalize_radius(radius)
        target = torch.tensor(radius, dtype=torch.float32)

        return points, target

    def get_stats(self) -> dict:
        """Get sample counts per object."""
        stats = {}
        for sample in self.samples:
            obj = sample["object_name"]
            stats[obj] = stats.get(obj, 0) + 1
        return stats

    def get_bag_stats(self) -> dict:
        """Get sample counts per bag."""
        stats = {}
        for sample in self.samples:
            bag = sample["bag_name"]
            stats[bag] = stats.get(bag, 0) + 1
        return stats


if __name__ == "__main__":
    # Test dataset
    split_file = Path(__file__).parent / "splits" / "default_split.json"

    print("Testing BagSplitDataset...")
    print(f"Split file: {split_file}")

    train_ds = BagSplitDataset(
        split_file=split_file,
        split="train",
        num_points=1024,
        use_intensity=True,
        use_normals=True,
    )

    val_ds = BagSplitDataset(
        split_file=split_file,
        split="val",
        num_points=1024,
        use_intensity=True,
        use_normals=True,
        augment=False,
    )

    print(f"\nTrain samples: {len(train_ds)}")
    print(f"Val samples: {len(val_ds)}")
    print(f"\nTrain distribution: {train_ds.get_stats()}")
    print(f"Val distribution: {val_ds.get_stats()}")

    if len(train_ds) > 0:
        points, target = train_ds[0]
        print(f"\nSample shape: {points.shape}")
        print(f"Target: {target}")
