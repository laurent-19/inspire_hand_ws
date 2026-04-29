"""Dataset for tactile point cloud data."""

import struct
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch

from .base_dataset import GraspRadiusDataset
from ..config import POINTNET_CONFIG, AUGMENTATION_CONFIG


def load_pcd_binary(filepath: Path, use_intensity: bool = True) -> np.ndarray:
    """
    Load binary PCD file and extract xyz + intensity.

    Args:
        filepath: Path to .pcd file
        use_intensity: Whether to include intensity channel

    Returns:
        points: (N, 4) array with xyz + intensity, or (N, 3) if no intensity
    """
    with open(filepath, "rb") as f:
        # Parse header
        fields = []
        sizes = []
        types = []
        num_points = 0

        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                types = line.split()[1:]
            elif line.startswith("POINTS"):
                num_points = int(line.split()[1])
            elif line.startswith("DATA"):
                break

        # Build dtype
        dtype_map = {"F": "f", "I": "i", "U": "u"}
        dtype_list = []
        for name, size, typ in zip(fields, sizes, types):
            dtype_list.append((name, f"{dtype_map.get(typ, 'f')}{size}"))

        # Read binary data
        data = np.frombuffer(f.read(), dtype=dtype_list)

    # Extract xyz
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32)

    if use_intensity and "intensity" in fields:
        intensity = data["intensity"].astype(np.float32).reshape(-1, 1)
        # Normalize 12-bit ADC (0-4095) to [0, 1]
        intensity = intensity / 4095.0
        return np.hstack([xyz, intensity])

    return xyz


def normalize_point_cloud(points: np.ndarray) -> np.ndarray:
    """
    Center and scale point cloud to [-1, 1] range.

    Adapted from 3d_process.py:84-106.
    """
    # Center about zero (xyz only)
    centroid = np.mean(points[:, :3], axis=0)
    points[:, :3] = points[:, :3] - centroid

    # Scale by maximum distance
    max_dist = np.max(np.sqrt(np.sum(points[:, :3] ** 2, axis=1)))
    if max_dist > 0:
        points[:, :3] = points[:, :3] / max_dist

    return points


def farthest_point_sample(points: np.ndarray, num_samples: int) -> np.ndarray:
    """
    Farthest point sampling for uniform spatial coverage.

    Adapted from 3d_process.py:109-139.

    Args:
        points: (N, D) point cloud
        num_samples: Number of points to sample

    Returns:
        sampled: (num_samples, D) sampled point cloud
    """
    N = len(points)

    # If we have fewer points than needed, pad with repeats
    if N <= num_samples:
        indices = np.concatenate([
            np.arange(N),
            np.random.choice(N, num_samples - N, replace=True)
        ])
        return points[indices]

    # FPS algorithm
    centroids = np.zeros(num_samples, dtype=np.int64)
    distances = np.full(N, np.inf)
    farthest = np.random.randint(0, N)

    for i in range(num_samples):
        centroids[i] = farthest
        centroid_xyz = points[farthest, :3]
        dist = np.sum((points[:, :3] - centroid_xyz) ** 2, axis=1)
        distances = np.minimum(distances, dist)
        farthest = np.argmax(distances)

    return points[centroids]


class TactilePointCloudDataset(GraspRadiusDataset):
    """
    Dataset for Model 2: Tactile point cloud.

    Args:
        root_dir: Path to non_deformable directory
        val_object: Object name to hold out for validation
        split: 'train' or 'val'
        num_points: Number of points to sample
        use_intensity: Whether to include intensity channel
        use_filtered: Whether to use filtered or raw point cloud
        augment: Whether to apply data augmentation
        normalize_target: Whether to normalize radius to [0, 1]
    """

    def __init__(
        self,
        root_dir: Optional[Path] = None,
        val_object: str = "mid_bottle",
        split: str = "train",
        num_points: int = 2048,
        use_intensity: bool = True,
        use_filtered: bool = True,
        augment: bool = True,
        normalize_target: bool = True,
    ):
        super().__init__(
            root_dir=root_dir,
            val_object=val_object,
            split=split,
            normalize_target=normalize_target,
        )

        self.num_points = num_points
        self.use_intensity = use_intensity
        self.use_filtered = use_filtered
        self.augment = augment and (split == "train")

    def _load_pointcloud(self, sample_path: Path) -> np.ndarray:
        """Load tactile point cloud from PCD file."""
        pcd_name = "tactile_pointcloud.pcd" if self.use_filtered else "tactile_pointcloud_raw.pcd"
        pcd_path = sample_path / pcd_name

        points = load_pcd_binary(pcd_path, use_intensity=self.use_intensity)
        return points

    def _augment_pointcloud(self, points: np.ndarray) -> np.ndarray:
        """Apply data augmentation to point cloud."""
        cfg = AUGMENTATION_CONFIG

        # Random Z-axis rotation
        if cfg["pc_rotation"]:
            theta = np.random.uniform(0, 2 * np.pi)
            rotation = np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1],
            ], dtype=np.float32)
            points[:, :3] = points[:, :3] @ rotation.T

        # Random scaling
        scale_low, scale_high = cfg["pc_scale_range"]
        scale = np.random.uniform(scale_low, scale_high)
        points[:, :3] = points[:, :3] * scale

        # Random translation
        trans_range = cfg["pc_translation_range"]
        translation = np.random.uniform(-trans_range, trans_range, size=(1, 3))
        points[:, :3] = points[:, :3] + translation

        # Random point dropout
        dropout_ratio = np.random.uniform(0, cfg["pc_dropout_ratio"])
        num_drop = int(len(points) * dropout_ratio)
        if num_drop > 0:
            drop_idx = np.random.choice(len(points), num_drop, replace=False)
            points[drop_idx] = points[0]  # Replace with first point

        # Intensity noise
        if points.shape[1] > 3:
            noise_std = cfg["pc_intensity_noise_std"]
            points[:, 3] = points[:, 3] + np.random.normal(0, noise_std, len(points))
            points[:, 3] = np.clip(points[:, 3], 0, 1)

        return points

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Get a sample.

        Returns:
            points: (C, N) tensor where C=4 (xyz + intensity) and N=num_points
            target: scalar radius
        """
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load point cloud
        points = self._load_pointcloud(sample_path)

        # Farthest point sampling
        points = farthest_point_sample(points, self.num_points)

        # Normalize xyz
        points = normalize_point_cloud(points)

        # Augmentation
        if self.augment:
            points = self._augment_pointcloud(points)

        # Convert to (C, N) format for PointNet
        points = torch.tensor(points, dtype=torch.float32).T

        # Get target
        target = torch.tensor(self.get_target(idx), dtype=torch.float32)

        return points, target
