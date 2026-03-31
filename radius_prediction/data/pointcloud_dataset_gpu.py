"""GPU-accelerated tactile point cloud dataset."""

from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch

from .base_dataset import GraspRadiusDataset
from .pointcloud_dataset import load_pcd_binary
from .gpu_ops import (
    farthest_point_sample_gpu,
    normalize_point_cloud_gpu,
    augment_point_cloud_gpu,
)
from ..config import AUGMENTATION_CONFIG


class TactilePointCloudDatasetGPU(GraspRadiusDataset):
    """
    GPU-accelerated dataset for tactile point clouds.

    All preprocessing (FPS, normalization, augmentation) happens on GPU.
    """

    def __init__(
        self,
        root_dir: Optional[Path] = None,
        val_object: str = "mid_bottle",
        split: str = "train",
        num_points: int = 1024,
        use_intensity: bool = True,
        use_filtered: bool = True,
        augment: bool = True,
        normalize_target: bool = True,
        device: str = "cuda",
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
        self.device = torch.device(device)

    def _load_pointcloud(self, sample_path: Path) -> np.ndarray:
        """Load tactile point cloud from PCD file (CPU only)."""
        pcd_name = "tactile_pointcloud.pcd" if self.use_filtered else "tactile_pointcloud_raw.pcd"
        pcd_path = sample_path / pcd_name
        return load_pcd_binary(pcd_path, use_intensity=self.use_intensity)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Get a sample with GPU-accelerated preprocessing.

        Returns:
            points: (C, N) tensor on GPU
            target: scalar tensor on GPU
        """
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load point cloud on CPU (fast binary read)
        points_np = self._load_pointcloud(sample_path)

        # Convert to GPU tensor immediately
        points = torch.from_numpy(points_np).float().to(self.device)

        # GPU-accelerated FPS
        points = farthest_point_sample_gpu(points, self.num_points)

        # GPU-accelerated normalization
        points = normalize_point_cloud_gpu(points)

        # GPU-accelerated augmentation
        if self.augment:
            points = augment_point_cloud_gpu(points, AUGMENTATION_CONFIG)

        # Convert to (C, N) format for PointNet
        points = points.T

        # Get target on GPU
        target = torch.tensor(self.get_target(idx), dtype=torch.float32, device=self.device)

        return points, target
