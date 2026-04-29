"""Dataset for tactile point cloud with surface normals."""

from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch

from .base_dataset import GraspRadiusDataset
from .pointcloud_dataset import load_pcd_binary, normalize_point_cloud, farthest_point_sample
from ..config import POINTNETPP_CONFIG, AUGMENTATION_CONFIG


def compute_normals_knn_pca(
    xyz: np.ndarray,
    k: int = 30,
) -> np.ndarray:
    """
    Compute surface normals using k-NN + PCA.

    For each point, fits a local plane to k nearest neighbors using PCA.
    The normal is the eigenvector with smallest eigenvalue.

    Args:
        xyz: (N, 3) point coordinates
        k: number of neighbors for local plane fitting

    Returns:
        normals: (N, 3) unit normal vectors
    """
    N = xyz.shape[0]

    # Limit k to available points
    k = min(k, N)

    # Compute pairwise squared distances
    diff = xyz[:, np.newaxis, :] - xyz[np.newaxis, :, :]  # (N, N, 3)
    dist_sq = np.sum(diff ** 2, axis=2)  # (N, N)

    # Find k nearest neighbors for each point
    knn_idx = np.argpartition(dist_sq, k, axis=1)[:, :k]  # (N, k)

    # Gather neighbor points
    neighbors = xyz[knn_idx]  # (N, k, 3)

    # Center neighbors (subtract mean)
    centered = neighbors - neighbors.mean(axis=1, keepdims=True)  # (N, k, 3)

    # Compute covariance matrices: (N, 3, 3)
    # cov = centered^T @ centered for each point
    cov = np.einsum('nki,nkj->nij', centered, centered)

    # Add small epsilon for numerical stability
    cov = cov + np.eye(3) * 1e-6

    # Eigendecomposition (eigenvalues in ascending order)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # Normal = eigenvector with smallest eigenvalue (index 0)
    normals = eigenvectors[:, :, 0]  # (N, 3)

    # Normalize to unit length
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / (norms + 1e-8)

    # Orient normals consistently
    # Heuristic: for tactile sensor, points typically face toward +z (sensor)
    # Flip normals pointing away from z-axis
    flip_mask = normals[:, 2] < 0
    normals[flip_mask] = -normals[flip_mask]

    return normals.astype(np.float32)


def compute_normals_knn_pca_torch(
    xyz: torch.Tensor,
    k: int = 30,
) -> torch.Tensor:
    """
    Compute surface normals using k-NN + PCA (PyTorch version).

    Args:
        xyz: (N, 3) point coordinates
        k: number of neighbors for local plane fitting

    Returns:
        normals: (N, 3) unit normal vectors
    """
    device = xyz.device
    N = xyz.shape[0]

    # Limit k to available points
    k = min(k, N)

    # Compute pairwise squared distances
    dist = torch.cdist(xyz, xyz)  # (N, N)

    # Find k nearest neighbors
    _, knn_idx = torch.topk(dist, k, largest=False)  # (N, k)

    # Gather neighbor points
    neighbors = xyz[knn_idx]  # (N, k, 3)

    # Center neighbors
    centered = neighbors - neighbors.mean(dim=1, keepdim=True)  # (N, k, 3)

    # Compute covariance matrices
    cov = torch.bmm(centered.transpose(1, 2), centered)  # (N, 3, 3)

    # Add epsilon for stability
    cov = cov + torch.eye(3, device=device).unsqueeze(0) * 1e-6

    # Eigendecomposition
    eigenvalues, eigenvectors = torch.linalg.eigh(cov)

    # Normal = eigenvector with smallest eigenvalue
    normals = eigenvectors[:, :, 0]  # (N, 3)

    # Normalize
    normals = F.normalize(normals, dim=1)

    # Orient toward +z
    flip_mask = normals[:, 2] < 0
    normals[flip_mask] = -normals[flip_mask]

    return normals


class TactilePointCloudDatasetWithNormals(GraspRadiusDataset):
    """
    Dataset for PointNet++: Tactile point cloud with surface normals.

    Returns points with xyz + intensity + normals (7 channels).

    Args:
        root_dir: Path to non_deformable directory
        val_object: Object name to hold out for validation
        split: 'train' or 'val'
        num_points: Number of points to sample
        use_intensity: Whether to include intensity channel
        use_normals: Whether to compute and include surface normals
        k_neighbors: Number of neighbors for normal computation
        use_filtered: Whether to use filtered or raw point cloud
        augment: Whether to apply data augmentation
        normalize_target: Whether to normalize radius to [0, 1]
    """

    def __init__(
        self,
        root_dir: Optional[Path] = None,
        val_object: str = "mid_bottle",
        split: str = "train",
        num_points: int = 1024,
        use_intensity: bool = True,
        use_normals: bool = True,
        k_neighbors: int = 30,
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
        self.use_normals = use_normals
        self.k_neighbors = k_neighbors
        self.use_filtered = use_filtered
        self.augment = augment and (split == "train")

    def _load_pointcloud(self, sample_path: Path) -> np.ndarray:
        """Load tactile point cloud from PCD file."""
        pcd_name = "tactile_pointcloud.pcd" if self.use_filtered else "tactile_pointcloud_raw.pcd"
        pcd_path = sample_path / pcd_name

        points = load_pcd_binary(pcd_path, use_intensity=self.use_intensity)
        return points

    def _augment_pointcloud_with_normals(self, points: np.ndarray) -> np.ndarray:
        """
        Apply data augmentation to point cloud with normals.

        Args:
            points: (N, 7) array - xyz(3) + intensity(1) + normals(3)
                    or (N, 6) if no intensity - xyz(3) + normals(3)

        Returns:
            augmented: (N, 7) or (N, 6) augmented points
        """
        cfg = AUGMENTATION_CONFIG

        # Determine channel layout
        if self.use_intensity and self.use_normals:
            # xyz(0:3), intensity(3), normals(4:7)
            xyz = points[:, :3]
            intensity = points[:, 3:4]
            normals = points[:, 4:7]
        elif self.use_normals:
            # xyz(0:3), normals(3:6)
            xyz = points[:, :3]
            intensity = None
            normals = points[:, 3:6]
        else:
            # xyz(0:3), intensity(3) - no normals
            xyz = points[:, :3]
            intensity = points[:, 3:4] if self.use_intensity else None
            normals = None

        # Random Z-axis rotation - apply to BOTH xyz AND normals
        if cfg["pc_rotation"]:
            theta = np.random.uniform(0, 2 * np.pi)
            rotation = np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1],
            ], dtype=np.float32)
            xyz = xyz @ rotation.T
            if normals is not None:
                normals = normals @ rotation.T

        # Random scaling - only xyz (normals are unit vectors)
        scale_low, scale_high = cfg["pc_scale_range"]
        scale = np.random.uniform(scale_low, scale_high)
        xyz = xyz * scale

        # Random translation - only xyz (normals are directions)
        trans_range = cfg["pc_translation_range"]
        translation = np.random.uniform(-trans_range, trans_range, size=(1, 3))
        xyz = xyz + translation

        # Random point dropout
        dropout_ratio = np.random.uniform(0, cfg["pc_dropout_ratio"])
        num_drop = int(len(points) * dropout_ratio)
        if num_drop > 0:
            drop_idx = np.random.choice(len(points), num_drop, replace=False)
            xyz[drop_idx] = xyz[0]
            if normals is not None:
                normals[drop_idx] = normals[0]
            if intensity is not None:
                intensity[drop_idx] = intensity[0]

        # Intensity noise
        if intensity is not None:
            noise_std = cfg["pc_intensity_noise_std"]
            intensity = intensity + np.random.normal(0, noise_std, intensity.shape)
            intensity = np.clip(intensity, 0, 1)

        # Reassemble points
        components = [xyz]
        if intensity is not None:
            components.append(intensity)
        if normals is not None:
            components.append(normals)

        return np.hstack(components)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Get a sample.

        Returns:
            points: (C, N) tensor where C=7 (xyz + intensity + normals)
            target: scalar radius
        """
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load point cloud (xyz + intensity)
        points = self._load_pointcloud(sample_path)  # (N, 4) or (N, 3)

        # Farthest point sampling FIRST (reduces from ~8000 to 1024 points)
        points = farthest_point_sample(points, self.num_points)

        # Compute surface normals AFTER sampling (much faster on 1024 points)
        if self.use_normals:
            xyz = points[:, :3]
            normals = compute_normals_knn_pca(xyz, self.k_neighbors)  # (N, 3)

            # Append normals
            if self.use_intensity:
                # xyz(3) + intensity(1) + normals(3) = 7 channels
                points = np.hstack([points, normals])
            else:
                # xyz(3) + normals(3) = 6 channels
                points = np.hstack([points[:, :3], normals])

        # Normalize xyz (preserves other channels)
        points = normalize_point_cloud(points)

        # Augmentation
        if self.augment:
            if self.use_normals:
                points = self._augment_pointcloud_with_normals(points)
            else:
                # Use original augmentation for backward compatibility
                points = self._augment_pointcloud_legacy(points)

        # Convert to (C, N) format for PointNet++
        points = torch.tensor(points, dtype=torch.float32).T

        # Get target
        target = torch.tensor(self.get_target(idx), dtype=torch.float32)

        return points, target

    def _augment_pointcloud_legacy(self, points: np.ndarray) -> np.ndarray:
        """Legacy augmentation without normals (copy from original dataset)."""
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
            points[drop_idx] = points[0]

        # Intensity noise
        if points.shape[1] > 3:
            noise_std = cfg["pc_intensity_noise_std"]
            points[:, 3] = points[:, 3] + np.random.normal(0, noise_std, len(points))
            points[:, 3] = np.clip(points[:, 3], 0, 1)

        return points


if __name__ == "__main__":
    # Test normal computation
    print("Testing surface normal computation...")

    # Create synthetic planar point cloud
    N = 100
    x = np.random.uniform(-1, 1, N)
    y = np.random.uniform(-1, 1, N)
    z = 0.1 * x + 0.2 * y + np.random.normal(0, 0.01, N)  # Slightly noisy plane
    xyz = np.stack([x, y, z], axis=1).astype(np.float32)

    normals = compute_normals_knn_pca(xyz, k=20)
    print(f"Input shape: {xyz.shape}")
    print(f"Normals shape: {normals.shape}")
    print(f"Mean normal: {normals.mean(axis=0)}")
    print(f"Expected (approx): [0, 0, 1] for flat plane")

    # Test dataset
    print("\nTesting dataset...")
    try:
        dataset = TactilePointCloudDatasetWithNormals(
            val_object="mid_bottle",
            split="val",
            num_points=1024,
            use_intensity=True,
            use_normals=True,
        )
        print(f"Dataset size: {len(dataset)}")

        if len(dataset) > 0:
            points, target = dataset[0]
            print(f"Points shape: {points.shape}")  # Should be (7, 1024)
            print(f"Target: {target}")
    except Exception as e:
        print(f"Dataset test skipped: {e}")
