"""GPU-accelerated point cloud operations using PyTorch CUDA."""

import torch
import numpy as np


def farthest_point_sample_gpu(points: torch.Tensor, num_samples: int) -> torch.Tensor:
    """
    GPU-accelerated Farthest Point Sampling.

    Args:
        points: (N, D) tensor on GPU
        num_samples: Number of points to sample

    Returns:
        sampled: (num_samples, D) tensor
    """
    device = points.device
    N = points.shape[0]

    # If we have fewer points than needed, pad with repeats
    if N <= num_samples:
        if N < num_samples:
            indices = torch.cat([
                torch.arange(N, device=device),
                torch.randint(0, N, (num_samples - N,), device=device)
            ])
        else:
            indices = torch.arange(N, device=device)
        return points[indices]

    # FPS algorithm on GPU
    centroids = torch.zeros(num_samples, dtype=torch.long, device=device)
    distances = torch.full((N,), float('inf'), device=device)
    farthest = torch.randint(0, N, (1,), device=device).item()

    for i in range(num_samples):
        centroids[i] = farthest
        centroid_xyz = points[farthest, :3]  # Only use xyz for distance

        # Compute distances in parallel on GPU
        dist = torch.sum((points[:, :3] - centroid_xyz) ** 2, dim=1)
        distances = torch.minimum(distances, dist)
        farthest = torch.argmax(distances).item()

    return points[centroids]


def normalize_point_cloud_gpu(points: torch.Tensor) -> torch.Tensor:
    """
    GPU-accelerated point cloud normalization.

    Args:
        points: (N, D) tensor on GPU, first 3 dims are xyz

    Returns:
        normalized: (N, D) tensor
    """
    # Center about zero (xyz only)
    centroid = torch.mean(points[:, :3], dim=0)
    points[:, :3] = points[:, :3] - centroid

    # Scale by maximum distance
    max_dist = torch.max(torch.sqrt(torch.sum(points[:, :3] ** 2, dim=1)))
    if max_dist > 0:
        points[:, :3] = points[:, :3] / max_dist

    return points


def augment_point_cloud_gpu(points: torch.Tensor, config: dict) -> torch.Tensor:
    """
    GPU-accelerated point cloud augmentation.

    Args:
        points: (N, D) tensor on GPU
        config: Augmentation config dict

    Returns:
        augmented: (N, D) tensor
    """
    # Random Z-axis rotation
    if config.get("pc_rotation", True):
        theta = torch.rand(1, device=points.device) * 2 * 3.14159265359
        cos_theta = torch.cos(theta)
        sin_theta = torch.sin(theta)

        rotation = torch.tensor([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ], dtype=torch.float32, device=points.device).squeeze()

        points[:, :3] = points[:, :3] @ rotation.T

    # Random scaling
    scale_low, scale_high = config.get("pc_scale_range", (0.8, 1.2))
    scale = torch.rand(1, device=points.device) * (scale_high - scale_low) + scale_low
    points[:, :3] = points[:, :3] * scale

    # Random translation
    trans_range = config.get("pc_translation_range", 0.1)
    translation = (torch.rand(3, device=points.device) * 2 - 1) * trans_range
    points[:, :3] = points[:, :3] + translation

    # Random point dropout
    dropout_ratio = torch.rand(1, device=points.device).item() * config.get("pc_dropout_ratio", 0.5)
    num_drop = int(len(points) * dropout_ratio)
    if num_drop > 0:
        drop_idx = torch.randperm(len(points), device=points.device)[:num_drop]
        points[drop_idx] = points[0]

    # Intensity noise
    if points.shape[1] > 3:
        noise_std = config.get("pc_intensity_noise_std", 0.02)
        points[:, 3] = points[:, 3] + torch.randn(len(points), device=points.device) * noise_std
        points[:, 3] = torch.clamp(points[:, 3], 0, 1)

    return points
