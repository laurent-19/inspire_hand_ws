"""PointNet++ for tactile point cloud radius prediction.

Implements hierarchical point set learning with Set Abstraction layers.
Based on Qi et al. "PointNet++: Deep Hierarchical Feature Learning on Point Sets"
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import List, Optional, Tuple

from ..config import POINTNETPP_CONFIG


def farthest_point_sample(xyz: torch.Tensor, npoint: int) -> torch.Tensor:
    """
    Farthest Point Sampling for batched point clouds.

    Args:
        xyz: (B, N, 3) point coordinates
        npoint: number of points to sample

    Returns:
        centroids: (B, npoint) indices of sampled points
    """
    device = xyz.device
    B, N, _ = xyz.shape

    centroids = torch.zeros(B, npoint, dtype=torch.long, device=device)
    distance = torch.full((B, N), 1e10, device=device)

    # Random starting point
    farthest = torch.randint(0, N, (B,), dtype=torch.long, device=device)

    for i in range(npoint):
        centroids[:, i] = farthest
        centroid = xyz[torch.arange(B, device=device), farthest, :].view(B, 1, 3)
        dist = torch.sum((xyz - centroid) ** 2, dim=-1)
        distance = torch.min(distance, dist)
        farthest = torch.argmax(distance, dim=-1)

    return centroids


def index_points(points: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """
    Gather points by indices.

    Args:
        points: (B, N, C) input points
        idx: (B, S) or (B, S, K) indices

    Returns:
        indexed_points: (B, S, C) or (B, S, K, C)
    """
    device = points.device
    B = points.shape[0]

    view_shape = list(idx.shape)
    view_shape[1:] = [1] * (len(view_shape) - 1)
    repeat_shape = list(idx.shape)
    repeat_shape[0] = 1

    batch_indices = torch.arange(B, dtype=torch.long, device=device)
    batch_indices = batch_indices.view(view_shape).repeat(repeat_shape)

    new_points = points[batch_indices, idx, :]
    return new_points


def ball_query(
    radius: float,
    nsample: int,
    xyz: torch.Tensor,
    new_xyz: torch.Tensor,
) -> torch.Tensor:
    """
    Ball query: find nsample points within radius for each centroid.

    Args:
        radius: ball query radius
        nsample: max number of neighbors
        xyz: (B, N, 3) all points
        new_xyz: (B, S, 3) query centroids

    Returns:
        group_idx: (B, S, nsample) neighbor indices
    """
    device = xyz.device
    B, N, _ = xyz.shape
    _, S, _ = new_xyz.shape

    # Compute pairwise distances
    sqrdists = torch.cdist(new_xyz, xyz, p=2) ** 2  # (B, S, N)

    # Find points within radius
    group_idx = torch.arange(N, dtype=torch.long, device=device)
    group_idx = group_idx.view(1, 1, N).repeat(B, S, 1)

    # Mask points outside radius
    group_idx[sqrdists > radius ** 2] = N

    # Sort and take first nsample
    group_idx = group_idx.sort(dim=-1)[0][:, :, :nsample]

    # Replace invalid indices with first valid neighbor
    group_first = group_idx[:, :, 0].view(B, S, 1).repeat(1, 1, nsample)
    mask = group_idx == N
    group_idx[mask] = group_first[mask]

    return group_idx


def sample_and_group(
    npoint: int,
    radius: float,
    nsample: int,
    xyz: torch.Tensor,
    points: Optional[torch.Tensor] = None,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """
    Sample centroids and group neighbors.

    Args:
        npoint: number of centroids
        radius: ball query radius
        nsample: neighbors per centroid
        xyz: (B, N, 3) coordinates
        points: (B, N, C) features (optional)

    Returns:
        new_xyz: (B, npoint, 3) centroid coordinates
        new_points: (B, npoint, nsample, 3+C) grouped points with local coords
    """
    B, N, _ = xyz.shape

    # FPS to select centroids
    fps_idx = farthest_point_sample(xyz, npoint)  # (B, npoint)
    new_xyz = index_points(xyz, fps_idx)  # (B, npoint, 3)

    # Ball query to find neighbors
    idx = ball_query(radius, nsample, xyz, new_xyz)  # (B, npoint, nsample)

    # Group xyz coordinates
    grouped_xyz = index_points(xyz, idx)  # (B, npoint, nsample, 3)
    # Make coordinates relative to centroid
    grouped_xyz_norm = grouped_xyz - new_xyz.view(B, npoint, 1, 3)

    # Group features if present
    if points is not None:
        grouped_points = index_points(points, idx)  # (B, npoint, nsample, C)
        new_points = torch.cat([grouped_xyz_norm, grouped_points], dim=-1)
    else:
        new_points = grouped_xyz_norm

    return new_xyz, new_points


class SetAbstraction(nn.Module):
    """
    PointNet++ Set Abstraction layer.

    Performs: FPS sampling -> Ball query grouping -> Mini-PointNet on groups
    """

    def __init__(
        self,
        npoint: int,
        radius: float,
        nsample: int,
        in_channel: int,
        mlp: List[int],
    ):
        """
        Args:
            npoint: number of centroids to sample
            radius: ball query radius
            nsample: neighbors per centroid
            in_channel: input feature channels (excluding xyz)
            mlp: output channels for each MLP layer
        """
        super().__init__()
        self.npoint = npoint
        self.radius = radius
        self.nsample = nsample

        # MLP layers (applied to grouped points)
        # Input: 3 (local xyz) + in_channel (features)
        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()

        last_channel = 3 + in_channel
        for out_channel in mlp:
            self.mlp_convs.append(nn.Conv2d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm2d(out_channel))
            last_channel = out_channel

    def forward(
        self,
        xyz: torch.Tensor,
        points: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            xyz: (B, N, 3) coordinates
            points: (B, N, C) features (optional)

        Returns:
            new_xyz: (B, npoint, 3) downsampled coordinates
            new_points: (B, npoint, mlp[-1]) output features
        """
        # Sample and group
        new_xyz, new_points = sample_and_group(
            self.npoint, self.radius, self.nsample, xyz, points
        )
        # new_points: (B, npoint, nsample, 3+C)

        # Permute for Conv2d: (B, 3+C, nsample, npoint)
        new_points = new_points.permute(0, 3, 2, 1)

        # Apply MLP
        for conv, bn in zip(self.mlp_convs, self.mlp_bns):
            new_points = F.relu(bn(conv(new_points)))

        # Max pool over neighbors: (B, mlp[-1], npoint)
        new_points = torch.max(new_points, dim=2)[0]

        # Permute back: (B, npoint, mlp[-1])
        new_points = new_points.permute(0, 2, 1)

        return new_xyz, new_points


class TactilePointNetPPRegressor(nn.Module):
    """
    PointNet++ regressor for tactile point cloud radius prediction.

    Architecture:
        Input (B, C, N) -> SA1 -> SA2 -> SA3 -> GlobalPool -> FC -> radius
    """

    def __init__(
        self,
        input_channels: int = 7,
        dropout: float = 0.4,
        sa_configs: Optional[List[dict]] = None,
    ):
        """
        Args:
            input_channels: number of input channels (xyz=3 + features)
            dropout: dropout rate for FC layers
            sa_configs: list of SA layer configs, each with:
                npoint, radius, nsample, mlp
        """
        super().__init__()

        if sa_configs is None:
            sa_configs = POINTNETPP_CONFIG["sa_configs"]

        # Feature channels (excluding xyz which is always 3)
        feature_channels = input_channels - 3

        # Build Set Abstraction layers
        self.sa_layers = nn.ModuleList()
        in_channel = feature_channels

        for cfg in sa_configs:
            sa = SetAbstraction(
                npoint=cfg["npoint"],
                radius=cfg["radius"],
                nsample=cfg["nsample"],
                in_channel=in_channel,
                mlp=cfg["mlp"],
            )
            self.sa_layers.append(sa)
            in_channel = cfg["mlp"][-1]

        # Final feature dimension after all SA layers
        self.final_feat_dim = sa_configs[-1]["mlp"][-1]

        # Regression head
        self.fc1 = nn.Linear(self.final_feat_dim, 512)
        self.bn1 = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, 256)
        self.bn2 = nn.BatchNorm1d(256)
        self.fc3 = nn.Linear(256, 1)

        self.dropout = nn.Dropout(p=dropout)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, C, N) point cloud where C = input_channels

        Returns:
            radius: (B, 1) predicted radius
        """
        B, C, N = x.shape

        # Separate xyz and features
        # Input is (B, C, N), convert to (B, N, C)
        x = x.permute(0, 2, 1)
        xyz = x[:, :, :3].contiguous()  # (B, N, 3)
        points = x[:, :, 3:].contiguous() if C > 3 else None  # (B, N, C-3)

        # Apply Set Abstraction layers
        for sa in self.sa_layers:
            xyz, points = sa(xyz, points)

        # Global max pooling: (B, npoint, C) -> (B, C)
        global_feat = torch.max(points, dim=1)[0]

        # Regression head
        x = F.relu(self.bn1(self.fc1(global_feat)))
        x = self.dropout(x)
        x = F.relu(self.bn2(self.fc2(x)))
        x = self.dropout(x)
        radius = F.relu(self.fc3(x))  # ReLU ensures positive radius

        return radius


def count_parameters(model: nn.Module) -> int:
    """Count trainable parameters."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    # Test model
    model = TactilePointNetPPRegressor(
        input_channels=7,  # xyz + intensity + normals
        dropout=0.4,
    )
    print(f"PointNet++ model: {count_parameters(model):,} parameters")

    # Test forward pass
    x = torch.randn(4, 7, 1024)  # (B, C, N)
    radius = model(x)
    print(f"Input shape: {x.shape}")
    print(f"Output shape: {radius.shape}")

    # Test with different input channels
    model_4ch = TactilePointNetPPRegressor(input_channels=4)
    x_4ch = torch.randn(4, 4, 1024)
    out_4ch = model_4ch(x_4ch)
    print(f"4-channel output shape: {out_4ch.shape}")
