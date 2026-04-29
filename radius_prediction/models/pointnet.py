"""PointNet-based tactile point cloud radius prediction.

Adapted from 3d_process.py and inspired by ge2018 paper.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

from ..config import POINTNET_CONFIG


class TNet(nn.Module):
    """
    Transformation Network for learning affine transformation matrix.

    Produces a k x k transformation matrix to transform input coordinates
    or features, making the network invariant to certain transformations.

    Adapted from 3d_process.py:391-445.
    """

    def __init__(self, k: int = 4):
        """
        Args:
            k: Input dimensionality (e.g., 4 for xyz + intensity)
        """
        super().__init__()
        self.k = k

        # Point-wise MLPs
        self.conv1 = nn.Conv1d(k, 64, 1)
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 1024, 1)

        # Batch normalization
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)

        # Fully connected layers
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, k * k)

        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, k, N) input features

        Returns:
            transform: (B, k, k) transformation matrix
        """
        batch_size = x.size(0)

        # Point-wise convolutions
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))

        # Global max pooling
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        # Fully connected layers
        x = F.relu(self.bn4(self.fc1(x)))
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.fc3(x)

        # Add identity matrix for stable initialization
        identity = torch.eye(self.k, dtype=torch.float32, device=x.device)
        identity = identity.view(1, self.k * self.k).repeat(batch_size, 1)
        x = x + identity

        # Reshape to transformation matrix
        x = x.view(-1, self.k, self.k)

        return x


class TactilePointNetEncoder(nn.Module):
    """
    PointNet encoder for tactile point clouds.

    Extracts global features from point cloud with optional
    input and feature transformations.

    Adapted from 3d_process.py:448-533.
    """

    def __init__(
        self,
        input_channels: int = 4,  # xyz + intensity
        feature_transform: bool = True,
    ):
        super().__init__()
        self.input_channels = input_channels
        self.feature_transform = feature_transform

        # Input transformation network
        self.input_tnet = TNet(k=input_channels)

        # First MLP
        self.conv1 = nn.Conv1d(input_channels, 64, 1)
        self.bn1 = nn.BatchNorm1d(64)

        # Feature transformation network (optional)
        if feature_transform:
            self.feature_tnet = TNet(k=64)

        # Second MLP
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 1024, 1)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)

    def forward(self, x: torch.Tensor):
        """
        Args:
            x: (B, C, N) point cloud where C=input_channels

        Returns:
            global_feat: (B, 1024) global features
            trans_input: (B, C, C) input transformation matrix
            trans_feat: (B, 64, 64) feature transformation matrix (or None)
        """
        batch_size, _, num_points = x.size()

        # Input transformation
        trans_input = self.input_tnet(x)
        x = x.transpose(2, 1)  # (B, N, C)
        x = torch.bmm(x, trans_input)  # Apply transformation
        x = x.transpose(2, 1)  # (B, C, N)

        # First MLP
        x = F.relu(self.bn1(self.conv1(x)))

        # Feature transformation
        if self.feature_transform:
            trans_feat = self.feature_tnet(x)
            x = x.transpose(2, 1)
            x = torch.bmm(x, trans_feat)
            x = x.transpose(2, 1)
        else:
            trans_feat = None

        # Second MLP
        x = F.relu(self.bn2(self.conv2(x)))
        x = self.bn3(self.conv3(x))

        # Global max pooling
        global_feat = torch.max(x, 2)[0]  # (B, 1024)

        return global_feat, trans_input, trans_feat


def feature_transform_regularization(trans: torch.Tensor) -> torch.Tensor:
    """
    Regularization loss for feature transformation matrix.

    Encourages orthogonality: ||T^T * T - I|| should be small.

    Adapted from 3d_process.py:536-553.

    Args:
        trans: (B, k, k) transformation matrices

    Returns:
        loss: scalar regularization loss
    """
    batch_size, k, _ = trans.size()
    identity = torch.eye(k, device=trans.device)[None, :, :]
    diff = torch.bmm(trans, trans.transpose(2, 1)) - identity
    loss = torch.norm(diff, dim=(1, 2))
    return torch.mean(loss)


class TactilePointNetRegressor(nn.Module):
    """
    PointNet regressor for tactile point cloud radius prediction.

    Architecture:
        Input (B, C, N) -> TNet -> MLP -> TNet -> MLP -> MaxPool -> FC -> radius
    """

    def __init__(
        self,
        input_channels: int = 4,
        feature_transform: bool = True,
        dropout: float = 0.4,
    ):
        super().__init__()
        self.feature_transform = feature_transform

        # Encoder
        self.encoder = TactilePointNetEncoder(
            input_channels=input_channels,
            feature_transform=feature_transform,
        )

        # Regression head
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 1)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)

        self.dropout = nn.Dropout(p=dropout)

    def forward(self, x: torch.Tensor):
        """
        Args:
            x: (B, C, N) point cloud

        Returns:
            radius: (B, 1) predicted radius
            trans_feat: (B, 64, 64) feature transform for regularization
        """
        # Encode
        global_feat, trans_input, trans_feat = self.encoder(x)

        # Regression head
        x = F.relu(self.bn1(self.fc1(global_feat)))
        x = F.relu(self.bn2(self.dropout(self.fc2(x))))
        radius = F.relu(self.fc3(x))  # ReLU ensures positive radius

        return radius, trans_feat


class PointNetLoss(nn.Module):
    """
    Combined loss for PointNet with feature transform regularization.

    Loss = MSE + weight * regularization

    Adapted from 3d_process.py:609-639.
    """

    def __init__(self, feature_transform_weight: float = 0.001):
        super().__init__()
        self.feature_transform_weight = feature_transform_weight
        self.mse_loss = nn.MSELoss()

    def forward(
        self,
        pred: torch.Tensor,
        target: torch.Tensor,
        trans_feat: torch.Tensor = None,
    ) -> torch.Tensor:
        """
        Args:
            pred: (B, 1) predicted radius
            target: (B,) or (B, 1) target radius
            trans_feat: (B, 64, 64) feature transform matrix

        Returns:
            loss: scalar loss value
        """
        # Ensure target shape matches pred
        if target.dim() == 1:
            target = target.unsqueeze(1)

        # MSE loss
        mse = self.mse_loss(pred, target)

        # Add regularization if feature transform is provided
        if trans_feat is not None and self.feature_transform_weight > 0:
            reg = feature_transform_regularization(trans_feat)
            return mse + self.feature_transform_weight * reg

        return mse


def count_parameters(model: nn.Module) -> int:
    """Count trainable parameters."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    # Test model
    model = TactilePointNetRegressor(
        input_channels=4,
        feature_transform=True,
        dropout=0.4,
    )
    print(f"PointNet model: {count_parameters(model):,} parameters")

    # Test forward pass
    x = torch.randn(4, 4, 2048)  # (B, C, N)
    radius, trans_feat = model(x)
    print(f"Output shape: {radius.shape}")
    print(f"Trans feat shape: {trans_feat.shape}")

    # Test loss
    criterion = PointNetLoss(feature_transform_weight=0.001)
    target = torch.rand(4)
    loss = criterion(radius, target, trans_feat)
    print(f"Loss: {loss.item():.4f}")
