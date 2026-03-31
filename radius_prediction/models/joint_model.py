"""Joint-based radius prediction model (optionally with RGB image fusion)."""

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models

from ..config import JOINT_MODEL_CONFIG


class JointEncoder(nn.Module):
    """MLP encoder for joint angle features."""

    def __init__(
        self,
        input_dim: int = 12,
        hidden_dims: list = None,
        output_dim: int = 128,
        dropout: float = 0.2,
    ):
        super().__init__()

        if hidden_dims is None:
            hidden_dims = [64, 128]

        layers = []
        prev_dim = input_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.BatchNorm1d(hidden_dim),
                nn.ReLU(inplace=True),
                nn.Dropout(dropout),
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, output_dim))
        self.encoder = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, num_joints) joint angles

        Returns:
            features: (B, output_dim)
        """
        return self.encoder(x)


class JointRadiusModel(nn.Module):
    """
    Radius prediction from joint states (optionally with RGB image).

    Mode A (use_image=False):
        Joint (12) -> MLP -> radius

    Mode B (use_image=True):
        Joint (12) -> MLP ---|
                             |---> Concat -> MLP -> radius
        Image -> ResNet -----|
    """

    def __init__(
        self,
        use_image: bool = False,
        num_joints: int = 12,
        backbone: str = "resnet18",
        pretrained: bool = True,
        freeze_backbone: bool = False,
    ):
        super().__init__()

        self.use_image = use_image
        self.num_joints = num_joints

        if use_image:
            # Image backbone
            self._setup_backbone(backbone, pretrained, freeze_backbone)

            # Joint encoder (smaller since fusing with image)
            self.joint_encoder = JointEncoder(
                input_dim=num_joints,
                hidden_dims=[64, 128],
                output_dim=128,
                dropout=0.2,
            )

            # Fusion layers
            fusion_input_dim = self.backbone_out_dim + 128
            self.fusion = nn.Sequential(
                nn.Linear(fusion_input_dim, 256),
                nn.BatchNorm1d(256),
                nn.ReLU(inplace=True),
                nn.Dropout(0.3),
                nn.Linear(256, 128),
                nn.BatchNorm1d(128),
                nn.ReLU(inplace=True),
                nn.Dropout(0.2),
            )

            # Output head
            self.output_head = nn.Sequential(
                nn.Linear(128, 1),
                nn.ReLU(),  # Radius must be positive
            )

        else:
            # Joint-only model: deeper MLP
            hidden_dims = JOINT_MODEL_CONFIG["hidden_dims"]

            layers = []
            prev_dim = num_joints

            for hidden_dim in hidden_dims:
                layers.extend([
                    nn.Linear(prev_dim, hidden_dim),
                    nn.BatchNorm1d(hidden_dim),
                    nn.ReLU(inplace=True),
                    nn.Dropout(0.2),
                ])
                prev_dim = hidden_dim

            layers.extend([
                nn.Linear(prev_dim, 1),
                nn.ReLU(),  # Radius must be positive
            ])

            self.joint_mlp = nn.Sequential(*layers)

    def _setup_backbone(self, backbone: str, pretrained: bool, freeze: bool):
        """Set up image backbone."""
        if backbone == "resnet18":
            weights = models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            model = models.resnet18(weights=weights)
            self.backbone_out_dim = 512
        elif backbone == "resnet34":
            weights = models.ResNet34_Weights.IMAGENET1K_V1 if pretrained else None
            model = models.resnet34(weights=weights)
            self.backbone_out_dim = 512
        elif backbone == "resnet50":
            weights = models.ResNet50_Weights.IMAGENET1K_V1 if pretrained else None
            model = models.resnet50(weights=weights)
            self.backbone_out_dim = 2048
        else:
            raise ValueError(f"Unsupported backbone: {backbone}")

        # Remove classification head (keep up to avgpool)
        self.backbone = nn.Sequential(*list(model.children())[:-1])

        if freeze:
            for param in self.backbone.parameters():
                param.requires_grad = False

    def freeze_backbone(self):
        """Freeze backbone weights."""
        if self.use_image:
            for param in self.backbone.parameters():
                param.requires_grad = False

    def unfreeze_backbone(self):
        """Unfreeze backbone weights."""
        if self.use_image:
            for param in self.backbone.parameters():
                param.requires_grad = True

    def forward(
        self,
        joints: torch.Tensor,
        image: torch.Tensor = None,
    ) -> torch.Tensor:
        """
        Forward pass.

        Args:
            joints: (B, num_joints) joint angles
            image: (B, 3, H, W) RGB image (only used if use_image=True)

        Returns:
            radius: (B, 1) predicted radius
        """
        if self.use_image:
            if image is None:
                raise ValueError("Image required when use_image=True")

            # Extract image features
            img_feat = self.backbone(image)
            img_feat = img_feat.flatten(1)  # (B, backbone_out_dim)

            # Extract joint features
            joint_feat = self.joint_encoder(joints)  # (B, 128)

            # Fuse features
            fused = torch.cat([img_feat, joint_feat], dim=1)
            fused = self.fusion(fused)

            # Output
            radius = self.output_head(fused)

        else:
            # Joint-only mode
            radius = self.joint_mlp(joints)

        return radius


def count_parameters(model: nn.Module) -> int:
    """Count trainable parameters."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    # Test joint-only model
    model_joint = JointRadiusModel(use_image=False)
    print(f"Joint-only model: {count_parameters(model_joint):,} parameters")

    joints = torch.randn(4, 12)
    out = model_joint(joints)
    print(f"Joint-only output shape: {out.shape}")

    # Test fusion model
    model_fusion = JointRadiusModel(use_image=True, backbone="resnet18")
    print(f"Fusion model: {count_parameters(model_fusion):,} parameters")

    image = torch.randn(4, 3, 224, 224)
    out = model_fusion(joints, image)
    print(f"Fusion output shape: {out.shape}")
