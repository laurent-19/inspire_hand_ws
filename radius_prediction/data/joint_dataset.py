"""Dataset for joint states (optionally with RGB images)."""

import json
import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch
from PIL import Image
import torchvision.transforms as T

from .base_dataset import GraspRadiusDataset
from ..config import JOINT_MODEL_CONFIG, AUGMENTATION_CONFIG


class JointDataset(GraspRadiusDataset):
    """
    Dataset for Model 1: Joint states (optionally with RGB images).

    Args:
        root_dir: Path to non_deformable directory
        val_object: Object name to hold out for validation
        split: 'train' or 'val'
        use_image: Whether to include RGB images
        image_size: Target image size (H, W)
        augment: Whether to apply data augmentation
        normalize_target: Whether to normalize radius to [0, 1]
    """

    def __init__(
        self,
        root_dir: Optional[Path] = None,
        val_object: str = "mid_bottle",
        split: str = "train",
        use_image: bool = False,
        image_size: Tuple[int, int] = (224, 224),
        augment: bool = True,
        normalize_target: bool = True,
    ):
        super().__init__(
            root_dir=root_dir,
            val_object=val_object,
            split=split,
            normalize_target=normalize_target,
        )

        self.use_image = use_image
        self.image_size = image_size
        self.augment = augment and (split == "train")

        # Image transforms
        if self.use_image:
            self._setup_image_transforms()

    def _setup_image_transforms(self):
        """Set up image transforms based on augmentation settings."""
        cfg = AUGMENTATION_CONFIG

        if self.augment:
            self.image_transform = T.Compose([
                T.Resize(self.image_size),
                T.RandomHorizontalFlip(p=cfg["image_h_flip"]),
                T.RandomRotation(degrees=cfg["image_rotation"]),
                T.ColorJitter(
                    brightness=cfg["image_brightness"],
                    contrast=cfg["image_contrast"],
                    saturation=cfg["image_saturation"],
                ),
                T.ToTensor(),
                T.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225],
                ),
            ])
        else:
            self.image_transform = T.Compose([
                T.Resize(self.image_size),
                T.ToTensor(),
                T.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225],
                ),
            ])

    def _load_joints(self, sample_path: Path) -> torch.Tensor:
        """Load joint states from JSON file."""
        joint_path = sample_path / "joint_state.json"

        with open(joint_path, "r") as f:
            data = json.load(f)

        # Extract positions (12 joint angles in radians)
        positions = data["position"]
        joints = torch.tensor(positions, dtype=torch.float32)

        # Normalize by pi to get [-1, 1] range
        joints = joints / math.pi

        # Apply augmentation (noise)
        if self.augment:
            noise_std = AUGMENTATION_CONFIG["joint_noise_std"]
            joints = joints + torch.randn_like(joints) * noise_std

        return joints

    def _load_image(self, sample_path: Path) -> torch.Tensor:
        """Load RGB image."""
        image_path = sample_path / "camera_rgb.png"
        image = Image.open(image_path).convert("RGB")
        return self.image_transform(image)

    def __getitem__(self, idx: int):
        """
        Get a sample.

        Returns:
            If use_image=False:
                joints: (12,) tensor of joint angles
                target: scalar radius
            If use_image=True:
                image: (3, H, W) tensor
                joints: (12,) tensor
                target: scalar radius
        """
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load joint states
        joints = self._load_joints(sample_path)

        # Get target
        target = torch.tensor(self.get_target(idx), dtype=torch.float32)

        if self.use_image:
            image = self._load_image(sample_path)
            return image, joints, target
        else:
            return joints, target
