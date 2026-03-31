"""Configuration for radius prediction models."""

import os
from pathlib import Path

# Paths
ROOT_DIR = Path(__file__).parent.parent
TRAINING_DATA_DIR = ROOT_DIR / "training_data"
NON_DEFORMABLE_DIR = TRAINING_DATA_DIR / "non_deformable"
CHECKPOINT_DIR = ROOT_DIR / "radius_prediction" / "checkpoints"

# Object name to radius mapping (in mm)
OBJECT_TO_RADIUS = {
    "250": 24.0,           #
    "330_fat": 33.0,       #
    "330_slim": 29.0,      #
    "500": 33.0,           #
    "small_bottle": 32.5,
    "mid_bottle": 39.5,
    "big_bottle": 46.0,
}

# All object names
ALL_OBJECTS = list(OBJECT_TO_RADIUS.keys())

# Radius range for normalization
RADIUS_MIN = 24.0  # Updated to match actual data
RADIUS_MAX = 46.0  # Updated to match actual data

# Default validation object for leave-one-out split
DEFAULT_VAL_OBJECT = "mid_bottle"

# Joint model config
JOINT_MODEL_CONFIG = {
    "num_joints": 12,
    "hidden_dims": [64, 128, 256, 128],
    "dropout": 0.2,
    "use_image": False,  # Set to True for fusion mode
    "image_size": (224, 224),
    "backbone": "resnet18",
    "freeze_backbone_epochs": 5,
}

# PointNet model config
POINTNET_CONFIG = {
    "num_points": 1024,  # Reduced from 2048 for faster training
    "input_channels": 4,  # xyz + intensity
    "feature_transform": True,
    "dropout": 0.4,
}

# Training config - Joint model (joint-only)
TRAIN_JOINT_CONFIG = {
    "batch_size": 32,
    "learning_rate": 1e-3,
    "weight_decay": 1e-4,
    "epochs": 100,
    "patience": 100,
    "scheduler": "cosine",
}

# Training config - Joint model (with image)
TRAIN_JOINT_IMAGE_CONFIG = {
    "batch_size": 16,
    "learning_rate": 1e-4,
    "weight_decay": 1e-4,
    "epochs": 50,
    "patience": 10,
    "scheduler": "cosine",
}

# Training config - PointNet model
TRAIN_POINTNET_CONFIG = {
    "batch_size": 32,  # Increased from 16 for better GPU utilization
    "learning_rate": 1e-3,
    "weight_decay": 1e-4,
    "epochs": 100,
    "patience": 100,
    "scheduler": "cosine",
    "feature_transform_weight": 0.001,
}

# Data augmentation config
AUGMENTATION_CONFIG = {
    # Joint augmentation
    "joint_noise_std": 0.01,

    # Image augmentation
    "image_h_flip": 0.5,
    "image_rotation": 10,
    "image_brightness": 0.2,
    "image_contrast": 0.2,
    "image_saturation": 0.2,

    # Point cloud augmentation
    "pc_rotation": True,          # Random Z-axis rotation
    "pc_scale_range": (0.8, 1.2),
    "pc_translation_range": 0.1,
    "pc_dropout_ratio": 0.5,
    "pc_intensity_noise_std": 0.02,
}


def normalize_radius(radius: float) -> float:
    """Normalize radius to [0, 1] range."""
    return (radius - RADIUS_MIN) / (RADIUS_MAX - RADIUS_MIN)


def denormalize_radius(normalized: float) -> float:
    """Denormalize radius from [0, 1] to mm."""
    return normalized * (RADIUS_MAX - RADIUS_MIN) + RADIUS_MIN
