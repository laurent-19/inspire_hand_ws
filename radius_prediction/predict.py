#!/usr/bin/env python3
"""Run radius prediction on a single sample."""

import argparse
from pathlib import Path
import json

import torch
import numpy as np
from PIL import Image
import torchvision.transforms as T

from radius_prediction.models.joint_model import JointRadiusModel
from radius_prediction.models.pointnet import TactilePointNetRegressor
from radius_prediction.data.pointcloud_dataset import (
    load_pcd_binary,
    farthest_point_sample,
    normalize_point_cloud,
)
from radius_prediction.config import denormalize_radius


def load_joint_model(checkpoint_path: Path, use_image: bool = False, device: str = "cuda"):
    """Load joint-based model from checkpoint."""
    checkpoint = torch.load(checkpoint_path, map_location=device)

    model = JointRadiusModel(use_image=use_image)
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    return model, checkpoint


def load_pointnet_model(checkpoint_path: Path, device: str = "cuda"):
    """Load PointNet model from checkpoint."""
    checkpoint = torch.load(checkpoint_path, map_location=device)
    config = checkpoint.get("config", {})

    input_channels = config.get("input_channels", 4)

    model = TactilePointNetRegressor(
        input_channels=input_channels,
        feature_transform=True,
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    return model, checkpoint


def predict_joint_only(model, sample_path: Path, device: str = "cuda"):
    """Predict radius using joint-only model."""
    # Load joint state
    joint_path = sample_path / "joint_state.json"
    with open(joint_path, "r") as f:
        data = json.load(f)

    # Prepare input
    joints = torch.tensor(data["position"], dtype=torch.float32) / np.pi
    joints = joints.unsqueeze(0).to(device)  # (1, 12)

    # Predict
    with torch.no_grad():
        pred = model(joints)

    return denormalize_radius(pred.squeeze().cpu().item())


def predict_joint_image(model, sample_path: Path, device: str = "cuda"):
    """Predict radius using joint + image model."""
    # Load joint state
    joint_path = sample_path / "joint_state.json"
    with open(joint_path, "r") as f:
        data = json.load(f)

    joints = torch.tensor(data["position"], dtype=torch.float32) / np.pi
    joints = joints.unsqueeze(0).to(device)  # (1, 12)

    # Load image
    image_path = sample_path / "camera_rgb.png"
    image = Image.open(image_path).convert("RGB")

    transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    image = transform(image).unsqueeze(0).to(device)  # (1, 3, 224, 224)

    # Predict
    with torch.no_grad():
        pred = model(joints, image)

    return denormalize_radius(pred.squeeze().cpu().item())


def predict_pointnet(model, sample_path: Path, num_points: int = 1024,
                     use_intensity: bool = True, device: str = "cuda"):
    """Predict radius using PointNet model."""
    # Load point cloud
    pcd_path = sample_path / "tactile_pointcloud.pcd"
    points = load_pcd_binary(pcd_path, use_intensity=use_intensity)

    # Preprocess
    points = farthest_point_sample(points, num_points)
    points = normalize_point_cloud(points)

    # Convert to tensor (C, N)
    points = torch.from_numpy(points).float().T.unsqueeze(0).to(device)  # (1, C, N)

    # Predict
    with torch.no_grad():
        pred, _ = model(points)

    return denormalize_radius(pred.squeeze().cpu().item())


def get_ground_truth(sample_path: Path):
    """Extract ground truth radius from sample path."""
    from radius_prediction.data.base_dataset import extract_radius

    # Get record name from path
    record_name = sample_path.parent.name
    return extract_radius(record_name)


def main():
    parser = argparse.ArgumentParser(description="Predict radius for a sample")
    parser.add_argument(
        "sample_path", type=str,
        help="Path to sample directory (e.g., training_data/non_deformable/record_250_1/sample_0000)"
    )
    parser.add_argument(
        "--checkpoint", type=str, required=True,
        help="Path to model checkpoint"
    )
    parser.add_argument(
        "--model_type", type=str, required=True,
        choices=["joint", "image_joint", "pointnet"],
        help="Type of model"
    )
    parser.add_argument(
        "--device", type=str, default="cuda",
        help="Device to use"
    )
    parser.add_argument(
        "--num_points", type=int, default=1024,
        help="Number of points for PointNet (default: 1024)"
    )
    args = parser.parse_args()

    sample_path = Path(args.sample_path)
    checkpoint_path = Path(args.checkpoint)
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")

    if not sample_path.exists():
        print(f"Error: Sample path not found: {sample_path}")
        return

    if not checkpoint_path.exists():
        print(f"Error: Checkpoint not found: {checkpoint_path}")
        return

    print(f"Sample: {sample_path}")
    print(f"Model: {args.model_type}")
    print(f"Device: {device}")
    print()

    # Get ground truth
    gt_radius = get_ground_truth(sample_path)

    # Load model and predict
    if args.model_type == "joint":
        model, checkpoint = load_joint_model(checkpoint_path, use_image=False, device=device)
        pred_radius = predict_joint_only(model, sample_path, device)
    elif args.model_type == "image_joint":
        model, checkpoint = load_joint_model(checkpoint_path, use_image=True, device=device)
        pred_radius = predict_joint_image(model, sample_path, device)
    elif args.model_type == "pointnet":
        model, checkpoint = load_pointnet_model(checkpoint_path, device=device)
        pred_radius = predict_pointnet(model, sample_path, args.num_points, device=device)
    else:
        print(f"Unknown model type: {args.model_type}")
        return

    # Print results
    error = abs(pred_radius - gt_radius)

    print("=" * 60)
    print(f"Ground Truth:  {gt_radius:.2f} mm")
    print(f"Prediction:    {pred_radius:.2f} mm")
    print(f"Error:         {error:.2f} mm")
    print("=" * 60)

    # Model info
    print(f"\nModel trained on: {checkpoint['epoch']} epochs")
    print(f"Best Val RMSE: {checkpoint.get('val_rmse', 'N/A'):.2f} mm")
    print(f"Best Val MAE:  {checkpoint.get('val_mae', 'N/A'):.2f} mm")


if __name__ == "__main__":
    main()
