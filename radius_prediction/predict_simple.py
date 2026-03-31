#!/usr/bin/env python3
"""Simple inference script with visualization."""

import argparse
from pathlib import Path
import json

import torch
import numpy as np

from radius_prediction.models.joint_model import JointRadiusModel
from radius_prediction.models.pointnet import TactilePointNetRegressor
from radius_prediction.data.pointcloud_dataset import (
    load_pcd_binary,
    farthest_point_sample,
    normalize_point_cloud,
)
from radius_prediction.data.base_dataset import extract_radius


def predict_joint(checkpoint_path: str, sample_path: str, device: str = "cuda"):
    """Predict using joint model."""
    # Load model
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model = JointRadiusModel(use_image=False)
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    # Load joint data
    joint_file = Path(sample_path) / "joint_state.json"
    with open(joint_file) as f:
        data = json.load(f)

    joints = torch.tensor(data["position"], dtype=torch.float32) / np.pi
    joints = joints.unsqueeze(0).to(device)

    # Predict (model outputs in mm directly with ReLU)
    with torch.no_grad():
        pred_mm = model(joints).squeeze().item()

    return pred_mm


def predict_pointnet(checkpoint_path: str, sample_path: str,
                     num_points: int = 1024, device: str = "cuda"):
    """Predict using PointNet model."""
    # Load model
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model = TactilePointNetRegressor(input_channels=4, feature_transform=True)
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    # Load point cloud
    pcd_file = Path(sample_path) / "tactile_pointcloud.pcd"
    points = load_pcd_binary(pcd_file, use_intensity=True)
    points = farthest_point_sample(points, num_points)
    points = normalize_point_cloud(points)

    points = torch.from_numpy(points).float().T.unsqueeze(0).to(device)

    # Predict (model outputs in mm directly with ReLU)
    with torch.no_grad():
        pred_mm, _ = model(points)

    return pred_mm.squeeze().item()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sample_path", help="Path to sample directory")
    parser.add_argument("--checkpoint", required=True, help="Model checkpoint")
    parser.add_argument("--model", required=True, choices=["joint", "pointnet"])
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()

    sample_path = Path(args.sample_path)

    # Get ground truth
    gt_radius = extract_radius(sample_path.parent.name)

    # Predict
    print(f"\nSample: {sample_path}")
    print(f"Model: {args.model}\n")

    if args.model == "joint":
        pred = predict_joint(args.checkpoint, sample_path, args.device)
    else:
        pred = predict_pointnet(args.checkpoint, sample_path, device=args.device)

    error = abs(pred - gt_radius)

    print(f"Ground Truth: {gt_radius:.2f} mm")
    print(f"Prediction:   {pred:.2f} mm")
    print(f"Error:        {error:.2f} mm\n")


if __name__ == "__main__":
    main()
