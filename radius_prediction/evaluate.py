#!/usr/bin/env python3
"""Evaluation script for radius prediction models."""

import argparse
from pathlib import Path
from collections import defaultdict

import torch
import numpy as np
from torch.utils.data import DataLoader

from radius_prediction.data.joint_dataset import JointDataset
from radius_prediction.data.pointcloud_dataset import TactilePointCloudDataset
from radius_prediction.models.joint_model import JointRadiusModel
from radius_prediction.models.pointnet import TactilePointNetRegressor
from radius_prediction.config import (
    NON_DEFORMABLE_DIR,
    CHECKPOINT_DIR,
    ALL_OBJECTS,
    denormalize_radius,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate radius prediction model")
    parser.add_argument(
        "checkpoint", type=str,
        help="Path to model checkpoint"
    )
    parser.add_argument(
        "--model_type", type=str, required=True,
        choices=["joint", "image_joint", "pointnet"],
        help="Type of model"
    )
    parser.add_argument(
        "--val_object", type=str, default=None,
        help="Object for validation (default: from checkpoint)"
    )
    parser.add_argument(
        "--device", type=str, default="cuda",
        help="Device to use"
    )
    return parser.parse_args()


def compute_metrics(predictions, targets, object_names):
    """Compute evaluation metrics."""
    predictions = np.array(predictions)
    targets = np.array(targets)
    object_names = np.array(object_names)

    # Overall metrics
    errors = np.abs(predictions - targets)
    mse = np.mean((predictions - targets) ** 2)
    rmse = np.sqrt(mse)
    mae = np.mean(errors)

    # R-squared
    ss_res = np.sum((targets - predictions) ** 2)
    ss_tot = np.sum((targets - np.mean(targets)) ** 2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    # Accuracy within thresholds
    acc_1mm = np.mean(errors <= 1.0) * 100
    acc_2mm = np.mean(errors <= 2.0) * 100
    acc_3mm = np.mean(errors <= 3.0) * 100

    # Per-object metrics
    per_object = {}
    for obj in np.unique(object_names):
        mask = object_names == obj
        obj_errors = errors[mask]
        per_object[obj] = {
            "mae": np.mean(obj_errors),
            "rmse": np.sqrt(np.mean(obj_errors ** 2)),
            "count": np.sum(mask),
        }

    return {
        "mse": mse,
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
        "acc_1mm": acc_1mm,
        "acc_2mm": acc_2mm,
        "acc_3mm": acc_3mm,
        "per_object": per_object,
    }


def evaluate_joint_model(checkpoint_path, val_object, use_image, device):
    """Evaluate joint-based model."""
    # Load checkpoint
    checkpoint = torch.load(checkpoint_path, map_location=device)

    # Create model
    model = JointRadiusModel(use_image=use_image)
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    # Load dataset
    dataset = JointDataset(
        root_dir=NON_DEFORMABLE_DIR,
        val_object=val_object,
        split="val",
        use_image=use_image,
        augment=False,
    )

    loader = DataLoader(dataset, batch_size=32, shuffle=False, num_workers=4)

    # Evaluate
    predictions = []
    targets = []
    object_names = []

    with torch.no_grad():
        for idx, batch in enumerate(loader):
            if use_image:
                images, joints, target = batch
                images = images.to(device)
                joints = joints.to(device)
                outputs = model(joints, images)
            else:
                joints, target = batch
                joints = joints.to(device)
                outputs = model(joints)

            # Denormalize
            preds_mm = denormalize_radius(outputs.squeeze().cpu()).numpy()
            targs_mm = denormalize_radius(target.cpu()).numpy()

            predictions.extend(preds_mm.tolist())
            targets.extend(targs_mm.tolist())

            # Get object names for this batch
            start_idx = idx * loader.batch_size
            end_idx = start_idx + len(target)
            for i in range(start_idx, end_idx):
                object_names.append(dataset.samples[i]["object_name"])

    return predictions, targets, object_names


def evaluate_pointnet_model(checkpoint_path, val_object, device):
    """Evaluate PointNet model."""
    # Load checkpoint
    checkpoint = torch.load(checkpoint_path, map_location=device)
    config = checkpoint.get("config", {})

    input_channels = config.get("input_channels", 4)
    use_intensity = config.get("use_intensity", True)
    num_points = config.get("num_points", 2048)

    # Create model
    model = TactilePointNetRegressor(
        input_channels=input_channels,
        feature_transform=True,
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    # Load dataset
    dataset = TactilePointCloudDataset(
        root_dir=NON_DEFORMABLE_DIR,
        val_object=val_object,
        split="val",
        num_points=num_points,
        use_intensity=use_intensity,
        augment=False,
    )

    loader = DataLoader(dataset, batch_size=16, shuffle=False, num_workers=4)

    # Evaluate
    predictions = []
    targets = []
    object_names = []

    with torch.no_grad():
        for idx, (points, target) in enumerate(loader):
            points = points.to(device)
            outputs, _ = model(points)

            # Denormalize
            preds_mm = denormalize_radius(outputs.squeeze().cpu()).numpy()
            targs_mm = denormalize_radius(target.cpu()).numpy()

            predictions.extend(preds_mm.tolist())
            targets.extend(targs_mm.tolist())

            # Get object names for this batch
            start_idx = idx * loader.batch_size
            end_idx = start_idx + len(target)
            for i in range(start_idx, end_idx):
                object_names.append(dataset.samples[i]["object_name"])

    return predictions, targets, object_names


def print_results(metrics, model_type, val_object):
    """Print evaluation results."""
    print("\n" + "=" * 60)
    print(f"Model: {model_type}")
    print(f"Validation Object: {val_object}")
    print("=" * 60)

    print("\nOverall Metrics:")
    print(f"  RMSE:  {metrics['rmse']:.3f} mm")
    print(f"  MAE:   {metrics['mae']:.3f} mm")
    print(f"  R2:    {metrics['r2']:.4f}")
    print(f"  Acc@1mm: {metrics['acc_1mm']:.1f}%")
    print(f"  Acc@2mm: {metrics['acc_2mm']:.1f}%")
    print(f"  Acc@3mm: {metrics['acc_3mm']:.1f}%")

    print("\nPer-Object Metrics:")
    for obj, obj_metrics in sorted(metrics["per_object"].items()):
        print(f"  {obj:15s}: MAE={obj_metrics['mae']:.3f}mm, "
              f"RMSE={obj_metrics['rmse']:.3f}mm, n={obj_metrics['count']}")


def main():
    args = parse_args()

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    checkpoint_path = Path(args.checkpoint)
    if not checkpoint_path.exists():
        print(f"Error: Checkpoint not found: {checkpoint_path}")
        return

    # Load checkpoint to get val_object if not specified
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    val_object = args.val_object or checkpoint.get("config", {}).get("val_object", "mid_bottle")

    print(f"\nEvaluating {args.model_type} model...")
    print(f"Checkpoint: {checkpoint_path}")

    # Evaluate based on model type
    if args.model_type == "joint":
        predictions, targets, object_names = evaluate_joint_model(
            checkpoint_path, val_object, use_image=False, device=device
        )
    elif args.model_type == "image_joint":
        predictions, targets, object_names = evaluate_joint_model(
            checkpoint_path, val_object, use_image=True, device=device
        )
    elif args.model_type == "pointnet":
        predictions, targets, object_names = evaluate_pointnet_model(
            checkpoint_path, val_object, device=device
        )
    else:
        print(f"Unknown model type: {args.model_type}")
        return

    # Compute and print metrics
    metrics = compute_metrics(predictions, targets, object_names)
    print_results(metrics, args.model_type, val_object)


if __name__ == "__main__":
    main()
