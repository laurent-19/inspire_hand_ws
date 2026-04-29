#!/usr/bin/env python3
"""Training script for joint-based radius prediction model."""

import argparse
import os
from pathlib import Path
from datetime import datetime

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torch.optim.lr_scheduler import CosineAnnealingLR
from tqdm import tqdm

from radius_prediction.data.joint_dataset import JointDataset
from radius_prediction.models.joint_model import JointRadiusModel, count_parameters
from radius_prediction.config import (
    NON_DEFORMABLE_DIR,
    CHECKPOINT_DIR,
    TRAIN_JOINT_CONFIG,
    TRAIN_JOINT_IMAGE_CONFIG,
    ALL_OBJECTS,
    denormalize_radius,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Train joint radius model")
    parser.add_argument(
        "--use_image", action="store_true",
        help="Use RGB image in addition to joint states"
    )
    parser.add_argument(
        "--val_object", type=str, default="mid_bottle",
        choices=ALL_OBJECTS,
        help="Object to hold out for validation"
    )
    parser.add_argument(
        "--epochs", type=int, default=None,
        help="Number of epochs (default: from config)"
    )
    parser.add_argument(
        "--batch_size", type=int, default=None,
        help="Batch size (default: from config)"
    )
    parser.add_argument(
        "--lr", type=float, default=None,
        help="Learning rate (default: from config)"
    )
    parser.add_argument(
        "--device", type=str, default="cuda",
        help="Device to use (cuda or cpu)"
    )
    parser.add_argument(
        "--checkpoint_dir", type=str, default=None,
        help="Directory to save checkpoints"
    )
    return parser.parse_args()


def train_epoch(model, dataloader, criterion, optimizer, device, use_image):
    """Train for one epoch."""
    model.train()
    total_loss = 0.0
    total_samples = 0

    pbar = tqdm(dataloader, desc="Training", leave=False)
    for batch in pbar:
        if use_image:
            images, joints, targets = batch
            images = images.to(device)
            joints = joints.to(device)
            targets = targets.to(device)
            outputs = model(joints, images)
        else:
            joints, targets = batch
            joints = joints.to(device)
            targets = targets.to(device)
            outputs = model(joints)

        loss = criterion(outputs.squeeze(), targets)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        batch_size = targets.size(0)
        total_loss += loss.item() * batch_size
        total_samples += batch_size

        pbar.set_postfix({"loss": f"{loss.item():.4f}"})

    return total_loss / total_samples


def validate(model, dataloader, criterion, device, use_image):
    """Validate model."""
    model.eval()
    total_loss = 0.0
    total_mae = 0.0
    total_samples = 0

    with torch.no_grad():
        for batch in dataloader:
            if use_image:
                images, joints, targets = batch
                images = images.to(device)
                joints = joints.to(device)
                targets = targets.to(device)
                outputs = model(joints, images)
            else:
                joints, targets = batch
                joints = joints.to(device)
                targets = targets.to(device)
                outputs = model(joints)

            loss = criterion(outputs.squeeze(), targets)

            # MAE in mm (denormalized)
            preds_mm = denormalize_radius(outputs.squeeze().cpu())
            targets_mm = denormalize_radius(targets.cpu())
            mae = torch.abs(preds_mm - targets_mm).mean()

            batch_size = targets.size(0)
            total_loss += loss.item() * batch_size
            total_mae += mae.item() * batch_size
            total_samples += batch_size

    avg_loss = total_loss / total_samples
    avg_mae = total_mae / total_samples
    rmse = (avg_loss ** 0.5) * (30.0 - 12.5)  # Denormalize RMSE

    return avg_loss, rmse, avg_mae


def main():
    args = parse_args()

    # Select config based on mode
    config = TRAIN_JOINT_IMAGE_CONFIG if args.use_image else TRAIN_JOINT_CONFIG

    # Override with CLI args
    epochs = args.epochs or config["epochs"]
    batch_size = args.batch_size or config["batch_size"]
    lr = args.lr or config["learning_rate"]
    weight_decay = config["weight_decay"]
    patience = config["patience"]

    # Device
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Checkpoint directory
    checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir else CHECKPOINT_DIR
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    # Dataset
    print(f"\nLoading data (val_object={args.val_object})...")
    train_dataset = JointDataset(
        root_dir=NON_DEFORMABLE_DIR,
        val_object=args.val_object,
        split="train",
        use_image=args.use_image,
        augment=True,
    )
    val_dataset = JointDataset(
        root_dir=NON_DEFORMABLE_DIR,
        val_object=args.val_object,
        split="val",
        use_image=args.use_image,
        augment=False,
    )

    print(f"Train samples: {len(train_dataset)}")
    print(f"Val samples: {len(val_dataset)}")
    print(f"Train distribution: {train_dataset.get_stats()}")

    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=8,
        pin_memory=True,
        persistent_workers=True,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=4,
        pin_memory=True,
    )

    # Model
    model = JointRadiusModel(use_image=args.use_image)
    model = model.to(device)
    print(f"\nModel parameters: {count_parameters(model):,}")

    # Loss and optimizer
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=lr,
        weight_decay=weight_decay,
    )
    scheduler = CosineAnnealingLR(optimizer, T_max=epochs)

    # Training loop
    best_rmse = float("inf")
    patience_counter = 0
    mode_str = "image_joint" if args.use_image else "joint"

    print(f"\nStarting training ({mode_str} mode)...")
    print(f"Epochs: {epochs}, Batch size: {batch_size}, LR: {lr}")

    for epoch in range(1, epochs + 1):
        # Freeze/unfreeze backbone
        if args.use_image:
            freeze_epochs = config.get("freeze_backbone_epochs", 5)
            if epoch <= freeze_epochs:
                model.freeze_backbone()
            else:
                model.unfreeze_backbone()

        # Train
        train_loss = train_epoch(
            model, train_loader, criterion, optimizer, device, args.use_image
        )

        # Validate
        val_loss, val_rmse, val_mae = validate(
            model, val_loader, criterion, device, args.use_image
        )

        scheduler.step()

        # Logging
        print(
            f"Epoch {epoch:3d}/{epochs} | "
            f"Train Loss: {train_loss:.4f} | "
            f"Val Loss: {val_loss:.4f} | "
            f"Val RMSE: {val_rmse:.2f}mm | "
            f"Val MAE: {val_mae:.2f}mm"
        )

        # Save best model
        if val_rmse < best_rmse:
            best_rmse = val_rmse
            patience_counter = 0

            checkpoint_path = checkpoint_dir / f"best_{mode_str}_{args.val_object}.pt"
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_rmse": val_rmse,
                "val_mae": val_mae,
                "config": {
                    "use_image": args.use_image,
                    "val_object": args.val_object,
                },
            }, checkpoint_path)
            print(f"  -> Saved best model (RMSE: {val_rmse:.2f}mm)")
        else:
            patience_counter += 1
            if patience_counter >= patience:
                print(f"\nEarly stopping at epoch {epoch}")
                break

    print(f"\nTraining complete! Best Val RMSE: {best_rmse:.2f}mm")


if __name__ == "__main__":
    main()
