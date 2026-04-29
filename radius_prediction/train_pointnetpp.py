#!/usr/bin/env python3
"""PointNet++ training with bag-level split and surface normals."""

import argparse
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torch.optim.lr_scheduler import CosineAnnealingLR
from tqdm import tqdm

from radius_prediction.data.bag_dataset import BagSplitDataset
from radius_prediction.models.pointnetpp import TactilePointNetPPRegressor, count_parameters
from radius_prediction.config import (
    CHECKPOINT_DIR,
    TRAIN_POINTNETPP_CONFIG,
    POINTNETPP_CONFIG,
    denormalize_radius,
)

# Default split file
DEFAULT_SPLIT = Path(__file__).parent / "data" / "splits" / "default_split.json"


def parse_args():
    parser = argparse.ArgumentParser(description="Train PointNet++ radius model")
    parser.add_argument(
        "--split_file", type=str, default=str(DEFAULT_SPLIT),
        help="Path to split JSON file"
    )
    parser.add_argument(
        "--num_points", type=int, default=None,
        help="Number of points to sample (default: from config)"
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
        "--no_intensity", action="store_true",
        help="Don't use intensity channel"
    )
    parser.add_argument(
        "--no_normals", action="store_true",
        help="Don't use surface normals"
    )
    parser.add_argument(
        "--k_neighbors", type=int, default=None,
        help="Neighbors for normal computation (default: from config)"
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


def train_epoch(model, dataloader, criterion, optimizer, device):
    """Train for one epoch."""
    model.train()
    total_loss = 0.0
    total_samples = 0

    pbar = tqdm(dataloader, desc="Training", leave=False)
    for points, targets in pbar:
        points = points.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)

        outputs = model(points)
        loss = criterion(outputs.squeeze(), targets)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        batch_size = targets.size(0)
        total_loss += loss.item() * batch_size
        total_samples += batch_size

        pbar.set_postfix({"loss": f"{loss.item():.4f}"})

    return total_loss / total_samples


def validate(model, dataloader, criterion, device):
    """Validate model."""
    model.eval()
    total_loss = 0.0
    total_mae = 0.0
    total_samples = 0

    with torch.no_grad():
        for points, targets in dataloader:
            points = points.to(device, non_blocking=True)
            targets = targets.to(device, non_blocking=True)

            outputs = model(points)
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
    rmse = (avg_loss ** 0.5) * (46.0 - 24.0)  # Denormalize RMSE

    return avg_loss, rmse, avg_mae


def main():
    args = parse_args()

    # Config
    config = TRAIN_POINTNETPP_CONFIG
    pp_config = POINTNETPP_CONFIG

    # Override with CLI args
    epochs = args.epochs or config["epochs"]
    batch_size = args.batch_size or config["batch_size"]
    lr = args.lr or config["learning_rate"]
    weight_decay = config["weight_decay"]
    patience = config["patience"]
    num_points = args.num_points or pp_config["num_points"]
    use_intensity = not args.no_intensity
    use_normals = not args.no_normals
    k_neighbors = args.k_neighbors or pp_config["normal_k_neighbors"]

    # Compute input channels
    input_channels = 3  # xyz
    if use_intensity:
        input_channels += 1
    if use_normals:
        input_channels += 3

    # Device
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Checkpoint directory
    checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir else CHECKPOINT_DIR
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    # Dataset with bag-level split
    split_file = Path(args.split_file)
    print(f"\nLoading data from split: {split_file}")
    print(f"Using normals: {use_normals} (k={k_neighbors})")

    train_dataset = BagSplitDataset(
        split_file=split_file,
        split="train",
        num_points=num_points,
        use_intensity=use_intensity,
        use_normals=use_normals,
        k_neighbors=k_neighbors,
        augment=True,
    )
    val_dataset = BagSplitDataset(
        split_file=split_file,
        split="val",
        num_points=num_points,
        use_intensity=use_intensity,
        use_normals=use_normals,
        k_neighbors=k_neighbors,
        augment=False,
    )

    print(f"Train samples: {len(train_dataset)} ({len(train_dataset.bags)} bags)")
    print(f"Val samples: {len(val_dataset)} ({len(val_dataset.bags)} bags)")
    print(f"Train distribution: {train_dataset.get_stats()}")
    print(f"Points per sample: {num_points}")
    print(f"Input channels: {input_channels}")
    channels_desc = ["xyz"]
    if use_intensity:
        channels_desc.append("intensity")
    if use_normals:
        channels_desc.append("normals")
    print(f"  Features: {' + '.join(channels_desc)}")

    # DataLoader
    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,
        pin_memory=True,
        persistent_workers=True,
        prefetch_factor=2,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=2,
        pin_memory=True,
    )

    # Model
    model = TactilePointNetPPRegressor(
        input_channels=input_channels,
        dropout=pp_config["dropout"],
        sa_configs=pp_config["sa_configs"],
    )
    model = model.to(device)
    print(f"\nModel parameters: {count_parameters(model):,}")

    # Print SA layer info
    print("Set Abstraction layers:")
    for i, cfg in enumerate(pp_config["sa_configs"]):
        print(f"  SA{i+1}: {cfg['npoint']} pts, r={cfg['radius']}, k={cfg['nsample']}, mlp={cfg['mlp']}")

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

    print(f"\nStarting training (PointNet++ with bag split)...")
    print(f"Epochs: {epochs}, Batch size: {batch_size}, LR: {lr}")

    for epoch in range(1, epochs + 1):
        # Train
        train_loss = train_epoch(model, train_loader, criterion, optimizer, device)

        # Validate
        val_loss, val_rmse, val_mae = validate(model, val_loader, criterion, device)

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

            checkpoint_path = checkpoint_dir / "best_pointnetpp.pt"
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_rmse": val_rmse,
                "val_mae": val_mae,
                "config": {
                    "num_points": num_points,
                    "input_channels": input_channels,
                    "use_intensity": use_intensity,
                    "use_normals": use_normals,
                    "k_neighbors": k_neighbors,
                    "split_file": str(split_file),
                    "sa_configs": pp_config["sa_configs"],
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
