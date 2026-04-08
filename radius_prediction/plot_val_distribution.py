#!/usr/bin/env python3
"""Plot predicted radius distribution per class on validation set."""

import argparse
from pathlib import Path
from collections import defaultdict

import numpy as np
import torch
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from tqdm import tqdm

from radius_prediction.data.bag_dataset import BagSplitDataset
from radius_prediction.models.pointnetpp import TactilePointNetPPRegressor
from radius_prediction.config import OBJECT_TO_RADIUS, denormalize_radius, CHECKPOINT_DIR

# Default paths
DEFAULT_SPLIT = Path(__file__).parent / "data" / "splits" / "default_split.json"
DEFAULT_CHECKPOINT = CHECKPOINT_DIR / "best_pointnetpp.pt"


def run_inference(model, dataloader, device):
    """Run inference on dataset and collect predictions per class."""
    model.eval()

    predictions_by_class = defaultdict(list)

    with torch.no_grad():
        for batch_idx, (points, targets) in enumerate(tqdm(dataloader, desc="Inference")):
            points = points.to(device)
            outputs = model(points)

            # Denormalize predictions
            preds_mm = denormalize_radius(outputs.squeeze().cpu()).numpy()

            # Get class info from dataset
            batch_size = points.size(0)
            start_idx = batch_idx * dataloader.batch_size

            for i in range(batch_size):
                sample_idx = start_idx + i
                if sample_idx >= len(dataloader.dataset):
                    break
                sample_info = dataloader.dataset.samples[sample_idx]
                obj_name = sample_info["object_name"]

                if preds_mm.ndim == 0:
                    pred = float(preds_mm)
                else:
                    pred = float(preds_mm[i])

                predictions_by_class[obj_name].append(pred)

    return predictions_by_class


def plot_distribution(predictions_by_class, output_path=None):
    """Plot radius distribution per class with ground truth line."""

    # Sort classes by ground truth radius
    classes = sorted(predictions_by_class.keys(),
                     key=lambda x: OBJECT_TO_RADIUS[x])

    n_classes = len(classes)
    fig, axes = plt.subplots(n_classes, 1, figsize=(10, 2 * n_classes),
                              sharex=True)

    if n_classes == 1:
        axes = [axes]

    # Color for predictions
    pred_color = '#5DADE2'  # Light blue

    for ax, obj_name in zip(axes, classes):
        preds = predictions_by_class[obj_name]
        gt_radius = OBJECT_TO_RADIUS[obj_name]
        n_samples = len(preds)

        # Plot histogram
        ax.hist(preds, bins=30, range=(20, 50), color=pred_color,
                alpha=0.7, edgecolor='white', linewidth=0.5)

        # Ground truth line
        ax.axvline(gt_radius, color='red', linestyle='--', linewidth=2,
                   label='ground truth')

        # Labels
        ax.set_ylabel(f'{obj_name}\n(n={n_samples})', fontsize=9)
        ax.set_xlim(20, 50)
        ax.tick_params(axis='y', labelsize=8)

        # Remove top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    # Common x-label
    axes[-1].set_xlabel('Predicted Radius (mm)', fontsize=11)

    # Title
    fig.suptitle('Predicted Radius Distribution per Class (Validation Set)',
                 fontsize=13, fontweight='bold')

    # Legend on first plot
    axes[0].legend(loc='upper right', fontsize=9)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to: {output_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot validation predictions")
    parser.add_argument("--checkpoint", type=str, default=str(DEFAULT_CHECKPOINT),
                        help="Path to model checkpoint")
    parser.add_argument("--split_file", type=str, default=str(DEFAULT_SPLIT),
                        help="Path to split JSON")
    parser.add_argument("--output", type=str, default="val_distribution.png",
                        help="Output plot path")
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Load checkpoint
    checkpoint = torch.load(args.checkpoint, map_location=device)
    config = checkpoint["config"]
    print(f"Loaded checkpoint from epoch {checkpoint['epoch']}")
    print(f"  Val RMSE: {checkpoint['val_rmse']:.2f}mm")

    # Create model
    model = TactilePointNetPPRegressor(
        input_channels=config["input_channels"],
        dropout=0.0,  # No dropout during inference
        sa_configs=config["sa_configs"],
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    # Create validation dataset
    val_dataset = BagSplitDataset(
        split_file=Path(args.split_file),
        split="val",
        num_points=config["num_points"],
        use_intensity=config["use_intensity"],
        use_normals=config["use_normals"],
        k_neighbors=config["k_neighbors"],
        augment=False,
    )
    print(f"Validation samples: {len(val_dataset)}")

    val_loader = DataLoader(
        val_dataset,
        batch_size=32,
        shuffle=False,
        num_workers=4,
    )

    # Run inference
    predictions_by_class = run_inference(model, val_loader, device)

    # Print stats
    print("\nPer-class statistics:")
    for obj_name in sorted(predictions_by_class.keys(),
                           key=lambda x: OBJECT_TO_RADIUS[x]):
        preds = predictions_by_class[obj_name]
        gt = OBJECT_TO_RADIUS[obj_name]
        mean_pred = np.mean(preds)
        std_pred = np.std(preds)
        mae = np.mean(np.abs(np.array(preds) - gt))
        print(f"  {obj_name}: GT={gt:.1f}mm, Mean={mean_pred:.1f}mm, "
              f"Std={std_pred:.1f}mm, MAE={mae:.1f}mm, n={len(preds)}")

    # Plot
    plot_distribution(predictions_by_class, args.output)


if __name__ == "__main__":
    main()
