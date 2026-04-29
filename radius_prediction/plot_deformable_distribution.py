#!/usr/bin/env python3
"""Plot predicted radius distribution on deformable objects (not used in training)."""

import argparse
import re
from pathlib import Path
from collections import defaultdict

import numpy as np
import torch
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm

from radius_prediction.data.pointcloud_dataset import load_pcd_binary, normalize_point_cloud, farthest_point_sample
from radius_prediction.data.pointcloud_dataset_normals import compute_normals_knn_pca
from radius_prediction.models.pointnetpp import TactilePointNetPPRegressor
from radius_prediction.config import OBJECT_TO_RADIUS, denormalize_radius, CHECKPOINT_DIR, TRAINING_DATA_DIR

# Default paths
DEFORMABLE_DIR = TRAINING_DATA_DIR / "deformable"
DEFAULT_CHECKPOINT = CHECKPOINT_DIR / "best_pointnetpp.pt"


class DeformableDataset(Dataset):
    """Dataset for deformable objects (empty bottles) - evaluation only."""

    def __init__(
        self,
        root_dir: Path = None,
        exclude_bags: list = None,
        num_points: int = 1024,
        use_intensity: bool = True,
        use_normals: bool = True,
        k_neighbors: int = 30,
    ):
        self.root_dir = Path(root_dir) if root_dir else DEFORMABLE_DIR
        self.exclude_bags = exclude_bags or [1, 2]
        self.num_points = num_points
        self.use_intensity = use_intensity
        self.use_normals = use_normals
        self.k_neighbors = k_neighbors

        self.samples = []
        self._scan_samples()

    def _extract_object_and_bag(self, record_name: str):
        """Extract object name and bag number from deformable record name."""
        # Remove 'record_' prefix and '_empty' suffix
        name = record_name.replace("record_", "").replace("_empty", "")

        # Extract trailing number
        match = re.match(r"(.+)_(\d+)$", name)
        if not match:
            return None, None

        obj_name = match.group(1)
        bag_num = int(match.group(2))

        if obj_name not in OBJECT_TO_RADIUS:
            return None, None

        return obj_name, bag_num

    def _scan_samples(self):
        """Scan deformable directory for samples."""
        if not self.root_dir.exists():
            print(f"Warning: Deformable directory not found: {self.root_dir}")
            return

        for record_dir in sorted(self.root_dir.iterdir()):
            if not record_dir.is_dir():
                continue

            record_name = record_dir.name
            obj_name, bag_num = self._extract_object_and_bag(record_name)

            if obj_name is None:
                continue

            # Skip excluded bags
            if bag_num in self.exclude_bags:
                continue

            radius = OBJECT_TO_RADIUS[obj_name]

            # Iterate over sample directories
            for sample_dir in sorted(record_dir.iterdir()):
                if not sample_dir.is_dir():
                    continue

                self.samples.append({
                    "path": sample_dir,
                    "record_name": record_name,
                    "object_name": obj_name,
                    "radius": radius,
                })

    def _load_pointcloud(self, sample_path: Path) -> np.ndarray:
        """Load tactile point cloud from PCD file."""
        pcd_path = sample_path / "tactile_pointcloud.pcd"
        return load_pcd_binary(pcd_path, use_intensity=self.use_intensity)

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load point cloud
        points = self._load_pointcloud(sample_path)

        # FPS
        points = farthest_point_sample(points, self.num_points)

        # Compute normals
        if self.use_normals:
            xyz = points[:, :3]
            normals = compute_normals_knn_pca(xyz, self.k_neighbors)
            if self.use_intensity:
                points = np.hstack([points, normals])
            else:
                points = np.hstack([points[:, :3], normals])

        # Normalize
        points = normalize_point_cloud(points)

        # Convert to tensor (C, N)
        points = torch.tensor(points, dtype=torch.float32).T

        return points, sample_info["object_name"], sample_info["radius"]


def run_inference(model, dataloader, device):
    """Run inference and collect predictions per class."""
    model.eval()

    predictions_by_class = defaultdict(list)
    gt_by_class = {}

    with torch.no_grad():
        for points, obj_names, radii in tqdm(dataloader, desc="Inference"):
            points = points.to(device)
            outputs = model(points)

            preds_mm = denormalize_radius(outputs.squeeze().cpu()).numpy()

            for i, (obj_name, gt_radius) in enumerate(zip(obj_names, radii)):
                if preds_mm.ndim == 0:
                    pred = float(preds_mm)
                else:
                    pred = float(preds_mm[i])

                predictions_by_class[obj_name].append(pred)
                gt_by_class[obj_name] = float(gt_radius)

    return predictions_by_class, gt_by_class


def plot_distribution(predictions_by_class, gt_by_class, output_path=None):
    """Plot radius distribution per class with ground truth line."""
    # ICRA-style rcParams
    plt.rcParams.update({
        "font.size": 14,
        "axes.titlesize": 14,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
    })

    # Sort classes by ground truth radius
    classes = sorted(predictions_by_class.keys(),
                     key=lambda x: gt_by_class[x])

    n_classes = len(classes)
    fig, axes = plt.subplots(n_classes, 1, figsize=(10, 1.8 * n_classes),
                              sharex=True)

    if n_classes == 1:
        axes = [axes]

    pred_color = '#ff7f0e'  # Orange for deformable

    for ax, obj_name in zip(axes, classes):
        preds = predictions_by_class[obj_name]
        gt_radius = gt_by_class[obj_name]
        n_samples = len(preds)

        # Plot histogram
        ax.hist(preds, bins=30, range=(20, 50), color=pred_color,
                alpha=0.7, edgecolor='white', linewidth=0.5)

        # Ground truth line
        ax.axvline(gt_radius, color='red', linestyle='--', linewidth=2,
                   label='Ground Truth')

        # Labels - add _empty suffix for clarity
        ax.set_ylabel(f'{obj_name}_empty\nn={n_samples}', rotation=0, ha='right', va='center')
        ax.set_xlim(20, 50)

        # Y-axis formatter
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.0f'))

        # Grid
        ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4)

        # Clean spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    axes[-1].set_xlabel('Predicted Radius (mm)')

    fig.suptitle('Predicted Radius Distribution - Deformable Objects\n(Model trained on non-deformable only)',
                 fontweight='bold', y=0.98)

    axes[0].legend(loc='upper right')

    plt.subplots_adjust(left=0.18, right=0.95, top=0.90, bottom=0.06, hspace=0.4)

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot to: {output_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Evaluate on deformable objects")
    parser.add_argument("--checkpoint", type=str, default=str(DEFAULT_CHECKPOINT),
                        help="Path to model checkpoint")
    parser.add_argument("--output", type=str, default="deformable_distribution.png",
                        help="Output plot path")
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Load checkpoint
    checkpoint = torch.load(args.checkpoint, map_location=device)
    config = checkpoint["config"]
    print(f"Loaded checkpoint from epoch {checkpoint['epoch']}")
    print(f"  Val RMSE: {checkpoint['val_rmse']:.2f}mm (on non-deformable)")

    # Create model
    model = TactilePointNetPPRegressor(
        input_channels=config["input_channels"],
        dropout=0.0,
        sa_configs=config["sa_configs"],
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    # Create deformable dataset
    dataset = DeformableDataset(
        num_points=config["num_points"],
        use_intensity=config["use_intensity"],
        use_normals=config["use_normals"],
        k_neighbors=config["k_neighbors"],
        exclude_bags=[1, 2],
    )
    print(f"Deformable samples: {len(dataset)} (excluding _1, _2 bags)")

    if len(dataset) == 0:
        print("No deformable samples found!")
        return

    dataloader = DataLoader(
        dataset,
        batch_size=32,
        shuffle=False,
        num_workers=4,
    )

    # Run inference
    predictions_by_class, gt_by_class = run_inference(model, dataloader, device)

    # Print stats
    print("\nPer-class statistics (deformable):")
    for obj_name in sorted(predictions_by_class.keys(),
                           key=lambda x: gt_by_class[x]):
        preds = predictions_by_class[obj_name]
        gt = gt_by_class[obj_name]
        mean_pred = np.mean(preds)
        std_pred = np.std(preds)
        mae = np.mean(np.abs(np.array(preds) - gt))
        print(f"  {obj_name}_empty: GT={gt:.1f}mm, Mean={mean_pred:.1f}mm, "
              f"Std={std_pred:.1f}mm, MAE={mae:.1f}mm, n={len(preds)}")

    # Plot
    plot_distribution(predictions_by_class, gt_by_class, args.output)


if __name__ == "__main__":
    main()
