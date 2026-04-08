#!/usr/bin/env python3
"""Evaluate model on both non-deformable and deformable datasets, save to CSV."""

import argparse
import csv
import json
import re
from pathlib import Path
from collections import defaultdict

import numpy as np
import torch
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm

from radius_prediction.data.pointcloud_dataset import load_pcd_binary, normalize_point_cloud, farthest_point_sample
from radius_prediction.data.pointcloud_dataset_normals import compute_normals_knn_pca
from radius_prediction.models.pointnetpp import TactilePointNetPPRegressor
from radius_prediction.config import OBJECT_TO_RADIUS, denormalize_radius, CHECKPOINT_DIR, TRAINING_DATA_DIR

# Paths
NON_DEFORMABLE_DIR = TRAINING_DATA_DIR / "non_deformable"
DEFORMABLE_DIR = TRAINING_DATA_DIR / "deformable"
DEFAULT_CHECKPOINT = CHECKPOINT_DIR / "best_pointnetpp.pt"
DEFAULT_CSV = Path(__file__).parent / "evaluation_results.csv"
DEFAULT_SPLIT = Path(__file__).parent / "data" / "splits" / "default_split.json"


class EvalDataset(Dataset):
    """Dataset for evaluation on any directory structure."""

    def __init__(
        self,
        root_dir: Path,
        dataset_type: str,  # "non_deformable" or "deformable"
        exclude_bags: list = None,
        allowed_bags: list = None,  # If provided, only include these bag names
        num_points: int = 1024,
        use_intensity: bool = True,
        use_normals: bool = True,
        k_neighbors: int = 30,
    ):
        self.root_dir = Path(root_dir)
        self.dataset_type = dataset_type
        self.exclude_bags = exclude_bags or [1, 2]
        self.allowed_bags = set(allowed_bags) if allowed_bags else None
        self.num_points = num_points
        self.use_intensity = use_intensity
        self.use_normals = use_normals
        self.k_neighbors = k_neighbors

        self.samples = []
        self._scan_samples()

    def _extract_info(self, record_name: str):
        """Extract object name and bag number from record name."""
        name = record_name.replace("record_", "")

        # Handle _empty suffix for deformable
        is_empty = "_empty" in name
        name = name.replace("_empty", "")

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
        """Scan directory for samples."""
        if not self.root_dir.exists():
            print(f"Warning: Directory not found: {self.root_dir}")
            return

        for record_dir in sorted(self.root_dir.iterdir()):
            if not record_dir.is_dir():
                continue

            record_name = record_dir.name
            obj_name, bag_num = self._extract_info(record_name)

            if obj_name is None:
                continue

            # If allowed_bags specified, only include those
            if self.allowed_bags is not None:
                if record_name not in self.allowed_bags:
                    continue
            else:
                # Otherwise use exclude_bags
                if bag_num in self.exclude_bags:
                    continue

            radius = OBJECT_TO_RADIUS[obj_name]

            for sample_dir in sorted(record_dir.iterdir()):
                if not sample_dir.is_dir():
                    continue

                self.samples.append({
                    "path": sample_dir,
                    "record_name": record_name,
                    "object_name": obj_name,
                    "bag_num": bag_num,
                    "radius": radius,
                    "dataset_type": self.dataset_type,
                })

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample_info = self.samples[idx]
        sample_path = sample_info["path"]

        # Load point cloud
        pcd_path = sample_path / "tactile_pointcloud.pcd"
        points = load_pcd_binary(pcd_path, use_intensity=self.use_intensity)

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

        return points, idx  # Return index to look up sample info


def run_inference(model, dataset, device, batch_size=32):
    """Run inference and return list of results."""
    model.eval()

    dataloader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=4,
    )

    results = []

    with torch.no_grad():
        for points, indices in tqdm(dataloader, desc=f"Inference ({dataset.dataset_type})"):
            points = points.to(device)
            outputs = model(points)
            preds_mm = denormalize_radius(outputs.squeeze().cpu()).numpy()

            if preds_mm.ndim == 0:
                preds_mm = [float(preds_mm)]

            for i, idx in enumerate(indices):
                idx = int(idx)
                sample_info = dataset.samples[idx]

                results.append({
                    "dataset_type": sample_info["dataset_type"],
                    "object_name": sample_info["object_name"],
                    "record_name": sample_info["record_name"],
                    "bag_num": sample_info["bag_num"],
                    "ground_truth_mm": sample_info["radius"],
                    "prediction_mm": float(preds_mm[i]),
                    "error_mm": float(preds_mm[i]) - sample_info["radius"],
                    "abs_error_mm": abs(float(preds_mm[i]) - sample_info["radius"]),
                })

    return results


def save_csv(results, output_path):
    """Save results to CSV file."""
    if not results:
        print("No results to save!")
        return

    fieldnames = list(results[0].keys())

    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print(f"Saved {len(results)} results to: {output_path}")


def load_csv(csv_path):
    """Load results from CSV file."""
    results = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric fields
            row["bag_num"] = int(row["bag_num"])
            row["ground_truth_mm"] = float(row["ground_truth_mm"])
            row["prediction_mm"] = float(row["prediction_mm"])
            row["error_mm"] = float(row["error_mm"])
            row["abs_error_mm"] = float(row["abs_error_mm"])
            results.append(row)
    return results


def plot_from_csv(csv_path, output_path=None):
    """Plot distribution from CSV file."""
    results = load_csv(csv_path)

    # Group by dataset_type and object_name
    groups = defaultdict(list)
    for r in results:
        key = (r["dataset_type"], r["object_name"])
        groups[key].append(r["prediction_mm"])

    # Get ground truths
    gt_map = {r["object_name"]: r["ground_truth_mm"] for r in results}

    # Sort: non-deformable first, then deformable, each sorted by radius
    sorted_keys = sorted(groups.keys(),
                         key=lambda x: (0 if x[0] == "non_deformable" else 1, gt_map[x[1]]))

    n_rows = len(sorted_keys)
    fig, axes = plt.subplots(n_rows, 1, figsize=(12, 1.5 * n_rows), sharex=True)
    fig.subplots_adjust(left=0.18)  # More space for horizontal labels

    if n_rows == 1:
        axes = [axes]

    # Colors
    colors = {
        "non_deformable": "#5DADE2",  # Light blue
        "deformable": "#85C1E9",      # Lighter blue
    }

    for ax, (dtype, obj_name) in zip(axes, sorted_keys):
        preds = groups[(dtype, obj_name)]
        gt = gt_map[obj_name]
        n_samples = len(preds)

        # Histogram
        ax.hist(preds, bins=30, range=(20, 50),
                color=colors.get(dtype, "#5DADE2"),
                alpha=0.7, edgecolor='white', linewidth=0.5)

        # Ground truth line
        ax.axvline(gt, color='red', linestyle='--', linewidth=2)

        # Y-axis label: class name on first line, n= on second line, horizontal
        label = f"{obj_name}" if dtype == "non_deformable" else f"{obj_name}_empty"
        ax.set_ylabel(f"{label}\nn={n_samples}", fontsize=9, rotation=0, ha='right', va='center')
        ax.set_xlim(20, 50)
        ax.tick_params(axis='y', labelsize=7)

        # Background color to distinguish types
        if dtype == "deformable":
            ax.set_facecolor('#f8f8f8')

        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    axes[-1].set_xlabel('Radius (mm)', fontsize=11)

    # Legend
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    legend_elements = [
        Patch(facecolor=colors["non_deformable"], alpha=0.7, label='non_deformable'),
        Patch(facecolor=colors["deformable"], alpha=0.7, label='deformable'),
        Line2D([0], [0], color='red', linestyle='--', linewidth=2, label='ground truth'),
    ]
    axes[0].legend(handles=legend_elements, loc='upper right', fontsize=8)

    fig.suptitle('Predicted Radius Distribution by Object Class', fontsize=13, fontweight='bold')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to: {output_path}")

    plt.show()


def generate_summary_table(csv_path, output_path=None):
    """Generate summary table with class, GT radius, mean, std, MAE."""
    results = load_csv(csv_path)

    # Group by dataset_type and object_name
    groups = defaultdict(list)
    for r in results:
        key = (r["dataset_type"], r["object_name"])
        groups[key].append(r)

    # Get ground truths
    gt_map = {r["object_name"]: r["ground_truth_mm"] for r in results}

    # Sort: non-deformable first, then deformable, each sorted by radius
    sorted_keys = sorted(groups.keys(),
                         key=lambda x: (0 if x[0] == "non_deformable" else 1, gt_map[x[1]]))

    # Build table rows
    rows = []
    for dtype, obj_name in sorted_keys:
        group = groups[(dtype, obj_name)]
        preds = [r["prediction_mm"] for r in group]
        gt = gt_map[obj_name]

        label = obj_name if dtype == "non_deformable" else f"{obj_name}_empty"
        mean_pred = np.mean(preds)
        std_pred = np.std(preds)
        mae = np.mean(np.abs(np.array(preds) - gt))

        rows.append({
            "class": label,
            "type": dtype,
            "n": len(preds),
            "gt_mm": gt,
            "mean_mm": mean_pred,
            "std_mm": std_pred,
            "mae_mm": mae,
        })

    # Print table
    print("\n" + "=" * 80)
    print("SUMMARY TABLE")
    print("=" * 80)
    print(f"{'Class':<20} {'Type':<15} {'n':>6} {'GT':>8} {'Mean':>8} {'Std':>8} {'MAE':>8}")
    print("-" * 80)

    for row in rows:
        print(f"{row['class']:<20} {row['type']:<15} {row['n']:>6} "
              f"{row['gt_mm']:>8.1f} {row['mean_mm']:>8.1f} {row['std_mm']:>8.1f} {row['mae_mm']:>8.1f}")

    # Overall stats per type
    print("-" * 80)
    for dtype in ["non_deformable", "deformable"]:
        dtype_rows = [r for r in rows if r["type"] == dtype]
        if dtype_rows:
            total_n = sum(r["n"] for r in dtype_rows)
            # Weighted average MAE
            weighted_mae = sum(r["mae_mm"] * r["n"] for r in dtype_rows) / total_n
            print(f"{'OVERALL ' + dtype:<35} {total_n:>6} {'-':>8} {'-':>8} {'-':>8} {weighted_mae:>8.1f}")

    # Save to CSV if path provided
    if output_path:
        fieldnames = ["class", "type", "n", "gt_mm", "mean_mm", "std_mm", "mae_mm"]
        with open(output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nSaved summary table to: {output_path}")

    return rows


def print_summary(results):
    """Print summary statistics."""
    # Group by dataset_type and object_name
    groups = defaultdict(list)
    for r in results:
        groups[(r["dataset_type"], r["object_name"])].append(r)

    print("\n" + "="*80)
    print("EVALUATION SUMMARY")
    print("="*80)

    for dtype in ["non_deformable", "deformable"]:
        print(f"\n{dtype.upper()}:")
        print("-" * 60)

        dtype_results = [r for r in results if r["dataset_type"] == dtype]
        if not dtype_results:
            print("  No samples")
            continue

        # Per-class stats
        for obj_name in sorted(set(r["object_name"] for r in dtype_results),
                               key=lambda x: OBJECT_TO_RADIUS[x]):
            obj_results = [r for r in dtype_results if r["object_name"] == obj_name]
            preds = [r["prediction_mm"] for r in obj_results]
            gt = obj_results[0]["ground_truth_mm"]
            mae = np.mean([r["abs_error_mm"] for r in obj_results])

            print(f"  {obj_name:15s}: GT={gt:5.1f}mm, Mean={np.mean(preds):5.1f}mm, "
                  f"Std={np.std(preds):4.1f}mm, MAE={mae:5.1f}mm, n={len(preds)}")

        # Overall stats
        overall_mae = np.mean([r["abs_error_mm"] for r in dtype_results])
        overall_rmse = np.sqrt(np.mean([r["error_mm"]**2 for r in dtype_results]))
        print(f"  {'OVERALL':15s}: MAE={overall_mae:.2f}mm, RMSE={overall_rmse:.2f}mm, n={len(dtype_results)}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate on all datasets")
    parser.add_argument("--checkpoint", type=str, default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--output_csv", type=str, default=str(DEFAULT_CSV))
    parser.add_argument("--output_plot", type=str, default="evaluation_plot.png")
    parser.add_argument("--summary_csv", type=str, default=None,
                        help="Output path for summary table CSV")
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--plot_only", action="store_true",
                        help="Only plot from existing CSV, skip inference")
    parser.add_argument("--table_only", action="store_true",
                        help="Only generate summary table from existing CSV")
    args = parser.parse_args()

    output_csv = Path(args.output_csv)
    output_plot = Path(args.output_plot)

    # Plot-only mode
    if args.plot_only:
        if not output_csv.exists():
            print(f"CSV not found: {output_csv}")
            return
        generate_summary_table(output_csv, args.summary_csv)
        plot_from_csv(output_csv, output_plot)
        return

    # Table-only mode
    if args.table_only:
        if not output_csv.exists():
            print(f"CSV not found: {output_csv}")
            return
        generate_summary_table(output_csv, args.summary_csv)
        return

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
        dropout=0.0,
        sa_configs=config["sa_configs"],
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    all_results = []

    # Load split file to get validation bags
    with open(DEFAULT_SPLIT, 'r') as f:
        split_data = json.load(f)
    val_bags = split_data["val"]
    print(f"Using {len(val_bags)} validation bags from split file")

    # Non-deformable dataset (validation only)
    print("\nLoading non-deformable dataset (validation only)...")
    non_def_dataset = EvalDataset(
        root_dir=NON_DEFORMABLE_DIR,
        dataset_type="non_deformable",
        allowed_bags=val_bags,  # Only validation bags
        num_points=config["num_points"],
        use_intensity=config["use_intensity"],
        use_normals=config["use_normals"],
        k_neighbors=config["k_neighbors"],
    )
    print(f"  Samples: {len(non_def_dataset)}")

    if len(non_def_dataset) > 0:
        results = run_inference(model, non_def_dataset, device)
        all_results.extend(results)

    # Deformable dataset
    print("\nLoading deformable dataset...")
    def_dataset = EvalDataset(
        root_dir=DEFORMABLE_DIR,
        dataset_type="deformable",
        exclude_bags=[1, 2],
        num_points=config["num_points"],
        use_intensity=config["use_intensity"],
        use_normals=config["use_normals"],
        k_neighbors=config["k_neighbors"],
    )
    print(f"  Samples: {len(def_dataset)}")

    if len(def_dataset) > 0:
        results = run_inference(model, def_dataset, device)
        all_results.extend(results)

    # Save CSV
    save_csv(all_results, output_csv)

    # Generate summary table
    generate_summary_table(output_csv, args.summary_csv)

    # Plot
    plot_from_csv(output_csv, output_plot)


if __name__ == "__main__":
    main()
