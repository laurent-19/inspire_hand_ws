#!/usr/bin/env python3
"""Evaluate PointNet++ model on the balls dataset."""

import argparse
import csv
import re
from pathlib import Path
from collections import defaultdict

import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm

from radius_prediction.data.pointcloud_dataset import load_pcd_binary, normalize_point_cloud, farthest_point_sample
from radius_prediction.data.pointcloud_dataset_normals import compute_normals_knn_pca
from radius_prediction.models.pointnetpp import TactilePointNetPPRegressor
from radius_prediction.config import denormalize_radius, CHECKPOINT_DIR

# Paths
ROOT_DIR = Path(__file__).parent.parent
BALLS_DATA_DIR = ROOT_DIR / "training_data_balls"
DEFAULT_CHECKPOINT = CHECKPOINT_DIR / "best_pointnetpp.pt"

# Ball radii (from training_data/README.md)
BALL_TO_RADIUS = {
    "tenis_ball": 32.0,
    "orange_ball": 29.4,
    "white_ball": 38.5,
}


class BallsEvalDataset(Dataset):
    """Dataset for evaluation on balls dataset."""

    def __init__(
        self,
        root_dir: Path,
        dataset_type: str,
        exclude_bags: list = None,
        num_points: int = 1024,
        use_intensity: bool = True,
        use_normals: bool = True,
        k_neighbors: int = 30,
    ):
        self.root_dir = Path(root_dir)
        self.dataset_type = dataset_type
        self.exclude_bags = exclude_bags or [1, 2]
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

        if obj_name not in BALL_TO_RADIUS:
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

            if bag_num in self.exclude_bags:
                continue

            radius = BALL_TO_RADIUS[obj_name]

            for sample_dir in sorted(record_dir.iterdir()):
                if not sample_dir.is_dir():
                    continue

                # Check if tactile_pointcloud.pcd exists
                pcd_path = sample_dir / "tactile_pointcloud.pcd"
                if not pcd_path.exists():
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

        pcd_path = sample_path / "tactile_pointcloud.pcd"
        points = load_pcd_binary(pcd_path, use_intensity=self.use_intensity)

        # Skip if too few points
        if len(points) < 10:
            # Return zeros - will be filtered later
            points = np.zeros((self.num_points, 7 if self.use_normals else 4), dtype=np.float32)
            return torch.tensor(points, dtype=torch.float32).T, idx

        points = farthest_point_sample(points, self.num_points)

        if self.use_normals:
            xyz = points[:, :3]
            normals = compute_normals_knn_pca(xyz, self.k_neighbors)
            if self.use_intensity:
                points = np.hstack([points, normals])
            else:
                points = np.hstack([points[:, :3], normals])

        points = normalize_point_cloud(points)
        points = torch.tensor(points, dtype=torch.float32).T

        return points, idx


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


def print_summary(results):
    """Print summary statistics."""
    groups = defaultdict(list)
    for r in results:
        groups[(r["dataset_type"], r["object_name"])].append(r)

    print("\n" + "=" * 80)
    print("BALLS DATASET EVALUATION")
    print("=" * 80)

    for dtype in ["non_deformable", "deformable"]:
        print(f"\n{dtype.upper()}:")
        print("-" * 60)

        dtype_results = [r for r in results if r["dataset_type"] == dtype]
        if not dtype_results:
            print("  No samples")
            continue

        for obj_name in sorted(set(r["object_name"] for r in dtype_results),
                               key=lambda x: BALL_TO_RADIUS[x]):
            obj_results = [r for r in dtype_results if r["object_name"] == obj_name]
            preds = [r["prediction_mm"] for r in obj_results]
            gt = obj_results[0]["ground_truth_mm"]
            mae = np.mean([r["abs_error_mm"] for r in obj_results])

            print(f"  {obj_name:15s}: GT={gt:5.1f}mm, Mean={np.mean(preds):5.1f}mm, "
                  f"Std={np.std(preds):4.1f}mm, MAE={mae:5.1f}mm, n={len(preds)}")

        overall_mae = np.mean([r["abs_error_mm"] for r in dtype_results])
        overall_rmse = np.sqrt(np.mean([r["error_mm"]**2 for r in dtype_results]))
        print(f"  {'OVERALL':15s}: MAE={overall_mae:.2f}mm, RMSE={overall_rmse:.2f}mm, n={len(dtype_results)}")


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

    print(f"\nSaved {len(results)} results to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate on balls dataset")
    parser.add_argument("--checkpoint", type=str, default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--output_csv", type=str, default="evaluation_balls.csv")
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
        dropout=0.0,
        sa_configs=config["sa_configs"],
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model = model.to(device)
    model.eval()

    all_results = []

    # Non-deformable (tenis_ball, white_ball)
    print("\nLoading non-deformable balls...")
    non_def_dataset = BallsEvalDataset(
        root_dir=BALLS_DATA_DIR / "non_deformable",
        dataset_type="non_deformable",
        exclude_bags=[1, 2],
        num_points=config["num_points"],
        use_intensity=config["use_intensity"],
        use_normals=config["use_normals"],
        k_neighbors=config["k_neighbors"],
    )
    print(f"  Samples: {len(non_def_dataset)}")

    if len(non_def_dataset) > 0:
        results = run_inference(model, non_def_dataset, device)
        all_results.extend(results)

    # Deformable (orange_ball_empty, tenis_ball_empty)
    print("\nLoading deformable balls...")
    def_dataset = BallsEvalDataset(
        root_dir=BALLS_DATA_DIR / "deformable",
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

    # Print summary
    print_summary(all_results)

    # Save CSV
    if args.output_csv:
        save_csv(all_results, Path(__file__).parent / args.output_csv)


if __name__ == "__main__":
    main()
