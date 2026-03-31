#!/usr/bin/env python3
"""Pre-cache point clouds to avoid runtime processing bottleneck."""

import pickle
from pathlib import Path
from tqdm import tqdm

from radius_prediction.data.pointcloud_dataset import (
    TactilePointCloudDataset,
    load_pcd_binary,
    farthest_point_sample,
    normalize_point_cloud,
)
from radius_prediction.config import NON_DEFORMABLE_DIR


def cache_dataset(num_points=1024, cache_dir="pointcloud_cache"):
    """Pre-process and cache all point clouds."""

    cache_path = Path(cache_dir)
    cache_path.mkdir(exist_ok=True)

    print(f"Caching point clouds ({num_points} points)...")

    # Create dataset without loading data
    dataset = TactilePointCloudDataset(
        root_dir=NON_DEFORMABLE_DIR,
        val_object="mid_bottle",
        split="train",
        num_points=num_points,
        use_intensity=True,
        augment=False,  # No augmentation for cache
    )

    # Add validation set
    val_dataset = TactilePointCloudDataset(
        root_dir=NON_DEFORMABLE_DIR,
        val_object="mid_bottle",
        split="val",
        num_points=num_points,
        use_intensity=True,
        augment=False,
    )

    all_samples = dataset.samples + val_dataset.samples

    cached_data = {}

    for sample_info in tqdm(all_samples, desc="Processing"):
        sample_path = sample_info["path"]
        cache_key = str(sample_path.relative_to(NON_DEFORMABLE_DIR))

        # Load and process
        pcd_path = sample_path / "tactile_pointcloud.pcd"
        points = load_pcd_binary(pcd_path, use_intensity=True)
        points = farthest_point_sample(points, num_points)
        points = normalize_point_cloud(points)

        cached_data[cache_key] = {
            "points": points,
            "radius": sample_info["radius"],
            "object_name": sample_info["object_name"],
        }

    # Save cache
    cache_file = cache_path / f"pointclouds_{num_points}.pkl"
    with open(cache_file, "wb") as f:
        pickle.dump(cached_data, f, protocol=4)

    print(f"Cached {len(cached_data)} samples to {cache_file}")
    print(f"Cache size: {cache_file.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_points", type=int, default=1024)
    args = parser.parse_args()

    cache_dataset(num_points=args.num_points)
