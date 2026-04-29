#!/usr/bin/env python3
"""Compute STD and MAE statistics for RANSAC results.csv file."""

import csv
import re
from pathlib import Path
from collections import defaultdict

import numpy as np

from radius_prediction.config import OBJECT_TO_RADIUS

# Paths
RANSAC_CSV = Path(__file__).parent.parent / "results.csv"

# Class order for table
CLASS_ORDER = ["250", "330_slim", "330_fat", "500", "small_bottle", "mid_bottle", "big_bottle"]

# Display names
DISPLAY_NAMES = {
    "250": "250ml can",
    "330_slim": "330ml slim",
    "330_fat": "330ml can",
    "500": "500ml can",
    "small_bottle": "500ml bot.",
    "mid_bottle": "1L bot.",
    "big_bottle": "1.5L bot.",
}


def extract_object_name_and_bag(record_name):
    """Extract object name and bag number from record name like 'record_250_empty_1'."""
    name = record_name.replace("record_", "")
    name = name.replace("_empty", "")

    # Remove trailing number
    match = re.match(r"(.+)_(\d+)$", name)
    if match:
        return match.group(1), int(match.group(2))
    return name, None


def load_ransac_csv(csv_path, exclude_bags=(1, 2)):
    """Load RANSAC results from CSV, grouped by object and type."""
    results = defaultdict(lambda: defaultdict(list))

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            record_name = row["level_2"]
            obj_name, bag_num = extract_object_name_and_bag(record_name)

            if obj_name not in OBJECT_TO_RADIUS:
                continue

            # Skip excluded bags
            if bag_num in exclude_bags:
                continue

            dtype = row["level_1"]  # "deformable" or "non_deformable"
            radius = float(row["radius_mm"])
            results[obj_name][dtype].append(radius)

    return results


def compute_stats(predictions, ground_truth):
    """Compute MAE and STD."""
    preds = np.array(predictions)
    mae = np.mean(np.abs(preds - ground_truth))
    std = np.std(preds)
    return mae, std


def main():
    print("Loading RANSAC results from:", RANSAC_CSV)
    data = load_ransac_csv(RANSAC_CSV)

    print("\n" + "=" * 100)
    print("RANSAC STATISTICS FOR LATEX TABLE")
    print("=" * 100)

    # Print header
    header = "Method & Metric"
    for obj_name in CLASS_ORDER:
        header += f" & {DISPLAY_NAMES[obj_name]}"
    header += " & Overall"
    print(f"\n{header}")
    print("-" * 100)

    # Collect for overall calculation
    all_rigid_errors = []
    all_deform_errors = []

    # Store results for printing
    rigid_mae = {}
    rigid_std = {}
    deform_mae = {}
    deform_std = {}

    for obj_name in CLASS_ORDER:
        gt = OBJECT_TO_RADIUS[obj_name]

        # Rigid (non_deformable)
        rigid_preds = data[obj_name].get("non_deformable", [])
        if rigid_preds:
            mae, std = compute_stats(rigid_preds, gt)
            rigid_mae[obj_name] = mae
            rigid_std[obj_name] = std
            all_rigid_errors.extend([abs(p - gt) for p in rigid_preds])
        else:
            rigid_mae[obj_name] = None
            rigid_std[obj_name] = None

        # Deformable
        deform_preds = data[obj_name].get("deformable", [])
        if deform_preds:
            mae, std = compute_stats(deform_preds, gt)
            deform_mae[obj_name] = mae
            deform_std[obj_name] = std
            all_deform_errors.extend([abs(p - gt) for p in deform_preds])
        else:
            deform_mae[obj_name] = None
            deform_std[obj_name] = None

    # Calculate overall MAE
    overall_rigid_mae = np.mean(all_rigid_errors) if all_rigid_errors else None
    overall_deform_mae = np.mean(all_deform_errors) if all_deform_errors else None

    # Print Rigid MAE
    line = "RANSAC & Rigid MAE"
    for obj_name in CLASS_ORDER:
        if rigid_mae[obj_name] is not None:
            line += f" & {rigid_mae[obj_name]:.1f}"
        else:
            line += " & -"
    line += f" & {overall_rigid_mae:.1f}" if overall_rigid_mae else " & -"
    print(line)

    # Print Rigid Std
    line = "RANSAC & Rigid Std"
    for obj_name in CLASS_ORDER:
        if rigid_std[obj_name] is not None:
            line += f" & {rigid_std[obj_name]:.1f}"
        else:
            line += " & -"
    line += " & -"
    print(line)

    # Print Deform MAE
    line = "RANSAC & Deform MAE"
    for obj_name in CLASS_ORDER:
        if deform_mae[obj_name] is not None:
            line += f" & {deform_mae[obj_name]:.1f}"
        else:
            line += " & -"
    line += f" & {overall_deform_mae:.1f}" if overall_deform_mae else " & -"
    print(line)

    # Print Deform Std
    line = "RANSAC & Deform Std"
    for obj_name in CLASS_ORDER:
        if deform_std[obj_name] is not None:
            line += f" & {deform_std[obj_name]:.1f}"
        else:
            line += " & -"
    line += " & -"
    print(line)

    print("-" * 100)

    # Print sample counts
    print("\nSample counts:")
    print("-" * 60)
    for obj_name in CLASS_ORDER:
        n_rigid = len(data[obj_name].get("non_deformable", []))
        n_deform = len(data[obj_name].get("deformable", []))
        print(f"  {DISPLAY_NAMES[obj_name]:12s}: rigid={n_rigid:4d}, deform={n_deform:4d}")

    # Print LaTeX-ready format
    print("\n" + "=" * 100)
    print("LATEX FORMAT (copy-paste ready)")
    print("=" * 100)

    # Rigid MAE
    line = "& Rigid MAE"
    for obj_name in CLASS_ORDER:
        if rigid_mae[obj_name] is not None:
            line += f" & {rigid_mae[obj_name]:.1f}"
        else:
            line += " & -"
    line += f" & {overall_rigid_mae:.1f} \\\\" if overall_rigid_mae else " & - \\\\"
    print(line)

    # Rigid Std
    line = "& Rigid Std"
    for obj_name in CLASS_ORDER:
        if rigid_std[obj_name] is not None:
            line += f" & {rigid_std[obj_name]:.1f}"
        else:
            line += " & -"
    line += " & - \\\\"
    print(line)

    # Deform MAE
    line = "& Deform MAE"
    for obj_name in CLASS_ORDER:
        if deform_mae[obj_name] is not None:
            line += f" & {deform_mae[obj_name]:.1f}"
        else:
            line += " & -"
    line += f" & {overall_deform_mae:.1f} \\\\" if overall_deform_mae else " & - \\\\"
    print(line)

    # Deform Std
    line = "& Deform Std"
    for obj_name in CLASS_ORDER:
        if deform_std[obj_name] is not None:
            line += f" & {deform_std[obj_name]:.1f}"
        else:
            line += " & -"
    line += " & - \\\\"
    print(line)


if __name__ == "__main__":
    main()
