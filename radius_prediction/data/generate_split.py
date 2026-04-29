#!/usr/bin/env python3
"""Generate train/val split at bag level for radius prediction.

Splits bags 80/20 per object class, excluding _1 and _2 bags (non-grasping).
"""

import json
import random
import re
from collections import defaultdict
from pathlib import Path

from ..config import NON_DEFORMABLE_DIR, OBJECT_TO_RADIUS


def extract_object_and_bag_num(record_name: str) -> tuple:
    """
    Extract object name and bag number from record directory name.

    Examples:
        'record_250_1' -> ('250', 1)
        'record_330_fat_5' -> ('330_fat', 5)
        'record_small_bottle_3' -> ('small_bottle', 3)
    """
    # Remove 'record_' prefix
    name = record_name.replace("record_", "")

    # Extract trailing number
    match = re.match(r"(.+)_(\d+)$", name)
    if not match:
        return None, None

    obj_name = match.group(1)
    bag_num = int(match.group(2))

    # Check if it matches a known object
    if obj_name not in OBJECT_TO_RADIUS:
        return None, None

    return obj_name, bag_num


def generate_split(
    root_dir: Path = None,
    train_ratio: float = 0.8,
    exclude_bags: list = None,
    seed: int = 42,
    output_path: Path = None,
) -> dict:
    """
    Generate bag-level train/val split.

    Args:
        root_dir: Path to non_deformable directory
        train_ratio: Fraction of bags for training (default 0.8)
        exclude_bags: Bag numbers to exclude (default [1, 2])
        seed: Random seed for reproducibility
        output_path: Where to save the split JSON

    Returns:
        Split dictionary with 'train', 'val', 'excluded' lists
    """
    if root_dir is None:
        root_dir = NON_DEFORMABLE_DIR

    if exclude_bags is None:
        exclude_bags = [1, 2]

    root_dir = Path(root_dir)
    random.seed(seed)

    # Group bags by object class
    bags_by_object = defaultdict(list)
    excluded = []

    for record_dir in sorted(root_dir.iterdir()):
        if not record_dir.is_dir():
            continue

        record_name = record_dir.name
        obj_name, bag_num = extract_object_and_bag_num(record_name)

        if obj_name is None:
            continue

        if bag_num in exclude_bags:
            excluded.append(record_name)
        else:
            bags_by_object[obj_name].append(record_name)

    # Split each object class 80/20
    train_bags = []
    val_bags = []

    print(f"Generating split (train_ratio={train_ratio}, seed={seed})")
    print(f"Excluding bags: {exclude_bags}")
    print()

    for obj_name in sorted(bags_by_object.keys()):
        bags = bags_by_object[obj_name]
        random.shuffle(bags)

        n_train = max(1, int(len(bags) * train_ratio))
        obj_train = sorted(bags[:n_train])
        obj_val = sorted(bags[n_train:])

        train_bags.extend(obj_train)
        val_bags.extend(obj_val)

        print(f"{obj_name}: {len(bags)} bags -> {len(obj_train)} train, {len(obj_val)} val")

    print()
    print(f"Total: {len(train_bags)} train, {len(val_bags)} val, {len(excluded)} excluded")

    # Build split dict
    split = {
        "train": sorted(train_bags),
        "val": sorted(val_bags),
        "excluded": sorted(excluded),
        "metadata": {
            "train_ratio": train_ratio,
            "exclude_bags": exclude_bags,
            "seed": seed,
        }
    }

    # Save if output path provided
    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(split, f, indent=2)
        print(f"\nSaved to: {output_path}")

    return split


def main():
    """Generate default split."""
    output_path = Path(__file__).parent / "splits" / "default_split.json"
    generate_split(output_path=output_path)


if __name__ == "__main__":
    main()
