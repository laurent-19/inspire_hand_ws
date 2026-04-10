#!/usr/bin/env python3
"""Plot comparison of PointNet++ vs RANSAC predictions for selected classes."""

import csv
import re
from pathlib import Path
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

from radius_prediction.config import OBJECT_TO_RADIUS

# Paths
POINTNET_CSV = Path(__file__).parent / "evaluation_results.csv"
RANSAC_CSV = Path(__file__).parent.parent / "results.csv"
OUTPUT_PLOT = Path(__file__).parent / "comparison_plot.png"

# Classes to include (internal name -> display name)
SELECTED_CLASSES = {
    "250": "250mL\ncan",
    "330_slim": "330mL\nslim can",
    "small_bottle": "500mL\nbottle",
}

# Colors
COLORS = {
    "non_deformable_pointnet": "#1f77b4",    # Blue
    "non_deformable_ransac": "#9467bd",      # Violet
    "deformable_pointnet": "#ff7f0e",        # Orange
    "deformable_ransac": "#ffbb33",          # Yellow
}


def load_pointnet_csv(csv_path):
    """Load PointNet++ results from CSV."""
    results = defaultdict(lambda: defaultdict(list))

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            obj_name = row["object_name"]
            if obj_name not in SELECTED_CLASSES:
                continue

            dtype = row["dataset_type"]
            pred = float(row["prediction_mm"])
            results[obj_name][dtype].append(pred)

    return results


def extract_object_name(record_name):
    """Extract object name from record name like 'record_250_empty_1'."""
    name = record_name.replace("record_", "")
    name = name.replace("_empty", "")

    # Remove trailing number
    match = re.match(r"(.+)_(\d+)$", name)
    if match:
        return match.group(1)
    return name


def load_ransac_csv(csv_path):
    """Load RANSAC results from CSV."""
    results = defaultdict(lambda: defaultdict(list))

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            record_name = row["level_2"]
            obj_name = extract_object_name(record_name)

            if obj_name not in SELECTED_CLASSES:
                continue

            dtype = row["level_1"]
            radius = float(row["radius_mm"])
            results[obj_name][dtype].append(radius)

    return results


def plot_comparison(output_path=None):
    """Create comparison plot."""
    # ICRA-style rcParams
    plt.rcParams.update({
        "font.size": 14,
        "axes.titlesize": 14,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
    })

    # Load data
    pointnet_data = load_pointnet_csv(POINTNET_CSV)
    ransac_data = load_ransac_csv(RANSAC_CSV)

    # Order classes
    class_order = ["250", "330_slim", "small_bottle"]

    n_classes = len(class_order)
    n_rows = n_classes
    n_cols = 2  # Non-deformable | Deformable

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(9, 0.6 * n_rows * 2), sharex=True)

    for row, obj_name in enumerate(class_order):
        gt = OBJECT_TO_RADIUS[obj_name]
        display_name = SELECTED_CLASSES[obj_name]

        for col, dtype in enumerate(["non_deformable", "deformable"]):
            ax = axes[row, col]

            # Get data
            pn_preds = pointnet_data[obj_name].get(dtype, [])
            ransac_preds = ransac_data[obj_name].get(dtype, [])

            # Choose colors
            pn_color = COLORS[f"{dtype}_pointnet"]
            ransac_color = COLORS[f"{dtype}_ransac"]

            # Compute histograms
            bins = np.linspace(10, 80, 29)

            pn_counts = np.zeros(len(bins) - 1)
            ransac_counts = np.zeros(len(bins) - 1)

            if pn_preds:
                pn_counts, _ = np.histogram(pn_preds, bins=bins)
            if ransac_preds:
                ransac_counts, _ = np.histogram(ransac_preds, bins=bins)

            # Compute overlap (minimum of both at each bin)
            overlap = np.minimum(pn_counts, ransac_counts)

            # Plot solid bars
            bin_centers = (bins[:-1] + bins[1:]) / 2
            bin_width = bins[1] - bins[0]

            if pn_preds:
                ax.bar(bin_centers, pn_counts, width=bin_width,
                       color=pn_color, alpha=0.7, edgecolor='white', linewidth=0.3)

            if ransac_preds:
                ax.bar(bin_centers, ransac_counts, width=bin_width,
                       color=ransac_color, alpha=0.5, edgecolor='white', linewidth=0.3)

            # Plot hatched overlap on top
            if np.any(overlap > 0):
                ax.bar(bin_centers, overlap, width=bin_width,
                       facecolor='none', edgecolor='black', linewidth=0.5,
                       hatch='//', alpha=0.8)

            # Ground truth line
            ax.axvline(gt, color='red', linestyle='--', linewidth=2.5, ymax=0.85)

            # Y-axis label (left column only)
            if col == 0:
                ax.set_ylabel(display_name, rotation=0, ha='right', va='center')

            ax.set_xlim(10, 80)
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.0f'))
            ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4)
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)

    # Set same y-scale for all subplots
    max_ylim = max(ax.get_ylim()[1] for ax in axes.flat)
    for ax in axes.flat:
        ax.set_ylim(0, max_ylim)

    # X-ticks
    regular_ticks = [10, 20, 30, 40, 50, 60, 70, 80]
    for col in range(2):
        axes[-1, col].set_xticks(regular_ticks)

    # Column titles (inside plot area)
    axes[0, 0].text(0.5, 0.85, 'Non-deformable', fontsize=14, ha='center', transform=axes[0, 0].transAxes)
    axes[0, 1].text(0.5, 0.85, 'Deformable', fontsize=14, ha='center', transform=axes[0, 1].transAxes)

    # Single centered x-axis label
    fig.text(0.5, -0.02, 'Radius (mm)', ha='center', fontsize=14)


    # Custom legend with split-color rectangles
    from matplotlib.legend_handler import HandlerBase
    from matplotlib.patches import Rectangle

    class SplitColorHandler(HandlerBase):
        def __init__(self, color1, color2, alpha1=0.7, alpha2=0.7):
            self.color1 = color1
            self.color2 = color2
            self.alpha1 = alpha1
            self.alpha2 = alpha2
            super().__init__()

        def create_artists(self, legend, orig_handle, xdescent, ydescent, width, height, fontsize, trans):
            # Left half (rigid/non-deformable)
            r1 = Rectangle((xdescent, ydescent), width/2, height,
                           facecolor=self.color1, alpha=self.alpha1,
                           edgecolor='white', linewidth=0.5, transform=trans)
            # Right half (deformable)
            r2 = Rectangle((xdescent + width/2, ydescent), width/2, height,
                           facecolor=self.color2, alpha=self.alpha2,
                           edgecolor='white', linewidth=0.5, transform=trans)
            return [r1, r2]

    # Dummy handles for legend
    pointnet_handle = Patch(label='PointNet++')
    ransac_handle = Patch(label='RANSAC')
    overlap_handle = Patch(facecolor='none', edgecolor='black', hatch='//', label='Overlap')
    gt_handle = Line2D([0], [0], color='red', linestyle='--', linewidth=2.5, label='Ground Truth')

    handler_map = {
        pointnet_handle: SplitColorHandler(COLORS["non_deformable_pointnet"], COLORS["deformable_pointnet"], 0.7, 0.7),
        ransac_handle: SplitColorHandler(COLORS["non_deformable_ransac"], COLORS["deformable_ransac"], 0.5, 0.5),
    }

    fig.legend(handles=[pointnet_handle, ransac_handle, overlap_handle, gt_handle],
               handler_map=handler_map,
               loc='upper center', bbox_to_anchor=(0.5, 1.02),
               ncol=4, fontsize=14, frameon=False, columnspacing=1.0)



    plt.subplots_adjust(left=0.12, right=0.95, top=0.88, bottom=0.10, hspace=0.4, wspace=0.2)

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot to: {output_path}")

    plt.show()


if __name__ == "__main__":
    plot_comparison(OUTPUT_PLOT)
