#!/bin/bash
#
# Process ROS2 bags and extract synchronized training data.
# Usage: ./process_bags.sh [bag_name]
#   If bag_name is provided, only process that specific bag.
#   Otherwise, process all bags in test_bags/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
BAGS_DIR="$WS_DIR/test_bags"
OUTPUT_DIR="$WS_DIR/training_data"
SAMPLE_INTERVAL=0.081

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# Create output directories
mkdir -p "$OUTPUT_DIR/deformable"
mkdir -p "$OUTPUT_DIR/non_deformable"

process_bag() {
    local bag_path="$1"
    local bag_name="$(basename "$bag_path")"

    # Classify: *_empty_* -> deformable, else -> non_deformable
    if [[ "$bag_name" == *"_empty"* ]]; then
        category="deformable"
    else
        category="non_deformable"
    fi

    # Skip if already processed
    local sample_dir="$OUTPUT_DIR/$category/$bag_name"
    if [[ -d "$sample_dir" && -n "$(ls -A "$sample_dir" 2>/dev/null)" ]]; then
        echo "Skipping $bag_name (already processed)"
        return 0
    fi

    echo "Processing: $bag_name -> $category"

    # Start sampler node in background
    ros2 run inspire_hand_ros2 bag_data_sampler.py \
        --ros-args \
        -p output_dir:="$OUTPUT_DIR" \
        -p category:="$category" \
        -p bag_name:="$bag_name" \
        -p sample_interval:="$SAMPLE_INTERVAL" &
    SAMPLER_PID=$!

    # Give sampler time to initialize
    sleep 1

    # Play bag with simulated clock
    ros2 bag play "$bag_path" --clock

    # Give sampler time to process final messages
    sleep 1

    # Force kill the sampler and any child processes
    kill -9 $SAMPLER_PID 2>/dev/null || true
    pkill -9 -f "bag_data_sampler" 2>/dev/null || true
    wait $SAMPLER_PID 2>/dev/null || true

    # Extra wait to ensure process is fully terminated
    sleep 1

    echo "Completed: $bag_name"
    echo "---"
}

# Main
if [[ $# -eq 1 ]]; then
    # Process single bag
    bag_path="$BAGS_DIR/$1"
    if [[ ! -d "$bag_path" ]]; then
        echo "Error: Bag not found: $bag_path"
        exit 1
    fi
    process_bag "$bag_path"
else
    # Process all bags
    for bag_path in "$BAGS_DIR"/record_*; do
        if [[ -d "$bag_path" ]]; then
            process_bag "$bag_path"
        fi
    done
fi

echo "All bags processed. Output in: $OUTPUT_DIR"
