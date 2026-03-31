#!/bin/bash
set -e

# Configuration
BAG_DIR="${1:-.}"
BAG_NAME="grasp_sequence_$(date +%Y%m%d_%H%M%S)"
#BAG_PATH="$BAG_DIR/$BAG_NAME"
BAG_PATH="$BAG_DIR"

# Create directory if it doesn't exist
#mkdir -p "$BAG_DIR"

echo "=========================================="
echo "Grasp Sequence with Data Recording"
echo "=========================================="
echo ""
echo "Recording topics:"
echo "  • Hand state, control, and tactile data"
echo "  • Tactile point clouds (with intensity field)"
echo "  • Cylinder projection (for grasp visualization)"
echo "  • Joint states and TF transforms"
echo ""

# Check if ROS2 is running
echo "Checking prerequisites..."
if ! ros2 node list &>/dev/null; then
    echo "ERROR: ROS2 not available or no daemon running"
    echo "Try: ros2 daemon start"
    exit 1
fi

# Check if inspire_hand node is running
if ! ros2 node list | grep -q inspire_hand_node; then
    echo "WARNING: inspire_hand_node not found!"
    echo "Please start it first:"
    echo "  ros2 launch inspire_hand_ros2 inspire_hand.launch.py"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "✓ Prerequisites OK"
echo ""
echo "Output: $BAG_PATH"
echo "=========================================="
echo ""

# Topics to record
TOPICS=(
    # Hand control and status
    "/inspire_hand/inspire_hand_node/control"
    "/inspire_hand/inspire_hand_node/grasp_status"
    "/inspire_hand/inspire_hand_node/state"
    "/inspire_hand/inspire_hand_node/touch"

    # Tactile point clouds (original and projected)
    "/inspire_hand/tactile_pointcloud"
    "/cylinder_projection/projected_pointcloud"
    #"/cylinder_projection/reference_cylinder"

    # 2D unwrapped images
    "/cylinder_projection/unwrapped_image"
    "/cylinder_projection/unwrapped_colormap"

    # Robot state and transforms
    "/joint_states"
    "/robot_description"
    "/tf"
    "/tf_static"

    "/camera/camera/color/image_raw"
    "/camera/camera/depth/image_rect_raw"
    "/camera/camera/aligned_depth_to_color/camera_info"
    "/camera/camera/aligned_depth_to_color/image_raw"
    "/camera/camera/depth/camera_info"
    "/processed_pointcloud"
)

# Cleanup function
cleanup() {
    if [[ -n "$RECORD_PID" ]] && kill -0 $RECORD_PID 2>/dev/null; then
        echo "Stopping recording..."
        kill $RECORD_PID
        wait $RECORD_PID 2>/dev/null || true
    fi
}

trap cleanup EXIT

# Start recording in background with nohup to prevent job control issues
echo "Starting ros2 bag record to: $BAG_PATH"
nohup ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}" > /tmp/bag_record.log 2>&1 &
RECORD_PID=$!
echo "Recording PID: $RECORD_PID"
echo ""

sleep 3

# Stop recording
echo "=========================================="
echo "Stopping recording..."
sleep 2  # Give time to finish writing
if kill -0 $RECORD_PID 2>/dev/null; then
    kill $RECORD_PID
fi
wait $RECORD_PID 2>/dev/null || true
sleep 1  # Wait for final flush

# Check if bag was created
if [ -d "$BAG_PATH" ] && [ -n "$(ls -A "$BAG_PATH" 2>/dev/null)" ]; then
    echo "✓ Recording saved successfully to:"
    echo "  $BAG_PATH"
    echo ""
    ls -lh "$BAG_PATH"
    echo ""
    echo "To play back:"
    echo "  ros2 bag play $BAG_PATH"
else
    echo "✗ Recording failed or empty!"
    echo "  Path: $BAG_PATH"
fi
echo "=========================================="
