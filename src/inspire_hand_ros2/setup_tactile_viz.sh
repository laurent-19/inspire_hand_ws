#!/bin/bash
# Setup script for tactile visualization dependencies

set -e

echo "Installing tactile visualization dependencies..."

# Fix NumPy version compatibility
echo "Checking NumPy version..."
NUMPY_VERSION=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "not installed")
echo "Current NumPy version: $NUMPY_VERSION"

if [[ "$NUMPY_VERSION" == "2."* ]]; then
    echo "NumPy 2.x detected. Downgrading to NumPy 1.x for compatibility..."
    pip3 install --user 'numpy<2.0' --force-reinstall
    echo "NumPy downgraded successfully"
fi

# Install trimesh
echo "Installing trimesh..."
pip3 install --user trimesh

echo ""
echo "Setup complete!"
echo ""
echo "To build and run:"
echo "  cd ~/rh56dfx-and-rh56e2/inspire_hand_ws"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select inspire_hand_ros2 inspire_hand_description"
echo "  source install/setup.bash"
echo "  ros2 launch inspire_hand_description display_tactile.launch.py"
