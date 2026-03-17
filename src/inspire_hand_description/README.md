# Inspire Hand Description

URDF models and visualization for the **RH56E2** and **RH56DFTP** Dexterous Hands by Beijing Inspire-Robots.

![Right Hand in RViz](doc/right_hand_rviz.png)

## Package Contents

- **URDF Models**: Complete kinematic descriptions for left and right hands
- **STL Meshes**: High-quality 3D meshes for all links (~32 files per hand)
- **RViz Configurations**: Pre-configured visualization setups
- **Launch Files**: Easy visualization of hand models
- **Tactile Point Cloud**: 3D spatial tactile visualization

## Features

- ✅ **Accurate Kinematics**: 6 DOF per hand with mimic joints
- ✅ **Visual & Collision Meshes**: Detailed STL models
- ✅ **Force Sensors**: Separate meshes for tactile sensor pads
- ✅ **Both Hands**: Left and right hand models
- ✅ **RViz Integration**: Launch files with pre-configured views
- ✅ **3D Tactile Visualization**: Real-time point cloud with tactile color mapping

## Quick Start

### View Hand Model in RViz

```bash
# Right hand
ros2 launch inspire_hand_description display.launch.py

# Left hand
ros2 launch inspire_hand_description display.launch.py hand:=left

# Without GUI (headless)
ros2 launch inspire_hand_description display.launch.py gui:=false
```

### 3D Tactile Point Cloud Visualization

Visualize all 1062 tactile sensors as a colored 3D point cloud:

```bash
# Standalone simulation (with joint state publisher GUI)
ros2 launch inspire_hand_description display_tactile.launch.py

# With real hardware (requires inspire_hand_ros2 running)
ros2 launch inspire_hand_ros2 tactile_pointcloud.launch.py
```

## URDF Structure

### Joints (6 DOF)

| Joint | Type | Range | Description |
|-------|------|-------|-------------|
| `right_thumb_1_joint` | Revolute | 90°-165° | Thumb rotation (abduction) |
| `right_thumb_2_joint` | Revolute | -13°-70° | Thumb bend (flexion) |
| `right_index_1_joint` | Revolute | 20°-176° | Index proximal joint |
| `right_middle_1_joint` | Revolute | 20°-176° | Middle proximal joint |
| `right_ring_1_joint` | Revolute | 20°-176° | Ring proximal joint |
| `right_little_1_joint` | Revolute | 20°-176° | Little proximal joint |

### Mimic Joints

Distal joints automatically follow proximal joints:

| Mimic Joint | Parent | Multiplier |
|-------------|--------|------------|
| `right_thumb_3_joint` | `right_thumb_2_joint` | 0.8392 |
| `right_thumb_4_joint` | `right_thumb_3_joint` | 0.891 |
| `right_index_2_joint` | `right_index_1_joint` | 1.0843 |
| `right_middle_2_joint` | `right_middle_1_joint` | 1.0843 |
| `right_ring_2_joint` | `right_ring_1_joint` | 1.0843 |
| `right_little_2_joint` | `right_little_1_joint` | 1.0843 |

### Links

Each hand has **33 links**:
- Base link (palm)
- 2 palm links
- Palm force sensor
- 5 fingers × 5 links each:
  - Main segment links (×4 per finger)
  - Force sensor pads (×3-4 per finger with tactile sensors)

### Meshes

Located in `meshes/right/` and `meshes/left/`:
- **32 STL files** per hand
- Meshes for main segments and force sensor pads
- Binary STL format

## Tactile Sensor Mapping

The hand has **1062 tactile sensors** distributed across force sensor pads:

**Regular Fingers (Index, Middle, Ring, Little):**
- Tip: 3×3 = 9 taxels (on `force_sensor_3`)
- Nail: 12×8 = 96 taxels (on `force_sensor_2`)
- Pad: 10×8 = 80 taxels (on `force_sensor_1`)
- **Total per finger: 185 taxels**

**Thumb:**
- Tip: 3×3 = 9 taxels (on `force_sensor_4`)
- Nail: 12×8 = 96 taxels (on `force_sensor_1`)
- Middle: 3×3 = 9 taxels (on `force_sensor_3`)
- Pad: 12×8 = 96 taxels (on `force_sensor_2`)
- **Total: 210 taxels**

**Palm:**
- 8×14 = 112 taxels (on `palm_force_sensor`, column-major layout)

**Grand Total: 4×185 + 210 + 112 = 1062 taxels**

## Files

```
inspire_hand_description/
├── urdf/
│   ├── rh56e2_right.urdf       # Right hand model
│   └── rh56e2_left.urdf        # Left hand model
├── meshes/
│   ├── right/                  # Right hand STL files (32 files)
│   └── left/                   # Left hand STL files (32 files)
├── rviz/
│   ├── right.rviz             # Right hand RViz config
│   ├── left.rviz              # Left hand RViz config
│   ├── display.rviz           # Basic display config
│   └── tactile_pointcloud.rviz # Tactile visualization config
├── launch/
│   ├── display.launch.py       # Basic hand visualization
│   └── display_tactile.launch.py # Tactile point cloud visualization
```

## Usage with inspire_hand_ros2

The hand model is automatically used when launching the ROS2 driver:

```bash
# Driver publishes /robot_description and /joint_states
ros2 launch inspire_hand_ros2 inspire_hand.launch.py

# View in RViz (in another terminal)
rviz2 -d install/inspire_hand_description/share/inspire_hand_description/rviz/right.rviz
```

## Parameters

The URDF uses `package://inspire_hand_description/` URIs for meshes. Make sure the package is installed:

```bash
colcon build --packages-select inspire_hand_description
source install/setup.bash
```

## Launch File Arguments

### display.launch.py

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `hand` | `right` | `right`, `left` | Which hand to display |
| `gui` | `true` | `true`, `false` | Show joint_state_publisher_gui |
| `rviz` | `true` | `true`, `false` | Launch RViz |

### display_tactile.launch.py

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_gui` | `true` | `true`, `false` | Show joint_state_publisher_gui |
| `rviz` | `true` | `true`, `false` | Launch RViz |

## Coordinate Frames

- **Base frame**: `base_footprint` (origin)
- **Palm frame**: `base_link` (offset -0.0305m in Z)
- All finger frames relative to `base_link`

## Integration

This package is referenced by:
- `inspire_hand_ros2` - Main driver package
- Custom manipulation stacks using the hand

To use in your own package:

```xml
<!-- package.xml -->
<depend>inspire_hand_description</depend>
```

```python
# Python launch file
from ament_index_python.packages import get_package_share_directory

description_pkg = get_package_share_directory('inspire_hand_description')
urdf_file = os.path.join(description_pkg, 'urdf', 'rh56e2_right.urdf')
```

## Tactile Point Cloud Visualization

![Tactile Point Cloud](doc/hand_tactile_pcl_random.png)

The 3D point cloud maps 1062 tactile sensors to colored mesh points:

**Sampling**: 16 points per taxel (configurable via `points_per_taxel` parameter)
- Equal visual weight per sensor
- ~17,000 tactile points + ~3,000 non-tactile = **~20,000 total**

**Hardware Data Format**:
- Each taxel: 16-bit integer (2 bytes, little-endian)
- Value range: 0-4095 (12-bit ADC)

**Color Mapping** (0-4095):
| Value | Color | Meaning |
|-------|-------|---------|
| 0 | Blue | No contact |
| 1024 | Cyan | Light contact |
| 2048 | Green | Medium contact |
| 3072 | Yellow | Firm contact |
| 4095 | Red | Maximum pressure |

**Point Distribution** (default 16 pts/taxel):

| Region | Grid | Taxels | Points |
|--------|------|--------|--------|
| Tip | 3×3 | 9 | 144 |
| Nail | 12×8 | 96 | 1,536 |
| Pad (finger) | 10×8 | 80 | 1,280 |
| Pad (thumb) | 12×8 | 96 | 1,536 |
| Palm | 8×14 | 112 | 1,792 |

**Quick test**:
```bash
# Terminal 1: Launch visualization
ros2 launch inspire_hand_description display_tactile.launch.py

# Terminal 2: Publish test tactile data
source install/setup.bash
python3 src/inspire_hand_ros2/scripts/publish_test_tactile.py --pattern grasp
```

**Configure density**:
```bash
# Higher density (32 pts/taxel)
ros2 launch inspire_hand_ros2 tactile_pointcloud.launch.py points_per_taxel:=32

# Lower density for performance (8 pts/taxel)
ros2 launch inspire_hand_ros2 tactile_pointcloud.launch.py points_per_taxel:=8
```

## Troubleshooting

### Meshes not loading

Check that package path is correct:
```bash
ros2 pkg prefix inspire_hand_description
```

Meshes should be in: `<prefix>/share/inspire_hand_description/meshes/`

### RViz shows no robot

Verify URDF is published:
```bash
ros2 topic echo /robot_description --once
```

### Joint state publisher not working

Make sure `joint_state_publisher_gui` is installed:
```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

## References

- **Hardware**: RH56E2/RH56DFTP User Manual, Beijing Inspire-Robots
- **URDF**: Exported from SolidWorks using `sw_urdf_exporter`