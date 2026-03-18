# Cylinder Unwrap Visualization

Creates a 2D "unwrapped" heatmap image from cylindrical tactile data, showing where contact occurs on a grasped cylinder.

## Quick Start

### 1. Launch Complete Tactile Visualization Pipeline

```bash
# Source workspace
source install/setup.bash

# Launch everything (tactile pointcloud + cylinder projection + unwrap)
ros2 launch inspire_hand_ros2 tactile_visualization.launch.py
```

### 2. Launch Individual Nodes

```bash
# Source workspace
source install/setup.bash

# Launch tactile pointcloud
ros2 launch inspire_hand_ros2 tactile_pointcloud.launch.py

# Launch cylinder projection
ros2 launch inspire_hand_ros2 cylinder_projection.launch.py

# Launch cylinder unwrap
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py
```

## Output Topics

### `/cylinder_projection/unwrapped_image`
- **Type**: `sensor_msgs/Image`
- **Encoding**: `32FC1` (32-bit float, single channel)
- **Content**: Raw intensity values (0-4095) or NaN
- **Use**: For quantitative analysis and data export

### `/cylinder_projection/unwrapped_colormap`
- **Type**: `sensor_msgs/Image`
- **Encoding**: `rgb8` (8-bit RGB)
- **Content**: Colorized heatmap using JET colormap
- **Use**: For visualization in RViz or other viewers

## Image Layout

```
       180°        270°       0°/360°      90°         180°
        ↓           ↓           ↓           ↓           ↓
     +-------------------------------------------------------+
 +H  |             |           |           |               |  ← Top of cylinder
 e   |             |           |           |               |
 i   |    TACTILE  |  CONTACT  |  HEATMAP  |               |
 g   |             |           |           |               |
 h   |             |           |           |               |
 t   |             |           |           |               |
     |             |           |           |               |
 -H  +-------------------------------------------------------+  ← Bottom of cylinder
     └───────────────── Angle ─────────────────────────────┘
     ↑                         ↑                           ↑
   Index 0                  Middle                     Index 359
   (Cut line)          (Palm contact)                  (Cut line)
```

**Key Layout:**
- **Left edge (index 0)**: 180° - Cut/seam line (opposite palm)
- **Center (index 180)**: 0°/360° - Palm contact edge
- **Right edge (index 359)**: 180° - Cut/seam line (wraps around)

This layout centers the palm contact in the image, making it easy to see the grasp pattern symmetrically.

### Cylinder Geometry

The cylinder is positioned with:
- **Axis**: X-axis of palm force sensor frame (finger direction)
- **Center**: Offset by radius in +Z direction from palm origin
- **Contact edge**: At Z=0 in palm frame (where cylinder touches palm sensor)

Looking down the cylinder axis (from +X direction):
```
           +Z (top, 180°)
             |
        _____|_____  ← CUT LINE (both edges of image)
       /           \
270° |      •       | 90°
      |      |      |
       \_____•_____/
             |
         0°/360° (palm contact, CENTER of image)
             |
           +Y (palm sensor direction)
```

**Unwrapping visualization:**
```
Cylinder wrapped:     Unwrapped image:

    180° (cut)        180° ... 270° ... 0° ... 90° ... 180°
       |              [====|====•====|====]
       •              Left edge  Center  Right edge
      / \
    270° 90°
      \ /
       •
       0°
```

### Axes
- **X-axis (columns)**: Angle around cylinder, **shifted to center palm contact**
  - **Index 0 (left edge)**: 180° - Cut/seam line (opposite palm)
  - Index 90 (1/4): 270° - Left side of cylinder
  - **Index 180 (center)**: 0°/360° - Palm contact edge
  - Index 270 (3/4): 90° - Right side of cylinder
  - **Index 359 (right edge)**: 180° - Cut/seam line (wraps to left edge)
- **Y-axis (rows)**: Height along cylinder axis (X-axis of palm frame)
  - Top = Positive height (+H, fingers direction)
  - Bottom = Negative height (-H, wrist direction)
- **Pixel value**: Tactile intensity
  - 0-4095 = Raw pressure from tactile sensors (12-bit ADC)
  - NaN (black) = No tactile data (structural geometry, no sensor)

**Note**: The image wraps at the edges - column 0 and column 359 represent the same physical location (180°, the cut line).

## Viewing in RViz

### 1. Add Image Display

```
RViz → Add → By topic → /cylinder_projection/unwrapped_colormap → Image
```

### 2. Configure Display

- **Image Topic**: `/cylinder_projection/unwrapped_colormap`
- **Transport Hint**: `raw`
- **Normalize Range**: Unchecked (already normalized)

### 3. View Raw Intensity

For raw intensity values:
- **Image Topic**: `/cylinder_projection/unwrapped_image`
- **Transport Hint**: `raw`
- **Normalize Range**: Checked

## Parameters

Configured in launch file or via command line:

```bash
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py \
    image_width:=360 \
    image_height:=200 \
    height_min:=-0.075 \
    height_max:=0.075
```

### Available Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `image_width` | 360 | Width in pixels (1 pixel = 1 degree) |
| `image_height` | 200 | Height in pixels |
| `height_min` | -0.075 | Min height along cylinder (m) |
| `height_max` | 0.075 | Max height along cylinder (m) |
| `intensity_min` | 0.0 | Min intensity for colormap |
| `intensity_max` | 4095.0 | Max intensity for colormap |
| `colormap` | 2 | OpenCV colormap (2=JET) |

### Colormap Options

Currently only JET colormap is supported (parameter value: 2):
- **JET** (default): Blue → Cyan → Green → Yellow → Red
  - Low pressure = Blue
  - Medium pressure = Green/Yellow
  - High pressure = Red

This matches the colormap used in the 3D tactile point cloud visualization.

## Recording and Export

### Record Unwrapped Images

The unwrapped images are automatically recorded when using `grasp_sequence_safe.sh`:

```bash
./grasp_sequence_safe.sh bags/
```

This now records:
- `/cylinder_projection/unwrapped_image` (raw intensity)
- `/cylinder_projection/unwrapped_colormap` (visualization)

### Extract Images from Bag

```bash
# Play bag
ros2 bag play bags/grasp_sequence_YYYYMMDD_HHMMSS

# Save images (in another terminal)
ros2 run image_view image_saver \
    --ros-args \
    -r image:=/cylinder_projection/unwrapped_colormap \
    -p filename_format:="unwrapped_%04d.png"
```

### Convert to Video

```bash
# Using ffmpeg
ffmpeg -framerate 50 -pattern_type glob -i 'unwrapped_*.png' \
    -c:v libx264 -pix_fmt yuv420p grasp_visualization.mp4
```

## Use Cases

### 1. Grasp Analysis
- Visualize contact distribution on cylindrical objects
- Identify high-pressure zones
- Verify symmetric grasping

### 2. Quality Control
- Check for missing sensors (NaN regions)
- Verify sensor calibration
- Detect anomalous contact patterns

### 3. Data Export
- Export raw intensity images for ML training
- Generate datasets for grasp recognition
- Create time-series visualizations

## Troubleshooting

### No Image Published

**Check prerequisites:**
```bash
# Verify cylinder projection is running
ros2 topic echo /cylinder_projection/projected_pointcloud --once

# Check if unwrap node is running
ros2 node list | grep cylinder_unwrap

# Check topics
ros2 topic list | grep cylinder_projection
```

### Image is All Black

**Possible causes:**
1. No tactile contact - all values are NaN
2. Intensity range misconfigured
3. Transform not available

**Debug:**
```bash
# Check raw intensity values
ros2 topic echo /cylinder_projection/projected_pointcloud --field data | head -100
```

### Image Resolution Issues

**Adjust parameters:**
```bash
# Higher resolution (720x400)
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py \
    image_width:=720 \
    image_height:=400

# Adjust height range
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py \
    height_min:=-0.10 \
    height_max:=0.10
```

## Example Workflow

### Complete Grasp Recording with Visualization

```bash
# Terminal 1: Launch hand driver
ros2 launch inspire_hand_ros2 inspire_hand.launch.py

# Terminal 2: Launch complete visualization
ros2 launch inspire_hand_ros2 tactile_visualization.launch.py

# Terminal 3: Launch RViz
rviz2

# In RViz: Add Image display for /cylinder_projection/unwrapped_colormap

# Terminal 4: Record grasp sequence
./grasp_sequence_safe.sh bags/

# Result: Full recording with 2D unwrapped visualization
```

## Advanced: Custom Processing

### Extract Raw Intensity Array

```python
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()

def image_callback(msg):
    # Convert to numpy array
    intensity_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    # Extract valid values (not NaN)
    valid_mask = ~np.isnan(intensity_image)
    valid_intensities = intensity_image[valid_mask]

    # Analyze
    print(f"Mean pressure: {valid_intensities.mean():.1f}")
    print(f"Max pressure: {valid_intensities.max():.1f}")
    print(f"Contact area: {valid_mask.sum()} pixels")
```

## Notes

- **Coordinate System**: Cylinder axis is X-axis of palm force sensor frame
- **Angle Convention**:
  - 0° at palm contact edge (where cylinder touches palm sensor)
  - 180° at opposite side (cut/seam line for unwrapping)
  - Computed as: `angle = atan2(z - radius, y) + 90°` in cylinder frame
- **Image Centering**:
  - Palm contact (0°) is centered in image for symmetric grasp visualization
  - Cut line (180°) appears at both left and right edges
  - Unwrapping formula: `column = ((angle + 180°) % 360°) / 360° * width`
- **Height Convention**: Positive height points in +X direction of palm frame (toward fingers)
- **Intensity Encoding**: Raw 12-bit ADC values (0-4095) from tactile sensors
- **NaN Handling**: Black pixels in colormap, skipped in quantitative analysis
- **Transform Requirement**: Node uses TF2 to transform points from base_footprint to cylinder frame
