# Data Collection Scripts

Scripts for extracting synchronized multi-modal training data from ROS2 bags.

## Overview

The pipeline processes ROS2 bag files containing robot hand grasping data and extracts synchronized samples of:
- Camera RGB images
- Tactile sensor heatmaps
- Tactile point clouds (with intensity)
- Hand actuator states
- Joint states

Bags are automatically classified as **deformable** (empty/soft objects) or **non-deformable** (filled/rigid objects) based on filename.

## Scripts

### `process_bags.sh`

Main orchestrator script that processes all bags in `test_bags/`.

**Usage:**
```bash
# Process all bags
./scripts/process_bags.sh

# Process a single bag
./scripts/process_bags.sh record_250_1
```

**Configuration** (edit script to change):
```bash
SAMPLE_INTERVAL=0.081  # Seconds between samples (~30 samples per 5s bag)
```

**Classification:**
- Bags with `_empty` in name → `training_data/deformable/`
- All other bags → `training_data/non_deformable/`

### `visualize_sample.py`

Interactive visualizer for inspecting extracted samples.

**Usage:**
```bash
./scripts/visualize_sample.py training_data/non_deformable/record_250_1/sample_0000
```

**Controls:**
1. Camera RGB image → press any key to continue
2. Tactile colormap → press any key to continue
3. Tactile point cloud viewer:
   - **A** - Toggle between RGB and Intensity coloring
   - **R** - Toggle between Filtered and Raw point cloud
   - **Q** - Quit
4. Camera point cloud viewer (if available):
   - **Q** - Quit


## Output Structure

```
training_data/
├── README.md                      # Dataset documentation
├── deformable/                    # Bags with "_empty" in name
│   └── record_250_empty_1/
│       ├── sample_0000/
│       │   ├── camera_rgb.png            # 1280x720 RGB camera image
│       │   ├── camera_pointcloud.pcd     # Depth camera point cloud (CUDA processed)
│       │   ├── tactile_colormap.png      # 360x200 tactile heatmap
│       │   ├── hand_state.json           # Actuator positions, forces, etc.
│       │   ├── joint_state.json          # Joint angles
│       │   ├── tactile_pointcloud.pcd    # 3D point cloud (grey filtered)
│       │   └── tactile_pointcloud_raw.pcd # 3D point cloud (unfiltered)
│       ├── sample_0001/
│       └── ...
└── non_deformable/                # All other bags
    └── record_250_1/
        └── ...
```

## Data Formats

### hand_state.json
```json
{
  "timestamp": {"sec": 1774010202, "nanosec": 21600595},
  "position_actual": [97, 73, 19, 93, 243, 813],
  "angle_actual": [999, 998, 999, 998, 1000, 492],
  "force_actual": [-78, 4, 5, 2, 17, 36],
  "current": [0, 0, 0, 0, 0, 0],
  "status": [2, 2, 2, 2, 2, 2],
  "error": [0, 0, 0, 0, 0, 0],
  "temperature": [56, 56, 56, 58, 50, 58]
}
```

**Field Ranges:**
- `position_actual`: 0-2000 (actuator stroke)
- `angle_actual`: 0-1000 (normalized finger angle)
- `force_actual`: -4000 to 4000 grams
- `current`: 0-2000 mA
- `temperature`: 0-100 °C

### joint_state.json
```json
{
  "timestamp": {"sec": 1774010202, "nanosec": 54589470},
  "name": ["right_little_1_joint", "right_ring_1_joint", ...],
  "position": [0.0, -0.0, 0.0, ...],
  "velocity": [],
  "effort": []
}
```

### tactile_pointcloud.pcd
Binary PCD v0.7 format with fields:
- `x, y, z` (float32) - 3D coordinates in meters
- `rgb` (packed float32) - Point color
- `intensity` (float32) - Tactile pressure (0-4095 raw ADC)

## ROS2 Topics Sampled

| Topic | Type | Rate | Output |
|-------|------|------|--------|
| `/camera/camera/color/image_raw` | Image | ~10 Hz | camera_rgb.png |
| `/cylinder_projection/unwrapped_colormap` | Image | ~10 Hz | tactile_colormap.png |
| `/inspire_hand/inspire_hand_node/state` | InspireHandState | ~30 Hz | hand_state.json |
| `/joint_states` | JointState | ~10 Hz | joint_state.json |
| `/inspire_hand/tactile_pointcloud` | PointCloud2 | ~10 Hz | tactile_pointcloud.pcd |

## Sample Data

### Deformable (Empty/Soft Objects)
**Example: `record_250_empty_10/sample_0005`**

| Camera RGB | Tactile Colormap |
|-----------|------------------|
| ![Camera](../training_data/deformable/record_250_empty_10/sample_0005/camera_rgb.png) | ![Tactile](../training_data/deformable/record_250_empty_10/sample_0005/tactile_colormap.png) |

### Non-Deformable (Rigid/Filled Objects)
**Example: `record_250_12/sample_0000`**

| Camera RGB | Tactile Colormap |
|-----------|------------------|
| ![Camera](../training_data/non_deformable/record_250_12/sample_0000/camera_rgb.png) | ![Tactile](../training_data/non_deformable/record_250_12/sample_0000/tactile_colormap.png) |

**View more samples:**
```bash
./scripts/visualize_sample.py training_data/deformable/record_250_empty_10/sample_0000
./scripts/visualize_sample.py training_data/deformable/record_250_empty_10/sample_0005
./scripts/visualize_sample.py training_data/deformable/record_250_empty_10/sample_0010
./scripts/visualize_sample.py training_data/non_deformable/record_250_12/sample_0000
./scripts/visualize_sample.py training_data/non_deformable/record_250_12/sample_0002
```

## Requirements

- ROS2 Humble
- Python packages: `opencv-python`, `numpy`, `open3d`, `pyyaml`
- Built workspace: `colcon build`
- Install dependencies: `pip install -r scripts/requirements.txt`

## Collected Dataset Statistics

| Metric | Value |
|--------|-------|
| Total Samples | 6,087 |
| Total Bags | 198 |
| Dataset Size | 11.08 GB |
| Sample Interval | 0.081s (~30 samples per bag) |

**Note:** All modalities are complete except `camera_pointcloud.pcd`, which is missing from 306 samples (5.0%) due to sparse depth camera recording in source bags. See `training_data/README.md` for affected bags.
