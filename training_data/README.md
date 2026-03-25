# Inspire Hand Grasping Dataset

Multi-modal training dataset for AI-based grasp analysis, collected from the RH56DFTP dexterous hand with integrated tactile sensors.

## Dataset Statistics

| Metric | Value |
|--------|-------|
| Total Samples | 6,087 |
| Deformable Samples | 2,229 |
| Non-deformable Samples | 3,858 |
| Total Bags | 198 |
| Deformable Bags | 83 |
| Non-deformable Bags | 115 |
| Dataset Size | 11.08 GB |
| Total Files | 42,303 |
| Sample Rate | 0.081s interval (~30 samples/bag) |
| Bag Duration | ~5 seconds each |
| Avg Sample Size | 1.86 MB |

## Classification

- **Deformable**: Empty/soft objects (bags with `_empty` in filename)
- **Non-deformable**: Filled/rigid objects (all other bags)

## Data Modalities

### 1. Camera RGB (`camera_rgb.png`)
- Resolution: 1280 x 720 pixels
- Format: PNG (RGB)
- Source: Intel RealSense color camera

### 2. Camera Point Cloud (`camera_pointcloud.pcd`)
- Format: Binary PCD v0.7
- Source: Intel RealSense depth camera
- Processing: CUDA-accelerated depth projection, 1cm voxel downsampling
- Fields: `x, y, z` (float32) - 3D coordinates in meters
- Typical size: ~90KB, ~7,500 points

### 3. Tactile Heatmap (`tactile_colormap.png`)
- Resolution: 360 x 200 pixels
- Format: PNG (RGB colormap)
- Visualization of tactile sensor pressure distribution

### 4. Tactile Point Cloud - Filtered (`tactile_pointcloud.pcd`)
- Format: Binary PCD v0.7
- Grey background points filtered out
- Typical size: ~330KB

| Field | Type | Description |
|-------|------|-------------|
| `x, y, z` | float32 | 3D coordinates (meters) |
| `rgb` | packed float32 | Point color |
| `intensity` | float32 | Tactile pressure (0-4095 raw) |

### 5. Tactile Point Cloud - Raw (`tactile_pointcloud_raw.pcd`)
- Format: Binary PCD v0.7
- All points including grey background
- Typical size: ~380KB
- Same fields as filtered version

**Tactile Sensor Specifications:**
- Resolution: 0.1 N per sensor
- Sensors per finger: tip (3x3), nail (12x8), pad (10x8)
- Thumb additional: middle section (3x3)
- Palm: 8x14 grid
- Raw value range: 0-4095 (12-bit ADC)
- Total taxels: ~1500 per hand

### 6. Hand State (`hand_state.json`)

Actuator feedback from the 6-DOF hand (100 Hz sampling rate).

| Field | Type | Range | Unit | Description |
|-------|------|-------|------|-------------|
| `position_actual` | int16[6] | 0-2000 | - | Actuator stroke position |
| `angle_actual` | int16[6] | 0-1000 | - | Normalized finger angle |
| `force_actual` | int16[6] | -4000 to 4000 | grams | Grip force per finger |
| `current` | int16[6] | 0-2000 | mA | Motor current |
| `status` | uint8[6] | 0-7 | - | Status code per DOF |
| `error` | uint8[6] | 0-31 | - | Error bitfield per DOF |
| `temperature` | uint8[6] | 0-100 | Celsius | Actuator temperature |

**DOF Mapping:**
- DOF 0: Little finger
- DOF 1: Ring finger
- DOF 2: Middle finger
- DOF 3: Index finger
- DOF 4: Thumb bend
- DOF 5: Thumb rotation

**Angle Physical Mapping:**
| DOF | Angle 0 | Angle 1000 | Physical Range |
|-----|---------|------------|----------------|
| Little/Ring/Middle/Index | Fully bent | Fully open | 20° - 176° |
| Thumb bend | Fully bent | Fully open | -13° - 70° |
| Thumb rotation | Minimum | Maximum | 90° - 165° |

**Status Codes:**
| Value | Meaning |
|-------|---------|
| 0 | Releasing (opening) |
| 1 | Grasping (moving to target) |
| 2 | Position reached (no object) |
| 3 | Force reached (object grasped) |
| 5 | Overcurrent protection |
| 6 | Stalled |
| 7 | Fault |

**Error Bits:**
| Bit | Meaning |
|-----|---------|
| 0 | Locked-rotor/stall |
| 1 | Over temperature |
| 2 | Overcurrent |
| 3 | Motor abnormal |
| 4 | Communication error |

### 7. Joint State (`joint_state.json`)

Robot joint angles for kinematic model.

| Field | Type | Description |
|-------|------|-------------|
| `name` | string[12] | Joint names |
| `position` | float[12] | Joint angles in radians |
| `velocity` | float[] | Angular velocities (if available) |
| `effort` | float[] | Joint torques (if available) |

**Joint Names:**
- `right_little_1_joint`, `right_little_2_joint`
- `right_ring_1_joint`, `right_ring_2_joint`
- `right_middle_1_joint`, `right_middle_2_joint`
- `right_index_1_joint`, `right_index_2_joint`
- `right_thumb_1_joint`, `right_thumb_2_joint`
- `right_thumb_3_joint`, `right_thumb_4_joint`

## Directory Structure

```
training_data/
├── README.md
├── deformable/                      # 83 bags, 2,281 samples
│   ├── record_250_empty_1/
│   │   ├── sample_0000/
│   │   │   ├── camera_rgb.png
│   │   │   ├── camera_pointcloud.pcd
│   │   │   ├── tactile_colormap.png
│   │   │   ├── tactile_pointcloud.pcd
│   │   │   ├── tactile_pointcloud_raw.pcd
│   │   │   ├── hand_state.json
│   │   │   └── joint_state.json
│   │   ├── sample_0001/
│   │   └── ...
│   └── ...
└── non_deformable/                  # 115 bags, 3,856 samples
    ├── record_250_1/
    └── ...
```

## Modality Completeness

**Camera Point Cloud Gap:** 306 samples (5.0%) missing `camera_pointcloud.pcd` due to sparse depth camera recording in source ROS2 bags. These 8 bags had no depth data published during collection:
- non_deformable/record_mid_bottle_8: 41 missing
- non_deformable/record_330_fat_12: 39 missing
- non_deformable/record_250_1: 30 missing
- deformable/record_mid_bottle_empty_11: 30 missing
- non_deformable/record_mid_bottle_10: 29 missing
- deformable/record_big_bottle_empty_15: 29 missing
- non_deformable/record_big_bottle_16: 27 missing
- non_deformable/record_small_bottle_12: 22 missing

**Why unfixable:** Source depth data doesn't exist in bags (despite metadata claims). Re-processing cannot recover missing source data.

## ROS2 Topics Recorded

| Topic | Message Type | Rate |
|-------|-------------|------|
| `/camera/camera/color/image_raw` | sensor_msgs/Image | ~10 Hz |
| `/camera/camera/depth/image_rect_raw` | sensor_msgs/Image | ~20 Hz |
| `/camera/camera/depth/camera_info` | sensor_msgs/CameraInfo | ~20 Hz |
| `/cylinder_projection/unwrapped_colormap` | sensor_msgs/Image | ~10 Hz |
| `/inspire_hand/inspire_hand_node/state` | InspireHandState | ~30 Hz |
| `/joint_states` | sensor_msgs/JointState | ~10 Hz |
| `/inspire_hand/tactile_pointcloud` | sensor_msgs/PointCloud2 | ~10 Hz |

## Hardware

- **Hand**: Beijing Inspire-Robots RH56DFTP Dexterous Hand
- **DOF**: 6 actuated degrees of freedom
- **Force Sensors**: 6 (one per DOF), 0.1 N resolution
- **Tactile Sensors**: 5 fingers + palm, ~1500 total taxels
- **Gripping Force**: Up to 30 N (five-finger)
- **Camera**: Intel RealSense D435 (RGB + Depth)

## Collection Parameters

| Parameter | Value |
|-----------|-------|
| Sample Interval | 0.081 seconds |
| Samples per Bag | ~30 |
| Bag Duration | ~5 seconds |
| QoS Profile | BEST_EFFORT, KEEP_LAST, depth=10 |
| Depth Voxel Size | 1cm (CUDA downsampling) |

## Usage

Visualize a sample:
```bash
./scripts/visualize_sample.py training_data/non_deformable/record_250_1/sample_0000
```

**Visualizer Controls:**
- **A** - Toggle RGB/Intensity coloring for point clouds
- **R** - Toggle Filtered/Raw tactile point cloud
- **Q** - Quit current view

Load in Python:
```python
import json
import cv2
import open3d as o3d

# Load images
camera = cv2.imread('sample_0000/camera_rgb.png')
tactile = cv2.imread('sample_0000/tactile_colormap.png')

# Load JSON
with open('sample_0000/hand_state.json') as f:
    hand = json.load(f)

# Load point clouds
tactile_pcd = o3d.io.read_point_cloud('sample_0000/tactile_pointcloud.pcd')
camera_pcd = o3d.io.read_point_cloud('sample_0000/camera_pointcloud.pcd')
```
