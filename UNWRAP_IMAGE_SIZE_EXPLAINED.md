# Cylinder Unwrap Image Size Calculation

## Overview

The unwrapped image size is determined by **configurable parameters** that map the 3D cylinder surface to a 2D rectangular image. The mapping preserves spatial relationships while creating a flat visualization.

---

## Default Configuration

### Parameters (from launch file)

```yaml
image_width: 360        # pixels
image_height: 200       # pixels
height_min: -0.075      # meters
height_max: 0.075       # meters
cylinder_radius: 0.02   # meters (must match cylinder_projection)
```

### Resulting Image Dimensions

**Output**: 360 × 200 pixel image (72,000 pixels total)

---

## Image Width Calculation

### Angular Resolution

**Default: 360 pixels = 360° around cylinder**

```
Resolution: 1 pixel per degree
Angular coverage: 360° (full wraparound)
Pixel width at cylinder surface: (2π × radius) / width
                                = (2π × 0.02 m) / 360 pixels
                                = 0.126 m / 360
                                ≈ 0.35 mm/pixel
```

### Mapping Formula

```python
# Angle (0-360°) → Column index (0 to width-1)
col_index = int((angle_unwrapped / 360.0) * image_width)
```

**Example mappings** (with width=360):
- 0° → column 0
- 90° → column 90
- 180° → column 180
- 270° → column 270
- 359.5° → column 359

### Physical Interpretation

For a cylinder with **radius = 0.02 m (20 mm)**:

```
Circumference = 2π × 0.02 m = 0.1257 m ≈ 125.7 mm
Pixel spacing = 125.7 mm / 360 pixels ≈ 0.35 mm/pixel
```

**Each column** represents a thin vertical strip around the cylinder, 0.35 mm wide.

---

## Image Height Calculation

### Height Range

**Default: -0.075 m to +0.075 m (150 mm total height)**

```
Height range = height_max - height_min
             = 0.075 - (-0.075)
             = 0.15 m = 150 mm

With 200 pixels:
Resolution = 150 mm / 200 pixels = 0.75 mm/pixel
```

### Mapping Formula

```python
# Height (meters) → Row index (0 to height-1)
height_range = height_max - height_min  # 0.15 m
row_index = int((height - height_min) / height_range * image_height)
row_index = clip(row_index, 0, image_height - 1)
```

**Example mappings** (with height=200, range -0.075 to +0.075):
- height = +0.075 m → row 0 (top of image)
- height = 0.000 m → row 100 (middle)
- height = -0.075 m → row 199 (bottom of image)

### Physical Interpretation

**Each row** represents a horizontal ring around the cylinder, 0.75 mm tall.

---

## Complete Pixel-to-Physical Mapping

### Image Pixel → Cylinder Surface

For a pixel at `(row, col)`:

```python
# Compute angle (0-360°)
angle_unwrapped = (col / image_width) * 360.0

# Shift back to physical angle (0° = palm, 180° = cut)
angle_physical = (angle_unwrapped - 180.0) % 360.0

# Compute height along cylinder axis (meters)
height = height_min + (row / image_height) * (height_max - height_min)

# Convert to 3D coordinates in cylinder frame
# (assuming cylinder axis = X, centered at Y=0, Z=radius)
x = height
y = radius * cos(angle_physical - 90°)
z = radius + radius * sin(angle_physical - 90°)
```

### Example: Center Pixel

**Pixel (100, 180)** → center of image

```
Column 180:
  angle_unwrapped = (180 / 360) * 360° = 180°
  angle_physical = (180 - 180) % 360 = 0° (palm contact)

Row 100:
  height = -0.075 + (100 / 200) * 0.15 = 0.0 m (middle)

3D position (in cylinder frame):
  x = 0.0 m (middle height)
  y = 0.0 m (at palm)
  z = 0.0 m (contact point)
```

This is the **center of the palm contact point at mid-height** - the most important grasp contact!

---

## Resolution Trade-offs

### Higher Resolution (more pixels)

**Pros:**
- Finer spatial detail
- Smoother visualization
- Better for small object contact analysis

**Cons:**
- Larger file sizes
- More computation time
- May exceed tactile sensor resolution

### Current Resolution Analysis

**Width: 360 pixels (1°/pixel, 0.35 mm/pixel)**
- Good match for tactile sensor spatial resolution
- Each taxel covers ~few pixels
- Sufficient for grasp pattern analysis

**Height: 200 pixels (0.75 mm/pixel)**
- Good coverage for typical grasp height
- Matches hand palm length
- Sufficient detail for contact distribution

---

## Adjusting Image Size

### For Higher Resolution

```bash
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py \
    image_width:=720 \
    image_height:=400
```

**New resolution:**
- Width: 720 pixels → 0.5°/pixel, 0.17 mm/pixel
- Height: 400 pixels → 0.375 mm/pixel

### For Larger Coverage

```bash
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py \
    height_min:=-0.10 \
    height_max:=0.10
```

**New coverage:**
- Height range: 200 mm (was 150 mm)
- Resolution: 200 mm / 200 pixels = 1 mm/pixel

### For Different Cylinder Size

If you change the cylinder radius in `cylinder_projection.launch.py`:

```bash
ros2 launch inspire_hand_ros2 cylinder_projection.launch.py \
    cylinder_radius:=0.03
```

You must also update the unwrap node:

```bash
ros2 launch inspire_hand_ros2 cylinder_unwrap.launch.py \
    cylinder_radius:=0.03
```

**New circumference:**
- radius = 0.03 m → circumference = 188.5 mm
- With width=360: 188.5 / 360 ≈ 0.52 mm/pixel

---

## Pixel Density vs Tactile Sensor Resolution

### Tactile Sensor Grid (typical finger pad)

```
Sensor: 10 rows × 8 columns = 80 taxels
Physical size: ~20 mm × 15 mm
Taxel spacing: ~2 mm × 1.9 mm
```

### Image Sampling

With `points_per_taxel=16` (from mesh sampler):
```
Each taxel → 16 points → ~16 pixels in unwrapped image
Taxel coverage: ~2 mm → ~6 pixels (at 0.35 mm/pixel)
```

**Result:** Image resolution is **higher** than sensor resolution, providing smooth interpolation between taxels.

---

## Memory and Performance

### Image Size Impact

**360 × 200 image:**

```
Raw intensity (32FC1): 360 × 200 × 4 bytes = 288 KB/frame
Colormap RGB (8UC3):   360 × 200 × 3 bytes = 216 KB/frame
Total: ~0.5 MB/frame

At 50 Hz: ~25 MB/s
```

**720 × 400 image (4x pixels):**

```
Raw intensity: 1.15 MB/frame
Colormap RGB:  864 KB/frame
Total: ~2 MB/frame

At 50 Hz: ~100 MB/s
```

### Recommended Settings

| Use Case | Width | Height | Reasoning |
|----------|-------|--------|-----------|
| Real-time viz | 360 | 200 | Default, good balance |
| High-detail analysis | 720 | 400 | 4x resolution, smooth |
| Recording/export | 360 | 200 | Manageable file sizes |
| Publications | 720 | 400 | High quality images |
| Large cylinders | 540 | 300 | More detail needed |

---

## Summary

The image size is computed from:

1. **Angular coverage**: 360° around cylinder → `image_width` columns
2. **Height coverage**: `(height_max - height_min)` → `image_height` rows
3. **Pixel resolution**:
   - Width: `circumference / image_width` ≈ 0.35 mm/pixel (default)
   - Height: `height_range / image_height` ≈ 0.75 mm/pixel (default)

**Key formula:**
```python
pixel_size_width = (2π × cylinder_radius) / image_width
pixel_size_height = (height_max - height_min) / image_height
```

The default 360×200 provides **good balance** between:
- Spatial detail (sub-millimeter resolution)
- Computational efficiency (50 Hz real-time)
- File size (manageable for recording)
- Visual quality (smooth heatmaps)
