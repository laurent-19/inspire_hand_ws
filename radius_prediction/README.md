# Radius Prediction from Grasp Data

Neural network models for predicting the radius of grasped cylindrical objects using multi-modal sensor data from the RH56DFTP dexterous hand.

## Overview

This package implements two complementary approaches:

1. **Model 1 (Joint-based)**: Predicts radius from 12 joint angles (optionally fused with RGB image)
2. **Model 2 (PointNet)**: Predicts radius from 3D tactile point cloud data

Both models are trained with a **leave-one-object-out** strategy to test generalization to unseen object types.

## Models

### Model 1: Joint States → Radius

**Architecture:**
```
Joint Angles (12) → MLP(64→128→256→128) → FC(1) → ReLU → Radius (mm)
```

**Characteristics:**
- **Input**: 12 joint angles (normalized by π)
- **Output**: Radius in mm (24-46mm range)
- **Parameters**: 76,353
- **Training time**: ~5 minutes on GPU
- **Performance**: ~4.5mm RMSE

**Optional Fusion Mode** (use `--use_image` flag):
```
RGB (224×224) → ResNet18 → 512-dim
                              ↓
Joint (12) → MLP(64→128) → Concat(640) → FC(256→128) → FC(1) → Radius
```
- **Parameters**: 11.4M (with pretrained backbone)
- **Training time**: ~20 minutes

### Model 2: Tactile Point Cloud → Radius

**Architecture** (PointNet-based):
```
Tactile PCD (1024×4) → TNet(4×4) → MLP(64) → TNet(64×64)
                                      ↓
                               MLP(128→1024) → MaxPool
                                      ↓
                               FC(512→256) → FC(1) → Radius
```

**Characteristics:**
- **Input**: 1024 points with xyz + intensity (pressure)
- **Output**: Radius in mm
- **Parameters**: 3,463,377
- **Training time**: ~3-4 hours on GPU
- **Performance**: ~8-9mm RMSE (needs more epochs)

**Key features:**
- Farthest Point Sampling (FPS) for uniform coverage
- Input and feature transformation networks (TNet)
- Feature transform regularization: ||T^T·T - I||
- Data augmentation: rotation, scale, translation, dropout

## Dataset

### Structure
```
training_data/non_deformable/
├── record_250_1/              # 24.0mm radius cylinder
├── record_330_fat_1/          # 33.0mm radius (fat variant)
├── record_330_slim_1/         # 29.0mm radius (slim variant)
├── record_500_1/              # 33.0mm radius cylinder
├── record_small_bottle_1/     # 32.5mm radius
├── record_mid_bottle_1/       # 39.5mm radius
└── record_big_bottle_1/       # 46.0mm radius
```

Each sample contains:
- `camera_rgb.png` - 1280×720 RGB image
- `joint_state.json` - 12 joint angles in radians
- `tactile_pointcloud.pcd` - 3D tactile point cloud (~7500-10000 points)

### Statistics
- **Total samples**: 3,858 (non-deformable only)
- **Radius range**: 24-46mm
- **Split strategy**: Leave-one-object-out (6 train, 1 val)
- **Why non-deformable?** Deformable (empty) objects change radius during grasp

### Label Mapping
```python
OBJECT_TO_RADIUS = {
    "250": 24.0,           # mm
    "330_fat": 33.0,
    "330_slim": 29.0,
    "500": 33.0,
    "small_bottle": 32.5,
    "mid_bottle": 39.5,
    "big_bottle": 46.0,
}
```

## Installation

### Prerequisites
```bash
cd ~/develop/inspire_hand_ws
source venv/bin/activate
pip install torch torchvision tqdm
```

### GPU Requirements
- CUDA-capable GPU (tested on RTX 4000 Ada, 21GB)
- PyTorch with CUDA 12.4+ support

## Training

### Model 1: Joint-based (Recommended for Quick Results)

**Basic training** (joint-only):
```bash
python -m radius_prediction.train_joint \
  --val_object mid_bottle \
  --epochs 100
```

**With image fusion**:
```bash
python -m radius_prediction.train_joint \
  --use_image \
  --val_object mid_bottle \
  --epochs 50
```

**Arguments:**
- `--val_object`: Object to hold out for validation (default: mid_bottle)
  - Choices: 250, 330_fat, 330_slim, 500, small_bottle, mid_bottle, big_bottle
- `--use_image`: Add RGB image input (fusion mode)
- `--epochs`: Number of training epochs
- `--batch_size`: Batch size (default: 32 for joint-only, 16 for fusion)
- `--lr`: Learning rate (default: 1e-3 for joint-only, 1e-4 for fusion)

**Expected output:**
```
Epoch   5/100 | Train Loss: 0.0440 | Val Loss: 0.0674 | Val RMSE: 4.54mm | Val MAE: 3.57mm
  -> Saved best model (RMSE: 4.54mm)
```

### Model 2: PointNet (Better for Tactile Analysis)

**Optimized training** (recommended):
```bash
python -m radius_prediction.train_pointcloud_fast \
  --val_object mid_bottle \
  --epochs 100
```

**Arguments:**
- `--val_object`: Object to hold out
- `--num_points`: Number of points to sample (default: 1024)
  - Use 512 for faster training, 2048 for higher accuracy
- `--epochs`: Number of epochs
- `--batch_size`: Batch size (default: 32)
- `--no_intensity`: Don't use intensity channel (xyz only)

**Optimizations:**
- 16 workers for parallel data loading
- Persistent workers to avoid respawn overhead
- Prefetch factor of 4 for better GPU utilization

**Expected time:**
- ~2-3 min/epoch with 1024 points
- ~3-4 hours for full training (100 epochs)

### Training Tips

**Early stopping:**
- Both models use early stopping (patience=100)
- Training stops if validation RMSE doesn't improve for 100 epochs

**Monitoring:**
- Models save checkpoints to `radius_prediction/checkpoints/`
- Best model: `best_{model_type}_{val_object}.pt`

**GPU utilization:**
- Joint model: ~10-20% GPU usage (CPU-bound from image loading)
- PointNet: ~30-50% GPU usage (CPU-bound from FPS)

## Inference

### Quick Testing (Helper Scripts)

**Single sample:**
```bash
# Using PointNet
./test_prediction.sh training_data/non_deformable/record_250_1/sample_0000 pointnet

# Using Joint model
./test_prediction.sh training_data/non_deformable/record_500_1/sample_0005 joint
```

**Batch testing:**
```bash
# Test on 5 samples from each object
./test_batch.sh pointnet 5

# Test joint model on 3 samples
./test_batch.sh joint 3
```

### Python API

**Using predict.py:**
```bash
python -m radius_prediction.predict \
  training_data/non_deformable/record_250_1/sample_0000 \
  --checkpoint radius_prediction/checkpoints/best_pointnet_mid_bottle.pt \
  --model_type pointnet
```

**Output:**
```
Sample: training_data/non_deformable/record_250_1/sample_0000
Model: pointnet
Device: cuda

============================================================
Ground Truth:  24.00 mm
Prediction:    28.45 mm
Error:         4.45 mm
============================================================

Model trained on: 30 epochs
Best Val RMSE: 5.03 mm
Best Val MAE:  3.51 mm
```

### Programmatic Use

```python
import torch
from pathlib import Path
from radius_prediction.models.joint_model import JointRadiusModel
from radius_prediction.data.joint_dataset import JointDataset

# Load model
device = torch.device('cuda')
checkpoint = torch.load('radius_prediction/checkpoints/best_joint_mid_bottle.pt')
model = JointRadiusModel(use_image=False)
model.load_state_dict(checkpoint['model_state_dict'])
model = model.to(device)
model.eval()

# Load sample
dataset = JointDataset(split='val', val_object='mid_bottle', use_image=False)
joints, target = dataset[0]
joints = joints.unsqueeze(0).to(device)

# Predict
with torch.no_grad():
    pred_mm = model(joints).item()

print(f"Predicted radius: {pred_mm:.2f} mm")
```

## Evaluation

**Full evaluation on validation set:**
```bash
python -m radius_prediction.evaluate \
  radius_prediction/checkpoints/best_pointnet_mid_bottle.pt \
  --model_type pointnet
```

**Output:**
```
Model: pointnet
Validation Object: mid_bottle
============================================================

Overall Metrics:
  RMSE:  5.03 mm
  MAE:   3.51 mm
  R2:    0.8234
  Acc@1mm: 12.5%
  Acc@2mm: 45.2%
  Acc@3mm: 67.8%

Per-Object Metrics:
  250               : MAE=4.45mm, RMSE=4.48mm, n=493
  330_fat           : MAE=4.56mm, RMSE=4.61mm, n=511
  500               : MAE=4.77mm, RMSE=4.82mm, n=665
  ...
```

## Logic & Approach

### Problem Formulation

**Goal:** Predict the radius of a grasped cylindrical object from sensor data

**Why this matters:**
- Object size estimation for grasp planning
- Quality control in manipulation tasks
- Shape inference from tactile feedback

**Challenge:** Generalization to unseen object types
- Training on 6 object types, testing on the 7th
- Tests if model learns radius from grasp patterns, not object-specific features

### Model Design Rationale

#### Model 1: Joint States
**Hypothesis:** Joint closure directly correlates with object radius
- Larger objects → joints less closed
- Smaller objects → joints more closed

**Why it works:**
- Direct mechanical relationship between radius and joint angles
- Fast and lightweight (76K parameters)
- Surprisingly effective: ~4.5mm error

**Limitation:**
- Doesn't use rich tactile information
- May struggle with deformable or irregularly shaped objects

#### Model 2: PointNet
**Hypothesis:** 3D tactile geometry encodes object curvature
- Point cloud reflects object surface shape
- Intensity (pressure) provides contact information

**Architecture choice:**
- PointNet: Permutation-invariant to point order
- TNet: Learns canonical alignment of tactile data
- Feature transform: Aligns features in learned space

**Why tactile?**
- Direct contact with object surface
- Captures local curvature information
- Pressure distribution varies with radius

**Limitation:**
- Requires more data to converge
- Computationally expensive
- Sensitive to tactile sensor calibration

### Data Pipeline

**Preprocessing:**
1. **FPS (Farthest Point Sampling)**: Downsample ~7500 points → 1024
   - Ensures uniform spatial coverage
   - Reduces computation while preserving geometry
2. **Normalization**: Center and scale to [-1, 1]
   - Makes model invariant to hand position
3. **Augmentation** (training only):
   - Rotation (Z-axis): Orientation invariance
   - Scale (0.8-1.2): Size robustness
   - Translation (±0.1): Position invariance
   - Point dropout (0-50%): Occlusion robustness
   - Intensity noise: Sensor noise robustness

**Why leave-one-object-out?**
- Tests true generalization, not memorization
- Simulates real-world scenario: grasping new objects
- More challenging than random split (prevents data leakage)

### Loss Function

**Joint model:** MSE Loss + ReLU output
```python
loss = MSE(pred_radius, gt_radius)
output = ReLU(linear(features))  # Ensures positive radius
```

**PointNet:** MSE + Feature Transform Regularization
```python
loss = MSE(pred, target) + λ * ||T^T·T - I||²
```
- Regularization prevents transformation network from learning arbitrary (unstable) transforms
- λ = 0.001 (from PointNet paper)

### Expected Performance

| Model | Training Objects | Val Object | RMSE | MAE |
|-------|-----------------|------------|------|-----|
| Joint | 250, 330_fat, 330_slim, 500, small_bottle, big_bottle | mid_bottle | ~4.5mm | ~3.6mm |
| PointNet | Same | mid_bottle | ~8-9mm | ~7-8mm |

**Note:** PointNet improves with more training epochs (100+)

### Future Improvements

1. **Data augmentation:** Synthetic occlusions, sensor dropout patterns
2. **Architecture:** Try PointNet++, DGCNN for better local feature learning
3. **Multi-task learning:** Joint prediction of radius + shape class
4. **Ensemble:** Combine joint and tactile predictions
5. **Active learning:** Collect more data for challenging object sizes

## File Structure

```
radius_prediction/
├── README.md                    # This file
├── config.py                    # Configuration and hyperparameters
├── __init__.py
│
├── data/                        # Dataset classes
│   ├── base_dataset.py          # Leave-one-out split logic
│   ├── joint_dataset.py         # Joint + optional image
│   ├── pointcloud_dataset.py    # Tactile point cloud
│   ├── gpu_ops.py               # GPU-accelerated ops (experimental)
│   └── cache_pointclouds.py     # Pre-caching utility
│
├── models/                      # Neural network models
│   ├── joint_model.py           # Joint-based MLP
│   └── pointnet.py              # PointNet architecture
│
├── train_joint.py               # Training script for joint model
├── train_pointcloud.py          # Training script for PointNet
├── train_pointcloud_fast.py     # Optimized PointNet training
├── train_pointcloud_gpu.py      # GPU preprocessing (experimental)
│
├── evaluate.py                  # Model evaluation
├── predict.py                   # Single sample inference
├── predict_simple.py            # Simplified prediction
│
└── checkpoints/                 # Saved model weights
    ├── best_joint_mid_bottle.pt
    └── best_pointnet_mid_bottle.pt
```

## References

- **PointNet**: Charles et al., "PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation", CVPR 2017
- **PointNet++**: Qi et al., "PointNet++: Deep Hierarchical Feature Learning on Point Sets", NeurIPS 2017
- **ge2018**: Ge et al., "Point-to-Point Regression PointNet for 3D Hand Pose Estimation", ECCV 2018 (inspiration for tactile processing)

## Troubleshooting

**Issue: "CUDA out of memory"**
- Reduce batch size: `--batch_size 16` or `--batch_size 8`
- Reduce points: `--num_points 512`

**Issue: Training is very slow**
- Use `train_pointcloud_fast.py` instead of `train_pointcloud.py`
- Reduce `--num_points` to 512 or 1024
- Check GPU utilization with `nvidia-smi`

**Issue: Poor predictions**
- Ensure you trained for enough epochs (100+ recommended)
- Check validation object in checkpoint config
- Verify normalization range matches training data

**Issue: Model predicts constant value**
- Model needs more training
- Check learning rate (may be too low/high)
- Verify data is properly normalized

## Citation

```bibtex
@software{radius_prediction_2026,
  author = {Laurent-19 and Claude Opus 4.6},
  title = {Neural Network Models for Grasp Radius Prediction},
  year = {2026},
  institution = {Analog Devices}
}
```
