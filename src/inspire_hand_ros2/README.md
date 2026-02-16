# Inspire Hand ROS2

ROS2 Humble wrapper for the **RH56DFTP Dexterous Hand** by Beijing Inspire-Robots with human-inspired grasp control.

## Features

- **6-DOF Control**: Position, angle, force, and speed control for all fingers
- **1062 Tactile Sensors**: Full tactile feedback from fingertips, finger pads, and palm
- **Human-Inspired Grasping**: Based on Romano et al. (IEEE TRO 2011)
  - Weber's law slip detection (12% threshold)
  - Adaptive force compensation
  - 6-state grasp controller
- **Grasp Presets**: Power, pinch, precision, cylindrical, hook
- **Action Server**: Real-time feedback during grasp operations
- **Qt Visualizer**: Real-time tactile heatmaps and state curves

## Hardware Requirements

- RH56DFTP Dexterous Hand
- Ethernet connection (default IP: 192.168.123.211, port: 6000)
- 24V power supply

## Installation

### Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Python dependencies
pip install pymodbus numpy

# For visualizer (optional)
pip install pyqtgraph colorcet PyQt5
```

### Build

```bash
cd ~/develop/inspire_hand_ws
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select inspire_hand_ros2

# Source the workspace
source install/setup.bash
```

## Quick Start

### Launch the Node

```bash
ros2 launch inspire_hand_ros2 inspire_hand.launch.py
```

With custom parameters:

```bash
ros2 launch inspire_hand_ros2 inspire_hand.launch.py \
  hand_ip:=192.168.123.211 \
  default_force:=600
```

### Execute a Grasp

```bash
# Power grasp (closes all fingers)
ros2 service call /inspire_hand/inspire_hand_node/grasp inspire_hand_ros2/srv/Grasp \
  "{grasp_type: 'power', target_force: 500}"

# Pinch grasp (thumb + index)
ros2 service call /inspire_hand/inspire_hand_node/grasp inspire_hand_ros2/srv/Grasp \
  "{grasp_type: 'pinch', target_force: 300}"

# Release (open hand)
ros2 service call /inspire_hand/inspire_hand_node/release inspire_hand_ros2/srv/Release \
  "{speed: 0.5}"
```

### Direct Control

```bash
# Set specific angles (0=closed, 1000=open)
ros2 topic pub --once /inspire_hand/inspire_hand_node/control inspire_hand_ros2/msg/InspireHandControl \
  "{angle_set: [500, 500, 500, 500, 500, 500], force_set: [500, 500, 500, 500, 500, 500], speed_set: [500, 500, 500, 500, 500, 500]}"
```

### Monitor Topics

```bash
# Hand state (100 Hz)
ros2 topic echo /inspire_hand/inspire_hand_node/state

# Tactile data (50 Hz)
ros2 topic echo /inspire_hand/inspire_hand_node/touch

# Grasp status
ros2 topic echo /inspire_hand/inspire_hand_node/grasp_status
```

### Visualizer

Launch the Qt visualizer to see real-time tactile heatmaps and state curves:

```bash
# In a separate terminal (while node is running)
ros2 run inspire_hand_ros2 visualizer.py
```

## ROS2 Interface

**Node name**: `inspire_hand_node`
**Namespace**: `inspire_hand` (default)
**Full path prefix**: `/inspire_hand/inspire_hand_node/`

### Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/inspire_hand/inspire_hand_node/state` | `InspireHandState` | 100 Hz | Actuator feedback (position, angle, force, status) |
| `/inspire_hand/inspire_hand_node/touch` | `InspireHandTouch` | 50 Hz | Tactile sensor data (1062 values) |
| `/inspire_hand/inspire_hand_node/control` | `InspireHandControl` | - | Control commands (subscriber) |
| `/inspire_hand/inspire_hand_node/grasp_status` | `GraspStatus` | 50 Hz | Grasp controller state |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/inspire_hand/inspire_hand_node/grasp` | `Grasp` | Execute grasp preset |
| `/inspire_hand/inspire_hand_node/release` | `Release` | Release and open hand |
| `/inspire_hand/inspire_hand_node/set_finger_position` | `SetFingerPosition` | Direct finger control |
| `/inspire_hand/inspire_hand_node/get_state` | `GetHandState` | Get current state snapshot |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/inspire_hand/inspire_hand_node/grasp_object` | `GraspObject` | Grasp with real-time feedback |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hand_ip` | string | `192.168.123.211` | Hand IP address |
| `hand_port` | int | `6000` | Modbus TCP port |
| `device_id` | int | `1` | Hand device ID (1-254) |
| `state_rate` | double | `100.0` | State publish rate (Hz) |
| `touch_rate` | double | `50.0` | Tactile publish rate (Hz) |
| `enable_tactile` | bool | `true` | Enable tactile reading |
| `default_force` | int | `500` | Default grip force (grams) |
| `default_speed` | int | `500` | Default movement speed (0-1000) |
| `slip_compensation` | bool | `true` | Enable slip detection |
| `namespace` | string | `inspire_hand` | Node namespace |

## Grasp Presets

| Preset | Description | Finger Configuration |
|--------|-------------|----------------------|
| `power` | Full hand power grasp | All fingers closed (angle=0) |
| `pinch` | Thumb-index pinch | Index + thumb closed, others open |
| `precision` | Three-finger precision | Middle, index, thumb closed |
| `cylindrical` | Wrapped grasp | Partially closed wrap |
| `hook` | Four-finger hook | 4 fingers closed, thumb open |
| `open` | Fully open hand | All fingers extended (angle=1000) |

**Note**: angle=0 is CLOSED (fingers curled), angle=1000 is OPEN (fingers extended)

## Message Definitions

### InspireHandState.msg

```
std_msgs/Header header
int16[6] position_actual      # Actuator position (0-2000)
int16[6] angle_actual         # Finger angle (0-1000)
int16[6] force_actual         # Grip force in grams (-4000 to 4000)
int16[6] current              # Motor current in mA
uint8[6] status               # Status codes (0-7)
uint8[6] error                # Error bitfield
uint8[6] temperature          # Temperature in Celsius
```

**Status Codes:**
- `0`: Releasing
- `1`: Grasping
- `2`: Position reached
- `3`: Force reached (object grasped)
- `5`: Current protection
- `6`: Stall
- `7`: Fault

### InspireHandControl.msg

```
std_msgs/Header header
int16[6] angle_set            # Target angle (0-1000, -1=no change)
int16[6] force_set            # Force limit in grams (0-3000)
int16[6] speed_set            # Speed (0-1000, -1=no change)
```

### GraspObject.action

```
# Goal
string grasp_type             # Preset name or "custom"
int16 target_force            # Force in grams
int16[6] custom_angles        # For custom grasps
float32 speed                 # 0.0-1.0
bool use_slip_compensation    # Enable slip response
float32 timeout               # Timeout in seconds
---
# Result
bool success
string message
int16[6] final_force
uint8[6] final_status
float32 elapsed_time
---
# Feedback
GraspStatus status
float32 progress              # 0.0 to 1.0
float32 elapsed_time
```

## Finger Indices

| Index | Finger | Angle Range |
|-------|--------|-------------|
| 0 | Little | 0-1000 (20°-176°) |
| 1 | Ring | 0-1000 (20°-176°) |
| 2 | Middle | 0-1000 (20°-176°) |
| 3 | Index | 0-1000 (20°-176°) |
| 4 | Thumb bend | 0-1000 (-13°-70°) |
| 5 | Thumb rotate | 0-1000 (90°-165°) |

## Grasp Controller

The grasp controller implements a 6-state finite state machine based on human grasping behavior:

```
IDLE → CLOSING → LOADING → HOLDING → REPLACING → UNLOADING → OPENING
                              ↑
                              └── SLIP_DETECTED (force adjustment)
```

### Slip Detection (Weber's Law)

Slip is detected when the tactile rate of change exceeds 12% of the current tactile value:

```python
slip = abs(d_tactile/dt) > tactile_sum * 0.12
```

### Slip Compensation

On slip detection, grip force increases by 5%:

```python
new_force = current_force * 1.05
```

## Python API Example

```python
import rclpy
from rclpy.node import Node
from inspire_hand_ros2.msg import InspireHandControl, InspireHandState
from inspire_hand_ros2.srv import Grasp

class GraspExample(Node):
    def __init__(self):
        super().__init__('grasp_example')

        # Subscribe to state
        self.state_sub = self.create_subscription(
            InspireHandState,
            '/inspire_hand/inspire_hand_node/state',
            self.state_callback,
            10
        )

        # Service client
        self.grasp_client = self.create_client(
            Grasp, '/inspire_hand/inspire_hand_node/grasp'
        )

    def state_callback(self, msg):
        # Check if any finger reached force threshold
        for i, status in enumerate(msg.status):
            if status == 3:  # Force reached
                self.get_logger().info(f'Finger {i} grasping object')

    async def execute_grasp(self):
        request = Grasp.Request()
        request.grasp_type = 'precision'
        request.target_force = 400
        request.speed = 0.5
        request.use_slip_compensation = True

        result = await self.grasp_client.call_async(request)
        return result.success
```

## Tactile Sensor Layout

Each finger has three tactile regions:
- **Tip**: 3×3 = 9 taxels
- **Nail/Top**: 12×8 = 96 taxels
- **Pad**: 10×8 = 80 taxels (thumb: 12×8 = 96)

Thumb has an additional **middle** region (3×3 = 9 taxels).

Palm: 8×14 = 112 taxels

**Total: 1062 tactile sensors**

## Troubleshooting

### Connection Issues

```bash
# Check network connectivity
ping 192.168.123.211

# Verify port is open
nc -zv 192.168.123.211 6000
```

### Hand Not Responding

The driver runs calibration on startup. If a finger isn't moving:

```bash
# Get current state to check status/errors
ros2 service call /inspire_hand/inspire_hand_node/get_state inspire_hand_ros2/srv/GetHandState "{}"

# Or restart the node to re-run calibration
```

### Finger Not Moving

If a specific finger doesn't respond after startup, the calibration may need to be re-run. Restart the node or power cycle the hand.

### Slow Tactile Updates

Reduce tactile rate if bandwidth limited:

```bash
ros2 launch inspire_hand_ros2 inspire_hand.launch.py touch_rate:=20.0
```

### Visualizer Not Showing Data

Make sure the main node is running and publishing:
```bash
ros2 topic hz /inspire_hand/inspire_hand_node/state
ros2 topic hz /inspire_hand/inspire_hand_node/touch
```

## References

1. **Hardware**: RH56DFTP User Manual v1.0.0, Beijing Inspire-Robots
2. **Grasp Algorithm**: Romano, Hsiao, Niemeyer, Chitta, Kuchenbecker, "Human-Inspired Robotic Grasp Control With Tactile Sensing", IEEE Transactions on Robotics, 2011

## License

MIT License

## Authors

- Developed with Claude Code
