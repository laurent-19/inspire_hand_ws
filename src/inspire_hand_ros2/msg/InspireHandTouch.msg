# InspireHandTouch.msg - Complete tactile sensor data
# Published at 50 Hz with full tactile array data (1062 taxels)

std_msgs/Header header

# Per-finger tactile data
inspire_hand_ros2/FingerTactile little    # Little finger (185 taxels)
inspire_hand_ros2/FingerTactile ring      # Ring finger (185 taxels)
inspire_hand_ros2/FingerTactile middle    # Middle finger (185 taxels)
inspire_hand_ros2/FingerTactile index     # Index finger (185 taxels)
inspire_hand_ros2/FingerTactile thumb     # Thumb (210 taxels)

# Palm tactile (8x14 = 112 taxels)
int16[112] palm

# Aggregate metrics for grasp monitoring
float32 total_contact         # Sum of all tactile values
uint8 fingers_in_contact      # Bitfield: bit 0=little, 1=ring, 2=middle, 3=index, 4=thumb
bool stable_contact           # True if multiple fingers in contact
