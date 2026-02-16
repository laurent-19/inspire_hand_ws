# GraspStatus.msg - Current grasp operation status
# Published during grasp operations

std_msgs/Header header

# Current state of the grasp controller
uint8 state

# State constants
uint8 STATE_IDLE = 0          # No active grasp operation
uint8 STATE_CLOSING = 1       # Fingers closing toward object
uint8 STATE_LOADING = 2       # Applying target grip force
uint8 STATE_HOLDING = 3       # Stable grasp, monitoring for slip
uint8 STATE_REPLACING = 4     # Placing object down
uint8 STATE_UNLOADING = 5     # Releasing grip force
uint8 STATE_OPENING = 6       # Opening hand
uint8 STATE_SLIP_DETECTED = 7 # Slip detected, adjusting force

# Grasp status
bool object_detected          # True if object contact detected (STATUS=3 on any finger)
bool grasp_stable             # True if grasp is stable (HOLD state)
string grasp_type             # Current grasp type: "power", "pinch", "precision", etc.

# Force feedback
int16[6] grip_force           # Current grip force per finger (grams)
int16 target_force            # Target grip force (grams)

# Tactile feedback
float32[6] tactile_contact    # Tactile sum per finger
bool[6] finger_contact        # Per-finger contact status
bool slip_detected            # True if slip was detected
