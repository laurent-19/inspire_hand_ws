# InspireHandState.msg - Hand actuator state feedback
# Published at 100 Hz from the hand hardware

std_msgs/Header header

# Actuator feedback (6 DOF)
# DOF 0-3: Little, Ring, Middle, Index fingers
# DOF 4: Thumb bend
# DOF 5: Thumb rotation
int16[6] position_actual      # Actuator position (0-2000)
int16[6] angle_actual         # Finger angle (0-1000)
int16[6] force_actual         # Grip force in grams (-4000 to 4000)
int16[6] current              # Motor current in mA (0-2000)

# Status and error information
uint8[6] status               # Status codes per DOF
uint8[6] error                # Error bitfield per DOF
uint8[6] temperature          # Actuator temperature in Celsius (0-100)

# Status code constants
uint8 STATUS_RELEASING = 0    # Hand is opening
uint8 STATUS_GRASPING = 1     # Moving toward target
uint8 STATUS_POSITION_REACHED = 2  # Target position reached (no object)
uint8 STATUS_FORCE_REACHED = 3     # Force threshold reached (object grasped)
uint8 STATUS_CURRENT_PROTECTION = 5  # Overcurrent protection
uint8 STATUS_STALL = 6        # Actuator stalled
uint8 STATUS_FAULT = 7        # Actuator fault

# Error bit definitions (bitfield)
uint8 ERROR_STALL = 1         # Bit 0: Locked-rotor/stall error
uint8 ERROR_OVERTEMP = 2      # Bit 1: Over temperature
uint8 ERROR_OVERCURRENT = 4   # Bit 2: Overcurrent
uint8 ERROR_MOTOR_ABNORMAL = 8  # Bit 3: Motor abnormal
uint8 ERROR_COMMUNICATION = 16  # Bit 4: Communication error
