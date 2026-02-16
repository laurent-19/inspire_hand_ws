# InspireHandControl.msg - Hand control commands
# Subscribe to this topic to control the hand

std_msgs/Header header

# Control setpoints (6 DOF)
# DOF 0-3: Little, Ring, Middle, Index fingers
# DOF 4: Thumb bend
# DOF 5: Thumb rotation
# Use -1 to indicate "no change" for a particular DOF
int16[6] angle_set            # Target angle (0-1000, -1=no change)
int16[6] force_set            # Force limit in grams (0-3000)
int16[6] speed_set            # Movement speed (0-1000, -1=no change)

# Angle values:
# 0 = Finger fully open
# 1000 = Finger fully closed/bent
