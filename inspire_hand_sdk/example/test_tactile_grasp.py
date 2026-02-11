#!/usr/bin/env python3
"""
Test script to close/open hands for tactile sensor testing.
Run the dds_subscribe.py in another terminal to see tactile data.

Usage:
    python test_tactile_grasp.py
"""
import sys
sys.path.insert(0, "/home/analog/develop/inspire_hand_ws/unitree_sdk2_python")
sys.path.insert(0, "/home/analog/develop/inspire_hand_ws/inspire_hand_sdk")

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_dds, inspire_hand_defaut
import time

print("Initializing DDS on domain 1, interface enp0s31f6...")
ChannelFactoryInitialize(1, "enp0s31f6")

# Create publishers for both hands
pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
pub_r.Init()
pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
pub_l.Init()
print("Publishers ready")

cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
cmd.mode = 0b0001  # Angle control mode

def send_command(angle, duration_sec=2.0, description=""):
    """Send angle command to both hands for specified duration."""
    print(f"\n>>> {description} (angle={angle}, {duration_sec}s)")
    cmd.angle_set = [angle] * 6
    iterations = int(duration_sec / 0.02)
    for i in range(iterations):
        pub_r.Write(cmd)
        pub_l.Write(cmd)
        time.sleep(0.02)

print("\n" + "="*50)
print("TACTILE SENSOR TEST - Grasp Sequence")
print("="*50)
print("Make sure the cylinder is positioned in front of the hands.")
print("Run dds_subscribe.py in another terminal to see tactile data.")
print("="*50)

input("\nPress ENTER to start the grasp sequence...")

# Open hands first
send_command(900, 2.0, "OPENING hands (preparing to grasp)")

# Slowly close to grasp
send_command(500, 2.0, "PARTIALLY CLOSING (approaching cylinder)")

# Close more to make contact
send_command(300, 3.0, "CLOSING MORE (should contact cylinder - check tactile!)")

# Grip firmly
send_command(150, 3.0, "FIRM GRIP (maximum contact - tactile values should be high)")

# Hold and let user observe
print("\n>>> HOLDING GRIP - Check tactile sensor values now!")
cmd.angle_set = [150] * 6
for i in range(150):  # 3 seconds
    pub_r.Write(cmd)
    pub_l.Write(cmd)
    time.sleep(0.02)

input("\nPress ENTER to release...")

# Release
send_command(900, 2.0, "RELEASING (opening hands)")

print("\n" + "="*50)
print("Test complete!")
print("="*50)
