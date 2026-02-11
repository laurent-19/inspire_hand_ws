#!/usr/bin/env python3
"""
Test script to move arms towards cylinder and grasp.
Controls both arm joints (via LowCmd) and hand joints (via inspire_hand_ctrl).

Usage:
    python test_arm_grasp.py
"""
import sys
sys.path.insert(0, "/home/analog/develop/inspire_hand_ws/unitree_sdk2_python")
sys.path.insert(0, "/home/analog/develop/inspire_hand_ws/inspire_hand_sdk")

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, MotorCmd_
from unitree_sdk2py.utils.crc import CRC
from inspire_sdkpy import inspire_dds, inspire_hand_defaut
import time
import math

print("Initializing DDS on domain 1, interface enp0s31f6...")
ChannelFactoryInitialize(1, "enp0s31f6")

# Arm control publisher
pub_arm = ChannelPublisher("rt/lowcmd", LowCmd_)
pub_arm.Init()
print("Arm publisher ready (rt/lowcmd)")

# Hand control publishers
pub_hand_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
pub_hand_r.Init()
pub_hand_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
pub_hand_l.Init()
print("Hand publishers ready")

# CRC calculator
crc = CRC()

def create_arm_cmd(positions):
    """Create LowCmd_ message with joint positions.

    Joint order (29 joints total):
    0-14: leg/torso joints
    15-21: left arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw)
    22-28: right arm (same order)
    """
    # Create motor commands (35 motors total, but we use 29)
    motor_cmds = []
    for i in range(35):
        pos = positions[i] if i < len(positions) else 0.0
        motor = MotorCmd_(
            mode=1,           # Position mode
            q=float(pos),     # Position
            dq=0.0,           # Velocity
            tau=0.0,          # Torque
            kp=100.0,         # Position gain
            kd=10.0,          # Velocity gain
            reserve=0         # Reserved
        )
        motor_cmds.append(motor)

    # Create LowCmd with all required arguments
    cmd = LowCmd_(
        mode_pr=0,
        mode_machine=0,
        motor_cmd=motor_cmds,
        reserve=[0, 0, 0, 0],
        crc=0
    )

    # Calculate CRC
    cmd = LowCmd_(
        mode_pr=cmd.mode_pr,
        mode_machine=cmd.mode_machine,
        motor_cmd=cmd.motor_cmd,
        reserve=cmd.reserve,
        crc=crc.Crc(cmd)
    )
    return cmd

def send_hand_cmd(angle, pub_r, pub_l, right_only=True):
    """Send hand angle command."""
    hand_cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    hand_cmd.mode = 0b0001  # Angle mode
    hand_cmd.angle_set = [angle] * 6
    pub_r.Write(hand_cmd)
    if not right_only:
        pub_l.Write(hand_cmd)

# Default positions for all joints (zeros)
default_positions = [0.0] * 29

# Arm reaching pose - move RIGHT arm to the LEFT to reach cylinder
# Left arm indices: 15-21, Right arm indices: 22-28
# Joint order: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw

reach_positions = default_positions.copy()

# Left arm - keep at default (no movement)
# reach_positions[15-21] = 0.0

# Right arm - reach forward and to the LEFT towards cylinder at (y=0.20, z=0.85)
reach_positions[22] = 0.9    # right_shoulder_pitch - raise arm forward more
reach_positions[23] = 0.8    # right_shoulder_roll - move to the LEFT
reach_positions[24] = 0.4    # right_shoulder_yaw - rotate arm inward
reach_positions[25] = 1.3    # right_elbow - bend elbow more to reach forward
reach_positions[26] = 0.0    # right_wrist_roll
reach_positions[27] = -0.3   # right_wrist_pitch - angle wrist down more
reach_positions[28] = 0.0    # right_wrist_yaw

print("\n" + "="*50)
print("RIGHT ARM + HAND GRASP TEST")
print("Moving right arm to the LEFT to reach cylinder")
print("="*50)

# Reset robot to default pose at startup
print("\n>>> Resetting robot to default pose...")
for _ in range(100):
    cmd = create_arm_cmd(default_positions)
    pub_arm.Write(cmd)
    send_hand_cmd(500, pub_hand_r, pub_hand_l, right_only=True)  # Neutral hand position
    time.sleep(0.02)
print("Robot reset to default pose")

input("\nPress ENTER to move RIGHT arm towards cylinder...")

# Open right hand first
print("\n>>> Opening right hand...")
for _ in range(50):
    send_hand_cmd(900, pub_hand_r, pub_hand_l, right_only=True)
    time.sleep(0.02)

# Move right arm to reach position (to the left towards cylinder)
print("\n>>> Moving RIGHT arm to the LEFT to reach cylinder...")
for i in range(100):
    # Interpolate from default to reach position
    t = i / 100.0
    interp_positions = [
        default_positions[j] + t * (reach_positions[j] - default_positions[j])
        for j in range(29)
    ]
    cmd = create_arm_cmd(interp_positions)
    pub_arm.Write(cmd)
    send_hand_cmd(900, pub_hand_r, pub_hand_l, right_only=True)  # Keep hands open
    time.sleep(0.03)

print("Arms in reach position")
time.sleep(1)

input("\nPress ENTER to close hands (grasp)...")

# Close hands to grasp
print("\n>>> Closing hands to grasp...")
for angle in range(900, 100, -20):
    for _ in range(5):
        cmd = create_arm_cmd(reach_positions)
        pub_arm.Write(cmd)
        send_hand_cmd(angle, pub_hand_r, pub_hand_l, right_only=True)
        time.sleep(0.02)

print("Grasping! Check tactile sensors in dds_subscribe.py")

# Hold grip
print("\n>>> Holding grip for 5 seconds...")
for _ in range(250):
    cmd = create_arm_cmd(reach_positions)
    pub_arm.Write(cmd)
    send_hand_cmd(100, pub_hand_r, pub_hand_l, right_only=True)
    time.sleep(0.02)

input("\nPress ENTER to release and return...")

# Open hands
print("\n>>> Releasing...")
for _ in range(50):
    cmd = create_arm_cmd(reach_positions)
    pub_arm.Write(cmd)
    send_hand_cmd(900, pub_hand_r, pub_hand_l, right_only=True)
    time.sleep(0.02)

# Return arms to default
print("\n>>> Returning arms to default position...")
for i in range(100):
    t = i / 100.0
    interp_positions = [
        reach_positions[j] + t * (default_positions[j] - reach_positions[j])
        for j in range(29)
    ]
    cmd = create_arm_cmd(interp_positions)
    pub_arm.Write(cmd)
    send_hand_cmd(900, pub_hand_r, pub_hand_l, right_only=True)
    time.sleep(0.03)

print("\n" + "="*50)
print("Test complete!")
print("="*50)
