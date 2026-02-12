#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Run Hand Grasping Policy on Real Inspire Hand

This script runs a trained ONNX grasping policy on the real Inspire Hand
using DDS communication.

Prerequisites:
- Trained ONNX policy (from export_hand_grasp_onnx.py)
- DDS communication working with the Inspire Hand
- Hand positioned near the object (arm manually positioned)

Usage:
    python run_grasp_policy.py --policy hand_grasp_policy.onnx
    python run_grasp_policy.py --policy hand_grasp_policy.onnx --hand r --duration 10.0
"""

import argparse
import time
import threading
import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("Error: onnxruntime not installed. Run: pip install onnxruntime")
    exit(1)

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from inspire_sdkpy import inspire_dds


def parse_args():
    parser = argparse.ArgumentParser(description="Run hand grasp policy on real hardware")
    parser.add_argument("--policy", type=str, required=True,
                        help="Path to ONNX policy file")
    parser.add_argument("--hand", type=str, default="r", choices=["l", "r"],
                        help="Hand side: 'l' (left) or 'r' (right)")
    parser.add_argument("--network", type=str, default=None,
                        help="Network interface (e.g., 'enp0s31f6')")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Maximum duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--speed", type=int, default=500,
                        help="Hand movement speed (0-1000)")
    parser.add_argument("--force", type=int, default=300,
                        help="Hand grip force limit (0-1000)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print debug information")
    return parser.parse_args()


class InspireGraspPolicy:
    """Run a trained grasping policy on the real Inspire Hand."""

    # Joint angle conversion constants
    # Inspire hand uses 0-1000 scale, simulation uses radians
    JOINT_RANGES = {
        'finger': (0.0, 1.7),      # indices 0,1,2,3
        'thumb_rot': (0.0, 0.5),   # index 5
        'thumb_flex': (-0.1, 1.3), # index 4
    }

    def __init__(
        self,
        policy_path: str,
        hand_side: str = 'r',
        network: str = None,
        speed: int = 500,
        force: int = 300,
        verbose: bool = False,
    ):
        """Initialize the policy runner.

        Args:
            policy_path: Path to ONNX policy file
            hand_side: 'l' or 'r'
            network: Network interface name
            speed: Movement speed (0-1000)
            force: Force limit (0-1000)
            verbose: Print debug info
        """
        self.hand_side = hand_side.lower()
        self.speed = speed
        self.force = force
        self.verbose = verbose

        # Load ONNX policy
        print(f"[policy] Loading ONNX policy: {policy_path}")
        self.session = ort.InferenceSession(policy_path)
        self.input_name = self.session.get_inputs()[0].name
        self.obs_dim = self.session.get_inputs()[0].shape[1]
        self.action_dim = self.session.get_outputs()[0].shape[1]
        print(f"[policy] obs_dim={self.obs_dim}, action_dim={self.action_dim}")

        # Initialize DDS
        print(f"[policy] Initializing DDS...")
        if network:
            ChannelFactoryInitialize(0, network)
        else:
            ChannelFactoryInitialize(0)

        # Setup publisher for control commands
        ctrl_topic = f"rt/inspire_hand/ctrl/{self.hand_side}"
        self.pub = ChannelPublisher(ctrl_topic, inspire_dds.inspire_hand_ctrl)
        self.pub.Init()
        print(f"[policy] Control publisher: {ctrl_topic}")

        # Setup subscriber for hand state
        state_topic = f"rt/inspire_hand/state/{self.hand_side}"
        self.sub = ChannelSubscriber(state_topic, inspire_dds.inspire_hand_state)
        self.sub.Init(self._state_callback, 10)
        print(f"[policy] State subscriber: {state_topic}")

        # State storage
        self.state_lock = threading.Lock()
        self.current_state = None
        self.last_state_time = 0

        # Running flag
        self.running = False

    def _state_callback(self, msg: inspire_dds.inspire_hand_state):
        """Callback for hand state updates."""
        with self.state_lock:
            self.current_state = {
                'angle_act': list(msg.angle_act),
                'force_act': list(msg.force_act),
                'pos_act': list(msg.pos_act),
            }
            self.last_state_time = time.time()

    def _get_joint_range(self, idx: int) -> tuple:
        """Get joint angle range for conversion."""
        if idx in [0, 1, 2, 3]:
            return self.JOINT_RANGES['finger']
        elif idx == 4:
            return self.JOINT_RANGES['thumb_rot']
        else:
            return self.JOINT_RANGES['thumb_flex']

    def _inspire_to_radians(self, inspire_val: int, idx: int) -> float:
        """Convert Inspire 0-1000 scale to radians."""
        min_val, max_val = self._get_joint_range(idx)
        normalized = np.clip(inspire_val / 1000.0, 0.0, 1.0)
        return (1.0 - normalized) * (max_val - min_val) + min_val

    def _radians_to_inspire(self, rad_val: float, idx: int) -> int:
        """Convert radians to Inspire 0-1000 scale."""
        min_val, max_val = self._get_joint_range(idx)
        normalized = np.clip((max_val - rad_val) / (max_val - min_val), 0.0, 1.0)
        return int(normalized * 1000)

    def _get_observation(self) -> np.ndarray:
        """Build observation vector from current state.

        The observation includes:
        - Hand joint positions (converted to radians)
        - Contact forces (converted to Newtons)
        - Object position relative to hand (estimated/zero)
        - Approach phase flag (always 1.0 on real robot)
        """
        with self.state_lock:
            if self.current_state is None:
                # Return zeros if no state yet
                return np.zeros((1, self.obs_dim), dtype=np.float32)

            state = self.current_state.copy()

        # Convert angles to radians (6 DOF)
        joint_positions = []
        for i in range(6):
            angle = state['angle_act'][i] if i < len(state['angle_act']) else 0
            joint_positions.append(self._inspire_to_radians(angle, i))

        # Convert forces to Newtons (grams / 102)
        contact_forces = []
        for i in range(6):
            force_grams = state['force_act'][i] if i < len(state['force_act']) else 0
            force_newtons = force_grams / 102.0  # grams to Newtons
            contact_forces.append(force_newtons)

        # Build observation vector
        # Note: The exact observation format depends on training config
        # This is a simplified version - may need adjustment
        obs = []

        # Add joint positions (12 for both hands in training, but we only have 6)
        obs.extend(joint_positions)
        obs.extend([0.0] * 6)  # Pad for left hand (or use actual if bilateral)

        # Add contact forces (tactile)
        obs.extend(contact_forces)

        # Add object position relative to hand (unknown on real robot - use zeros)
        obs.extend([0.0, 0.0, 0.0])

        # Add approach phase flag (always 1.0 - approach complete)
        obs.append(1.0)

        # Pad or truncate to match expected obs_dim
        if len(obs) < self.obs_dim:
            obs.extend([0.0] * (self.obs_dim - len(obs)))
        elif len(obs) > self.obs_dim:
            obs = obs[:self.obs_dim]

        return np.array([obs], dtype=np.float32)

    def _send_action(self, action: np.ndarray):
        """Send action to the hand via DDS.

        Args:
            action: Array of joint position targets in radians (6 DOF)
        """
        # Convert radians to Inspire scale
        angle_set = []
        for i in range(min(6, len(action))):
            inspire_val = self._radians_to_inspire(float(action[i]), i)
            angle_set.append(inspire_val)

        # Pad to 6
        while len(angle_set) < 6:
            angle_set.append(500)  # Default mid position

        # Build control message
        msg = inspire_dds.inspire_hand_ctrl(
            angle_set=angle_set,
            speed_set=[self.speed] * 6,
            force_set=[self.force] * 6,
            mode=1,  # Position control mode
        )

        self.pub.Write(msg)

        if self.verbose:
            print(f"[policy] Action: {angle_set}")

    def run(self, duration: float = 30.0, rate: float = 50.0):
        """Run the policy loop.

        Args:
            duration: Maximum run duration in seconds
            rate: Control loop rate in Hz
        """
        print(f"[policy] Starting policy loop (duration={duration}s, rate={rate}Hz)")
        print(f"[policy] Press Ctrl+C to stop")

        self.running = True
        dt = 1.0 / rate
        start_time = time.time()
        step = 0

        try:
            while self.running:
                loop_start = time.time()

                # Check duration
                elapsed = loop_start - start_time
                if elapsed > duration:
                    print(f"\n[policy] Duration limit reached ({duration}s)")
                    break

                # Check for recent state
                if loop_start - self.last_state_time > 1.0 and step > 10:
                    print("\n[policy] Warning: No state update in 1s")

                # Get observation
                obs = self._get_observation()

                # Run policy
                outputs = self.session.run(None, {self.input_name: obs})
                action = outputs[0][0]  # (1, action_dim) -> (action_dim,)

                # Send action
                self._send_action(action)

                # Status print
                if step % int(rate) == 0:
                    with self.state_lock:
                        if self.current_state:
                            forces = self.current_state['force_act'][:4]
                            angles = self.current_state['angle_act'][:4]
                            print(f"[policy] t={elapsed:.1f}s | "
                                  f"angles={angles} | forces={forces}")

                step += 1

                # Rate limiting
                sleep_time = dt - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[policy] Interrupted by user")
        finally:
            self.running = False
            print(f"[policy] Policy loop stopped after {step} steps")

    def open_hand(self):
        """Open the hand (release grasp)."""
        print("[policy] Opening hand...")
        msg = inspire_dds.inspire_hand_ctrl(
            angle_set=[1000, 1000, 1000, 1000, 1000, 500],  # Fully open + thumb neutral
            speed_set=[500] * 6,
            force_set=[300] * 6,
            mode=1,
        )
        self.pub.Write(msg)

    def close_hand(self):
        """Close the hand (simple grasp)."""
        print("[policy] Closing hand...")
        msg = inspire_dds.inspire_hand_ctrl(
            angle_set=[0, 0, 0, 0, 0, 0],  # Fully closed
            speed_set=[300] * 6,
            force_set=[self.force] * 6,
            mode=1,
        )
        self.pub.Write(msg)


def main():
    args = parse_args()

    # Create policy runner
    policy = InspireGraspPolicy(
        policy_path=args.policy,
        hand_side=args.hand,
        network=args.network,
        speed=args.speed,
        force=args.force,
        verbose=args.verbose,
    )

    # Wait for initial state
    print("[main] Waiting for hand state...")
    for i in range(50):  # Wait up to 5 seconds
        time.sleep(0.1)
        if policy.current_state is not None:
            print("[main] Hand state received!")
            break
    else:
        print("[main] Warning: No hand state received, continuing anyway...")

    # Open hand before starting
    policy.open_hand()
    time.sleep(1.0)

    # Run policy
    policy.run(duration=args.duration, rate=args.rate)

    # Open hand after finishing
    policy.open_hand()
    print("[main] Done!")


if __name__ == "__main__":
    main()
