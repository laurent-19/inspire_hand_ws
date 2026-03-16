#!/usr/bin/env python3
"""
Publish test tactile data to simulate contact on the Inspire Hand.

Hardware tactile sensor range: 0-4095 (12-bit ADC, stored as 16-bit little-endian)

Usage:
    # Continuous random contact
    python3 publish_test_tactile.py

    # Simulated grasp pattern
    python3 publish_test_tactile.py --grasp

    # High pressure on index and thumb
    python3 publish_test_tactile.py --pinch
"""

import rclpy
from rclpy.node import Node
from inspire_hand_ros2.msg import InspireHandTouch, FingerTactile
import numpy as np
import argparse

# Hardware tactile sensor range (12-bit ADC)
TACTILE_MIN = 0
TACTILE_MAX = 4095


class TactileTestPublisher(Node):
    def __init__(self, pattern='random'):
        super().__init__('tactile_test_publisher')

        self.publisher = self.create_publisher(
            InspireHandTouch,
            '/inspire_hand/inspire_hand_node/touch',
            10
        )

        self.pattern = pattern
        self.timer = self.create_timer(0.05, self.publish_test_data)  # 20 Hz
        self.counter = 0

        self.get_logger().info(f'Publishing test tactile data with pattern: {pattern}')

    def create_finger_msg(self, name, tip_vals, nail_vals, pad_vals, middle_vals=None):
        """Create a FingerTactile message."""
        msg = FingerTactile()
        msg.name = name
        msg.tip = tip_vals
        msg.nail = nail_vals
        msg.pad = pad_vals
        msg.middle_section = middle_vals if middle_vals else []
        msg.total_pressure = float(np.sum(tip_vals + nail_vals + pad_vals))
        msg.max_pressure = float(np.max(tip_vals + nail_vals + pad_vals))
        msg.contact_count = int(np.sum(np.array(tip_vals + nail_vals + pad_vals) > 100))
        return msg

    def publish_test_data(self):
        """Publish test tactile data based on selected pattern."""
        msg = InspireHandTouch()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.pattern == 'random':
            msg = self.random_pattern()
        elif self.pattern == 'grasp':
            msg = self.grasp_pattern()
        elif self.pattern == 'pinch':
            msg = self.pinch_pattern()
        elif self.pattern == 'wave':
            msg = self.wave_pattern()
        else:
            msg = self.zero_pattern()

        self.publisher.publish(msg)
        self.counter += 1

    def random_pattern(self):
        """Random contact values across full 0-4095 range."""
        msg = InspireHandTouch()
        msg.header.stamp = self.get_clock().now().to_msg()

        for finger_name in ['little', 'ring', 'middle', 'index']:
            # Tips get higher pressure (0-3000)
            tip = [int(np.random.uniform(0, 3000)) for _ in range(9)]
            # Nails get medium pressure (0-2000)
            nail = [int(np.random.uniform(0, 2000)) for _ in range(96)]
            # Pads get lower pressure (0-1500)
            pad = [int(np.random.uniform(0, 1500)) for _ in range(80)]

            finger_msg = self.create_finger_msg(finger_name, tip, nail, pad)
            setattr(msg, finger_name, finger_msg)

        # Thumb (has middle section) - higher pressure range
        thumb_tip = [int(np.random.uniform(0, 4000)) for _ in range(9)]
        thumb_nail = [int(np.random.uniform(0, 2500)) for _ in range(96)]
        thumb_middle = [int(np.random.uniform(0, 2000)) for _ in range(9)]
        thumb_pad = [int(np.random.uniform(0, 2000)) for _ in range(96)]
        msg.thumb = self.create_finger_msg('thumb', thumb_tip, thumb_nail, thumb_pad, thumb_middle)

        # Palm - lower pressure (0-1500)
        msg.palm = [int(np.random.uniform(0, 1500)) for _ in range(112)]

        return msg

    def grasp_pattern(self):
        """Simulate a power grasp with high contact on fingertips and palm."""
        msg = InspireHandTouch()
        msg.header.stamp = self.get_clock().now().to_msg()

        # All fingers have high tip contact (~75% of max)
        for finger_name in ['little', 'ring', 'middle', 'index']:
            tip = [int(3000 + np.random.uniform(-300, 300)) for _ in range(9)]
            nail = [int(1500 + np.random.uniform(-200, 200)) for _ in range(96)]
            pad = [int(2000 + np.random.uniform(-200, 200)) for _ in range(80)]

            finger_msg = self.create_finger_msg(finger_name, tip, nail, pad)
            setattr(msg, finger_name, finger_msg)

        # Thumb with very high pressure (~90% of max on tip)
        thumb_tip = [int(3700 + np.random.uniform(-300, 300)) for _ in range(9)]
        thumb_nail = [int(2500 + np.random.uniform(-200, 200)) for _ in range(96)]
        thumb_middle = [int(2000 + np.random.uniform(-200, 200)) for _ in range(9)]
        thumb_pad = [int(3000 + np.random.uniform(-200, 200)) for _ in range(96)]
        msg.thumb = self.create_finger_msg('thumb', thumb_tip, thumb_nail, thumb_pad, thumb_middle)

        # High palm contact (~50% of max)
        msg.palm = [int(2000 + np.random.uniform(-300, 300)) for _ in range(112)]

        msg.total_contact = 100000.0
        msg.fingers_in_contact = 0b11111  # All fingers
        msg.stable_contact = True

        return msg

    def pinch_pattern(self):
        """Simulate precision pinch - only index and thumb have contact."""
        msg = InspireHandTouch()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Other fingers - no contact
        for finger_name in ['little', 'ring', 'middle']:
            tip = [0] * 9
            nail = [0] * 96
            pad = [0] * 80
            finger_msg = self.create_finger_msg(finger_name, tip, nail, pad)
            setattr(msg, finger_name, finger_msg)

        # Index finger - very high tip contact (~85% max)
        index_tip = [int(3500 + np.random.uniform(-200, 200)) for _ in range(9)]
        index_nail = [int(1500 + np.random.uniform(-100, 100)) for _ in range(96)]
        index_pad = [int(800 + np.random.uniform(-100, 100)) for _ in range(80)]
        msg.index = self.create_finger_msg('index', index_tip, index_nail, index_pad)

        # Thumb - maximum tip contact (~95% max)
        thumb_tip = [int(3900 + np.random.uniform(-100, 100)) for _ in range(9)]
        thumb_nail = [int(2000 + np.random.uniform(-100, 100)) for _ in range(96)]
        thumb_middle = [int(1200 + np.random.uniform(-100, 100)) for _ in range(9)]
        thumb_pad = [int(1000 + np.random.uniform(-100, 100)) for _ in range(96)]
        msg.thumb = self.create_finger_msg('thumb', thumb_tip, thumb_nail, thumb_pad, thumb_middle)

        # Minimal palm contact
        msg.palm = [int(np.random.uniform(0, 200)) for _ in range(112)]

        msg.total_contact = 30000.0
        msg.fingers_in_contact = 0b11000  # Index and thumb
        msg.stable_contact = True

        return msg

    def wave_pattern(self):
        """Animated wave pattern across fingers using full 0-4095 range."""
        msg = InspireHandTouch()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Create wave effect across full range
        phase = (self.counter * 0.1) % (2 * np.pi)

        for i, finger_name in enumerate(['little', 'ring', 'middle', 'index']):
            offset = i * np.pi / 2
            # Wave from 0 to 4095
            intensity = int(TACTILE_MAX * (0.5 + 0.5 * np.sin(phase + offset)))

            tip = [intensity] * 9
            nail = [int(intensity * 0.6)] * 96
            pad = [int(intensity * 0.4)] * 80

            finger_msg = self.create_finger_msg(finger_name, tip, nail, pad)
            setattr(msg, finger_name, finger_msg)

        # Thumb follows with delay
        thumb_intensity = int(TACTILE_MAX * (0.5 + 0.5 * np.sin(phase + 4 * np.pi / 2)))
        thumb_tip = [thumb_intensity] * 9
        thumb_nail = [int(thumb_intensity * 0.7)] * 96
        thumb_middle = [int(thumb_intensity * 0.5)] * 9
        thumb_pad = [int(thumb_intensity * 0.6)] * 96
        msg.thumb = self.create_finger_msg('thumb', thumb_tip, thumb_nail, thumb_pad, thumb_middle)

        # Palm waves at 50% intensity
        msg.palm = [int(TACTILE_MAX * 0.5 * (0.5 + 0.5 * np.sin(phase)))] * 112

        return msg

    def zero_pattern(self):
        """All zeros - no contact."""
        msg = InspireHandTouch()
        msg.header.stamp = self.get_clock().now().to_msg()

        for finger_name in ['little', 'ring', 'middle', 'index']:
            finger_msg = self.create_finger_msg(finger_name, [0]*9, [0]*96, [0]*80)
            setattr(msg, finger_name, finger_msg)

        msg.thumb = self.create_finger_msg('thumb', [0]*9, [0]*96, [0]*96, [0]*9)
        msg.palm = [0] * 112
        msg.total_contact = 0.0
        msg.fingers_in_contact = 0
        msg.stable_contact = False

        return msg


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish test tactile data')
    parser.add_argument('--pattern', type=str, default='random',
                       choices=['random', 'grasp', 'pinch', 'wave', 'zero'],
                       help='Tactile pattern to publish')

    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = TactileTestPublisher(pattern=cli_args.pattern)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
