#!/usr/bin/env python3
"""
Inspire Hand ROS2 Visualizer

Qt-based visualization for hand state and tactile data using ROS2 topics.
"""

import sys
import os
import signal
import threading
import numpy as np

# Add package directory to path
_pkg_dir = os.path.dirname(os.path.abspath(__file__))
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from inspire_hand_ros2.msg import InspireHandState, InspireHandTouch

# Import local visualization modules
import viz_qt
import viz_data


class ROS2DataHandler(Node):
    """
    ROS2 subscriber that provides the same interface as DDSHandler.

    Subscribes to:
        - /inspire_hand/inspire_hand_node/state
        - /inspire_hand/inspire_hand_node/touch

    Provides:
        - read() method returning {'states': dict, 'touch': dict}
    """

    def __init__(self, namespace='inspire_hand/inspire_hand_node'):
        super().__init__('inspire_hand_visualizer')

        self.data_state_lock = threading.Lock()
        self.data_touch_lock = threading.Lock()

        # Initialize with default values
        self.states = {
            'POS_ACT': [0] * 6,
            'ANGLE_ACT': [0] * 6,
            'FORCE_ACT': [0] * 6,
            'CURRENT': [0] * 6,
            'ERROR': [0] * 6,
            'STATUS': [0] * 6,
            'TEMP': [0] * 6
        }

        # Initialize touch data structure from data sheet
        self.data = viz_data.data_sheet
        self.touch = {}
        for name, addr, length, size, var in self.data:
            self.touch[var] = np.zeros(size)

        # Create subscribers
        self.state_sub = self.create_subscription(
            InspireHandState,
            f'/{namespace}/state',
            self.state_callback,
            10
        )

        self.touch_sub = self.create_subscription(
            InspireHandTouch,
            f'/{namespace}/touch',
            self.touch_callback,
            10
        )

        self.get_logger().info(f'Visualizer subscribing to /{namespace}/state and /{namespace}/touch')

    def state_callback(self, msg: InspireHandState):
        """Update state from ROS2 message."""
        with self.data_state_lock:
            self.states = {
                'POS_ACT': list(msg.position_actual),
                'ANGLE_ACT': list(msg.angle_actual),
                'FORCE_ACT': list(msg.force_actual),
                'CURRENT': list(msg.current),
                'ERROR': list(msg.error),
                'STATUS': list(msg.status),
                'TEMP': list(msg.temperature)
            }

    def touch_callback(self, msg: InspireHandTouch):
        """Update touch data from ROS2 message."""
        with self.data_touch_lock:
            # Map ROS2 message fields to SDK data structure variable names
            # SDK uses: fingerone_tip_touch, fingerone_top_touch, fingerone_palm_touch, etc.
            finger_map = {
                'little': 'fingerone',
                'ring': 'fingertwo',
                'middle': 'fingerthree',
                'index': 'fingerfour',
                'thumb': 'fingerfive'
            }

            for ros_name, sdk_prefix in finger_map.items():
                finger = getattr(msg, ros_name)

                # Get sizes from data sheet (uses _touch suffix)
                tip_size = self._get_size(f'{sdk_prefix}_tip_touch')
                top_size = self._get_size(f'{sdk_prefix}_top_touch')
                palm_size = self._get_size(f'{sdk_prefix}_palm_touch')

                if len(finger.tip) > 0:
                    self.touch[f'{sdk_prefix}_tip_touch'] = np.array(finger.tip).reshape(tip_size)
                if len(finger.nail) > 0:
                    self.touch[f'{sdk_prefix}_top_touch'] = np.array(finger.nail).reshape(top_size)
                if len(finger.pad) > 0:
                    self.touch[f'{sdk_prefix}_palm_touch'] = np.array(finger.pad).reshape(palm_size)

            # Thumb middle (only thumb has this)
            if len(msg.thumb.middle_section) > 0:
                middle_size = self._get_size('fingerfive_middle_touch')
                self.touch['fingerfive_middle_touch'] = np.array(msg.thumb.middle_section).reshape(middle_size)

            # Palm
            if len(msg.palm) > 0:
                palm_size = self._get_size('palm_touch')
                self.touch['palm_touch'] = np.array(msg.palm).reshape(palm_size)

    def _get_size(self, var_name):
        """Get the size tuple for a touch variable from data sheet."""
        for name, addr, length, size, var in self.data:
            if var == var_name:
                return size
        return (1, 1)  # Default fallback

    def read(self):
        """Return current state and touch data (same interface as DDSHandler)."""
        with self.data_state_lock:
            with self.data_touch_lock:
                return {'states': self.states.copy(), 'touch': self.touch.copy()}


def main():
    # Parse arguments
    namespace = 'inspire_hand/inspire_hand_node'
    if len(sys.argv) > 1:
        namespace = sys.argv[1]

    # Initialize ROS2
    rclpy.init()

    # Create data handler node
    data_handler = ROS2DataHandler(namespace=namespace)

    # Create executor and spin in background thread
    executor = SingleThreadedExecutor()
    executor.add_node(data_handler)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Create Qt application
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
    app = QApplication(sys.argv)

    # Allow Ctrl+C to work with Qt - periodic timer lets Python handle signals
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # Empty callback to process Python signals
    timer.start(100)

    window = viz_qt.MainWindow(
        data_handler=data_handler,
        dt=50,  # Update every 50ms
        name="Inspire Hand ROS2 Visualizer"
    )
    window.reflash()
    window.show()

    # Run Qt event loop
    ret = app.exec_()

    # Cleanup
    executor.shutdown()
    data_handler.destroy_node()
    rclpy.shutdown()

    sys.exit(ret)


if __name__ == '__main__':
    main()
