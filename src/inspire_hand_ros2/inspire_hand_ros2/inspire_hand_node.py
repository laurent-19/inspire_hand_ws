#!/usr/bin/env python3
"""
InspireHandNode - Main ROS2 node for Inspire Hand control

Provides:
- State and tactile publishers
- Control subscriber
- Grasp/Release services
- GraspObject action server with real-time feedback
"""

import sys
import os

# Add the package directory to path for imports when running as script
_pkg_dir = os.path.dirname(os.path.abspath(__file__))
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header

import time
import threading
import asyncio

# Import custom messages (generated from .msg files)
from inspire_hand_ros2.msg import (
    InspireHandState,
    InspireHandControl,
    InspireHandTouch,
    FingerTactile,
    GraspStatus
)
from inspire_hand_ros2.srv import (
    Grasp,
    Release,
    SetFingerPosition,
    GetHandState
)
from inspire_hand_ros2.action import GraspObject

# Import our modules (use absolute import since we added to path)
from inspire_hand_driver import InspireHandDriver
from tactile_processor import TactileProcessor
from grasp_controller import GraspController, GraspState


class InspireHandNode(Node):
    """
    Main ROS2 node for Inspire Hand control.

    Publishers:
        ~/state (InspireHandState): Actuator state at configured rate
        ~/touch (InspireHandTouch): Tactile data at configured rate
        ~/grasp_status (GraspStatus): Grasp controller status

    Subscribers:
        ~/control (InspireHandControl): Control commands

    Services:
        ~/grasp: Execute grasp preset
        ~/release: Release and open hand
        ~/set_finger_position: Direct finger control
        ~/get_state: Get current state

    Actions:
        ~/grasp_object: Grasp with real-time feedback
    """

    def __init__(self):
        super().__init__('inspire_hand_node')

        # Declare parameters
        self.declare_parameter('hand_ip', '192.168.11.210')
        self.declare_parameter('hand_port', 6000)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('state_rate', 100.0)
        self.declare_parameter('touch_rate', 50.0)
        self.declare_parameter('enable_tactile', True)
        self.declare_parameter('default_force', 500)
        self.declare_parameter('default_speed', 500)
        self.declare_parameter('slip_compensation', True)

        # Get parameters
        hand_ip = self.get_parameter('hand_ip').value
        hand_port = self.get_parameter('hand_port').value
        device_id = self.get_parameter('device_id').value
        state_rate = self.get_parameter('state_rate').value
        touch_rate = self.get_parameter('touch_rate').value
        self.enable_tactile = self.get_parameter('enable_tactile').value
        default_force = self.get_parameter('default_force').value
        default_speed = self.get_parameter('default_speed').value
        slip_compensation = self.get_parameter('slip_compensation').value

        self.get_logger().info(f'Connecting to Inspire Hand at {hand_ip}:{hand_port}')

        # Initialize driver
        self.driver = InspireHandDriver(
            ip=hand_ip,
            port=hand_port,
            device_id=device_id
        )

        if not self.driver.is_connected():
            self.get_logger().error('Failed to connect to hand!')
        else:
            self.get_logger().info('Connected to hand successfully')

        # Initialize tactile processor and grasp controller
        self.tactile_processor = TactileProcessor(sample_rate=touch_rate)
        self.grasp_controller = GraspController(
            tactile_processor=self.tactile_processor,
            default_force=default_force,
            default_speed=default_speed,
            slip_compensation_enabled=slip_compensation
        )

        # Callback group for concurrent callbacks
        self.cb_group = ReentrantCallbackGroup()

        # State storage
        self._latest_state = None
        self._latest_tactile = None
        self._state_lock = threading.Lock()
        self._shutting_down = False

        # Publishers
        self.state_pub = self.create_publisher(InspireHandState, '~/state', 10)
        self.touch_pub = self.create_publisher(InspireHandTouch, '~/touch', 10)
        self.grasp_status_pub = self.create_publisher(GraspStatus, '~/grasp_status', 10)

        # Subscriber
        self.control_sub = self.create_subscription(
            InspireHandControl,
            '~/control',
            self.control_callback,
            10,
            callback_group=self.cb_group
        )

        # Services
        self.grasp_srv = self.create_service(
            Grasp, '~/grasp', self.grasp_callback,
            callback_group=self.cb_group
        )
        self.release_srv = self.create_service(
            Release, '~/release', self.release_callback,
            callback_group=self.cb_group
        )
        self.set_finger_srv = self.create_service(
            SetFingerPosition, '~/set_finger_position', self.set_finger_callback,
            callback_group=self.cb_group
        )
        self.get_state_srv = self.create_service(
            GetHandState, '~/get_state', self.get_state_callback,
            callback_group=self.cb_group
        )

        # Action server
        self._action_server = ActionServer(
            self,
            GraspObject,
            '~/grasp_object',
            execute_callback=self.grasp_action_execute,
            goal_callback=self.grasp_action_goal,
            cancel_callback=self.grasp_action_cancel,
            callback_group=self.cb_group
        )

        # Timers
        self.state_timer = self.create_timer(
            1.0 / state_rate,
            self.state_timer_callback,
            callback_group=self.cb_group
        )

        if self.enable_tactile:
            self.touch_timer = self.create_timer(
                1.0 / touch_rate,
                self.touch_timer_callback,
                callback_group=self.cb_group
            )

        self.get_logger().info('Inspire Hand node started')

    def state_timer_callback(self):
        """Read and publish hand state."""
        if self._shutting_down:
            return
        try:
            state = self.driver.read_state()

            with self._state_lock:
                self._latest_state = state

            # Convert to ROS message
            msg = self._state_to_msg(state)
            self.state_pub.publish(msg)

            # Update grasp controller if active
            if self.grasp_controller.is_active():
                tactile = self._latest_tactile or {}
                grasp_state, command = self.grasp_controller.update(state, tactile)

                # Send command if needed
                if command:
                    angles, forces, speeds = command
                    self.driver.write_control(angles, forces, speeds)

                # Publish grasp status
                status_msg = self._grasp_status_to_msg()
                self.grasp_status_pub.publish(status_msg)

        except Exception as e:
            if not self._shutting_down:
                self.get_logger().error(f'Error reading state: {e}')

    def touch_timer_callback(self):
        """Read and publish tactile data."""
        if self._shutting_down:
            return
        try:
            tactile = self.driver.read_tactile()

            with self._state_lock:
                self._latest_tactile = tactile

            # Convert to ROS message
            msg = self._tactile_to_msg(tactile)
            self.touch_pub.publish(msg)

        except Exception as e:
            if not self._shutting_down:
                self.get_logger().error(f'Error reading tactile: {e}')

    def control_callback(self, msg: InspireHandControl):
        """Handle direct control commands."""
        try:
            angles = list(msg.angle_set)
            forces = list(msg.force_set)
            speeds = list(msg.speed_set)

            self.driver.write_control(angles, forces, speeds)

        except Exception as e:
            self.get_logger().error(f'Error writing control: {e}')

    def grasp_callback(self, request, response):
        """Handle grasp service request."""
        try:
            grasp_type = request.grasp_type or 'power'
            target_force = request.target_force if request.target_force > 0 else None
            speed = int(request.speed * 1000) if request.speed > 0 else None

            # Start grasp
            angles, forces, speeds = self.grasp_controller.start_grasp(
                grasp_type=grasp_type,
                target_force=target_force,
                speed=speed
            )

            # Send command to hand
            self.driver.write_control(angles, forces, speeds)

            # Wait for grasp to complete (with timeout)
            timeout = 5.0
            start_time = time.time()

            while not self.grasp_controller.is_complete():
                if time.time() - start_time > timeout:
                    break
                time.sleep(0.05)

            response.success = self.grasp_controller.is_holding()
            response.message = f'Grasp {grasp_type}: {"success" if response.success else "timeout"}'
            response.status = self._grasp_status_to_msg()

        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'

        return response

    def release_callback(self, request, response):
        """Handle release service request."""
        try:
            speed = int(request.speed * 1000) if request.speed > 0 else 500

            # Release grasp
            angles, forces, speeds = self.grasp_controller.release()
            speeds = [speed] * 6

            self.driver.write_control(angles, forces, speeds)

            # Wait for hand to open
            time.sleep(1.0)

            response.success = True
            response.message = 'Released'

        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'

        return response

    def set_finger_callback(self, request, response):
        """Handle set finger position service request."""
        try:
            # Build control arrays
            angles = [-1] * 6
            forces = [-1] * 6
            speeds = [-1] * 6

            for i, idx in enumerate(request.finger_indices):
                if idx < 6:
                    if i < len(request.angles):
                        angles[idx] = request.angles[i]
                    if i < len(request.force_limits):
                        forces[idx] = request.force_limits[i]
                    if i < len(request.speeds):
                        speeds[idx] = request.speeds[i]

            self.driver.write_control(angles, forces, speeds)

            response.success = True
            response.message = 'Command sent'

        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'

        return response

    def get_state_callback(self, request, response):
        """Handle get state service request."""
        try:
            with self._state_lock:
                state = self._latest_state or self.driver.read_state()
                tactile = self._latest_tactile or self.driver.read_tactile()

            response.state = self._state_to_msg(state)
            response.touch = self._tactile_to_msg(tactile)
            response.grasp_status = self._grasp_status_to_msg()

        except Exception as e:
            self.get_logger().error(f'Error getting state: {e}')

        return response

    def grasp_action_goal(self, goal_request):
        """Accept or reject grasp action goal."""
        return GoalResponse.ACCEPT

    def grasp_action_cancel(self, goal_handle):
        """Handle grasp action cancellation."""
        self.get_logger().info('Grasp action cancelled')
        self.grasp_controller.abort()
        self.driver.open_hand()
        return CancelResponse.ACCEPT

    async def grasp_action_execute(self, goal_handle):
        """Execute grasp action with feedback."""
        self.get_logger().info(f'Executing grasp: {goal_handle.request.grasp_type}')

        request = goal_handle.request
        result = GraspObject.Result()

        try:
            # Start grasp
            grasp_type = request.grasp_type or 'power'
            target_force = request.target_force if request.target_force > 0 else None
            speed = int(request.speed * 1000) if request.speed > 0 else None
            custom_angles = list(request.custom_angles) if len(request.custom_angles) == 6 else None

            angles, forces, speeds = self.grasp_controller.start_grasp(
                grasp_type=grasp_type,
                target_force=target_force,
                speed=speed,
                custom_angles=custom_angles
            )

            self.driver.write_control(angles, forces, speeds)

            # Monitor and send feedback
            timeout = request.timeout if request.timeout > 0 else 10.0
            start_time = time.time()

            while not self.grasp_controller.is_complete():
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.grasp_controller.abort()
                    self.driver.open_hand()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled'
                    return result

                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > timeout:
                    break

                # Read state and update controller
                state = self.driver.read_state()
                tactile = self.driver.read_tactile() if self.enable_tactile else {}

                grasp_state, command = self.grasp_controller.update(state, tactile)

                if command:
                    self.driver.write_control(*command)

                # Send feedback
                feedback = GraspObject.Feedback()
                feedback.status = self._grasp_status_to_msg()
                feedback.progress = self.grasp_controller.get_progress()
                feedback.elapsed_time = elapsed
                goal_handle.publish_feedback(feedback)

                await asyncio.sleep(0.02)  # 50 Hz

            # Get final state
            final_state = self.driver.read_state()

            result.success = self.grasp_controller.is_holding()
            result.message = 'Grasp complete' if result.success else 'Grasp failed'
            result.final_force = list(final_state.get('force_act', [0] * 6))
            result.final_status = list(final_state.get('status', [0] * 6))
            result.elapsed_time = time.time() - start_time

            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f'Grasp action error: {e}')
            result.success = False
            result.message = f'Error: {e}'
            goal_handle.abort()

        return result

    def _state_to_msg(self, state: dict) -> InspireHandState:
        """Convert state dict to ROS message."""
        msg = InspireHandState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'inspire_hand'

        msg.position_actual = list(state.get('pos_act', [0] * 6))
        msg.angle_actual = list(state.get('angle_act', [0] * 6))
        msg.force_actual = list(state.get('force_act', [0] * 6))
        msg.current = list(state.get('current', [0] * 6))
        msg.status = list(state.get('status', [0] * 6))
        msg.error = list(state.get('error', [0] * 6))
        msg.temperature = list(state.get('temperature', [0] * 6))

        return msg

    def _tactile_to_msg(self, tactile: dict) -> InspireHandTouch:
        """Convert tactile dict to ROS message."""
        msg = InspireHandTouch()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'inspire_hand'

        # Helper to create FingerTactile message
        def make_finger(name: str, tip_key: str, nail_key: str,
                        pad_key: str, middle_key: str = None) -> FingerTactile:
            f = FingerTactile()
            f.name = name
            f.tip = list(tactile.get(tip_key, []))
            f.nail = list(tactile.get(nail_key, []))
            f.pad = list(tactile.get(pad_key, []))
            f.middle_section = list(tactile.get(middle_key, [])) if middle_key else []
            f.total_pressure = float(sum(f.tip) + sum(f.nail) + sum(f.pad) + sum(f.middle_section))
            f.max_pressure = float(max(f.tip + f.nail + f.pad + f.middle_section, default=0))
            f.contact_count = sum(1 for v in f.tip + f.nail + f.pad + f.middle_section if v > 100)
            return f

        msg.little = make_finger('little', 'fingerone_tip', 'fingerone_nail', 'fingerone_pad')
        msg.ring = make_finger('ring', 'fingertwo_tip', 'fingertwo_nail', 'fingertwo_pad')
        msg.middle = make_finger('middle', 'fingerthree_tip', 'fingerthree_nail', 'fingerthree_pad')
        msg.index = make_finger('index', 'fingerfour_tip', 'fingerfour_nail', 'fingerfour_pad')
        msg.thumb = make_finger('thumb', 'fingerfive_tip', 'fingerfive_nail',
                                'fingerfive_pad', 'fingerfive_middle')

        msg.palm = list(tactile.get('palm', [0] * 112))

        # Aggregate metrics
        msg.total_contact = (msg.little.total_pressure + msg.ring.total_pressure +
                            msg.middle.total_pressure + msg.index.total_pressure +
                            msg.thumb.total_pressure + sum(msg.palm))

        # Fingers in contact bitfield
        contact_threshold = 500
        msg.fingers_in_contact = 0
        if msg.little.total_pressure > contact_threshold:
            msg.fingers_in_contact |= 1
        if msg.ring.total_pressure > contact_threshold:
            msg.fingers_in_contact |= 2
        if msg.middle.total_pressure > contact_threshold:
            msg.fingers_in_contact |= 4
        if msg.index.total_pressure > contact_threshold:
            msg.fingers_in_contact |= 8
        if msg.thumb.total_pressure > contact_threshold:
            msg.fingers_in_contact |= 16

        msg.stable_contact = bin(msg.fingers_in_contact).count('1') >= 2

        return msg

    def _grasp_status_to_msg(self) -> GraspStatus:
        """Convert grasp controller status to ROS message."""
        msg = GraspStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'inspire_hand'

        msg.state = self.grasp_controller.state.value
        msg.object_detected = self.grasp_controller.is_holding()
        msg.grasp_stable = self.grasp_controller.is_holding()
        msg.grasp_type = (self.grasp_controller.current_preset.name
                         if self.grasp_controller.current_preset else '')

        # Get latest force from state
        with self._state_lock:
            if self._latest_state:
                msg.grip_force = list(self._latest_state.get('force_act', [0] * 6))
            else:
                msg.grip_force = [0] * 6

        msg.target_force = self.grasp_controller.target_force

        # Tactile contact
        signals = None
        with self._state_lock:
            if self._latest_tactile:
                signals = self.tactile_processor.process(self._latest_tactile)

        if signals:
            msg.tactile_contact = [float(f) for f in signals.finger_force]
            msg.finger_contact = list(signals.finger_contact)
            msg.slip_detected = self.tactile_processor.any_slip_detected(signals.finger_slip)
        else:
            msg.tactile_contact = [0.0] * 6
            msg.finger_contact = [False] * 6
            msg.slip_detected = False

        return msg

    def shutdown(self):
        """Prepare for shutdown - stop timers and set flag."""
        self._shutting_down = True

        # Cancel timers first
        if hasattr(self, 'state_timer'):
            self.state_timer.cancel()
        if hasattr(self, 'touch_timer'):
            self.touch_timer.cancel()

        # Wait briefly for in-flight callbacks to complete
        time.sleep(0.1)

        self.get_logger().info('Shutting down Inspire Hand node')

        # Open hand and disconnect
        if hasattr(self, 'driver') and self.driver.is_connected():
            try:
                self.driver.open_hand()
            except Exception:
                pass  # Ignore errors during shutdown
            self.driver.disconnect()

    def destroy_node(self):
        """Cleanup on shutdown."""
        if not self._shutting_down:
            self.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = InspireHandNode()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown cleanly
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
