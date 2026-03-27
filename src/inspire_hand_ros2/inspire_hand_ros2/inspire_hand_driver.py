"""
InspireHandDriver - SDK wrapper for ROS2 integration

Wraps the ModbusDataHandler from inspire_hand_sdk, disabling internal DDS
and providing thread-safe methods for ROS2 node integration.
"""

import threading
import struct
import sys
import os

# Add the SDK to the path
sdk_path = os.path.join(os.path.dirname(__file__), '../../../../inspire_hand_sdk')
if sdk_path not in sys.path:
    sys.path.insert(0, sdk_path)

from pymodbus.client import ModbusTcpClient
from pymodbus.client import ModbusSerialClient


class InspireHandDriver:
    """
    Wraps ModbusDataHandler for ROS2 integration.

    This class communicates directly with the Inspire Hand via Modbus TCP,
    without using the internal DDS system (we use ROS2 instead).
    """

    # Register addresses (from hardware documentation)
    REG_POS_SET = 1474
    REG_ANGLE_SET = 1486
    REG_FORCE_SET = 1498
    REG_SPEED_SET = 1522

    REG_POS_ACT = 1534
    REG_ANGLE_ACT = 1546
    REG_FORCE_ACT = 1582
    REG_CURRENT = 1594
    REG_ERROR = 1606
    REG_STATUS = 1612
    REG_TEMP = 1618

    REG_CLEAR_ERROR = 1004

    # Tactile register definitions
    TACTILE_REGISTERS = [
        ("fingerone_tip", 3000, 9),       # Little finger tip (3x3)
        ("fingerone_nail", 3018, 96),     # Little finger nail (12x8)
        ("fingerone_pad", 3210, 80),      # Little finger pad (10x8)
        ("fingertwo_tip", 3370, 9),       # Ring finger tip
        ("fingertwo_nail", 3388, 96),     # Ring finger nail
        ("fingertwo_pad", 3580, 80),      # Ring finger pad
        ("fingerthree_tip", 3740, 9),     # Middle finger tip
        ("fingerthree_nail", 3758, 96),   # Middle finger nail
        ("fingerthree_pad", 3950, 80),    # Middle finger pad
        ("fingerfour_tip", 4110, 9),      # Index finger tip
        ("fingerfour_nail", 4128, 96),    # Index finger nail
        ("fingerfour_pad", 4320, 80),     # Index finger pad
        ("fingerfive_tip", 4480, 9),      # Thumb tip
        ("fingerfive_nail", 4498, 96),    # Thumb nail
        ("fingerfive_middle", 4690, 9),   # Thumb middle
        ("fingerfive_pad", 4708, 96),     # Thumb pad
        ("palm", 4900, 112),              # Palm (8x14)
    ]

    def __init__(self, ip: str = '192.168.123.211', port: int = 6000,
                 device_id: int = 1, use_serial: bool = False,
                 serial_port: str = '/dev/ttyUSB0', baudrate: int = 115200,
                 max_retries: int = 5, retry_delay: float = 2.0,
                 auto_calibrate: bool = False):
        """
        Initialize the Inspire Hand driver.

        Args:
            ip: Modbus TCP IP address
            port: Modbus TCP port
            device_id: Hand device ID (1-254)
            use_serial: Use serial instead of TCP
            serial_port: Serial port path
            baudrate: Serial baud rate
            max_retries: Connection retry attempts
            retry_delay: Delay between retries in seconds
            auto_calibrate: Run calibration on startup (default False)
        """
        self.device_id = device_id
        self.use_serial = use_serial
        self._lock = threading.Lock()
        self._connected = False

        # Initialize Modbus client
        if use_serial:
            self.client = ModbusSerialClient(
                method='rtu',
                port=serial_port,
                baudrate=baudrate,
                timeout=1
            )
        else:
            self.client = ModbusTcpClient(ip, port=port)

        # Connect with retry
        self._connect(max_retries, retry_delay)

        # Clear errors on startup
        if self._connected:
            self.clear_errors()

            # Only calibrate if explicitly requested
            if auto_calibrate:
                print("[InspireHandDriver] Auto-calibration enabled")
                self.calibrate()

    def _connect(self, max_retries: int, retry_delay: float):
        """Connect to Modbus server with retry logic."""
        import time

        for attempt in range(max_retries):
            try:
                if self.client.connect():
                    self._connected = True
                    print(f"[InspireHandDriver] Connected to hand successfully")
                    return
            except Exception as e:
                print(f"[InspireHandDriver] Connection attempt {attempt + 1} failed: {e}")

            if attempt < max_retries - 1:
                print(f"[InspireHandDriver] Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)

        print(f"[InspireHandDriver] Failed to connect after {max_retries} attempts")
        self._connected = False

    def is_connected(self) -> bool:
        """Check if driver is connected."""
        return self._connected

    def _read_registers_short(self, address: int, count: int) -> list:
        """Read registers and parse as signed 16-bit integers."""
        with self._lock:
            try:
                response = self.client.read_holding_registers(address, count=count, device_id=self.device_id)
                if response.isError():
                    return None

                # Pack as unsigned, unpack as signed
                packed = struct.pack('>' + 'H' * count, *response.registers)
                return list(struct.unpack('>' + 'h' * count, packed))
            except Exception as e:
                print(f"[InspireHandDriver] Read error: {e}")
                return None

    def _read_registers_byte(self, address: int, count: int) -> list:
        """Read registers and parse as bytes (2 bytes per register)."""
        with self._lock:
            try:
                response = self.client.read_holding_registers(address, count=count, device_id=self.device_id)
                if response.isError():
                    return None

                byte_list = []
                for reg in response.registers:
                    byte_list.append((reg >> 8) & 0xFF)  # High byte
                    byte_list.append(reg & 0xFF)          # Low byte
                return byte_list
            except Exception as e:
                print(f"[InspireHandDriver] Read error: {e}")
                return None

    def _write_registers(self, address: int, values: list) -> bool:
        """Write registers. All 6 values must be provided."""
        with self._lock:
            try:
                # Skip if all values are -1 (no change)
                if all(v == -1 for v in values):
                    return True

                # Convert signed to unsigned for Modbus
                # -1 means "no change" - we need to read current value first
                unsigned_values = []
                for v in values:
                    if v == -1:
                        # Keep as 0xFFFF (will be handled below)
                        unsigned_values.append(0xFFFF)
                    elif v < 0:
                        unsigned_values.append(v & 0xFFFF)
                    else:
                        unsigned_values.append(v)

                # If any value is 0xFFFF (-1), we need to read current values and merge
                if 0xFFFF in unsigned_values:
                    # Read current register values
                    count = len(values)
                    response = self.client.read_holding_registers(address, count=count, device_id=self.device_id)
                    if response.isError():
                        print(f"[InspireHandDriver] Read for merge failed: {response}")
                        return False
                    current = response.registers
                    # Merge: keep current value where -1 was specified
                    for i, v in enumerate(unsigned_values):
                        if v == 0xFFFF:
                            unsigned_values[i] = current[i]

                print(f"[InspireHandDriver] Writing to {address}: {unsigned_values}")
                result = self.client.write_registers(address, unsigned_values, device_id=self.device_id)
                if hasattr(result, 'isError') and result.isError():
                    print(f"[InspireHandDriver] Write failed: {result}")
                    return False
                return True
            except Exception as e:
                print(f"[InspireHandDriver] Write error: {e}")
                return False

    def read_state(self) -> dict:
        """
        Read current hand state.

        Returns:
            Dictionary with keys: pos_act, angle_act, force_act, current,
                                  status, error, temperature
        """
        state = {
            'pos_act': self._read_registers_short(self.REG_POS_ACT, 6) or [0] * 6,
            'angle_act': self._read_registers_short(self.REG_ANGLE_ACT, 6) or [0] * 6,
            'force_act': self._read_registers_short(self.REG_FORCE_ACT, 6) or [0] * 6,
            'current': self._read_registers_short(self.REG_CURRENT, 6) or [0] * 6,
            'error': self._read_registers_byte(self.REG_ERROR, 3) or [0] * 6,
            'status': self._read_registers_byte(self.REG_STATUS, 3) or [0] * 6,
            'temperature': self._read_registers_byte(self.REG_TEMP, 3) or [0] * 6,
        }
        return state

    def read_tactile(self) -> dict:
        """
        Read tactile sensor data.

        Returns:
            Dictionary with tactile arrays for each finger region and palm.
        """
        tactile = {}

        for name, address, count in self.TACTILE_REGISTERS:
            values = self._read_registers_short(address, count)
            tactile[name] = values if values else [0] * count

        return tactile

    def read_all(self) -> dict:
        """
        Read both state and tactile data.

        Returns:
            Dictionary with 'state' and 'tactile' keys.
        """
        return {
            'state': self.read_state(),
            'tactile': self.read_tactile()
        }

    def write_angle(self, angles: list) -> bool:
        """
        Set target angles for all 6 DOF.

        Args:
            angles: List of 6 angle values (0-1000, -1 for no change)

        Returns:
            True if successful
        """
        return self._write_registers(self.REG_ANGLE_SET, angles)

    def write_force(self, forces: list) -> bool:
        """
        Set force limits for all 6 DOF.

        Args:
            forces: List of 6 force values in grams (0-3000)

        Returns:
            True if successful
        """
        return self._write_registers(self.REG_FORCE_SET, forces)

    def write_speed(self, speeds: list) -> bool:
        """
        Set movement speeds for all 6 DOF.

        Args:
            speeds: List of 6 speed values (0-1000, -1 for no change)

        Returns:
            True if successful
        """
        return self._write_registers(self.REG_SPEED_SET, speeds)

    def write_position(self, positions: list) -> bool:
        """
        Set actuator positions for all 6 DOF.

        Args:
            positions: List of 6 position values (0-2000, -1 for no change)

        Returns:
            True if successful
        """
        return self._write_registers(self.REG_POS_SET, positions)

    def write_control(self, angles: list = None, forces: list = None,
                      speeds: list = None) -> bool:
        """
        Write complete control command.

        Args:
            angles: Target angles (0-1000, -1 for no change)
            forces: Force limits in grams (0-3000)
            speeds: Movement speeds (0-1000, -1 for no change)

        Returns:
            True if all writes successful
        """
        success = True

        if angles is not None:
            success = success and self.write_angle(angles)

        if forces is not None:
            success = success and self.write_force(forces)

        if speeds is not None:
            success = success and self.write_speed(speeds)

        return success

    def clear_errors(self) -> bool:
        """Clear error flags on all actuators."""
        with self._lock:
            try:
                result = self.client.write_register(self.REG_CLEAR_ERROR, 1, device_id=self.device_id)
                return not result.isError() if hasattr(result, 'isError') else True
            except Exception as e:
                print(f"[InspireHandDriver] Clear errors failed: {e}")
                return False

    def calibrate(self, reset_to_factory: bool = False) -> bool:
        """
        Run force sensor calibration.

        IMPORTANT: Hand must be fully open with no object contact before calling.

        Args:
            reset_to_factory: Reset all parameters to factory defaults before calibration

        Returns:
            True if calibration successful
        """
        import time
        try:
            print("[InspireHandDriver] Running force sensor calibration...")

            # Optionally reset parameters (register 1006)
            if reset_to_factory:
                print("[InspireHandDriver] Resetting to factory defaults...")
                with self._lock:
                    self.client.write_register(1006, 1, device_id=self.device_id)
                time.sleep(0.5)

            # Step 1: Command hand to fully open (angles = 1000)
            # Must set force and speed first for hand to move
            print("[InspireHandDriver] Commanding hand to open...")
            with self._lock:
                self.client.write_registers(self.REG_FORCE_SET, [500, 500, 500, 500, 500, 500], device_id=self.device_id)
                self.client.write_registers(self.REG_SPEED_SET, [500, 500, 500, 500, 500, 500], device_id=self.device_id)
                self.client.write_registers(self.REG_ANGLE_SET, [1000, 1000, 1000, 1000, 1000, 1000], device_id=self.device_id)

            # Step 2: Verify hand is actually fully open by checking actual angles
            print("[InspireHandDriver] Waiting for hand to reach fully open position...")
            open_threshold = 950  # Consider "open" if angle >= 950 (out of 1000)
            max_wait_time = 5.0   # Maximum 5 seconds to open
            poll_interval = 0.1   # Check every 100ms

            start_time = time.time()
            hand_fully_open = False

            while time.time() - start_time < max_wait_time:
                # Read actual angles and error status (these methods handle their own locking)
                actual_angles = self._read_registers_short(self.REG_ANGLE_ACT, 6)
                error_codes = self._read_registers_byte(self.REG_ERROR, 3)

                if actual_angles is None:
                    print("[InspireHandDriver] Warning: Could not read actual angles")
                    time.sleep(poll_interval)
                    continue

                # Check for errors that would prevent opening (first 5 fingers)
                if error_codes:
                    errors = error_codes[:5]  # First 5 fingers
                    if any(err != 0 for err in errors):
                        print(f"[InspireHandDriver] WARNING: Finger errors detected: {errors}")
                        print("[InspireHandDriver] Clear errors or check for obstructions")
                        # Clear errors and try to continue
                        self.clear_errors()

                # Check if all fingers (0-4) are open (skip finger 5 - thumb rotation)
                fingers_open = [angle >= open_threshold for angle in actual_angles[:5]]

                if all(fingers_open):
                    print(f"[InspireHandDriver] Hand fully open (angles: {actual_angles})")
                    hand_fully_open = True
                    break

                # Still moving, wait a bit
                time.sleep(poll_interval)

            if not hand_fully_open:
                # Timeout - hand didn't fully open
                actual_angles = self._read_registers_short(self.REG_ANGLE_ACT, 6)
                error_codes = self._read_registers_byte(self.REG_ERROR, 3)
                print(f"[InspireHandDriver] WARNING: Hand not fully open after {max_wait_time}s")
                print(f"[InspireHandDriver] Actual angles: {actual_angles}")
                print(f"[InspireHandDriver] Error codes: {error_codes[:6] if error_codes else 'N/A'}")
                print("[InspireHandDriver] Check for obstructions or mechanical issues")
                print("[InspireHandDriver] Proceeding with calibration anyway (may be inaccurate)")

            # Give a small settling time
            time.sleep(0.3)

            # Step 3: Trigger force sensor calibration (register 1009)
            print("[InspireHandDriver] Triggering force sensor calibration...")
            with self._lock:
                self.client.write_register(1009, 1, device_id=self.device_id)
            time.sleep(1.0)  # Wait for calibration to complete

            print("[InspireHandDriver] Calibration complete")
            return True
        except Exception as e:
            print(f"[InspireHandDriver] Calibration failed: {e}")
            return False

    def open_hand(self, speed: int = 500) -> bool:
        """Open hand (all fingers extended)."""
        # angle=1000 is OPEN (extended), angle=0 is CLOSED (curled)
        return self.write_control(
            angles=[1000, 1000, 1000, 1000, 1000, 500],
            forces=[500] * 6,  # Need some force to allow movement
            speeds=[speed] * 6
        )

    def close_hand(self, force: int = 500, speed: int = 500) -> bool:
        """Close hand (all fingers curled with force limit)."""
        # angle=0 is CLOSED (curled for grasping)
        return self.write_control(
            angles=[0, 0, 0, 0, 0, 500],
            forces=[force] * 6,
            speeds=[speed] * 6
        )

    def disconnect(self):
        """Disconnect from hand."""
        if self.client:
            self.client.close()
            self._connected = False

    def __del__(self):
        """Cleanup on deletion."""
        self.disconnect()
