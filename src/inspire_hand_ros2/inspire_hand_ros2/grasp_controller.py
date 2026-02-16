"""
GraspController - Human-inspired grasp state machine

Implements a 6-state grasp controller based on Romano et al. (IEEE TRO 2011):
IDLE -> CLOSE -> LOAD -> HOLD -> REPLACE -> UNLOAD -> OPEN

Uses hardware force control with tactile-based slip detection and compensation.
"""

from enum import Enum, auto
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import time

from tactile_processor import TactileProcessor, TactileSignals, AdaptiveForceEstimator


class GraspState(Enum):
    """Grasp controller states."""
    IDLE = 0
    CLOSING = 1      # Closing fingers toward object
    LOADING = 2      # Applying target grip force
    HOLDING = 3      # Stable grasp, monitoring for slip
    REPLACING = 4    # Placing object down
    UNLOADING = 5    # Releasing grip force
    OPENING = 6      # Opening hand
    SLIP_DETECTED = 7  # Slip detected, adjusting force


@dataclass
class GraspPreset:
    """Predefined grasp configuration."""
    name: str
    angles: List[int]        # Target angles for 6 DOF
    default_force: int       # Default force in grams
    active_fingers: List[int]  # Which fingers are active
    description: str


class GraspController:
    """
    Human-inspired grasp controller with hardware force control.

    Features:
    - 6-state finite state machine
    - Predefined grasp presets (power, pinch, precision, etc.)
    - Weber's law slip detection (12% threshold)
    - Adaptive force compensation
    - Hardware force limiting (STATUS=3 detection)
    """

    # Grasp presets
    # NOTE: angle=0 is CLOSED (fingers curled), angle=1000 is OPEN (fingers extended)
    PRESETS = {
        'power': GraspPreset(
            name='power',
            angles=[0, 0, 0, 0, 0, 500],  # All fingers closed, thumb neutral
            default_force=800,
            active_fingers=[0, 1, 2, 3, 4, 5],
            description='Full hand power grasp for large objects'
        ),
        'pinch': GraspPreset(
            name='pinch',
            angles=[1000, 1000, 1000, 0, 0, 200],  # Only index+thumb close
            default_force=400,
            active_fingers=[3, 4, 5],
            description='Thumb-index pinch for small objects'
        ),
        'precision': GraspPreset(
            name='precision',
            angles=[1000, 1000, 200, 100, 0, 400],  # Middle, index, thumb close
            default_force=300,
            active_fingers=[2, 3, 4, 5],
            description='Three-finger precision for delicate objects'
        ),
        'cylindrical': GraspPreset(
            name='cylindrical',
            angles=[200, 200, 200, 200, 400, 600],  # Partially closed wrap
            default_force=600,
            active_fingers=[0, 1, 2, 3, 4, 5],
            description='Wrapped grasp for bottles and handles'
        ),
        'hook': GraspPreset(
            name='hook',
            angles=[0, 0, 0, 0, 1000, 1000],  # 4 fingers closed, thumb open
            default_force=700,
            active_fingers=[0, 1, 2, 3],
            description='Hook grasp for bags and handles'
        ),
        'open': GraspPreset(
            name='open',
            angles=[1000, 1000, 1000, 1000, 1000, 500],  # All open
            default_force=500,
            active_fingers=[],
            description='Fully open hand'
        ),
    }

    # Status code indicating force threshold reached
    STATUS_FORCE_REACHED = 3

    # Angle step to close fingers more on slip detection
    # angle=0 is CLOSED, so we subtract this to close more
    SLIP_ANGLE_STEP = 100  # ~10% of full range for faster grip

    def __init__(self,
                 tactile_processor: Optional[TactileProcessor] = None,
                 force_estimator: Optional[AdaptiveForceEstimator] = None,
                 default_force: int = 500,
                 default_speed: int = 500,
                 slip_compensation_enabled: bool = True):
        """
        Initialize grasp controller.

        Args:
            tactile_processor: TactileProcessor instance (creates one if None)
            force_estimator: AdaptiveForceEstimator instance (creates one if None)
            default_force: Default grip force in grams
            default_speed: Default movement speed (0-1000)
            slip_compensation_enabled: Enable slip detection and force adjustment
        """
        self.tactile = tactile_processor or TactileProcessor()
        self.force_estimator = force_estimator or AdaptiveForceEstimator()

        self.default_force = default_force
        self.default_speed = default_speed
        self.slip_compensation_enabled = slip_compensation_enabled

        # State
        self.state = GraspState.IDLE
        self.current_preset: Optional[GraspPreset] = None
        self.target_force = default_force
        self.target_angles = [0] * 6
        self.target_speeds = [default_speed] * 6

        # Timing
        self.state_start_time = 0.0
        self.grasp_start_time = 0.0

        # Monitoring
        self.slip_count = 0
        self.max_force_during_load = 0.0
        self.last_slip_time = 0.0
        self.SLIP_COOLDOWN = 0.5  # Wait 0.5 seconds after slip before detecting again
        self.INITIAL_HOLD_DELAY = 1.0  # Wait 1 second after reaching HOLD before enabling slip detection
        self.hold_start_time = 0.0

    def start_grasp(self, grasp_type: str = 'power',
                    target_force: Optional[int] = None,
                    speed: Optional[int] = None,
                    custom_angles: Optional[List[int]] = None) -> Tuple[List[int], List[int], List[int]]:
        """
        Start a grasp operation.

        Args:
            grasp_type: Preset name or 'custom'
            target_force: Target grip force in grams (uses preset default if None)
            speed: Movement speed 0-1000 (uses default if None)
            custom_angles: Custom angles for grasp_type='custom'

        Returns:
            Tuple of (angles, forces, speeds) to send to hand
        """
        # Get preset
        if grasp_type == 'custom' and custom_angles:
            self.current_preset = GraspPreset(
                name='custom',
                angles=custom_angles,
                default_force=target_force or self.default_force,
                active_fingers=[i for i, a in enumerate(custom_angles) if a > 0],
                description='Custom grasp'
            )
        else:
            self.current_preset = self.PRESETS.get(grasp_type, self.PRESETS['power'])

        # Set targets
        self.target_force = target_force if target_force is not None else self.current_preset.default_force
        self.target_angles = self.current_preset.angles.copy()
        self.target_speeds = [speed or self.default_speed] * 6

        # Reset state
        self.state = GraspState.CLOSING
        self.state_start_time = time.time()
        self.grasp_start_time = time.time()
        self.slip_count = 0
        self.max_force_during_load = 0.0
        self.tactile.reset()
        self.force_estimator.set_force(self.target_force)

        # Return control command
        return (
            self.target_angles,
            [self.target_force] * 6,
            self.target_speeds
        )

    def update(self, state: Dict, tactile_data: Dict) -> Tuple[GraspState, Optional[Tuple[List[int], List[int], List[int]]]]:
        """
        Update grasp controller state based on feedback.

        Args:
            state: Hand state dictionary with keys: force_act, status, etc.
            tactile_data: Tactile data dictionary

        Returns:
            Tuple of (current_state, control_command_or_none)
            control_command is (angles, forces, speeds) or None if no update needed
        """
        # Process tactile signals
        signals = self.tactile.process(tactile_data)

        # Get status codes
        status = state.get('status', [0] * 6)
        force_act = state.get('force_act', [0] * 6)

        # Check if any finger reached force threshold
        fingers_holding = [s == self.STATUS_FORCE_REACHED for s in status]

        # State machine
        command = None

        if self.state == GraspState.CLOSING:
            # Wait for contact detection
            if any(fingers_holding) or any(signals.finger_contact):
                self._transition_to(GraspState.LOADING)
                # Track max force for adaptive estimation
                self.max_force_during_load = max(signals.finger_force)

        elif self.state == GraspState.LOADING:
            # Track max force
            current_max = max(signals.finger_force)
            self.max_force_during_load = max(self.max_force_during_load, current_max)

            # Wait for stable contact (multiple fingers)
            contact_count = self.tactile.get_contact_count(signals.finger_contact)
            holding_count = sum(fingers_holding)

            if contact_count >= 2 or holding_count >= 2:
                self._transition_to(GraspState.HOLDING)
                self.hold_start_time = time.time()

            # Timeout check
            elif self._time_in_state() > 3.0:
                # Force reached on at least one finger
                if holding_count >= 1:
                    self._transition_to(GraspState.HOLDING)
                    self.hold_start_time = time.time()

        elif self.state == GraspState.HOLDING:
            # Monitor for slip (with delays to let grip stabilize)
            time_since_slip = time.time() - self.last_slip_time
            time_in_hold = time.time() - self.hold_start_time

            # Wait for initial stabilization and cooldown after last slip response
            slip_cooldown_passed = time_since_slip > self.SLIP_COOLDOWN
            initial_delay_passed = time_in_hold > self.INITIAL_HOLD_DELAY

            if (self.slip_compensation_enabled and
                initial_delay_passed and
                slip_cooldown_passed and
                self.tactile.any_slip_detected(signals.finger_slip)):

                self._transition_to(GraspState.SLIP_DETECTED)
                self.last_slip_time = time.time()

                # Increase force
                new_force = self.force_estimator.adjust_for_slip()
                print(f"[GRASP] SLIP #{self.slip_count + 1}: force {self.target_force} -> {new_force}, "
                      f"hold_time={time_in_hold:.1f}s")
                self.target_force = new_force
                self.slip_count += 1

                # Get current angles and compute tighter grip
                # angle=0 is CLOSED, angle=1000 is OPEN
                # Reduce angle by SLIP_ANGLE_STEP to close fingers more
                current_angles = state.get('angle_act', self.target_angles)
                new_angles = []
                for i, angle in enumerate(current_angles):
                    if i in (self.current_preset.active_fingers if self.current_preset else range(6)):
                        # Close finger more (reduce angle toward 0)
                        new_angle = max(0, angle - self.SLIP_ANGLE_STEP)
                        new_angles.append(new_angle)
                    else:
                        new_angles.append(-1)  # Don't change inactive fingers

                # Update target angles for tracking
                for i, a in enumerate(new_angles):
                    if a >= 0:
                        self.target_angles[i] = a

                # Send command with lower angles AND higher force
                # This restarts finger movement toward more closed position
                command = (
                    new_angles,
                    [new_force] * 6,
                    [-1] * 6   # Don't change speeds
                )

        elif self.state == GraspState.SLIP_DETECTED:
            # Brief state for slip handling, return to holding
            if self._time_in_state() > 0.1:
                self._transition_to(GraspState.HOLDING)

        elif self.state == GraspState.REPLACING:
            # Wait for contact with surface (via slip or tactile spike)
            # This is typically triggered by external robot motion
            if signals.total_contact > self.tactile.contact_threshold * 2:
                self._transition_to(GraspState.UNLOADING)

        elif self.state == GraspState.UNLOADING:
            # Gradually reduce force
            elapsed = self._time_in_state()
            unload_duration = 0.5  # 500ms

            if elapsed < unload_duration:
                # Linear force reduction
                progress = elapsed / unload_duration
                new_force = int(self.target_force * (1 - progress))
                command = (
                    [-1] * 6,
                    [new_force] * 6,
                    [-1] * 6
                )
            else:
                self._transition_to(GraspState.OPENING)
                command = (
                    [0, 0, 0, 0, 0, 0],
                    [0] * 6,
                    self.target_speeds
                )

        elif self.state == GraspState.OPENING:
            # Wait for hand to open
            all_open = all(a < 50 for a in state.get('angle_act', [1000] * 6))
            if all_open or self._time_in_state() > 2.0:
                self._transition_to(GraspState.IDLE)

        return self.state, command

    def release(self) -> Tuple[List[int], List[int], List[int]]:
        """
        Release grasp and open hand.

        Returns:
            Tuple of (angles, forces, speeds) to send to hand
        """
        self._transition_to(GraspState.OPENING)
        # angle=1000 is OPEN, angle=0 is CLOSED
        return (
            [1000, 1000, 1000, 1000, 1000, 500],  # Open position
            [500] * 6,  # Need some force to allow movement
            self.target_speeds
        )

    def place(self):
        """Signal that we're placing the object (start Replace phase)."""
        if self.state == GraspState.HOLDING:
            self._transition_to(GraspState.REPLACING)

    def abort(self) -> Tuple[List[int], List[int], List[int]]:
        """Abort current grasp and open hand."""
        self._transition_to(GraspState.IDLE)
        self.tactile.reset()
        # angle=1000 is OPEN, angle=0 is CLOSED
        return (
            [1000, 1000, 1000, 1000, 1000, 500],  # Open position
            [500] * 6,  # Need some force to allow movement
            [self.default_speed] * 6
        )

    def _transition_to(self, new_state: GraspState):
        """Transition to new state."""
        self.state = new_state
        self.state_start_time = time.time()

    def _time_in_state(self) -> float:
        """Time spent in current state."""
        return time.time() - self.state_start_time

    def get_elapsed_time(self) -> float:
        """Total time since grasp started."""
        return time.time() - self.grasp_start_time

    def get_progress(self) -> float:
        """
        Estimate grasp progress (0.0 to 1.0).

        Returns:
            Progress value from 0 (starting) to 1 (complete)
        """
        if self.state == GraspState.IDLE:
            return 0.0
        elif self.state == GraspState.CLOSING:
            return min(0.3, self._time_in_state() / 2.0)
        elif self.state == GraspState.LOADING:
            return 0.3 + min(0.3, self._time_in_state() / 2.0)
        elif self.state in [GraspState.HOLDING, GraspState.SLIP_DETECTED]:
            return 1.0
        elif self.state == GraspState.REPLACING:
            return 0.8
        elif self.state == GraspState.UNLOADING:
            return 0.5 + 0.3 * self._time_in_state() / 0.5
        elif self.state == GraspState.OPENING:
            return 0.2
        return 0.0

    def is_complete(self) -> bool:
        """Check if grasp is complete (holding or idle)."""
        return self.state in [GraspState.HOLDING, GraspState.IDLE]

    def is_holding(self) -> bool:
        """Check if currently holding an object."""
        return self.state == GraspState.HOLDING

    def is_active(self) -> bool:
        """Check if grasp operation is active."""
        return self.state not in [GraspState.IDLE]

    def get_status_dict(self, signals: Optional[TactileSignals] = None) -> Dict:
        """
        Get current status as dictionary (for ROS message).

        Args:
            signals: Optional TactileSignals from latest update

        Returns:
            Dictionary with grasp status fields
        """
        return {
            'state': self.state.value,
            'state_name': self.state.name,
            'grasp_type': self.current_preset.name if self.current_preset else 'none',
            'target_force': self.target_force,
            'elapsed_time': self.get_elapsed_time(),
            'progress': self.get_progress(),
            'slip_count': self.slip_count,
            'is_holding': self.is_holding(),
            'finger_contact': signals.finger_contact if signals else [False] * 6,
            'finger_force': signals.finger_force if signals else [0.0] * 6,
        }

    @classmethod
    def get_preset_names(cls) -> List[str]:
        """Get list of available preset names."""
        return list(cls.PRESETS.keys())

    @classmethod
    def get_preset(cls, name: str) -> Optional[GraspPreset]:
        """Get preset by name."""
        return cls.PRESETS.get(name)
