"""
TactileProcessor - Human-inspired tactile signal processing

Implements SA-I and FA-I signal processing based on Romano et al. (IEEE TRO 2011)
for contact detection, slip detection, and grasp monitoring.
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class TactileSignals:
    """Processed tactile signals for a single update."""
    finger_force: List[float]      # SA-I: Total tactile per finger
    finger_derivative: List[float] # FA-I: Rate of change
    finger_contact: List[bool]     # Contact detected per finger
    finger_slip: List[bool]        # Slip detected per finger
    total_contact: float           # Total tactile sum
    palm_contact: float            # Palm tactile sum


class TactileProcessor:
    """
    Process tactile data for contact and slip detection.

    Implements human-inspired tactile signal processing:
    - SA-I (Merkel): Steady-state force from tactile sum
    - FA-I (Meissner): Force disturbance from high-pass filtered tactile

    Based on Romano et al., "Human-Inspired Robotic Grasp Control
    With Tactile Sensing", IEEE TRO 2011.
    """

    # Finger names for reference
    FINGER_NAMES = ['little', 'ring', 'middle', 'index', 'thumb', 'thumb_rotate']

    def __init__(self, sample_rate: float = 50.0,
                 contact_threshold: float = 500.0,
                 derivative_threshold: float = 100.0,
                 slip_threshold: float = 0.12,
                 min_contact_for_slip: float = 200.0):
        """
        Initialize tactile processor.

        Args:
            sample_rate: Tactile update rate in Hz
            contact_threshold: Tactile sum threshold for contact detection
            derivative_threshold: Derivative threshold for contact detection
            slip_threshold: Weber's law threshold (0.12 = 12%)
            min_contact_for_slip: Minimum contact to consider slip
        """
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate

        # Thresholds
        self.contact_threshold = contact_threshold
        self.derivative_threshold = derivative_threshold
        self.slip_threshold = slip_threshold
        self.min_contact_for_slip = min_contact_for_slip

        # State for derivative computation
        self.prev_finger_force = [0.0] * 6
        self.prev_palm_force = 0.0

        # Simple first-order high-pass filter state
        # Cutoff frequency ~5Hz for FA-I signal
        self.alpha = 2 * math.pi * 5.0 * self.dt / (2 * math.pi * 5.0 * self.dt + 1)
        self.filtered_force = [0.0] * 6

    def compute_finger_force(self, tactile: dict) -> List[float]:
        """
        Compute SA-I signal: sum of tactile values per finger.

        Args:
            tactile: Dictionary with tactile arrays

        Returns:
            List of 6 force values (one per finger, thumb_rotate = 0)
        """
        def safe_sum(key: str) -> float:
            val = tactile.get(key, [])
            return float(sum(val)) if val else 0.0

        return [
            # Little finger (DOF 0)
            safe_sum('fingerone_tip') + safe_sum('fingerone_nail') + safe_sum('fingerone_pad'),
            # Ring finger (DOF 1)
            safe_sum('fingertwo_tip') + safe_sum('fingertwo_nail') + safe_sum('fingertwo_pad'),
            # Middle finger (DOF 2)
            safe_sum('fingerthree_tip') + safe_sum('fingerthree_nail') + safe_sum('fingerthree_pad'),
            # Index finger (DOF 3)
            safe_sum('fingerfour_tip') + safe_sum('fingerfour_nail') + safe_sum('fingerfour_pad'),
            # Thumb (DOF 4)
            safe_sum('fingerfive_tip') + safe_sum('fingerfive_nail') +
            safe_sum('fingerfive_middle') + safe_sum('fingerfive_pad'),
            # Thumb rotation (DOF 5) - no tactile sensors
            0.0
        ]

    def compute_palm_force(self, tactile: dict) -> float:
        """Compute palm tactile sum."""
        palm = tactile.get('palm', [])
        return float(sum(palm)) if palm else 0.0

    def compute_derivative(self, current: List[float]) -> List[float]:
        """
        Compute FA-I signal: high-pass filtered force (detects changes).

        Uses a simple first-order high-pass filter approximation.

        Args:
            current: Current finger force values

        Returns:
            List of derivative values per finger
        """
        derivative = []
        for i, (curr, prev) in enumerate(zip(current, self.prev_finger_force)):
            # Simple derivative
            d = (curr - prev) * self.sample_rate

            # Apply high-pass filter (exponential smoothing of derivative)
            self.filtered_force[i] = self.alpha * d + (1 - self.alpha) * self.filtered_force[i]
            derivative.append(self.filtered_force[i])

        self.prev_finger_force = current.copy()
        return derivative

    def detect_contact(self, finger_force: List[float],
                       finger_derivative: List[float]) -> List[bool]:
        """
        Detect contact on each finger.

        Contact is detected when either:
        - Tactile sum exceeds threshold (SA-I)
        - Derivative exceeds threshold (FA-I, more sensitive)

        Args:
            finger_force: SA-I signal (tactile sums)
            finger_derivative: FA-I signal (derivatives)

        Returns:
            List of contact booleans per finger
        """
        return [
            (force > self.contact_threshold) or (abs(deriv) > self.derivative_threshold)
            for force, deriv in zip(finger_force, finger_derivative)
        ]

    def detect_slip(self, finger_force: List[float],
                    finger_derivative: List[float]) -> List[bool]:
        """
        Detect slip using Weber's Law.

        Slip is detected when the rate of change exceeds a percentage
        of the current force level. This matches human perception
        where relative changes are more important than absolute values.

        From Romano et al.: slip_threshold ~0.12 (12%)

        Args:
            finger_force: SA-I signal (tactile sums)
            finger_derivative: FA-I signal (derivatives)

        Returns:
            List of slip booleans per finger
        """
        return [
            # Must be in contact and derivative > percentage of current force
            (force > self.min_contact_for_slip) and
            (abs(deriv) > force * self.slip_threshold)
            for force, deriv in zip(finger_force, finger_derivative)
        ]

    def process(self, tactile: dict) -> TactileSignals:
        """
        Process tactile data and return all signals.

        Args:
            tactile: Dictionary with tactile arrays from driver

        Returns:
            TactileSignals dataclass with all processed signals
        """
        # Compute SA-I signal (steady force)
        finger_force = self.compute_finger_force(tactile)
        palm_contact = self.compute_palm_force(tactile)

        # Compute FA-I signal (force disturbance)
        finger_derivative = self.compute_derivative(finger_force)

        # Detect contact and slip
        finger_contact = self.detect_contact(finger_force, finger_derivative)
        finger_slip = self.detect_slip(finger_force, finger_derivative)

        # Total contact
        total_contact = sum(finger_force) + palm_contact

        return TactileSignals(
            finger_force=finger_force,
            finger_derivative=finger_derivative,
            finger_contact=finger_contact,
            finger_slip=finger_slip,
            total_contact=total_contact,
            palm_contact=palm_contact
        )

    def reset(self):
        """Reset processor state."""
        self.prev_finger_force = [0.0] * 6
        self.prev_palm_force = 0.0
        self.filtered_force = [0.0] * 6

    def get_contact_count(self, contacts: List[bool]) -> int:
        """Count number of fingers in contact."""
        return sum(contacts)

    def get_contact_bitfield(self, contacts: List[bool]) -> int:
        """Convert contact list to bitfield."""
        bitfield = 0
        for i, c in enumerate(contacts):
            if c:
                bitfield |= (1 << i)
        return bitfield

    def is_stable_grasp(self, contacts: List[bool], min_fingers: int = 2) -> bool:
        """
        Check if grasp is stable (multiple fingers in contact).

        Args:
            contacts: Contact list per finger
            min_fingers: Minimum fingers required for stable grasp

        Returns:
            True if stable grasp detected
        """
        return self.get_contact_count(contacts) >= min_fingers

    def any_slip_detected(self, slips: List[bool]) -> bool:
        """Check if slip detected on any finger."""
        return any(slips)


class AdaptiveForceEstimator:
    """
    Estimate appropriate grip force based on object properties.

    Based on Romano et al.'s adaptive force selection during
    the Load phase of grasping.
    """

    def __init__(self, k_hardness: float = 0.25,
                 k_slip: float = 1.05,
                 max_force: int = 2000,
                 min_force: int = 100):
        """
        Initialize force estimator.

        Args:
            k_hardness: Scaling factor for hardness-based force
            k_slip: Force multiplier on slip (1.05 = 5% increase)
            max_force: Maximum allowed force in grams
            min_force: Minimum force in grams
        """
        self.k_hardness = k_hardness
        self.k_slip = k_slip
        self.max_force = max_force
        self.min_force = min_force

        self.current_force = 500  # Default starting force

    def estimate_initial_force(self, max_force_during_contact: float,
                               close_speed: float = 500.0) -> int:
        """
        Estimate initial grip force based on object stiffness.

        From Romano et al.:
        F_target = max_force_during_settle * K_HARDNESS / close_speed

        Args:
            max_force_during_contact: Maximum tactile during initial contact
            close_speed: Closing speed used (0-1000)

        Returns:
            Estimated grip force in grams
        """
        if close_speed <= 0:
            close_speed = 500.0

        # Normalize speed to 0-1 range
        normalized_speed = close_speed / 1000.0

        # Estimate force
        estimated = int(max_force_during_contact * self.k_hardness / normalized_speed)

        # Clamp to valid range
        self.current_force = max(self.min_force, min(self.max_force, estimated))
        return self.current_force

    def adjust_for_slip(self) -> int:
        """
        Increase force after slip detection.

        Returns:
            New force value in grams
        """
        self.current_force = min(
            int(self.current_force * self.k_slip),
            self.max_force
        )
        return self.current_force

    def set_force(self, force: int) -> int:
        """Set force directly."""
        self.current_force = max(self.min_force, min(self.max_force, force))
        return self.current_force

    def get_force(self) -> int:
        """Get current force target."""
        return self.current_force

    def reset(self, initial_force: int = 500):
        """Reset to initial force."""
        self.current_force = max(self.min_force, min(self.max_force, initial_force))
