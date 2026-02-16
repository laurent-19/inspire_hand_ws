"""
Inspire Hand ROS2 Package

ROS2 wrapper for the RH56DFTP Dexterous Hand with human-inspired grasp control.
"""

# When running as installed scripts, imports are direct
try:
    from tactile_processor import TactileProcessor
    from grasp_controller import GraspController, GraspState
except ImportError:
    # Fallback for when imported as a package
    from .tactile_processor import TactileProcessor
    from .grasp_controller import GraspController, GraspState

__all__ = ['TactileProcessor', 'GraspController', 'GraspState']
