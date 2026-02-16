"""
Visualization data definitions (extracted from inspire_hand_sdk).
"""

import threading

modbus_lock = threading.Lock()

# Data definition for tactile sensors
data_sheet = [
    ("Little finger tip", 3000, 18, (3, 3), "fingerone_tip_touch"),
    ("Little finger nail", 3018, 192, (12, 8), "fingerone_top_touch"),
    ("Little finger pad", 3210, 160, (10, 8), "fingerone_palm_touch"),
    ("Ring finger tip", 3370, 18, (3, 3), "fingertwo_tip_touch"),
    ("Ring finger nail", 3388, 192, (12, 8), "fingertwo_top_touch"),
    ("Ring finger pad", 3580, 160, (10, 8), "fingertwo_palm_touch"),
    ("Middle finger tip", 3740, 18, (3, 3), "fingerthree_tip_touch"),
    ("Middle finger nail", 3758, 192, (12, 8), "fingerthree_top_touch"),
    ("Middle finger pad", 3950, 160, (10, 8), "fingerthree_palm_touch"),
    ("Index finger tip", 4110, 18, (3, 3), "fingerfour_tip_touch"),
    ("Index finger nail", 4128, 192, (12, 8), "fingerfour_top_touch"),
    ("Index finger pad", 4320, 160, (10, 8), "fingerfour_palm_touch"),
    ("Thumb tip", 4480, 18, (3, 3), "fingerfive_tip_touch"),
    ("Thumb nail", 4498, 192, (12, 8), "fingerfive_top_touch"),
    ("Thumb middle", 4690, 18, (3, 3), "fingerfive_middle_touch"),
    ("Thumb pad", 4708, 192, (12, 8), "fingerfive_palm_touch"),
    ("Palm", 4900, 224, (14, 8), "palm_touch")
]

status_codes = {
    0: "Releasing",
    1: "Grasping",
    2: "Position reached",
    3: "Force reached",
    5: "Current protection",
    6: "Stall",
    7: "Fault",
    255: "Error"
}

error_descriptions = {
    0: "Stall fault",
    1: "Over-temperature",
    2: "Over-current",
    3: "Motor abnormal",
    4: "Communication fault"
}


def get_error_description(error_value):
    error_reasons = []
    for bit, description in error_descriptions.items():
        if error_value & (1 << bit):
            error_reasons.append(description)
    return error_reasons


def update_error_label(ERROR):
    error_summary = []
    for e in ERROR:
        binary_error = '{:04b}'.format(int(e))
        error_reasons = get_error_description(int(e))
        if error_reasons:
            error_summary.append(f"ERR {e}: " + ', '.join(error_reasons))
        else:
            error_summary.append(f"OK")
    return "\t".join(error_summary)
