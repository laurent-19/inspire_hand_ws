

from .inspire_dds import inspire_hand_touch,inspire_hand_ctrl,inspire_hand_state
import threading
modbus_lock = threading.Lock()

# Data definition   
data_sheet = [
    ("Little finger tip tactile data", 3000, 18, (3, 3), "fingerone_tip_touch"),      # Little finger tip tactile data
    ("Little finger top tactile data", 3018, 192, (12, 8), "fingerone_top_touch"),      # Little finger top tactile data
    ("Little finger palm tactile data", 3210, 160, (10, 8), "fingerone_palm_touch"),     # Little finger palm tactile data
    ("Ring finger tip tactile data", 3370, 18, (3, 3), "fingertwo_tip_touch"),      # Ring finger tip tactile data
    ("Ring finger top tactile data", 3388, 192, (12, 8), "fingertwo_top_touch"),      # Ring finger top tactile data
    ("Ring finger palm tactile data", 3580, 160, (10, 8), "fingertwo_palm_touch"),     # Ring finger palm tactile data
    ("Middle finger tip tactile data", 3740, 18, (3, 3), "fingerthree_tip_touch"),    # Middle finger tip tactile data
    ("Middle finger top tactile data", 3758, 192, (12, 8), "fingerthree_top_touch"),    # Middle finger top tactile data
    ("Middle finger palm tactile data", 3950, 160, (10, 8), "fingerthree_palm_touch"),   # Middle finger palm tactile data
    ("Index finger tip tactile data", 4110, 18, (3, 3), "fingerfour_tip_touch"),     # Index finger tip tactile data
    ("Index finger top tactile data", 4128, 192, (12, 8), "fingerfour_top_touch"),     # Index finger top tactile data
    ("Index finger palm tactile data", 4320, 160, (10, 8), "fingerfour_palm_touch"),    # Index finger palm tactile data
    ("Thumb tip tactile data", 4480, 18, (3, 3), "fingerfive_tip_touch"),     # Thumb tip tactile data
    ("Thumb top tactile data", 4498, 192, (12, 8), "fingerfive_top_touch"),     # Thumb top tactile data
    ("Thumb middle tactile data", 4690, 18, (3, 3), "fingerfive_middle_touch"),  # Thumb middle tactile data
    ("Thumb palm tactile data", 4708, 192, (12, 8), "fingerfive_palm_touch"),    # Thumb palm tactile data
    ("Palm tactile data", 4900, 224, (14, 8), "palm_touch")                # Palm tactile data
]
status_codes = {
    0: "Releasing",
    1: "Grasping",
    2: "Position reached, stopped",
    3: "Force control reached, stopped",
    5: "Current protection, stopped",
    6: "Actuator stall, stopped",
    7: "Actuator fault, stopped",
    255: "Error"
}
error_descriptions = {
    0: "Stall fault",
    1: "Over-temperature fault",
    2: "Over-current fault",
    3: "Motor abnormal",
    4: "Communication fault"
}
def get_error_description(error_value):
    error_reasons = []
    # Check if each bit is 1, if so add the corresponding fault description
    for bit, description in error_descriptions.items():
        if error_value & (1 << bit):  # Use bitwise operation to check if the bit is 1
            error_reasons.append(description)
    return error_reasons

# Print combined fault reasons
def update_error_label(ERROR):
    error_summary = []
    for e in ERROR:
        binary_error = '{:04b}'.format(int(e))  # Convert to 4-bit binary representation
        error_reasons = get_error_description(int(e))  # Get fault reason list
        if error_reasons:
            error_summary.append(f"ERROR {e} ({binary_error}): " + ', '.join(error_reasons))
        else:
            error_summary.append(f"ERROR {e} ({binary_error}): No fault")
    # Update label content
    # print("\n".join(error_summary))
    return"\t".join(error_summary)
       
       
 
def get_inspire_hand_touch():
    return inspire_hand_touch(
        fingerone_tip_touch=[0 for _ in range(9)],        # Little finger tip tactile data
        fingerone_top_touch=[0 for _ in range(96)],       # Little finger top tactile data
        fingerone_palm_touch=[0 for _ in range(80)],      # Little finger palm tactile data
        fingertwo_tip_touch=[0 for _ in range(9)],        # Ring finger tip tactile data
        fingertwo_top_touch=[0 for _ in range(96)],       # Ring finger top tactile data
        fingertwo_palm_touch=[0 for _ in range(80)],      # Ring finger palm tactile data
        fingerthree_tip_touch=[0 for _ in range(9)],      # Middle finger tip tactile data
        fingerthree_top_touch=[0 for _ in range(96)],     # Middle finger top tactile data
        fingerthree_palm_touch=[0 for _ in range(80)],    # Middle finger palm tactile data
        fingerfour_tip_touch=[0 for _ in range(9)],       # Index finger tip tactile data
        fingerfour_top_touch=[0 for _ in range(96)],      # Index finger top tactile data
        fingerfour_palm_touch=[0 for _ in range(80)],     # Index finger palm tactile data
        fingerfive_tip_touch=[0 for _ in range(9)],       # Thumb tip tactile data
        fingerfive_top_touch=[0 for _ in range(96)],      # Thumb top tactile data
        fingerfive_middle_touch=[0 for _ in range(9)],    # Thumb middle tactile data
        fingerfive_palm_touch=[0 for _ in range(96)],     # Thumb palm tactile data
        palm_touch=[0 for _ in range(112)]                # Palm tactile data
    )
    
def get_inspire_hand_state():
    return inspire_hand_state(
        pos_act=[0 for _ in range(6)],        # Actual position for each DOF
        angle_act=[0 for _ in range(6)],       # Actual angle for each DOF
        force_act=[0 for _ in range(6)],      # Actual force for each DOF
        current=[0 for _ in range(6)],        # Current for each DOF
        err=[0 for _ in range(6)],        # Error codes for each DOF
        status=[0 for _ in range(6)],        # Status for each DOF
        temperature=[0 for _ in range(6)],        # Temperature for each DOF
    ) 

def get_inspire_hand_ctrl():
    return inspire_hand_ctrl(
        pos_set=[0 for _ in range(6)],        # Position setpoint for each DOF
        angle_set=[0 for _ in range(6)],       # Angle setpoint for each DOF
        force_set=[0 for _ in range(6)],      # Force setpoint for each DOF
        speed_set=[0 for _ in range(6)],        # Speed setpoint for each DOF
        mode=0b0000
    ) 

defaut_ip='192.168.11.210'