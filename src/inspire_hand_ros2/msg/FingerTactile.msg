# FingerTactile.msg - Tactile data for a single finger
# Each finger has tip, nail (top), and pad regions
# Thumb additionally has a middle region

string name                   # Finger name: "little", "ring", "middle", "index", "thumb"

int16[] tip                   # Tip tactile values (3x3 = 9 values)
int16[] nail                  # Nail/top tactile values (12x8 = 96 values)
int16[] pad                   # Pad/palm tactile values (10x8=80 or 12x8=96 for thumb)
int16[] middle_section        # Middle section (3x3 = 9, thumb only, empty for others)

# Aggregate metrics
float32 total_pressure        # Sum of all tactile values for this finger
float32 max_pressure          # Maximum tactile value
int32 contact_count           # Number of taxels above threshold
