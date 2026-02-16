
---

# Inspire Hand SDK Usage Guide

## Virtual Environment Management

It is recommended to use `venv` for managing the virtual environment:

```bash
python -m venv venv  # or  Unzip venv_x86.tar.xz, and place the.venv in inspire_hand_ws/.venv

# Then execute the script to modify venv:
python update_venv_path.py.venv
python update_bin_files.py.venv 

source venv/bin/activate  # Activate the virtual environment for Linux/MacOS

```

## Installation

1. When configuring the environment yourself, you need to install project dependencies; if you use Unzip venv_x86.tar.xz to set up the environment, you do not need to run the following command:

    ```bash
    pip install -r requirements.txt
    ```

2. Initialize and update submodules:

    ```bash
    git submodule init  # Initialize submodules
    git submodule update  # Update submodules to the latest version
    ```

3. Install the two SDKs:

    ```bash
    cd unitree_sdk2_python
    pip install -e .

    cd ../inspire_hand_sdk
    pip install -e .
    ```

## Control Modes

The Inspire Hand SDK supports multiple control modes, defined as follows:

- **Mode 0**: `0000` (No operation)
- **Mode 1**: `0001` (Angle)
- **Mode 2**: `0010` (Position)
- **Mode 3**: `0011` (Angle + Position)
- **Mode 4**: `0100` (Force control)
- **Mode 5**: `0101` (Angle + Force control)
- **Mode 6**: `0110` (Position + Force control)
- **Mode 7**: `0111` (Angle + Position + Force control)
- **Mode 8**: `1000` (Velocity)
- **Mode 9**: `1001` (Angle + Velocity)
- **Mode 10**: `1010` (Position + Velocity)
- **Mode 11**: `1011` (Angle + Position + Velocity)
- **Mode 12**: `1100` (Force control + Velocity)
- **Mode 13**: `1101` (Angle + Force control + Velocity)
- **Mode 14**: `1110` (Position + Force control + Velocity)
- **Mode 15**: `1111` (Angle + Position + Force control + Velocity)

## Network Configuration

All DDS scripts use consistent default settings:
- **Default Network Interface**: `enp0s31f6` (adjust to match your network interface)
- **Default DDS Domain**: `0` (all components must use the same domain to communicate)
- **Default Hand IP**: `192.168.123.211` (for Headless_driver)

To find your network interface, run: `ip addr` or `ifconfig`

## Usage Examples

Below are instructions for using common examples:

### Step 1: Start the Headless Driver (required for DDS communication)

The Headless Driver bridges the Inspire Hand's ModbusTCP connection to DDS:

```bash
# Default settings (hand IP: 192.168.123.211, interface: enp0s31f6, domain: 0)
python inspire_hand_sdk/example/Headless_driver_double.py

# Custom settings
python inspire_hand_sdk/example/Headless_driver_double.py [interface] [domain]
# Example: python inspire_hand_sdk/example/Headless_driver_double.py enp0s31f6 0
```

### Step 2: Publish Control Commands

Publish control commands to the Inspire Hand via DDS:

```bash
# Default settings (interface: enp0s31f6, domain: 0)
python inspire_hand_sdk/example/dds_publish.py

# Custom settings
python inspire_hand_sdk/example/dds_publish.py [interface] [domain]
# Example: python inspire_hand_sdk/example/dds_publish.py enp0s31f6 0
```

### Step 3: Subscribe to Hand State and Sensor Data

Visualize the Inspire Hand status and tactile sensor data:

```bash
# Default settings (interface: enp0s31f6, domain: 0, hand: right)
python inspire_hand_sdk/example/dds_subscribe.py

# Custom settings
python inspire_hand_sdk/example/dds_subscribe.py [interface] [l/r] [domain]
# Example: python inspire_hand_sdk/example/dds_subscribe.py enp0s31f6 r 0
```

---

## Additional Tools

3. **Inspire Hand DDS Driver (Headless Mode)**:

    Use the following script for the headless mode driver:
    ```bash
    python inspire_hand_sdk/example/Headless_driver.py
    ```

4. **Inspire Hand Configuration Panel**:

    Run the following script to use the Inspire Hand configuration panel:
    ```bash
    python inspire_hand_sdk/example/init_set_inspire_hand.py
    ```

5. **Inspire Hand DDS Driver (Panel Mode)**:

    Use the following script to enter panel mode for the Inspire Hand DDS driver:
    ```bash
    python inspire_hand_sdk/example/Vision_driver.py
    ```

---
