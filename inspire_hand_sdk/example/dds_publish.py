import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.utils.thread import Thread

from inspire_sdkpy import inspire_hand_defaut,inspire_dds
import numpy as np

if __name__ == '__main__':
    # Use loopback interface for local DDS communication
    network_interface = "lo"
    
    # Domain 1 for simulation, domain 0 for local testing
    domain = 1 if len(sys.argv) > 1 else 0
    interface = sys.argv[1] if len(sys.argv) > 1 else network_interface
    ChannelFactoryInitialize(domain, interface)
    # Create a publisher to publish the data defined in UserData class
    pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pubr.Init()
    
    publ = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
    publ.Init()
    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    short_value=1000


    cmd.angle_set=[1000,1000,1000,1000,1000,1000]
    cmd.mode=0b0001
    publ.Write(cmd)
    pubr.Write(cmd)

    time.sleep(1.0)

    cmd.angle_set=[1000,1000,1000,1000,1000,1000]
    cmd.mode=0b0001
    publ.Write(cmd)
    pubr.Write(cmd)

    time.sleep(3.0)

    for cnd in range(1000000): 

            # Register start address, 0x05CE corresponds to 1486
        start_address = 1486            
        num_registers = 6  # 6 registers
        # Generate the list of values to write, each register is a short value

        if (cnd+1) % 10 == 0:
            short_value = 1000-short_value  # Short value to write



        values_to_write = [short_value] * num_registers
        values_to_write[-1]=1000-values_to_write[-1]
        values_to_write[-2]=1000-values_to_write[-2]

        value_to_write_np=np.array(values_to_write)
        value_to_write_np=np.clip(value_to_write_np,200,800)
        # value_to_write_np[3]=800

        # Combined modes implemented in binary
        # mode 0:  0000 (no operation)
        # mode 1:  0001 (angle)
        # mode 2:  0010 (position)
        # mode 3:  0011 (angle + position)
        # mode 4:  0100 (force control)
        # mode 5:  0101 (angle + force control)
        # mode 6:  0110 (position + force control)
        # mode 7:  0111 (angle + position + force control)
        # mode 8:  1000 (speed)
        # mode 9:  1001 (angle + speed)
        # mode 10: 1010 (position + speed)
        # mode 11: 1011 (angle + position + speed)
        # mode 12: 1100 (force control + speed)
        # mode 13: 1101 (angle + force control + speed)
        # mode 14: 1110 (position + force control + speed)
        # mode 15: 1111 (angle + position + force control + speed)  
        cmd.angle_set=value_to_write_np.tolist()
        cmd.mode=0b0001
        # Publish message
        publ.Write(cmd)
        pubr.Write(cmd)

        time.sleep(0.1)
        
