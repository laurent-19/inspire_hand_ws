import multiprocessing
import time
from inspire_sdkpy import inspire_sdk, inspire_hand_defaut

def worker(ip,LR,name,network=None):
    handler=inspire_sdk.ModbusDataHandler(network=network,ip=ip, LR=LR, device_id=1)

    call_count = 0
    start_time = time.perf_counter()
    time.sleep(0.5)
    
    try:
        while True:
            data_dict = handler.read()
            call_count += 1
            time.sleep(0.001)
            
            if call_count % 10 == 0:
                elapsed_time = time.perf_counter() - start_time
                frequency = call_count / elapsed_time
                print(f"{name} Current frequency: {frequency:.2f} Hz, Call count: {call_count}, Elapsed time: {elapsed_time:.6f} s")
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - start_time
        frequency = call_count / elapsed_time if elapsed_time > 0 else 0
        print(f"{name} Program ended. Total calls: {call_count}, Total time: {elapsed_time:.6f} s, Final frequency: {frequency:.2f} Hz")

if __name__ == "__main__":
    # Example using default IP address
    # Specify network interface for DDS (use 'lo' for localhost or your interface name)
    network_interface = "lo"  # Use loopback for local DDS communication
    
    process_r = multiprocessing.Process(target=worker, args=('192.168.123.211','r',"Right hand process", network_interface))
    #process_l = multiprocessing.Process(target=worker, args=('192.168.123.210','l',"Left hand process", network_interface))

    process_r.start()
    time.sleep(0.6)
    #process_l.start()

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        process_r.terminate()
        #process_l.terminate()
