import multiprocessing
import time
import sys
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
    # Usage: python Headless_driver_double.py [interface] [domain]
    # For default: python Headless_driver_double.py
    # Custom settings: python Headless_driver_double.py enp0s31f6 0
    
    hand_ip = '192.168.123.211'
    network_interface = sys.argv[1] if len(sys.argv) > 1 else "enp0s31f6"
    domain = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    
    print(f"Starting Headless Driver: hand_ip={hand_ip}, interface={network_interface}, domain={domain}")
    
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
