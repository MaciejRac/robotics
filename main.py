import time
import struct
from st_hsdatalog.HSD_link.HSDLink import HSDLink

# 1. Initialize Connection
hsd_link = HSDLink()
hsd_link_instance = hsd_link.create_hsd_link()
device_id = 0

if hsd_link_instance is None:
    print("Could not connect.")
    exit()

# 2. Get the v1 Device Object (This returns a Class Instance, NOT a dictionary)
device = hsd_link.get_device(hsd_link_instance, device_id)

if device is None:
    print("Failed to get device info. Try resetting the board.")
    exit()

# Access the board name using dot notation
board_name = device.device_info.alias if hasattr(device, 'device_info') else "SensorTile.box"
print(f"Connected to: {board_name}")

# 3. Start Streaming
hsd_link.start_log(hsd_link_instance, device_id)
print("Log started. Polling for data...")

try:
    while True:
        # Loop through using enumerate so we can match the descriptor to the status
        for s_idx, sensor in enumerate(device.sensor):
            s_id = sensor.id

            for ss_idx, sub_sensor in enumerate(sensor.sensor_descriptor.sub_sensor_descriptor):
                ss_id = sub_sensor.id 
                sensor_type = sub_sensor.sensor_type 
                
                # --- IMPORTANT: sensor cheatsheet   ---
                # --- FORMAT: sensor_id.subsensor_id ---
                # ACC: 0.0, 2.0, 4.0
                # GYRO: 4.1
                # MIC: 6.0
                if sensor_type == 'ACC' or sensor_type == "":
                    
                    # Fetch the live data
                    sensor_data = hsd_link_instance.get_sensor_data(device_id, s_id, ss_id)
                    
                    if sensor_data is not None and len(sensor_data) == 2:
                        data_length = sensor_data[0] # Number of valid bytes/samples
                        raw_bytes = sensor_data[1]   # The raw byte string
                        
                        # 2. Check the active Full Scale (fs) to determine sensitivity
                        # This grabs the dynamic status for this specific sub-sensor
                        status = device.sensor[s_idx].sensor_status.sub_sensor_status[ss_idx]
                        
                        sensitivity = status.sensitivity
                        
                        # Each ACC sample is 3 axes (X, Y, Z). 
                        # Each axis is a 16-bit signed integer (2 bytes). Total = 6 bytes per sample.
                        bytes_per_sample = 6
                        
                        # Loop through the raw bytes in chunks of 6
                        for i in range(0, len(raw_bytes), bytes_per_sample):
                            chunk = raw_bytes[i:i+bytes_per_sample]
                            
                            # Ensure we have a full 6-byte chunk to unpack
                            if len(chunk) == bytes_per_sample:
                                # '<hhh' unpacks three Little-Endian 16-bit signed integers
                                raw_x, raw_y, raw_z = struct.unpack('<hhh', chunk)
                                
                                # Multiply by sensitivity to get the real values (usually in mg)
                                real_x = raw_x * sensitivity
                                real_y = raw_y * sensitivity
                                real_z = raw_z * sensitivity
                                
                                #print(f"ACC [mg] -> X: {real_x:.2f}, Y: {real_y:.2f}, Z: {real_z:.2f}")
                    
                    
                        
        time.sleep(0.05) # Poll buffer every 50ms

except KeyboardInterrupt:
    hsd_link.stop_log(hsd_link_instance, device_id)
    print("\nStream safely stopped.")