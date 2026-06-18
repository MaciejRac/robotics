# import time
# import struct
# from st_hsdatalog.HSD_link.HSDLink import HSDLink

# # --- IMPORTANT: sensor cheatsheet   ---
# # --- FORMAT: sensor_id.subsensor_id ---
# # ACC: 0.0, 2.0, 4.0
# # GYRO: 4.1
# # MIC: 6.0
# ACC_S_ID = 0
# ACC_SS_ID = 0
# GYRO_S_ID = 4
# GYRO_SS_ID = 1

# DEVICE_ID = 0

# # 1. Initialize Connection
# hsd_link_factory = HSDLink()
# hsd_link = hsd_link_factory.create_hsd_link()

# if hsd_link is None:
#     print("Could not connect.")
#     exit()

# # 2. Get the v1 Device Object (This returns a Class Instance, NOT a dictionary)
# device = hsd_link_factory.get_device(hsd_link, DEVICE_ID)

# if device is None:
#     print("Failed to get device info. Try resetting the board.")
#     exit()

# # Access the board name using dot notation
# board_name = device.device_info.alias if hasattr(device, 'device_info') else "SensorTile.box"
# print(f"Connected to: {board_name}")

# # 3. Start Streaming
# hsd_link_factory.start_log(hsd_link, DEVICE_ID)
# print("Log started. Polling for data...")

# try:
#     while True:
#         acc_data = hsd_link.get_sensor_data(DEVICE_ID, ACC_S_ID, ACC_SS_ID)
#         gyro_data = hsd_link.get_sensor_data(DEVICE_ID, GYRO_S_ID, GYRO_SS_ID)

#         acc_sensitivity = device.sensor[ACC_S_ID].sensor_status.sub_sensor_status[ACC_SS_ID].sensitivity
#         gyro_sensitivity = device.sensor[GYRO_S_ID].sensor_status.sub_sensor_status[GYRO_SS_ID].sensitivity

#         if acc_data is not None and len(acc_data) == 2:
#             data_length = acc_data[0] # Number of valid bytes/samples
#             raw_bytes = acc_data[1]   # The raw byte string
            

#             # PRINTING ACCELEROMETER DATA


#             # Each ACC sample is 3 axes (X, Y, Z). 
#             # Each axis is a 16-bit signed integer (2 bytes). Total = 6 bytes per sample.
#             bytes_per_sample = 6
            
#             # Loop through the raw bytes in chunks of 6
#             for i in range(0, len(raw_bytes), bytes_per_sample):
#                 chunk = raw_bytes[i:i+bytes_per_sample]
                
#                 # Ensure we have a full 6-byte chunk to unpack
#                 if len(chunk) == bytes_per_sample:
#                     # '<hhh' unpacks three Little-Endian 16-bit signed integers
#                     raw_x, raw_y, raw_z = struct.unpack('<hhh', chunk)
                    
#                     # Multiply by sensitivity to get the real values (usually in mg)
#                     real_x = raw_x * acc_sensitivity
#                     real_y = raw_y * acc_sensitivity
#                     real_z = raw_z * acc_sensitivity
                    
#                     print(f"ACC [mg] -> X: {real_x:.2f}, Y: {real_y:.2f}, Z: {real_z:.2f}")
                    
                    
                        
#         time.sleep(0.05) # Poll buffer every 50ms

# except KeyboardInterrupt:
#     hsd_link_factory.stop_log(hsd_link, DEVICE_ID)
#     print("\nStream safely stopped.")

import time
import struct
from st_hsdatalog.HSD_link.HSDLink import HSDLink

# --- IMPORTANT: sensor cheatsheet   ---
# --- FORMAT: sensor_id.subsensor_id ---
# ACC: 0.0, 2.0, 4.0
# GYRO: 4.1
# MIC: 6.0
ACC_S_ID = 0
ACC_SS_ID = 0
GYRO_S_ID = 4
GYRO_SS_ID = 1

DEVICE_ID = 0

# 1. Initialize Connection
hsd_link_factory = HSDLink()
hsd_link = hsd_link_factory.create_hsd_link()

if hsd_link is None:
    print("Could not connect.")
    exit()

# 2. Get the v1 Device Object (This returns a Class Instance, NOT a dictionary)
device = hsd_link_factory.get_device(hsd_link, DEVICE_ID)

if device is None:
    print("Failed to get device info. Try resetting the board.")
    exit()

# Access the board name using dot notation
board_name = device.device_info.alias if hasattr(device, 'device_info') else "SensorTile.box"
print(f"Connected to: {board_name}")

# 3. Start Streaming
hsd_link_factory.start_log(hsd_link, DEVICE_ID)
print("Log started. Polling for data...")

try:
    while True:
        # Pętla po wszystkich głównych czujnikach
        for s_idx, sensor in enumerate(device.sensor):
            s_id = sensor.id

            # Pętla po sub-czujnikach (np. ACC i GYRO często dzielą ten sam fizyczny sensor)
            for ss_idx, sub_sensor in enumerate(sensor.sensor_descriptor.sub_sensor_descriptor):
                ss_id = sub_sensor.id 
                sensor_type = sub_sensor.sensor_type 
                
                # Interesują nas tylko ACC, GYRO i MAG
                if sensor_type in ['ACC', 'GYRO', 'MAG']:
                    
                    # Pobierz dane z bufora
                    sensor_data = hsd_link.get_sensor_data(DEVICE_ID, s_id, ss_id)
                    
                    if sensor_data is not None and len(sensor_data) == 2:
                        data_length = sensor_data[0] # Liczba ważnych bajtów
                        raw_bytes = sensor_data[1]   # Surowe dane
                        
                        # Pobierz aktualną czułość dla danego sub-czujnika
                        status = device.sensor[s_idx].sensor_status.sub_sensor_status[ss_idx]
                        sensitivity = status.sensitivity
                        
                        # Każda próbka to 3 osie (X, Y, Z), każda to 2 bajty = 6 bajtów na próbkę
                        bytes_per_sample = 6
                        
                        # Przechodzimy przez surowe bajty w paczkach po 6
                        for i in range(0, len(raw_bytes), bytes_per_sample):
                            chunk = raw_bytes[i:i+bytes_per_sample]
                            
                            if len(chunk) == bytes_per_sample:
                                # '<hhh' rozpakowuje trzy 16-bitowe liczby całkowite ze znakiem (Little-Endian)
                                raw_x, raw_y, raw_z = struct.unpack('<hhh', chunk)
                                
                                # Przemnóż przez czułość, żeby uzyskać realne wartości
                                real_x = raw_x * sensitivity
                                real_y = raw_y * sensitivity
                                real_z = raw_z * sensitivity
                                
                                # Wyświetl dane z odpowiednimi jednostkami w zależności od typu
                                if sensor_type == 'ACC':
                                    print(f"ACC [mg]     -> X: {real_x:8.2f}, Y: {real_y:8.2f}, Z: {real_z:8.2f}")
                                elif sensor_type == 'GYRO':
                                    print(f"GYRO [mdps]  -> X: {real_x:8.2f}, Y: {real_y:8.2f}, Z: {real_z:8.2f}")
                                elif sensor_type == 'MAG':
                                    print(f"MAG [mgauss] -> X: {real_x:8.2f}, Y: {real_y:8.2f}, Z: {real_z:8.2f}")
                                    
        time.sleep(0.05) # Odpytuj bufor co 50ms

except KeyboardInterrupt:
    hsd_link_factory.stop_log(hsd_link, DEVICE_ID)
    print("\nStream safely stopped.")