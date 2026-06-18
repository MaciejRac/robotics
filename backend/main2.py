import time
import struct
import json
import numpy as np
from websocket import create_connection
from ahrs.filters import Madgwick
from ahrs.common.quaternion import Quaternion
from st_hsdatalog.HSD_link.HSDLink import HSDLink

# --- 1. Konfiguracja WebSocket ---
WS_URL = "ws://localhost:8080"
try:
    ws = create_connection(WS_URL)
    print("Połączono z serwerem WebSocket.")
except Exception as e:
    print(f"Błąd WebSocket: {e}")
    exit()

# --- 2. Inicjalizacja Zmiennych do Podwójnej Całki ---
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])

position = np.zeros(3) # [x, y, z] w metrach
velocity = np.zeros(3) # [x, y, z] w m/s

last_time = time.time()

# --- PARAMETRY "MAGII" (Możesz je dostrajać) ---
GRAVITY_MS2 = 9.81
ACCEL_DEADBAND = 0.25   # Odrzucamy przyspieszenia poniżej 0.25 m/s^2 (szum)
VELOCITY_DAMPING = 0.92 # Wirtualne tarcie. 1.0 = brak tarcia (dryf!), 0.0 = brak ruchu. Zmniejsz, jeśli długopis ucieka.

ACC_S = 4
ACC_SS = 0

GYRO_S = 4
GYRO_SS = 1

MAG_S = 3
MAG_SS = 0

DEVICE_ID= 0

sensors_id = {
    "ACC": (ACC_S, ACC_SS),
    "GYRO": (GYRO_S, GYRO_SS),
    "MAG": (MAG_S, MAG_SS)
}

normalize_coeff = {
    "ACC" : GRAVITY_MS2,
    "GYRO":  (np.pi / 180000.0),
    "MAG": (1.0 / 10.0)
}

def get_latest_sample(link_instance, device, sensor_str):
    s_id, ss_id = sensors_id[sensor_str]

    sensor_data = link_instance.get_sensor_data(DEVICE_ID, s_id, ss_id)

    if sensor_data is None:
        return None

    raw_bytes = sensor_data[1]

    samples = len(raw_bytes) // 6

    for i in range(samples):
        offset = i * 6
        chunk = raw_bytes[offset:offset+6]

        raw_x, raw_y, raw_z = struct.unpack('<hhh', chunk)

        status = device.sensor[s_id].sensor_status.sub_sensor_status[ss_id]
        sensitivity = status.sensitivity

        latest_acc = np.array([raw_x * sensitivity, raw_y * sensitivity, raw_z * sensitivity]) * normalize_coeff[sensor_str]
    
    return latest_acc

# --- 3. Połączenie z płytką ---
hsd_link = HSDLink()
hsd_link_instance = hsd_link.create_hsd_link()

if hsd_link_instance is None: exit()
device = hsd_link.get_device(hsd_link_instance, DEVICE_ID)
hsd_link.start_log(hsd_link_instance, DEVICE_ID)

try:
    while True:
        data_updated = False
        latest_acc = latest_gyro = latest_mag = None

        latest_acc = get_latest_sample(hsd_link_instance, device, "ACC")
        latest_gyro = get_latest_sample(hsd_link_instance, device, "GYRO")
        latest_mag = get_latest_sample(hsd_link_instance, device, "MAG")
                                
        # --- 4. OBLICZENIA I PODWÓJNA CAŁKA ---
        if (latest_acc is not None) and (latest_gyro is not None) and (latest_mag is not None):
            
            # Obliczanie precyzyjnego dt
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Zabezpieczenie przed skokami przy starcie
            if dt > 0.1: dt = 0.02 

            # KROK 1: Orientacja przestrzenna
            q = madgwick.updateMARG(q, gyr=latest_gyro, acc=latest_acc, mag=latest_mag)
            
            # KROK 2: Kompensacja grawitacji
            # Używamy kwaternionu, aby obrócić odczyt z akcelerometru do globalnego układu Ziemi
            quat_obj = Quaternion(q)
            acc_earth_g = quat_obj.rotate(latest_acc) 
            
            # Zakładamy, że w układzie Ziemi grawitacja działa w osi Z. Odcinamy 1G.
            # Zależnie od kalibracji Madgwicka, może być to +1 lub -1. Zakładamy [0, 0, 1].
            acc_linear_g = acc_earth_g - np.array([0.0, 0.0, 1.0])
            
            # Konwersja na m/s^2
            acc_linear_ms2 = acc_linear_g * GRAVITY_MS2
            
            # KROK 3: Martwa strefa (czyszczenie mikroszumów)
            # Jeśli przyspieszenie w jakiejś osi jest mniejsze niż np. 0.25 m/s^2, uznajemy że stoi w miejscu.
            acc_linear_ms2[np.abs(acc_linear_ms2) < ACCEL_DEADBAND] = 0.0

            # KROK 4: PIERWSZA CAŁKA (Prędkość)
            velocity += acc_linear_ms2 * dt
            
            # KROK 5: Tłumienie (Leaky Integrator) - to ratuje nas przed nieskończonym dryfem
            velocity *= VELOCITY_DAMPING
            
            # KROK 6: DRUGA CAŁKA (Pozycja)
            position += velocity * dt
            
            # --- Wysyłka do frontendu ---
            payload = {
                "quaternion": {"w": q[0], "x": q[1], "y": q[2], "z": q[3]},
                "position": {"x": position[0], "y": position[1], "z": position[2]}
            }
            
            try:
                ws.send(json.dumps(payload))
            except Exception:
                break
                
        time.sleep(0.01) # Szybsze próbkowanie = lepsza całka

except KeyboardInterrupt:
    hsd_link.stop_log(hsd_link_instance, DEVICE_ID)
    ws.close()
    print("\nStrumień bezpiecznie zatrzymany.")