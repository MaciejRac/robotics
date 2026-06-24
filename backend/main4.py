import time
import json
import numpy as np
import threading
import queue
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

#ws_queue = queue.Queue()

class EKFPositionTracker:
    def __init__(self, dt):
        self.dt = dt
        self.x = np.zeros(6) # [px, py, pz, vx, vy, vz]
        self.F = np.eye(6)
        self.F[0:3, 3:6] = np.eye(3) * dt
        self.P = np.eye(6) * 1.0
        self.Q = np.eye(6) * 0.01
        self.H = np.zeros((3, 6))
        self.H[:, 3:6] = np.eye(3)
        self.R = np.eye(3) * 0.1

    def predict(self, acc_linear):
        B = np.zeros((6, 3))
        B[0:3, :] = np.eye(3) * (0.5 * self.dt**2)
        B[3:6, :] = np.eye(3) * self.dt
        self.x = self.F.dot(self.x) + B.dot(acc_linear)
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q

    def update_zupt(self): # Zerowanie prędkości (Zero Velocity Update)
        y = np.zeros(3) - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + (self.R * 0.1)
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        self.P = (np.eye(6) - K.dot(self.H)).dot(self.P)

# --- 2. Inicjalizacja Zmiennych do Podwójnej Całki ---
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])
ekf = EKFPositionTracker(0.01)

position = np.zeros(3) # [x, y, z] w metrach
velocity = np.zeros(3) # [x, y, z] w m/s

# --- PARAMETRY "MAGII" (Możesz je dostrajać) ---
GRAVITY_MS2 = 9.81
ACCEL_DEADBAND = 0.1   # Odrzucamy przyspieszenia poniżej 0.35 m/s^2 (szum)
VELOCITY_DAMPING = 0.999 

# Bardzo delikatna wirtualna sprężyna.
# 0.9995^1000 = ~0.60 (Pozycja wraca o 40% w stronę zera co sekundę)
POSITION_RETURN_SPEED = 0.9995

# CZĘSTOTLIWOŚĆ SPRZĘTOWA (Zmień jeśli twój czujnik ma inną częstotliwość)
# Jeśli spts=1000 co sekundę, to próbkuje z częstotliwością 1000 Hz.
HARDWARE_DT = 0.001     

DEVICE_ID = 0

normalize_coeff = {
    "ACC" : 1.0,            # Konwersja z mg na g (1g = 1.0)
    "GYRO": (np.pi / 180000.0),     # Konwersja z mdps na rad/s
    "MAG":  (1.0 / 1000.0)          # Konwersja z mgauss na Gauss
}

sensor_configs = {
    "ACC":  {"s_id": 4, "ss_id": 0, "spts": 1000, "buffer": bytearray(), "aligned": False},
    "GYRO": {"s_id": 4, "ss_id": 1, "spts": 1000, "buffer": bytearray(), "aligned": False},
    "MAG":  {"s_id": 3, "ss_id": 0, "spts": 100,  "buffer": bytearray(), "aligned": False}
}

# --- Inicjalizacja parametrów z urządzenia ---
def init_sensor_params(device):
    for s_name, cfg in sensor_configs.items():
        status = device.sensor[cfg["s_id"]].sensor_status.sub_sensor_status[cfg["ss_id"]]
        
        dynamic_spts = getattr(status, 'samples_per_ts', 0)
        
        if dynamic_spts is not None and dynamic_spts > 0:
            cfg["spts"] = dynamic_spts
        else:
            print(f"[{s_name}] Ostrzeżenie: spts=0 z płytki. Używam domyślnego: {cfg['spts']}")

        print(f"{s_name}: {dynamic_spts}")
            
        cfg["sensitivity"] = status.sensitivity
        
        cfg["block_size"] = 8 + (cfg["spts"] * 3 * 2) 

        cfg["dtype"] = np.dtype([('ts', '<f8'), ('samples', '<i2', (cfg["spts"], 3))])
        
        print(f"[{s_name}] Gotowy! Czułość: {cfg['sensitivity']}, BlockSize: {cfg['block_size']}")

# --- ZMIANA: Zwracamy wszystkie przetworzone próbki z bufora ---
def get_pending_samples(link_instance, sensor_str):
    cfg = sensor_configs[sensor_str]
    res = link_instance.get_sensor_data(DEVICE_ID, cfg["s_id"], cfg["ss_id"])
    if res is None: return None

    if len(res[1]) == 0:
        return None
    
    cfg["buffer"].extend(res[1])
    n_samples = len(cfg["buffer"]) // 6
    
    if n_samples > 0:
        total_bytes = n_samples * 6
        data = np.frombuffer(bytes(cfg["buffer"][:total_bytes]), dtype='<i2').reshape(-1, 3)
        del cfg["buffer"][:total_bytes]
        
        samples = data * cfg["sensitivity"] * normalize_coeff[sensor_str]
        
        # --- RÓWNANIE OSI W LOCIE ---
        # Sprawdzamy, która oś ma w danej paczce największą wartość (grawitację ~1.0)
        # i "przesuwamy" ją tak, aby zawsze była na Z (indeks 2)
        aligned_samples = np.zeros_like(samples)
        for i in range(len(samples)):
            row = samples[i]
            # Znajdź indeks osi z grawitacją (wartość bliska 1.0)
            gravity_axis = np.argmax(np.abs(row)) 
            
            if gravity_axis == 0: # X ma grawitację
                aligned_samples[i] = [row[1], row[2], row[0]]
            elif gravity_axis == 1: # Y ma grawitację
                aligned_samples[i] = [row[0], row[2], row[1]]
            else: # Z ma grawitację (poprawnie)
                aligned_samples[i] = row
                
        return aligned_samples
    return None

def flush_usb_buffers(link_instance):
    """Ściąga z płytki zalegające pakiety, żeby zacząć obliczenia na czysto."""
    print("Oczyszczanie buforów USB z zaległych paczek...")
    for _ in range(20):
        hsd_link_instance.get_sensor_data(DEVICE_ID, 4, 0) # ACC
        hsd_link_instance.get_sensor_data(DEVICE_ID, 4, 1) # GYRO
        time.sleep(0.1)

# --- 3. Połączenie z płytką ---
hsd_link = HSDLink()
hsd_link_instance = hsd_link.create_hsd_link()

if hsd_link_instance is None: 
    print("Brak połączenia z płytką!")
    exit()

device = hsd_link.get_device(hsd_link_instance, DEVICE_ID)

hsd_link.start_log(hsd_link_instance, DEVICE_ID)
init_sensor_params(device)
print("\n--- Start Akwizycji i Fuzji Danych ---")

madgwick.Dt = HARDWARE_DT

time.sleep(1.0) # Zostawiamy czujnik w spokoju
flush_usb_buffers(hsd_link_instance)

# print("\nTrwa kalibracja żyroskopu (1 sekunda). NIE RUSZAJ CZUJNIKIEM...")
# gyro_bias = np.zeros(3)
# calib_samples = 0

# while calib_samples < 1000:
#     chunk = get_pending_samples(hsd_link_instance, "GYRO")
#     if chunk is not None:
#         for g in chunk:
#             if calib_samples < 1000:
#                 gyro_bias += g
#                 calib_samples += 1
#     time.sleep(0.01)

# gyro_bias /= 1000.0
# print(f"Kalibracja zakończona! Bias żyroskopu: {gyro_bias}")
# print("\n--- Start Akwizycji i Fuzji Danych ---")

last_time = time.time()

try:
    while True:
        # Pobieramy pełne pakiety próbek (np. macierze 1000x3)
        latest_acc = get_pending_samples(hsd_link_instance, "ACC")
        print(latest_acc)
        latest_gyro = get_pending_samples(hsd_link_instance, "GYRO")
        # mag_chunk = get_pending_samples(hsd_link_instance, "MAG") 
                                
        # --- 4. OBLICZENIA I PODWÓJNA CAŁKA ---
        if latest_acc is not None and latest_gyro is not None:
            #print("yalla")

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # [KROK 1 do 5 pozostają bez zmian...]
            madgwick.Dt = dt 
            q = madgwick.updateIMU(q, gyr=latest_gyro, acc=latest_acc)

            quat_obj = Quaternion(q)

            acc_earth = quat_obj.rotate(latest_acc)
            acc_linear = (acc_earth - np.array([0.0, 0.0, 1.0])) * GRAVITY_MS2
            #print(np.linalg.norm(acc_linear))
            
            # Filtracja szumu
            if np.linalg.norm(acc_linear) < ACCEL_DEADBAND:
                ekf.update_zupt() # Jeśli nie ma ruchu, wymuś zero
            else:
                ekf.predict(acc_linear)
            
            # Wysyłka najświeższych danych
            pos = ekf.x[0:3]

            payload = {
                "quaternion": {"w": q[0], "x": q[1], "y": q[2], "z": q[3]},
                "position": {"x": pos[0], "y": pos[1], "z": pos[2]}
            }
            
            try:
                ws.send(json.dumps(payload))
            except Exception:
                print("Przerwano połączenie WebSocket.")
                break

        time.sleep(0.01)


except KeyboardInterrupt:
    print("\nZatrzymywanie...")
finally:
    hsd_link.stop_log(hsd_link_instance, DEVICE_ID)
    #ws_queue.put(None) # Zabija wątek WebSocket
    ws.close()
    print("Strumień bezpiecznie zatrzymany.")