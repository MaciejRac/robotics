import time
import json
import numpy as np
import shutil
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
ACCEL_DEADBAND = 0.05   # Odrzucamy przyspieszenia poniżej 0.25 m/s^2 (szum)
VELOCITY_DAMPING = 0.8 # Wirtualne tarcie. 1.0 = brak tarcia (dryf!), 0.0 = brak ruchu.
POSITION_RETURN_SPEED = 0.98

DEVICE_ID = 0

normalize_coeff = {
    "ACC" : 1,         # Konwersja z mg na g (1g = 1.0)
    "GYRO": (np.pi / 180000.0),     # Konwersja z mdps na rad/s
    "MAG":  (1.0 / 1000.0)          # Konwersja z mgauss na Gauss
}

# Stanowe bufory i parametry dla każdego z czujników osobno
# Oparte na strukturze Twojego urządzenia: ACC/GYRO (1000 próbek), MAG (100 próbek)
sensor_configs = {
    "ACC":  {"s_id": 4, "ss_id": 0, "spts": 1000, "buffer": bytearray(), "aligned": False, "last_val": None},
    "GYRO": {"s_id": 4, "ss_id": 1, "spts": 1000, "buffer": bytearray(), "aligned": False, "last_val": None},
    "MAG":  {"s_id": 3, "ss_id": 0, "spts": 100,  "buffer": bytearray(), "aligned": False, "last_val": None}
}

# --- Inicjalizacja parametrów z urządzenia ---
def init_sensor_params(device):
    for s_name, cfg in sensor_configs.items():
        status = device.sensor[cfg["s_id"]].sensor_status.sub_sensor_status[cfg["ss_id"]]
        
        # Read dynamically from the board
        dynamic_spts = getattr(status, 'samples_per_ts', 0)
        
        # SAFETY CHECK: If the board returns 0, keep the dictionary's default (1000 or 100)
        if dynamic_spts is not None and dynamic_spts > 0:
            cfg["spts"] = dynamic_spts
        else:
            print(f"[{s_name}] Ostrzeżenie: spts=0 z płytki. Używam domyślnego: {cfg['spts']}")
            
        cfg["sensitivity"] = status.sensitivity
        
        # Recalculate block size safely
        cfg["block_size"] = 8 + (cfg["spts"] * 3 * 2)

        cfg["dtype"] = np.dtype([('ts', '<f8'), ('samples', '<i2', (cfg["spts"], 3))])
        
        print(f"[{s_name}] Gotowy! Czułość: {cfg['sensitivity']}, BlockSize: {cfg['block_size']}")

# --- Nowa, bezpieczna funkcja pobierająca najświeższą próbkę ---
def get_latest_sample(link_instance, sensor_str):
    cfg = sensor_configs[sensor_str]
    res = link_instance.get_sensor_data(DEVICE_ID, cfg["s_id"], cfg["ss_id"])

    if res is not None:
        size, raw_bytes = res
        cfg["buffer"].extend(raw_bytes)
        bs = cfg["block_size"]

        # ETAP 1: Szukanie poprawnego początku ramki (Pomijanie uciętych TSów)
        if not cfg["aligned"]:
            if len(cfg["buffer"]) >= bs * 3:
                align_idx = -1
                for i in range(len(cfg["buffer"]) - bs * 2):
                    ts1 = np.frombuffer(bytes(cfg["buffer"][i:i+8]), dtype='<f8')[0]
                    ts2 = np.frombuffer(bytes(cfg["buffer"][i+bs:i+bs+8]), dtype='<f8')[0]
                    # Logiczne odstępy czasu między znacznikami (np. ~0.15s)
                    if 0.0 < ts1 < 1e9 and ts2 > ts1 and (ts2 - ts1) < 2.0:
                        align_idx = i
                        break
                if align_idx != -1:
                    print(f"[{sensor_str}] Zsynchronizowano strumień!")
                    del cfg["buffer"][:align_idx]
                    cfg["aligned"] = True
                else:
                    del cfg["buffer"][:bs]

        # ETAP 2: Wyciąganie najnowszej próbki ze zsynchronizowanego strumienia
        if cfg["aligned"]:
            latest_raw_xyz = None
            
            # Opróżniamy bufor blok po bloku
            while len(cfg["buffer"]) >= bs:
                block_bytes = bytes(cfg["buffer"][:bs])
                del cfg["buffer"][:bs]

                parsed = np.frombuffer(block_bytes, dtype=cfg["dtype"])
                ts = parsed['ts'][0]

                # Awaryjny reset przy zgubieniu pakietu USB
                if not (0.0 <= ts < 1e9):
                    cfg["aligned"] = False
                    cfg["buffer"].clear()
                    break

                # Bierzemy OSTATNIĄ próbkę [X,Y,Z] z bloku (indeks -1)
                latest_raw_xyz = parsed['samples'][0][-1]

            if latest_raw_xyz is not None:
                # Obliczamy jednostki fizyczne i zapamiętujemy
                val = latest_raw_xyz * cfg["sensitivity"] * normalize_coeff[sensor_str]
                cfg["last_val"] = val

    # Zwracamy najnowszą wartość z pamięci podręcznej
    return cfg["last_val"]


# --- 3. Połączenie z płytką ---
hsd_link = HSDLink()
hsd_link_instance = hsd_link.create_hsd_link(acquisition_folder="temp")

if hsd_link_instance is None: 
    print("Brak połączenia z płytką!")
    exit()

device = hsd_link.get_device(hsd_link_instance, DEVICE_ID)

hsd_link.start_log(hsd_link_instance, DEVICE_ID)
# Inicjujemy bufory, sprawdzamy czułości w locie
init_sensor_params(device)
print("\n--- Start Akwizycji i Fuzji Danych ---")

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


gyro_bias = np.zeros(3)
calib_samples = 0

while calib_samples < 1000:
    chunk = get_pending_samples(hsd_link_instance, "GYRO")
    if chunk is not None:
        for g in chunk:
            if calib_samples < 1000:
                gyro_bias += g
                calib_samples += 1
    time.sleep(0.01)

gyro_bias /= 1000.0
print(f"Kalibracja zakończona! Bias żyroskopu: {gyro_bias}")
print("\n--- Start Akwizycji i Fuzji Danych ---")

try:
    while True:
        # Odczyt pobiera dane, parsuje bloki, odrzuca timestampy i zwraca sam czysty pomiar

        latency_start = time.time()

        latest_acc = get_latest_sample(hsd_link_instance, "ACC")
        #print(latest_acc)
        latest_gyro = get_latest_sample(hsd_link_instance, "GYRO")
        print(latest_gyro)
                                
        # --- 4. OBLICZENIA I PODWÓJNA CAŁKA ---
        if (latest_acc is not None) and (latest_gyro is not None):
            # Obliczanie precyzyjnego dt
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            latest_gyro -= gyro_bias
            
            # Zabezpieczenie przed skokami przy starcie
            if dt > 0.1: dt = 0.02 

            # KROK 1: Orientacja przestrzenna
            madgwick.Dt = dt 
            q = madgwick.updateIMU(q, gyr=latest_gyro, acc=latest_acc)
            
            # KROK 2: Kompensacja grawitacji
            quat_obj = Quaternion(q)
            acc_earth_g = quat_obj.rotate(latest_acc) 
            
            acc_linear_g = acc_earth_g - np.array([0.0, 0.0, 1.0])
            acc_linear_ms2 = acc_linear_g * GRAVITY_MS2
            
            # KROK 3: Martwa strefa (czyszczenie mikroszumów)
            acc_linear_ms2[np.abs(acc_linear_ms2) < ACCEL_DEADBAND] = 0.0
            
            #alternatywne (chyba lepsze rozwiązanie)
            # if np.linalg.norm(acc_linear_ms2) < ACCEL_DEADBAND:
            #     acc_linear_ms2 = np.zeros(3)
            #     velocity *= 0.5 # Szybkie wygaszanie prędkości gdy czujnik się zatrzymuje
            # else:
            #     # KROK 4: PIERWSZA CAŁKA (Prędkość)
            #     velocity += acc_linear_ms2 * dt

            # KROK 4: PIERWSZA CAŁKA (Prędkość)
            velocity += acc_linear_ms2 * dt
            
            # KROK 5: Tłumienie (Leaky Integrator)
            velocity *= VELOCITY_DAMPING
            
            # KROK 6: DRUGA CAŁKA (Pozycja)
            position += velocity * dt
            position *= POSITION_RETURN_SPEED
            
            # --- Wysyłka do frontendu ---
            payload = {
                "quaternion": {"w": q[0], "x": q[1], "y": q[2], "z": q[3]},
                "position": {"x": position[0], "y": position[1], "z": position[2]},
                "timestamp": latency_start
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
    ws.close()
    shutil.rmtree("temp")
    print("Strumień bezpiecznie zatrzymany.")