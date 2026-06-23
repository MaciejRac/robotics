
import time
import numpy as np
from st_hsdatalog.HSD_link.HSDLink import HSDLink

def main():
    print("Nawiązywanie połączenia...")
    hsd_factory = HSDLink()
    hsd_link = hsd_factory.create_hsd_link(acquisition_folder=None)

    device_list = HSDLink.get_devices(hsd_link)
    if not device_list:
        print("Nie znaleziono podłączonej płytki STBOX_001!")
        return

    d_id = 0
    s_id = 4     # Główne ID dla LSM6DSOX
    ss_id = 0    # SubID dla akcelerometru
    
    # ---------------------------------------------------------
    # KONFIGURACJA WYŚWIETLANIA:
    # 10 = ~660 printów/sekundę (bardzo gęsto)
    # 20 = ~330 printów/sekundę (optymalnie)
    # 100 = ~66 printów/sekundę
    print_step = 10 
    # ---------------------------------------------------------

    device = hsd_link.get_device(0)
    status = device.sensor[s_id].sensor_status.sub_sensor_status[ss_id]
    sensitivity = status.sensitivity
    
    samples_per_ts = HSDLink.get_sensor_spts(hsd_link, d_id, s_id=s_id, ss_id=ss_id)
    if samples_per_ts is None:
        samples_per_ts = 1000 
        
    odr = HSDLink.get_sensor_odr(hsd_link, d_id, s_id=s_id, ss_id=ss_id)
    if odr is None or odr == 0:
        odr = 6667.0

    # Przewidywany czas między blokami i rozmiar bloku (8 bajtów TS + próbki int16)
    expected_ts_diff = samples_per_ts / odr
    block_size_bytes = 8 + (samples_per_ts * 3 * 2)
    
    block_dtype = np.dtype([
        ('timestamp', '<f8'),
        ('samples', '<i2', (samples_per_ts, 3))
    ])

    print(f"Konfiguracja: Czułość={sensitivity} g/LSB | Rozmiar bloku={block_size_bytes} bajtów")
    
    # Zaczynamy strumieniowanie z USB
    HSDLink.start_log(hsd_link, d_id)
    print("\n--- Szukam synchronizacji strumienia... ---")

    byte_buffer = bytearray()
    is_aligned = False

    try:
        while True:
            read_data = hsd_link.get_sensor_data(d_id, s_id, ss_id)
            



    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika.")
    finally:
        HSDLink.stop_log(hsd_link, d_id)
        print("Zakończono komunikację i zamknięto port.")


    try:
        while True:
            # Szybki odczyt z kabla
            res = hsd_link.get_sensor_data(d_id, s_id, ss_id)

            if res is not None:
                size, raw_bytes = res

                byte_buffer.extend(raw_bytes)
                
                # --- ETAP 1: Szukanie poprawnego początku ramki ---
                if not is_aligned:
                    if len(byte_buffer) >= block_size_bytes * 3:
                        align_idx = -1
                        
                        for i in range(len(byte_buffer) - block_size_bytes * 2):
                            ts1 = np.frombuffer(bytes(byte_buffer[i:i+8]), dtype='<f8')[0]
                            ts2 = np.frombuffer(bytes(byte_buffer[i+block_size_bytes:i+block_size_bytes+8]), dtype='<f8')[0]
                            
                            if 0.0 < ts1 < 1e9 and ts2 > ts1:
                                diff = ts2 - ts1
                                if abs(diff - expected_ts_diff) < (expected_ts_diff * 0.1):
                                    align_idx = i
                                    break
                                    
                        if align_idx != -1:
                            print(f"SUKCES: Zsynchronizowano strumień! (Odrzucono {align_idx} śmieciowych bajtów)")
                            print(f"--- Zbieram dane (Wyświetlam co {print_step} próbkę) ---")
                            del byte_buffer[:align_idx]
                            is_aligned = True
                        else:
                            del byte_buffer[:block_size_bytes]
                    continue

                # --- ETAP 2: Szybki i prawidłowy odczyt ---
                while len(byte_buffer) >= block_size_bytes:
                    
                    block_bytes = bytes(byte_buffer[:block_size_bytes])
                    del byte_buffer[:block_size_bytes] 
                    
                    parsed_block = np.frombuffer(block_bytes, dtype=block_dtype)
                    ts = parsed_block['timestamp'][0]
                    
                    if not (0.0 <= ts < 1e9):
                        print("\nUWAGA: USB zgubiło pakiet. Wznawiam szukanie synchronizacji...")
                        is_aligned = False
                        byte_buffer.clear()
                        break
                    
                    # Dekodujemy wszystkie próbki z bloku (domyślnie 1000 x 3 osie)
                    xyz_data = parsed_block['samples'][0]
                    physical_data = xyz_data * sensitivity
                    
                    # WYŚWIETLANIE: Przechodzimy po bloku i wypisujemy próbki z zadanym krokiem
                    for i in range(0, samples_per_ts, print_step):
                        x, y, z = physical_data[i]
                        
                        # Dokładny czas w sekundach dla tej konkretnej mikrosekundy
                        exact_time = ts + (i / odr)
                        
                        print(f"TS: {exact_time:.4f}s | ACC [g]: X={x:+.3f} | Y={y:+.3f} | Z={z:+.3f}")

    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika.")
    finally:
        HSDLink.stop_log(hsd_link, d_id)
        print("Zakończono komunikację i zamknięto port.")

if __name__ == "__main__":
    main()