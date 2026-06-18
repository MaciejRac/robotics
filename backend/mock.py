import time
import json
import math
from websocket import create_connection

WS_URL = "ws://localhost:8080"

print("Próbuję połączyć się z serwerem WebSocket na porcie 8080...")
try:
    ws = create_connection(WS_URL)
    print("Połączono pomyślnie! Rozpoczynam wysyłanie mockowanych danych.")
except Exception as e:
    print(f"Błąd połączenia: {e}")
    print("Upewnij się, że aplikacja Electron z serwerem WS działa.")
    exit()

# Zmienna czasu do napędzania równań parametrycznych
t = 0.0
dt = 0.03 # Prędkość animacji

try:
    while True:
        # --- 1. Generowanie sztucznej pozycji ---
        # Rysujemy okrąg w płaszczyźnie X/Y (promień 0.3 metra)
        # i dodajemy delikatne falowanie w górę i w dół na osi Z
        x = 0.3 * math.cos(t)
        y = 0.3 * math.sin(t)
        z = 0.1 * math.sin(t * 2.0) 

        # --- 2. Generowanie sztucznej orientacji (Kwaternion) ---
        # Obracamy nasz "długopis" wokół osi Z, żeby podążał za kierunkiem ruchu.
        # Wzór na kwaternion dla obrotu o kąt theta wokół osi Z:
        # w = cos(theta/2), x = 0, y = 0, z = sin(theta/2)
        theta = t 
        qw = math.cos(theta / 2.0)
        qx = 0.0
        qy = 0.0
        qz = math.sin(theta / 2.0)

        # --- 3. Pakowanie i wysyłka ---
        payload = {
            "quaternion": {"w": qw, "x": qx, "y": qy, "z": qz},
            "position": {"x": x, "y": y, "z": z}
        }

        try:
            ws.send(json.dumps(payload))
        except Exception as e:
            print(f"Utracono połączenie: {e}")
            break
        
        t += dt
        time.sleep(0.016) # ~60 FPS (1/60 sekundy)

except KeyboardInterrupt:
    print("\nSymulacja zatrzymana.")
    ws.close()