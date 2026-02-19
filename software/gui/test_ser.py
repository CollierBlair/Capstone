import time
import threading
import serial
from pynput import keyboard

# ---------- SERIAL CONFIG ----------
SERIAL_PORT = "COM3"      # Change this to your port
BAUDRATE = 115200
# -----------------------------------

key_state = {'w': False, 'a': False, 's': False, 'd': False}
lock = threading.Lock()

try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    ser.flush()
    print(f"Opened serial port {SERIAL_PORT} at {BAUDRATE} baud")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# ---------- KEY HANDLING ----------
def on_press(key):
    try:
        k = key.char.lower()
        if k in key_state:
            with lock:
                key_state[k] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char.lower()
        if k in key_state:
            with lock:
                key_state[k] = False
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# ---------- MAIN LOOP ----------
try:
    while True:
        packet = []
        with lock:
            for k in ['w', 'a', 's', 'd']:
                packet.append(0xA0 if key_state[k] else 0x05)

        # Debug print
        print("Sending packet:", [hex(b) for b in packet])

        # Send over serial
        ser.write(bytes(packet))
        ser.flush()  # Make sure it's sent immediately
        time.sleep(0.05)  # 50ms

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
    listener.stop()