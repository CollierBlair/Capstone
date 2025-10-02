###############################################################
###### Receive basic commands from GUI using LoRa module ######
###############################################################
# Lora appears to RPi as a serial port
# Lora Specifications
# Communication Protocol: UART
# Baud Rate: 1200 - 115200 bps
# 
###############################################################

import serial
import time

# Open LoRa serial port for receiving motor commands
lora = serial.Serial(
    port="/dev/serial0",  # or "/dev/ttyS0"
    baudrate=9600,        # slow bit rate
    timeout=10            # long timeout period (seconds)
)

print("Listening for LoRa messages...")

try:
    while True:

        if lora.in_waiting > 0:             # if bytes available in buffer
            motor_dir = lora.read(lora.in_waiting)      # read all available bytes
            print("Received (raw bytes):", data)

            try:
                print("As string:", motor_dir.decode("utf-8").strip())

                if motor_dir[0].decode("utf-8") == "W"

                    
                else if motor_dir[0].decode("utf-8") == "A"
                else if motor_dir[0].decode("utf-8") == "S"
                else if motor_dir[0].deocde("utf-8") == "D"

                
            except UnicodeDecodeError:
                print("Non-text bytes received")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    lora.close()