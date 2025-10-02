###############################################################
###### Receive basic commands from GUI using LoRa module ######
###############################################################
# Lora appears to RPi as a serial port
# Lora Specifications
## Communication Protocol: UART
## Baud Rate: 1200 - 115200 bps
###############################################################

import serial
import time

from simple_motor import SimpleMotor

# Open LoRa serial port for receiving motor commands
lora = serial.Serial(
    port="/dev/serial0",  # or "/dev/ttyS0"
    baudrate=9600,        # slow bit rate
    timeout=10            # long timeout period (seconds)
)

# replace with actual RPi GPIO pins
leftMotorPin1 = 1
leftMotorPin2 = 2
rightMotorPin1 = 3
rightMotorPin2 = 4

motors = SimpleMotor(4, [leftMotorPin1, leftMotorPin2, rightMotorPin1, rightMotorPin2])

print("Configured motor pins")
print("Listening for LoRa messages...")

try:
    while True:

        if lora.in_waiting > 0:             # if bytes available in buffer
            packets = lora.read(lora.in_waiting)      # read all available bytes
            print("Received (raw bytes):", packets)

            try:
                print("As string:", packets.decode("utf-8").strip())

                if packets[0].decode("utf-8") == "W":
                    # drive forward
                    motors.forward()

                elif packets[0].decode("utf-8") == "A":
                    # drive left
                    motors.left()

                elif packets[0].decode("utf-8") == "S":
                    # drive backwards
                    motors.backward()

                elif packets[0].decode("utf-8") == "D":
                    # drive right
                    motors.right()

                elif packets[0].decode("utf-8") == "T":
                    # stop motors
                    motors.stop()

                
            except UnicodeDecodeError:
                print("Non-text bytes received")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    lora.close()