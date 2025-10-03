###############################################################
############### Basic motor control commands ##################
###############################################################

import RPi.GPIO as GPIO

class SimpleMotor:

    # constructor, run when object is initialized
    def __init__(self, numMotorPins, motorPins):
        self.numMotorPins = numMotorPins        # should be 4, 2 for each motor (each H-bridge)
        self.motorPins = motorPins              # [leftMotorPin1, leftMotorPin2, rightMotorPin1, rightMotorPin2]

        GPIO.setmode(GPIO.BCM)
        for pin in motorPins:                   # set motor pins to be outputs
            GPIO.setup(pin, GPIO.OUT)            

    def forward(self):
        # both motors spin forwards
        GPIO.output(self.motorPins[0], GPIO.HIGH)
        GPIO.output(self.motorPins[1], GPIO.LOW)
        GPIO.output(self.motorPins[2], GPIO.HIGH)
        GPIO.output(self.motorPins[3], GPIO.LOW)

    # left and right GPIO directions may need to be reversed
    def left(self):
        # right motor forward, left motor backward
        GPIO.output(self.motorPins[0], GPIO.LOW)
        GPIO.output(self.motorPins[1], GPIO.HIGH)
        GPIO.output(self.motorPins[2], GPIO.HIGH)
        GPIO.output(self.motorPins[3], GPIO.LOW)


    def right(self):
        # left motor forward, right motor backward
        GPIO.output(self.motorPins[0], GPIO.HIGH)
        GPIO.output(self.motorPins[1], GPIO.LOW)
        GPIO.output(self.motorPins[2], GPIO.LOW)
        GPIO.output(self.motorPins[3], GPIO.HIGH)

    def backward(self):
        # both motors spin backwards
        GPIO.output(self.motorPins[0], GPIO.LOW)
        GPIO.output(self.motorPins[1], GPIO.HIGH)
        GPIO.output(self.motorPins[2], GPIO.LOW)
        GPIO.output(self.motorPins[3], GPIO.HIGH)


    def stop(self):
        # all pins off
        for pin in self.motorPins:
            GPIO.output(pin, GPIO.LOW)