#!/usr/bin/python

#   Calder Phillips-Grafflin

import serial
import time

class ArduinoValveDriver:

    def __init__(self, port):
        self.arduino = serial.Serial(port, 9600, 8, 'N', 1, timeout=0.1)

    def SetDutyCycle(self, duty_cycle):
        # Check the commanded value
        if (duty_cycle < 0.0):
            duty_cycle = 0.0
        elif (duty_cycle > 1.0):
            duty_cycle = 1.0
        # Make the command
        command_bytes = bytearray(3)
        command_bytes[0] = '$'
        command_bytes[2] = '\n'
        command_bytes[1] = int(duty_cycle * 100.0)
        self.arduino.write(command_bytes)

