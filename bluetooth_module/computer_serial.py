import serial
import time

s = serial.Serial("COM7", 9600)
time.sleep(2)

s.write(b'1')  # for turning on

time.sleep(5)
s.write(b'0')  # for turning off
