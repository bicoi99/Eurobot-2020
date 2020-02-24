# Goes on the Pi onboard

import socket
import serial

Pi_IP = '192.168.137.246'
Pi_Port = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((Pi_IP, Pi_Port))

s = serial.Serial('/dev/ttyACM0', 9600)

while True:
    data, addr = sock.recvfrom(1024)
    print("Received:", data.decode('utf-8'))
    s.write(data)
