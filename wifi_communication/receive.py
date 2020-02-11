# Goes on the Pi onboard

import socket

Pi_IP = '192.168.137.246'
Pi_Port = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((Pi_IP, Pi_Port))

while True:
    data, addr = sock.recvfrom(1024)
    print("Received:", data.decode('utf-8'))
