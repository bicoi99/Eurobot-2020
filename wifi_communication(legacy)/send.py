# Goes on the Pi with vision

import socket

Pi_IP = '192.168.137.246'
Pi_Port = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    data = bytes(input("Type the message you want to send: "), 'utf-8')
    sock.sendto(data, (Pi_IP, Pi_Port))
    print("I have sent your message!\n")
