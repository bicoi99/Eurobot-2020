import socket
import serial

rose_ip = '192.168.137.148'
rose_port = 1235

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((jack_ip, jack_port))

s = serial.Serial('/dev/ttyS0', 9600)

while True:
    data, addr = sock.recvfrom(1024)
    print('Received: ', data.decode('utf-8'))
    s.write(data)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
