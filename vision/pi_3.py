import cv2
import math
from time import sleep
import socket

threshold = 300**2

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

jack_ip = '192.168.137.206'
jack_port = 1234

rose_ip = '192.168.137.148'
rose_port = 1235

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def logic(x1, y1, x2, y2):
    return (x2 - x1)**2 + (y2-y1)**2


class Robot():
    x, y = None, None

    def __init__(self, id):
        self.id = id


cap = cv2.VideoCapture(0)
print('Start')
collision_flag = False
while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    # sleep(0.2)
    # frame_marker = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)

    jack = Robot(1)
    rose = Robot(2)
    enemy = Robot(0)

    if len(corners) >= 2:
        for i in range(len(ids)):
            if ids[i] == jack.id:
                jack.x, jack.y = corners[i][0][:,
                                               0].mean(), corners[i][0][:, 1].mean()
            elif ids[i] == enemy.id:
                enemy.x, enemy.y = corners[i][0][:,
                                                 0].mean(), corners[i][0][:, 1].mean()

        if (jack.x or rose.x) and enemy.x:
            if logic(jack.x, jack.y, enemy.x, enemy.y) < threshold:
                print('Collision!')
                if not collision_flag:
                    data = b'1'
                    sock.sendto(data, (jack_ip, jack_port))
                collision_flag = True
            else:
                print(f"Found {len(corners)} codes")
                if collision_flag:
                    data = b'0'
                    sock.sendto(data, (jack_ip, jack_port))
                collision_flag = False
    else:
        # print('Less than 2 codes')
        # s.write(b'0')
        if collision_flag:
            data = b'0'
            sock.sendto(data, (jack_ip, jack_port))
        collision_flag = False
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
