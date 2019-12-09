import cv2
import numpy as np
from cv2 import aruco
import matplotlib.pyplot as plt
import math
from time import sleep
import serial

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

s = serial.Serial('com3', 9600)
sleep(2)


def distance(x_1, y_1, x_2, y_2):
    return math.sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2)

# img = aruco.drawMarker(aruco_dict, 1, 500)
# plt.imshow(img)
# plt.show()


# PICTURE TEST
# pic = cv2.imread("aruco_detection/test_2.jpg")

# gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
# parameters = aruco.DetectorParameters_create()
# corners, ids, rejectedImgPoints = aruco.detectMarkers(
#     gray, aruco_dict, parameters=parameters)
# frame_markers = aruco.drawDetectedMarkers(pic.copy(), corners, ids)

# x_1, y_1 = corners[0][0][0][0], corners[0][0][0][1]
# x_2, y_2 = corners[1][0][0][0], corners[1][0][0][1]

# print(distance(x_1, y_1, x_2, y_2))

# plt.imshow(frame_markers)
# plt.show()


class Robot():
    x, y = None, None

    def __init__(self, id):
        self.id = id


# VIDEO
cap = cv2.VideoCapture(0)  # ensure that not on video call
while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    # sleep(0.1)
    frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
    print('Running')

    jack = Robot(1)
    enemy = Robot(0)

    if len(corners) >= 2:
        for i in range(len(ids)):
            if ids[i] == jack.id:
                jack.x, jack.y = corners[i][0][:,
                                               0].mean(), corners[i][0][:, 1].mean()
            elif ids[i] == enemy.id:
                enemy.x, enemy.y = corners[i][0][:,
                                                 0].mean(), corners[i][0][:, 1].mean()
        if jack.x and enemy.x:
            cv2.line(frame_markers, (jack.x, jack.y),
                     (enemy.x, enemy.y), (0, 0, 255), 3)
            if distance(jack.x, jack.y, enemy.x, enemy.y) < 100:
                print('Collision!')
                s.write(b'1')
            else:
                print('Detect 2 or more')
                s.write(b'0')
    else:
        # print('Less than 2 codes')
        s.write(b'0')
    cv2.imshow("Webcam", frame_markers)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
s.close()
