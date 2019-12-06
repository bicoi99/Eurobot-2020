import cv2
import numpy as np
from cv2 import aruco
import matplotlib.pyplot as plt
import math

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)


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


# VIDEO
cap = cv2.VideoCapture(0)  # ensure that not on video call
while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    if len(corners) >= 2:
        x_1, y_1 = corners[0][0][0][0], corners[0][0][0][1]
        x_2, y_2 = corners[1][0][0][0], corners[1][0][0][1]
        if distance(x_1, y_1, x_2, y_2) < 100:
            print('Collision!')
    frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
    cv2.imshow("Webcam", frame_markers)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
