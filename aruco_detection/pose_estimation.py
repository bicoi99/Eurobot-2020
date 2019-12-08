import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import numpy as np

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
pic = cv2.imread("aruco_detection/test_2.jpg")

gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
parameters = aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(
    gray, aruco_dict, parameters=parameters)
frame_markers = aruco.drawDetectedMarkers(pic.copy(), corners, ids)

print(corners)
print(ids)

plt.figure()
plt.imshow(frame_markers)
for i in range(len(ids)):
    c = corners[i][0]
    plt.plot([c[:, 0].mean()], [c[:, 1].mean()],
             "o", label="id={0}".format(ids[i]))
plt.plot([corners[0][0][:, 0].mean(), corners[1][0][:, 0].mean()],
         [corners[0][0][:, 1].mean(), corners[1][0][:, 1].mean()], 'r-')
plt.legend()
# plt.show()

# rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05)
# aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

# plt.imshow(frame_markers)
# plt.show()
