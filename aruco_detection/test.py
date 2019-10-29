import cv2
import numpy as np
from cv2 import aruco
import matplotlib.pyplot as plt

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# img = aruco.drawMarker(aruco_dict, 1, 500)
# plt.imshow(img)
# plt.show()

#pic = cv2.imread("rule_book_aruco_1.jpg")
# plt.imshow(pic)
# plt.show()

# gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
# parameters = aruco.DetectorParameters_create()
# corners, ids, rejectedImgPoints = aruco.detectMarkers(
#     gray, aruco_dict, parameters=parameters)
# frame_markers = aruco.drawDetectedMarkers(pic.copy(), corners, ids)

# print(ids)

# plt.imshow(frame_markers)
# plt.show()

cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
    cv2.imshow("Webcam", frame_markers)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
