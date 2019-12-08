from cv2 import aruco
import matplotlib.pyplot as plt

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

img = [aruco.drawMarker(aruco_dict, i, 500) for i in range(4)]

for i in range(4):
    plt.figure()
    plt.imshow(img[i], 'gray')

plt.show()
