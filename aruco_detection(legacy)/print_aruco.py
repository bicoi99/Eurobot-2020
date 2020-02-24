from cv2 import aruco
import matplotlib.pyplot as plt
import os

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

img = [aruco.drawMarker(aruco_dict, i, 500) for i in range(4)]

# for i in range(4):
#     plt.figure()
#     plt.imshow(img[i], 'gray')

plt.figure(figsize=(11.69, 8.27))
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.imshow(img[i], 'gray')
    plt.xticks([]), plt.yticks([])
plt.subplots_adjust(left=0.01, right=1, top=0.97, bottom=0.03)

f = plt.gcf()
f.set_size_inches(11.69, 8.27)
f.savefig('aruco_detection/aruco_codes_images/test.png')

plt.show()
