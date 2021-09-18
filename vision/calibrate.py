import cv2
import numpy as np

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters_create()

res_size = (500, 700)  # width x height

cap = cv2.VideoCapture(0)
res = np.zeros(res_size)

while True:
    _, img = cap.read()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        img, aruco_dict, parameters=params)
    img = cv2.aruco.drawDetectedMarkers(img, corners, ids)

    if len(corners) == 4:
        centres = np.array([corner[0].mean(axis=0) for corner in corners])[
            np.argsort(ids, axis=0).flatten()]
        matrix = cv2.getPerspectiveTransform(
            centres, np.float32([[0, 0], [res_size[0], 0], [0, res_size[1]], [res_size[0], res_size[1]]]))
        res = cv2.warpPerspective(img, matrix, res_size)

    cv2.imshow("Image", img)
    cv2.imshow("Result", res)

    if cv2.waitKey(1) == ord('q'):
        # Save the transform matrix
        with open("vision/transform.npy", "wb") as f:
            np.save(f, matrix)
        break

cap.release()
cv2.destroyAllWindows()
