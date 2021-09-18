import cv2
import socket
from time import sleep
import math
import numpy as np

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def stop_robot(ip, port):
    s.connect((ip, port))
    s.sendall(b'1')
    print(s.recv(1024))

# stop_robot("10.177.133.75", 1234)
# stop_robot("10.177.133.75", 1235)


def find_centre(corner):
    """Find centre coordiante of code given coordinates of corners"""
    return corner.mean(axis=0)


def find_robot(corners, ids, robot_id):
    """Find centre of Jack's code (id 1) if present"""
    if robot_id not in ids:
        return None
    return find_centre(corners[np.argwhere(ids.flatten() == robot_id)[0][0]][0])


def distance(pt1, pt2):
    """Calculate Euclidean distance between two points. 
       Do not compute sqrt to improve performance"""
    return (pt2[1] - pt1[1])**2 + (pt2[0] - pt1[0])**2


aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
thresh = 100**2

cap = cv2.VideoCapture(0)
while True:
    _, img = cap.read()
    corners, ids, _ = cv2.aruco.detectMarkers(
        img, aruco_dict, parameters=parameters)
    # Check if any code is detected
    if ids is not None:
        # Draw markers
        img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        # Check Jack
        jack = find_robot(corners, ids, 1)
        if jack is not None:
            cv2.circle(img, tuple(jack.astype(int)), 5, (0, 0, 255), -1)
        # Check Rose
        rose = find_robot(corners, ids, 2)
        if rose is not None:
            cv2.circle(img, tuple(rose.astype(int)), 5, (0, 255, 0), -1)
        # Check enemy
        enemy = find_robot(corners, ids, 0)
        if enemy is not None:
            cv2.circle(img, tuple(enemy.astype(int)), 5, (0, 255, 255), -1)

        # Line between Jack and enemy if both in frame
        if jack is not None and enemy is not None:
            cv2.line(img, tuple(jack.astype(int)),
                     tuple(enemy.astype(int)), (255, 255, 0), 3)
            cv2.putText(img, f"{distance(jack, enemy):.1f}", tuple(
                ((jack+enemy)/2).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 0), 2)
            # If distance is below threshold, send stop signal
            if distance(jack, enemy) < thresh:
                stop_robot("10.177.133.75", 1234)

        # Line between Rose and enemy if both in frame
        if rose is not None and enemy is not None:
            cv2.line(img, tuple(rose.astype(int)),
                     tuple(enemy.astype(int)), (255, 0, 0), 3)
            cv2.putText(img, f"{distance(rose, enemy):.1f}", tuple(
                ((rose+enemy)/2).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 0), 2)

        # Line between Jack and Rose if both in frame
        if jack is not None and rose is not None:
            cv2.line(img, tuple(jack.astype(int)),
                     tuple(rose.astype(int)), (255, 0, 255), 3)
            cv2.putText(img, f"{distance(jack, rose):.1f}", tuple(
                ((jack+rose)/2).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 2)

    cv2.imshow("Feed", img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
