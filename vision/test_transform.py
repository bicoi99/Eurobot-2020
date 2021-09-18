from pickle import FALSE
import cv2
import numpy as np
import math
import socket

try:
    jack_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    jack_sock.connect(("10.177.133.75", 1234))
except:
    print("Please connect to Jack")

try:
    rose_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rose_sock.connect(("10.177.133.75", 1235))
except:
    print("Please connect to Rose")

jack_id = 0
rose_id = 1
enemy1_id = 2
enemy2_id = 3

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters_create()
res_size = (500, 700)
conv_fact = 20 / 500  # Convert between px to cm

is_jack_collision = False
is_rose_collision = False

cap = cv2.VideoCapture(0)
res = np.zeros(res_size)
with open("vision/transform.npy", "rb") as f:
    matrix = np.load(f)


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
    return math.sqrt((pt2[1] - pt1[1])**2 + (pt2[0] - pt1[0])**2)


# def check_collision(robot1, robot2, is_jack):
#     """Check distance between robot1 and robot2"""
#     global jack_collision_flag, rose_collision_flag, checking
#     cv2.line(res, tuple(robot1.astype(int)), tuple(
#         robot2.astype(int)), (255, 255, 0), 3)
#     dist = distance(robot1, robot2)*conv_fact
#     cv2.putText(res, f"{dist:.1f}", tuple(
#         ((robot1+robot2)/2).astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
#         .5, (255, 255, 0), 2)
#     if is_jack:
#         if dist < 6:
#             if not jack_collision_flag:
#                 print("Jack collision")
#                 jack_sock.send(b'1')
#                 jack_collision_flag = True
#         else:
#             if jack_collision_flag and not checking:
#                 print("Jack's clear")
#                 jack_sock.send(b'0')
#                 jack_collision_flag = False
#     else:
#         if dist < 6:
#             if not rose_collision_flag:
#                 print("Rose collision")
#                 rose_sock.send(b'1')
#                 rose_collision_flag = True
#         else:
#             if rose_collision_flag and not checking:
#                 print("Rose's clear")
#                 rose_sock.send(b'0')
#                 rose_collision_flag = False

def check_collision(robot1, robot2):
    """Check distance between robot1 and robot2"""
    cv2.line(res, tuple(robot1.astype(int)), tuple(
        robot2.astype(int)), (255, 255, 0), 3)
    dist = distance(robot1, robot2)*conv_fact
    cv2.putText(res, f"{dist:.1f}", tuple(
        ((robot1+robot2)/2).astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
        .5, (255, 255, 0), 2)
    return True if dist < 6 else False


while True:
    _, img = cap.read()
    # corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
    #     img, aruco_dict, parameters=params)
    # img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
    # print(ids)

    # Transform image
    res = cv2.warpPerspective(img, matrix, res_size)

    # Detect codes from transformed image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        res, aruco_dict, parameters=params)
    # res = cv2.aruco.drawDetectedMarkers(res, corners, ids)
    # ids = ids.flatten()
    # print(ids)
    # print(np.where(ids == 1))
    # corner = corners[np.where(ids == 1)]
    # res = cv2.aruco.drawDetectedMarkers(res, [corner], [[1]])
    if ids is not None:
        # Draw markers
        res = cv2.aruco.drawDetectedMarkers(res, corners, ids)

        # Find robots (jack, rose, enemy1, enemy2)
        robots = {
            "jack": find_robot(corners, ids, jack_id),
            "rose": find_robot(corners, ids, rose_id),
            "enemy1": find_robot(corners, ids, enemy1_id),
            "enemy2": find_robot(corners, ids, enemy2_id)
        }

        # Check Jack collisions
        if robots["jack"] is not None:
            collisions = []
            for robot in [robots["rose"], robots["enemy1"], robots["enemy2"]]:
                if robot is None:
                    continue
                collisions.append(check_collision(robots["jack"], robot))
            if any(collisions):
                if not is_jack_collision:
                    print("Jack collision")
                    jack_sock.send(b'1')
                    is_jack_collision = True
            else:
                if is_jack_collision:
                    print("Jack's fine")
                    jack_sock.send(b'0')
                    is_jack_collision = False

        # Check Rose collisions
        if robots["rose"] is not None:
            collisions = []
            for robot in [robots["jack"], robots["enemy1"], robots["enemy2"]]:
                if robot is None:
                    continue
                collisions.append(check_collision(robots["rose"], robot))
            if any(collisions):
                if not is_rose_collision:
                    print("Rose collision")
                    rose_sock.send(b'1')
                    is_rose_collision = True
            else:
                if is_rose_collision:
                    print("Rose's fine")
                    rose_sock.send(b'0')
                    is_rose_collision = False

        # # Check Jack
        # jack = find_robot(corners, ids, jack_id)
        # # if jack is not None:
        # #     cv2.circle(res, tuple(jack.astype(int)), 5, (0, 0, 255), -1)

        # # Check Rose
        # rose = find_robot(corners, ids, rose_id)
        # # if rose is not None:
        # #     cv2.circle(res, tuple(rose.astype(int)), 5, (0, 0, 255), -1)

        # # Check Enemy1
        # enemy1 = find_robot(corners, ids, enemy1_id)
        # # if enemy1 is not None:
        # #     cv2.circle(res, tuple(enemy1.astype(int)), 5, (0, 0, 255), -1)

        # # Check Enemy2
        # enemy2 = find_robot(corners, ids, enemy2_id)
        # # if enemy2 is not None:
        # #     cv2.circle(res, tuple(enemy2.astype(int)), 5, (0, 255, 0), -1)

        # # Check Jack collisions
        # if jack is not None:
        #     checking = True
        #     # Check Rose
        #     if rose is not None:
        #         check_collision(jack, rose, True)
        #     # Check Enemy1
        #     if enemy1 is not None:
        #         check_collision(jack, enemy1, True)
        #     # Check Enemy2
        #     if enemy2 is not None:
        #         check_collision(jack, enemy2, True)
        #     checking = False

        # # Check Rose collisions
        # if rose is not None:
        #     checking = True
        #     # Check Jack
        #     if jack is not None:
        #         check_collision(rose, jack, False)
        #     # Check Enemy1
        #     if enemy1 is not None:
        #         check_collision(rose, enemy1, False)
        #     # Check Enemy2
        #     if enemy2 is not None:
        #         check_collision(rose, enemy2, False)
        #     checking = False

        # if jack is not None and enemy is not None:
        #     cv2.line(res, tuple(jack.astype(int)), tuple(
        #         enemy.astype(int)), (255, 255, 0), 3)
        #     dist = distance(jack, enemy)*conv_fact
        #     cv2.putText(res, f"{dist:.1f}", tuple(
        #         ((jack+enemy)/2).astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
        #         .5, (255, 255, 0), 2)
        #     if dist < 6:
        #         if not collision_flag:
        #             print("Collision")
        #             jack_sock.send(b'1')
        #             collision_flag = True
        #     else:
        #         if collision_flag:
        #             print("Fine")
        #             jack_sock.send(b'0')
        #             collision_flag = False

    cv2.imshow("Image", img)
    cv2.imshow("Result", res)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
jack_sock.close()
