import socket
from API.controller import JointSpaceJogController, TaskSpaceJogController
import numpy as np
import time

haptic_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
haptic_sock.bind(("127.0.0.1", 8081))
haptic_sock.settimeout(0.001)

robot = TaskSpaceJogController(ip="10.10.10.10",
                               rate_hz=100,
                               velocity=1,
                               acceleration=1,
                               treshold_position=0.001,
                               treshold_angel=1)
message = []

robot.start()

R = np.array([[1, 0, 0],
              [0, -1, 0],
              [0, 0, -1]], dtype=float)

while True:
    try:
        data, addr = haptic_sock.recvfrom(1024)
        message = np.array(list(map(float, data.decode()[1:-1].split(","))))

    except socket.timeout:
        data, addr = None, None  # или просто continue

    if len(message):
        # print(message)
        # R = np.array([message[3:6], message[6:9], message[9:12]])
        robot.set_target(message[0:3], R)

robot.stop()