import socket
from API.controller import JointSpaceJogController, TaskSpaceJogController
import numpy as np
import time

robot = TaskSpaceJogController(ip="10.10.10.10",
                               rate_hz=100,
                               velocity=1,
                               acceleration=1,
                               treshold_position=0.001,
                               treshold_angel=1)

robot.start()

position = robot.get_current_tcp()

# position[0] += 0.1
# position[1] += 0.2

R = np.array([[1, 0, 0],
              [0, -1, 0],
              [0, 0, -1]], dtype=float)

# robot.set_target(position[0:3], R)

# while True:
#     time.sleep(0.01)

# for i in range(250):
robot.set_direction([0,1,1,0,0,0])
time.sleep(3)

robot.set_direction([0,0,0,0,0,0])
time.sleep(1)

robot.set_direction([1,-1,0,0,0,0])
time.sleep(3)

robot.set_direction([0,0,0,0,0,0])

time.sleep(1)

robot.set_target(position[0:3], R)

while True:
    time.sleep(0.01)

robot.stop()