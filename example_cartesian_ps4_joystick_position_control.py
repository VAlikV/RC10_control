"""Realtime SpaceMouse control using Cartesian linear jog

Uses linear.jog_once_all_axis() at 100Hz with velocity feedthrough,
the SpaceMouse's smoothed velocities are mapped directly to jog directions.
As long as we are pushing/twisting the space mouse, the robot keeps moving. No position
tracking, no start/stop jitter.
"""

import time
import numpy as np

from rc10_api.controller import TaskSpaceJogController
from rc10_api.ps4_joystick import PS4Joystick
from rc10_api.gripper import Gripper



ROBOT_IP = "10.10.10.10"

def main():
    ctrl = TaskSpaceJogController(ip=ROBOT_IP, velocity=1, acceleration=1.0)
    ctrl.start()

    joy = PS4Joystick(max_speed=0.2, x_init=0.095, y_init=0.35, z_init=0.23, roll_init=np.pi, pitch_init=0.0, yaw_init=0.0)

    gripper = Gripper()

    print("Cartesian jog + PS4")
    try:
        while True:
            # For now we are not using the x, y, z, r, p, y for position control inside the cartesian jog, 
            # we just simply take space mouse values and directly set the directions like a velocity control
            x, y, z, roll, pitch, yaw = joy.get_joystick()
            # vx, vy, vz, vyaw = joy.get_delta_velocities()
            ctrl.set_target(x, y, z, roll, pitch, yaw)
            print(f"X: {x:+.4f}m  Y: {y:+.4f}m  Z: {z:+.4f}m  "
                  f"R: {roll:+.4f}rad  P: {pitch:+.4f}rad  Y: {yaw:+.4f}rad",
                  end="\r")
            gripper.send(joy.get_gripper_state())
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        ctrl.stop()
        joy.stop()


if __name__ == "__main__":
    main()
