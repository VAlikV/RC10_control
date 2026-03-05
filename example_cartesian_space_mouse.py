"""Realtime SpaceMouse control using Cartesian linear jog

Uses linear.jog_once_all_axis() at 100Hz with velocity feedthrough,
the SpaceMouse's smoothed velocities are mapped directly to jog directions.
As long as we are pushing/twisting the space mouse, the robot keeps moving. No position
tracking, no start/stop jitter.
"""

import time
import numpy as np

from rc10_api.controller import CartesianJogController
from rc10_api.space_mouse import SpaceMouseWrapper



ROBOT_IP = "10.10.10.10"

def main():
    ctrl = CartesianJogController(ip=ROBOT_IP, velocity=1, acceleration=1.0)
    ctrl.start()

    sm = SpaceMouseWrapper(
        max_speed=0.05, max_rot_speed=0.1,
        x_init=0.5, y_init=0.5, z_init=0.5,
        roll_init=np.pi, pitch_init=0.0, yaw_init=0.0
        )

    print("Cartesian jog + SpaceMouse")
    try:
        while True:
            # For now we are not using the x, y, z, r, p, y for position control inside the cartesian jog, 
            # we just simply take space mouse values and directly set the directions like a velocity control
            x, y, z, roll, pitch, yaw = sm.get_pose()
            vx, vy, vz, vyaw = sm.get_velocity()
            ctrl.set_velocities(vx, vy, vz, vyaw)
            # print(f"X: {x:+.4f}m  Y: {y:+.4f}m  Z: {z:+.4f}m  "
            #       f"R: {roll:+.4f}rad  P: {pitch:+.4f}rad  Y: {yaw:+.4f}rad",
            #       end="\r")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        ctrl.stop()
        sm.stop()


if __name__ == "__main__":
    main()
