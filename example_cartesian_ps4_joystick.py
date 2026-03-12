"""Real-time joystick control using Cartesian linear jog"""

import time
import numpy as np

from rc10_api.controller import CartesianJogController
from rc10_api.ps4_joystick import PS4Joystick

from rc10_api.gripper import Gripper


ROBOT_IP = "10.10.10.10"

def main():
    ctrl = CartesianJogController(ip=ROBOT_IP, velocity=1, acceleration=1.0)
    ctrl.start()

    joy = PS4Joystick(max_speed=0.05, x_init=0.5, y_init=0.5, z_init=0.5, roll_init=np.pi, pitch_init=0.0, yaw_init=0.0)

    gripper = Gripper()

    print("Cartesian jog + joystick active")
    print(f"Current TCP: {ctrl.get_current_tcp()}")

    try:
        while True:
            dvx, dvy, dvz, dvroll, dvpitch, dvyaw = joy.get_delta_velocities()
            ctrl.set_velocities(dvx, dvy, dvz, dvroll, dvpitch, dvyaw)
            print(f"Delta velocities: [{dvx:.4f}, {dvy:.4f}, {dvz:.4f}, {dvroll:.4f}, {dvpitch:.4f}, {dvyaw:.4f}]", end='\r')
            gripper.send(joy.get_gripper_state())
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        ctrl.stop()
        joy.stop()


if __name__ == "__main__":
    main()
