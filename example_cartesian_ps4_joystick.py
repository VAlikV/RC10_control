"""Real-time joystick control using Cartesian linear jog"""

import time
import numpy as np

from rc10_api.controller import CartesianJogController
from rc10_api.ps4_joystick import PS4Joystick


ROBOT_IP = "10.10.10.10"

def main():
    ctrl = CartesianJogController(ip=ROBOT_IP, velocity=1, acceleration=1.0)
    ctrl.start()

    joy = PS4Joystick(max_speed=0.05, x_init=0.5, y_init=0.5, z_init=0.5, roll_init=np.pi, pitch_init=0.0, yaw_init=0.0)

    print("Cartesian jog + joystick active")
    print(f"Current TCP: {ctrl.get_current_tcp()}")

    try:
        while True:
            dx, dy, dz, droll, dpitch, dyaw = joy.get_delta()
            ctrl.set_velocities(dx, dy, dz, droll, dpitch, dyaw)
            print(f"Delta velocities: [{dx:.4f}, {dy:.4f}, {dz:.4f}, {droll:.4f}, {dpitch:.4f}, {dyaw:.4f}]", end='\r')
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        ctrl.stop()
        joy.stop()


if __name__ == "__main__":
    main()
