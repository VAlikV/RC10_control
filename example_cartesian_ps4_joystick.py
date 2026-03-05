"""Real-time joystick control using Cartesian linear jog"""

import time
import numpy as np

from rc10_api.controller import CartesianJogController
from rc10_api.ps4_joystick import PS4Joystick


ROBOT_IP = "10.10.10.10"

def main():
    ctrl = CartesianJogController(ip=ROBOT_IP, velocity=1, acceleration=1.0)
    ctrl.start()

    joy = PS4Joystick(max_speed=0.05, x_init=0.5, y_init=0.5, z_init=0.5)
    joy.start()

    print("Cartesian jog + joystick active")
    print(f"Current TCP: {ctrl.get_current_tcp()}")

    try:
        while True:
            x, y, z = joy.get_joystick()
            ctrl.set_target(x, y, z)
            print(f"Target: [{x:.4f}, {y:.4f}, {z:.4f}]", end='\r')
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        ctrl.stop()
        joy.stop()


if __name__ == "__main__":
    main()
