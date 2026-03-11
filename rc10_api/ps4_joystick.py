import pygame
import time
import threading
import numpy as np


class PS4Joystick:
    """Non-blocking PS4 joystick controller that integrates stick input into position values"""

    def __init__(self, max_speed=0.05, max_rot_speed=0.1, deadzone=0.05, alpha=0.3, poll_rate=100,
                 x_init=0.0, y_init=0.0, z_init=0.0, roll_init=np.pi, pitch_init=0.0, yaw_init=0.0):
        self.max_speed = max_speed
        self.max_rot_speed = max_rot_speed
        self.deadzone = deadzone
        self.alpha = alpha
        self.poll_rate = poll_rate
        self.dt = 1.0 / poll_rate

        # Position accumulators
        self.x_meter = x_init
        self.y_meter = y_init
        self.z_meter = z_init

        # Orientation accumulator
        self.roll_radian = roll_init
        self.pitch_radian = pitch_init
        self.yaw_radian = yaw_init

        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.droll = 0.0
        self.dpitch = 0.0
        self.dyaw = 0.0


        # Smoothed joystick values
        self._smoothed_x = 0.0
        self._smoothed_y = 0.0
        self._smoothed_z = 0.0
        self._smoothed_roll = 0.0
        self._smoothed_pitch = 0.0
        self._smoothed_yaw = 0.0


        # Button state: 1.0 = open (default), -1.0 = closed
        self._gripper_state = 1.0

        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller found")
        self._controller = pygame.joystick.Joystick(0)
        self._controller.init()
        self.start()
        print(f"Connected: {self._controller.get_name()}")

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return (value - (self.deadzone if value > 0 else -self.deadzone)) / (1 - self.deadzone)

    def _poll_loop(self):
        while self._running:
            pygame.event.pump()

            raw_x = self._controller.get_axis(0)  # Left stick horizontal
            raw_y = self._controller.get_axis(1)  # Left stick vertical
            raw_z = self._controller.get_axis(4)  # Right stick vertical

            # TODO: add roll and pitch maping to joystick later, not needed right now
            raw_roll = 0.0 # Set this later
            raw_pitch = 0.0 # Set this later 
            raw_yaw = self._controller.get_axis(3)  # Right stick horizontal

            if self._controller.get_button(0):  # cross/x button
                self._toggle_gripper_state()

            dead_x = self._apply_deadzone(raw_x)
            dead_y = self._apply_deadzone(raw_y)
            dead_z = self._apply_deadzone(raw_z)
            dead_roll = self._apply_deadzone(raw_roll)
            dead_pitch = self._apply_deadzone(raw_pitch)

            dead_yaw = self._apply_deadzone(raw_yaw)

            self._smoothed_x = (self.alpha * dead_x) + ((1 - self.alpha) * self._smoothed_x)
            self._smoothed_y = (self.alpha * dead_y) + ((1 - self.alpha) * self._smoothed_y)
            self._smoothed_z = (self.alpha * dead_z) + ((1 - self.alpha) * self._smoothed_z)

            self._smoothed_roll = (self.alpha * dead_roll) + ((1 - self.alpha) * self._smoothed_roll)
            self._smoothed_pitch = (self.alpha * dead_pitch) + ((1 - self.alpha) * self._smoothed_pitch)
            self._smoothed_yaw = (self.alpha * dead_yaw) + ((1 - self.alpha) * self._smoothed_yaw)

            # Integrate: joystick value is velocity, accumulate into position
            # UP (negative smoothed_y) → increase X_meter, so negate
            # LEFT (negative smoothed_x) → increase Y_meter, so negate
            # Right stick UP (negative smoothed_z) → increase Z_meter, so negate
            dx = -self._smoothed_y * self.max_speed * self.dt
            dy = -self._smoothed_x * self.max_speed * self.dt
            dz = -self._smoothed_z * self.max_speed * self.dt

            # Rotation:
            droll = -self._smoothed_roll * self.max_rot_speed * self.dt
            dpitch = -self._smoothed_pitch * self.max_rot_speed * self.dt
            dyaw = -self._smoothed_yaw * self.max_rot_speed * self.dt

            with self._lock:
                self.x_meter += dx
                self.y_meter += dy
                self.z_meter += dz
                self.roll_radian += droll
                self.pitch_radian += dpitch
                self.yaw_radian += dyaw

                self.dx = dx
                self.dy = dy
                self.dz = dz

                self.droll = droll
                self.dpitch = dpitch
                self.dyaw = dyaw

            time.sleep(self.dt)

    def start(self):
        """Start the background polling thread"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the background polling thread"""
        self._running = False
        if self._thread:
            self._thread.join()
        pygame.quit()

    def get_joystick(self):
        """Returns (X_meter, Y_meter, Z_meter) - current accumulated position"""
        with self._lock:
            return self.x_meter, self.y_meter, self.z_meter, self.roll_radian, self.pitch_radian, self.yaw_radian
        
    def get_delta(self):
        """Returns (dX_meter, dY_meter, dZ_meter) - current accumulated position"""
        with self._lock:
            return self.dx, self.dy, self.dz, self.droll, self.dpitch, self.dyaw

    def get_gripper_state(self):
        """Returns 1.0 (open) or -1.0 (closed). Toggled by PS4 button x press."""
        with self._lock:
            return self._gripper_state
        
    def _toggle_gripper_state(self):
        """Toggle the gripper state"""
        with self._lock:
            self._gripper_state = -self._gripper_state

    def reset(self):
        """Reset position to origin"""
        with self._lock:
            self.x_meter = 0.0
            self.y_meter = 0.0
            self.z_meter = 0.0
            self.yaw_radian = 0.0


if __name__ == "__main__":
    joy = PS4Joystick(max_speed=0.05)
    joy.start()
    try:
        while True:
            x, y, z, yaw = joy.get_joystick()
            print(f"X_meter: {x:.4f}, Y_meter: {y:.4f}, Z_meter: {z:.4f} "
                  f"yaw_radian: {yaw: 4f}",
                  end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        joy.stop()
