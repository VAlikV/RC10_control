import pygame
import time
import threading


class PS4Joystick:
    """Non-blocking PS4 joystick controller that integrates stick input into position values"""

    def __init__(self, max_speed=0.05, deadzone=0.05, alpha=0.3, poll_rate=100,
                 x_init=0.0, y_init=0.0, z_init=0.0):
        self.max_speed = max_speed
        self.deadzone = deadzone
        self.alpha = alpha
        self.poll_rate = poll_rate
        self.dt = 1.0 / poll_rate

        # Position accumulators
        self.x_meter = x_init
        self.y_meter = y_init
        self.z_meter = z_init

        # Smoothed joystick values
        self._smoothed_x = 0.0
        self._smoothed_y = 0.0
        self._smoothed_z = 0.0

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

            dead_x = self._apply_deadzone(raw_x)
            dead_y = self._apply_deadzone(raw_y)
            dead_z = self._apply_deadzone(raw_z)

            self._smoothed_x = (self.alpha * dead_x) + ((1 - self.alpha) * self._smoothed_x)
            self._smoothed_y = (self.alpha * dead_y) + ((1 - self.alpha) * self._smoothed_y)
            self._smoothed_z = (self.alpha * dead_z) + ((1 - self.alpha) * self._smoothed_z)

            # Integrate: joystick value is velocity, accumulate into position
            # UP (negative smoothed_y) → increase X_meter, so negate
            # LEFT (negative smoothed_x) → increase Y_meter, so negate
            # Right stick UP (negative smoothed_z) → increase Z_meter, so negate
            dx = -self._smoothed_y * self.max_speed * self.dt
            dy = -self._smoothed_x * self.max_speed * self.dt
            dz = -self._smoothed_z * self.max_speed * self.dt

            with self._lock:
                self.x_meter += dx
                self.y_meter += dy
                self.z_meter += dz

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
            return self.x_meter, self.y_meter, self.z_meter

    def reset(self):
        """Reset position to origin"""
        with self._lock:
            self.x_meter = 0.0
            self.y_meter = 0.0
            self.z_meter = 0.0


if __name__ == "__main__":
    joy = PS4Joystick(max_speed=0.05)
    joy.start()
    try:
        while True:
            x, y, z = joy.get_joystick()
            print(f"X_meter: {x:.4f}, Y_meter: {y:.4f}, Z_meter: {z:.4f}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        joy.stop()
