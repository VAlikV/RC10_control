import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class TaskSpaceJogController:
    def __init__(self, robot_interface, rate_hz=100):
        """
        :param robot_interface: объект, предоставляющий метод jog(joint_deltas)
        :param rate_hz: частота управления (Гц)
        """
        self.robot = robot_interface
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        self.robot.payload.set(mass=0, tcp_mass_center=(0, 0, 0))
        self.robot.motion.scale_setup.set(velocity=1, acceleration=1)
        self.robot.controller_state.set('run', await_sec=120)

        self.target_pose = None  # целевая конфигурация в joint_space
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

        self.treshold_pos = 0.001
        self.treshold_angel = 1 * np.pi/180

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_loop)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def set_target(self, target_pos, target_rot):

        roll, pitch, yaw = R.from_matrix(target_rot).as_euler('xyz', degrees=False)

        target_pose = np.array([target_pos[0], target_pos[1], target_pos[2], roll, pitch, yaw])

        with self.lock:
            self.target_pose = target_pose

    def get_current_joint(self):
        rtd = self.robot._rtd_receiver.rt_data
        q = np.array(rtd.act_q)

        return q
    
    def get_current_tcp(self):
        rtd = self.robot._rtd_receiver.rt_data
        tcp = np.array(rtd.act_tcp_x)

        return tcp
        
    def _run_loop(self):
        while self.running:
            start_time = time.time()

            with self.lock:
                target_pose = self.target_pose

            # print(target_q)

            if target_pose is not None:
                current_pose = self.get_current_tcp()
                error = target_pose - current_pose
                dir = self._compute_jog_step(error)
                self.robot.motion.linear.jog_once_all_axis(dir)

            elapsed = time.time() - start_time
            time_to_sleep = self.dt - elapsed
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

    def _compute_jog_step(self, error):

        error_pos = np.sign(np.where((np.abs(error[0:3])<self.treshold_pos), 0, error[0:3]))
        error_angle = np.sign(np.where((np.abs(error[3:6])<self.treshold_angel), 0, error[3:6]))

        dir = ['+' if i > 0 else '-' if i < 0 else '0' for i in error_pos] + ['0']*3 # ['+' if i > 0 else '-' if i < 0 else '0' for i in error_angle]
        
        return dir
    
