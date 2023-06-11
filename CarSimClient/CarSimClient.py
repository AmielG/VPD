import airsim
import time
import keyboard
import numpy as np
import atexit
from datetime import datetime
import pandas as pd
from pytransform3d import rotations as rt


class CarSimClient:

    def __init__(self, logfile):
        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        self.records = None
        self.start_time = None
        self.logfile = logfile

        #  Write records to file when on script termination
        atexit.register(self.write_records_to_file)

    def run(self):
        while True:
            self.client.setCarControls(self.car_controls)
            self.record_state()
            time.sleep(0.04)

    def write_records_to_file(self):
        logfile = "logs/" + self.logfile
        columns = ['timestamp', 'has_collided', 'throttle', 'brake', 'steering', 'handbrake', 'gear',
                   'x', 'y', 'z', 'x_vel', 'y_vel', 'z_vel', 'x_acc', 'y_acc', 'z_acc',
                   'q_x', 'q_y', 'q_z', 'q_w', 'q_x_vel', 'q_y_vel', 'q_z_vel', 'q_x_acc', 'q_y_acc', 'q_z_acc']
        df = pd.DataFrame(self.records, columns=columns)
        df.to_pickle(logfile)
        print(f"{self.records.shape[0]} Records saved!")

    def record_state(self):
        log_row = self.get_flatten_car_data()
        if self.records is None:
            self.records = log_row
        else:
            self.records = np.vstack((self.records, log_row))

        if self.records.shape[0] % 100 == 0:
            print(f"{self.records.shape[0]} records")

    def get_flatten_car_data(self):
        state = self.client.getCarState()
        controls = self.client.getCarControls()
        collision_info = self.client.simGetCollisionInfo()
        flatten_kinematics_estimated = np.array([self.format_timestamp(state.timestamp), collision_info.has_collided, controls.throttle, controls.brake, controls.steering, controls.handbrake, state.gear])
        flatten_kinematics_estimated = np.append(flatten_kinematics_estimated, state.kinematics_estimated.position.to_numpy_array())
        flatten_kinematics_estimated = np.append(flatten_kinematics_estimated, state.kinematics_estimated.linear_velocity.to_numpy_array())
        flatten_kinematics_estimated = np.append(flatten_kinematics_estimated, state.kinematics_estimated.linear_acceleration.to_numpy_array())
        flatten_kinematics_estimated = np.append(flatten_kinematics_estimated, state.kinematics_estimated.orientation.to_numpy_array())
        flatten_kinematics_estimated = np.append(flatten_kinematics_estimated, state.kinematics_estimated.angular_velocity.to_numpy_array())
        flatten_kinematics_estimated = np.append(flatten_kinematics_estimated, state.kinematics_estimated.angular_acceleration.to_numpy_array())
        return flatten_kinematics_estimated

    def get_flatten_car_pose_and_measurements(self):
        state = self.client.getCarState()

        q = state.kinematics_estimated.orientation.to_numpy_array()
        pose = np.array([self.format_timestamp(state.timestamp)])
        pose = np.append(pose, state.kinematics_estimated.position.to_numpy_array())
        pose = np.append(pose, q)

        measurements = self.transform_vector_to_body_frame(q, state.kinematics_estimated.linear_velocity.to_numpy_array())
        measurements = np.append(measurements, state.kinematics_estimated.angular_velocity.to_numpy_array())
        a = self.transform_vector_to_body_frame(q, state.kinematics_estimated.linear_acceleration.to_numpy_array())
        measurements = np.append(measurements, a)
        measurements = np.append(measurements, state.kinematics_estimated.angular_acceleration.to_numpy_array())
        return self.format_timestamp(state.timestamp), pose, measurements

    @staticmethod
    def transform_vector_to_body_frame(q, v):
        q = [-q[3], q[0], q[1], q[2]]
        rotation = rt.matrix_from_quaternion(q)
        v = np.array([[v[2], -v[0], v[1]]]).T
        b = rotation @ v
        return np.atleast_1d(b)

    def format_timestamp(self, timestamp):
        if self.start_time is None:
            self.start_time = timestamp
        return (timestamp - self.start_time) * 1e-9

    def start_keyboard_control(self):
        # Subscribe to a and d press event for steering
        keyboard.on_press_key("d", lambda _: self.set_steering(1))
        keyboard.on_press_key("a", lambda _: self.set_steering(-1))
        keyboard.on_release_key("d", lambda _: self.set_steering(0))
        keyboard.on_release_key("a", lambda _: self.set_steering(0))

        # Subscribe to w and s press event for throttle
        keyboard.on_press_key("w", lambda _: self.set_throttle(1, 0, True))
        keyboard.on_press_key("s", lambda _: self.set_throttle(0, -1, True))
        keyboard.on_release_key("w", lambda _: self.set_throttle(0, 0, True))
        keyboard.on_release_key("s", lambda _: self.set_throttle(0, 0, True))

    def set_steering(self, steering):
        self.car_controls.steering = steering

    def set_throttle(self, throttle, brake, is_forward):
        self.car_controls.set_throttle(throttle, is_forward)
        self.car_controls.brake = brake
