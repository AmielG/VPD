import socket
import struct
import numpy as np
from threading import Thread
import time


class RCIClient:

    def __init__(self):
        self.update_rate = 60
        self.socket = None
        self.init_socket()
        socket.setdefaulttimeout(None)

        # Send the state to the RCI in 30Hz
        self.car_pose = None
        self.car_pose_nm1 = None
        self.camera_pose = None

        # Read the control input from the RCI in 30Hz
        self.steering = 0
        self.throttle = 0
        self.brake = 0
        self.is_forward = True

    def init_socket(self):
        self.socket = socket.socket()
        self.socket.bind(('127.0.0.1', 60000))
        self.socket.listen(30)

    def set_car_pose(self, car_pose, car_pose_nm1):
        self.car_pose = car_pose
        self.car_pose_nm1 = car_pose_nm1

    def set_camera_pose(self, camera_pose):
        self.camera_pose = camera_pose

    def publish_state_and_subscribe_control(self):
        message = np.hstack((self.car_pose, self.camera_pose, self.car_pose_nm1))
        if message.shape[0] != 3*8:
            return

        c, addr = self.socket.accept()  # when port connected

        # Receive control command
        bytes_received = c.recv(4000)  # received bytes
        control = np.frombuffer(bytes_received, dtype=np.float32)  # converting into float array
        throttle = float(control[0])
        self.throttle = throttle if throttle > 0 else 0
        self.brake = throttle if throttle < 0 else 0
        self.is_forward = True
        self.steering = float(control[1])

        # Send camera and car pose
        bytes_to_send = struct.pack('%sf' % len(message), *message)  # converting float to byte
        c.sendall(bytes_to_send)  # sending back

        c.close()
