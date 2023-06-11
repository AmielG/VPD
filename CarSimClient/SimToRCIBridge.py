import time
from CarSimClient import CarSimClient
from RCIClient import RCIClient
from DNNEstimator import DNNEstimator
from utilities import *
import numpy as np
import atexit
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt

import sys
sys.path.insert(1, 'C:/Users/user/Projects/CarDNN')
from DataLoader import DataLoader
from CarStateDataset import CarStateDataset


def record_state(a, b, c, d):
    global records
    log_row = np.hstack((np.array(a), b, c, d))
    if records is None:
        records = log_row
    else:
        records = np.vstack((records, log_row))

    if records.shape[0] % 100 == 0:
        print(f"{records.shape[0]} records")


def write_records_to_file():
    global records
    columns = ['timestamp_np1_hat', 'x_np1_hat', 'y_np1_hat', 'z_np1_hat',
               'timestamp', 'x', 'y', 'z',
               'timestamp_nm1', 'x_nm1', 'y_nm1', 'z_nm1']
    df = pd.DataFrame(records, columns=columns)
    df.to_pickle("exps/" + logfile)
    print(f"{records.shape[0]} Records saved!")


def append_to_commands_history(commands_history, command, current_time, delay):
    commands_history.append(command)
    oldest_command_h = commands_history[0]
    if current_time - oldest_command_h["time"] > 2.5 * delay:
        commands_history.pop(0)
    return commands_history


def predict_car_pose():
    z_nm1 = oldest_measurement["measurements"]
    x_hat = oldest_measurement["pose"]
    time_hat = oldest_measurement["time"]

    trajectory = []
    united_commands = commands_history + commands_queue
    # united_commands = commands_queue
    if united_commands.__len__() < 2:
        return x_hat

    for i, c in enumerate(united_commands):
        if i != united_commands.__len__() - 1:
            dt = united_commands[i + 1]["time"] - c["time"]
        else:
            dt = c["time"] - united_commands[i - 1]["time"]
        y_hat = estimator.dynamics(z_nm1, c["command"], dt)
        t_hat = y_hat[:3]
        z_nm1 = y_hat[3:]
        time_hat = time_hat + dt
        x_hat = add_translation(x_hat, t_hat, time_hat)
        trajectory.append(x_hat)

    return x_hat


MODE = input("Enter sim mode (WO/PD/PVD)-(s/h): ")
CAMERA_DELAY = float(input("Enter the delay for camera: [s] "))  # [s]
CONTROL_DELAY = float(input("Enter the delay for the control and the measurements: [s] "))  # [s]
MEASUREMENTS_DELAY = CONTROL_DELAY  # [s]
UPDATE_RATE = 1 / 45  # [Hz]
atexit.register(write_records_to_file)
records = None

if __name__ == "__main__":
    print(f"start AirSim to RCI bridge, CAMERA_DELAY: {CAMERA_DELAY}, CONTROL_DELAY: {CONTROL_DELAY}, MEASUREMENTS_DELAY: {MEASUREMENTS_DELAY}")
    logfile = f'data_{datetime.now().strftime("%d-%m--%H-%M-%S")}_{CAMERA_DELAY}-{CONTROL_DELAY}-{MODE}.pkl'

    cs = CarSimClient(logfile)
    cs.client.reset()  # Reset the car pose on start
    rc = RCIClient()

    camera_queue = []
    commands_queue = []
    commands_history = []
    measurements_queue = []

    m_x, m_y = DataLoader.load("data/car-data--22-08-2022--14-11-05.pkl")
    dataset = CarStateDataset(m_x, m_y)
    estimator = DNNEstimator(dataset)

    timer = 0
    while True:
        cs.record_state()

        # Get the state from the AirSim and add to the state_queue
        new_timer, car_pose, measurements = cs.get_flatten_car_pose_and_measurements()

        # Skip the iteration if the data from the simulation didn't refresh
        if new_timer == timer:
            continue
        timer = new_timer
        camera_queue.append({"time": timer, "pose": car_pose})
        measurements_queue.append({"time": timer, "measurements": measurements, "pose": car_pose})

        # Get the control commands from the RCI and add to the commands_queue
        rc.publish_state_and_subscribe_control()
        commands_queue.append({"time": timer, "command": [rc.throttle, rc.brake, rc.steering]})

        # Estimate the vehicle state using delayed measurements and pass the pose of the virtual car to the RCI with delay
        oldest_measurement = measurements_queue[0]
        if timer - oldest_measurement["time"] >= MEASUREMENTS_DELAY:
            x_hat = predict_car_pose()
            # Logs
            record_state(x_hat[:4], timer, car_pose[1:4], oldest_measurement["pose"][:4])

            rc.set_car_pose(x_hat, oldest_measurement["pose"])
            # rc.set_car_pose(x_hat, car_pose)
            measurements_queue.pop(0)

        # Pass the control from the RCI to the AirSim with delay
        oldest_command = commands_queue[0]
        if timer - oldest_command["time"] >= CONTROL_DELAY:
            command = oldest_command["command"]
            cs.set_steering(command[2])
            cs.set_throttle(command[0], command[1], True)
            cs.client.setCarControls(cs.car_controls)
            oldest_command = commands_queue.pop(0)
            commands_history = append_to_commands_history(commands_history, oldest_command, timer, CONTROL_DELAY)

        # Pass the camera pose from the AirSim to the RCI with delay
        oldest_camera_pose = camera_queue[0]
        if timer - oldest_camera_pose["time"] >= CAMERA_DELAY:
            rc.set_camera_pose(oldest_camera_pose["pose"])
            camera_queue.pop(0)

        time.sleep(UPDATE_RATE)


