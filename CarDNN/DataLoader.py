import torch
import torch.utils.data
from glob import glob
import pandas as pd
import numpy as np
from pytransform3d import transformations as tf
from pytransform3d import rotations as rt
from CarStateDataset import CarStateDataset
import math


class DataLoader:

    @staticmethod
    def load_the_last_log(df=False):
        logs = glob("data/*.pkl")
        data = pd.read_pickle(logs[-1])
        if df:
            return data
        x, y = DataLoader.format_data(data)
        return x, y

    @staticmethod
    def load(pkl_file):
        data = pd.read_pickle(pkl_file)
        x, y = DataLoader.format_data(data)
        return x, y

    @staticmethod
    def load_data_as_frames(pkl_file):
        data = pd.read_pickle(pkl_file)
        return DataLoader.calc_car_frame(data)

    @staticmethod
    def format_data(data):
        print(data.shape)
        # TODO: To filter the data near collision events
        # Filter records with the same timestamp
        data = data.drop_duplicates(subset='timestamp', keep="first")

        # Format the input data - the user control signals and car state
        x = data[['throttle', 'brake', 'steering', 'x_vel', 'y_vel', 'z_vel',
                   'q_x_vel', 'q_y_vel', 'q_z_vel', 'x_acc', 'y_acc', 'z_acc', 'q_x_acc', 'q_y_acc', 'q_z_acc']].to_numpy()

        timestamp = data[['timestamp']].to_numpy()
        dt = np.diff(timestamp, axis=0)

        # Transform velocity to body frame
        velocities = DataLoader.transform_velocity_to_body_frame(data)
        x[:, 3:6] = velocities

        # Calc the linear acceleration
        acceleration = DataLoader.transform_acc_to_body_frame(data)
        x[:, 9:12] = acceleration

        # Format the output data - the translation of the car: x, y and yaw
        y = DataLoader.calc_translation_in_car_frame(data)
        y = np.hstack((y, x[1:, 3:]))  # Add the velocity and acceleration to the prediction

        x = np.hstack((dt, x[:-1, :]))  # Add dt to the input vector and remove the last row from the data

        y[:, 0:3] = y[:, 0:3] - DataLoader.calc_translations_by_linear_dynamic(x)
        return x, y  # Remove the last row from the data

    @staticmethod
    def calc_translations_by_linear_dynamic(x):
        dt = x[:, 0]
        velocity = x[:, 4:7]
        angular_velocity = x[:, -4]

        longitudinal = np.linalg.norm(np.atleast_2d(dt).T * velocity, axis=1)
        yaw = -np.atleast_2d(dt * angular_velocity)

        x = np.cos(yaw) * longitudinal
        y = np.sin(yaw) * longitudinal

        return np.vstack((x, y, yaw)).T

    @staticmethod
    def calc_translation_in_car_frame(data):
        translations = []
        frames = DataLoader.calc_car_frame(data)

        for i, frame in enumerate(frames[:-1]):
            translation = np.linalg.inv(frame) @ frames[i+1]
            translations.append(DataLoader.convert_frame_to_xy_yaw(translation))

        return np.array(translations)

    @staticmethod
    def calc_car_frame(data):
        frames = []
        for index, row in data.iterrows():
            # The w element is in minus due to a bug, probably when converting from the Unity frame of reference
            frame = tf.transform_from_pq([row["x"], row["y"], row["z"], -row["q_w"], row["q_x"], row["q_y"], row["q_z"]])
            frames.append(frame)
        return frames

    @staticmethod
    def transform_velocity_to_body_frame(data):
        velocities = []
        for index, row in data.iterrows():
            velocity_g = np.array([[row['z_vel'], -row['x_vel'], row['y_vel']]]).T
            q = [-row["q_w"], row["q_x"], row["q_y"], row["q_z"]]
            rotation = rt.matrix_from_quaternion(q)
            velocity_b = rotation @ velocity_g
            velocities.append(velocity_b.T)
        velocities = np.array(velocities)
        return velocities.reshape(velocities.shape[0], velocities.shape[2])

    @staticmethod
    def transform_acc_to_body_frame(data):
        aac = []
        for index, row in data.iterrows():
            acc_g = np.array([[row['z_acc'], -row['x_acc'], row['y_acc']]]).T
            q = [-row["q_w"], row["q_x"], row["q_y"], row["q_z"]]
            rotation = rt.matrix_from_quaternion(q)
            acc_b = rotation @ acc_g
            aac.append(acc_b.T)
        aac = np.array(aac)
        return aac.reshape(aac.shape[0], aac.shape[2])

    @staticmethod
    def calc_first_order_derivative(data, dt):
        diff = np.diff(data, axis=0)
        return diff / dt

    @staticmethod
    def moving_average(x, w):
        x_smooth = []
        for col in x.T:
            s = np.convolve(col, np.ones(w), 'valid') / w
            x_smooth.append(np.hstack((col[:w-1], s)))
        return np.array(x_smooth).T

    @staticmethod
    def convert_frame_to_xy_yaw(frame):
        pq = tf.pq_from_transform(frame)
        position, quaternion = pq[:3], pq[3:]
        roll_x, pitch_y, yaw_z = DataLoader.euler_from_quaternion(quaternion)
        return [position[0], position[1], yaw_z]

    @staticmethod
    def euler_from_quaternion(quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    @staticmethod
    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qw, qx, qy, qz]

    @staticmethod
    def split_train_test(dataset, test_ratio, batch_size=500):
        assert abs(test_ratio) < 1
        train_size = int(len(dataset) * test_ratio)
        test_size = len(dataset) - train_size
        train_set, val_set = torch.utils.data.random_split(dataset, [train_size, test_size])
        print("Total records in the dataset: {}, Train size: {}, Test size: {}".format(len(dataset), train_size, test_size))

        training_loader = torch.utils.data.DataLoader(train_set, batch_size=batch_size, shuffle=True, num_workers=2)
        validation_loader = torch.utils.data.DataLoader(val_set, batch_size=batch_size, shuffle=False, num_workers=2)
        return training_loader, validation_loader


def main():
    x, y = DataLoader.load("data/car-data--02-06-2022--18-12-41-lite-sim.pkl")
    dataset = CarStateDataset(x, y)
    training_loader, validation_loader = DataLoader.split_train_test(dataset, 0.7)


if __name__ == "__main__":
    main()

