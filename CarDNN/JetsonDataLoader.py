import torch
import torch.utils.data
from glob import glob
import pandas as pd
import numpy as np
from pytransform3d import transformations as tf
from pytransform3d import rotations as rt
from CarStateDataset import CarStateDataset
import math
from FramePlotter import FramePlotter
import matplotlib.pyplot as plt


class JetsonDataLoader:

    @staticmethod
    def load(log):
        columns = ['timestamp', 'un', 'throttle', 'brake', 'steering', 'x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w', 'x_vel',
                   'y_vel', 'z_vel', 'q_x_vel', 'q_y_vel', 'q_z_vel', 'x_acc', 'y_acc', 'z_acc']
        df = pd.read_csv(log)
        df.columns = columns
        return df

    @staticmethod
    def load_all():
        logs = glob("dataset/*.csv")
        data = JetsonDataLoader.load(logs[0])
        x, y = JetsonDataLoader.format_data(data)
        for log in logs[1:]:
            data = JetsonDataLoader.load(log)
            _x, _y = JetsonDataLoader.format_data(data)
            x = np.vstack((x, _x))
            y = np.vstack((y, _y))
        return x, y

    @staticmethod
    def load_data_as_frames(csv_file):
        data = JetsonDataLoader.load(csv_file)
        data = data.drop_duplicates(subset=['x', 'y', 'z', 'q_x', 'q_z', 'q_y', 'q_w'], keep="first")
        return JetsonDataLoader.calc_car_frame(data)

    @staticmethod
    def format_data(data):
        print(data.shape)
        # TODO: To filter the data near collision events
        # Filter records with the same position and orientation
        data = data.drop_duplicates(subset=['x', 'y', 'z', 'q_x', 'q_z', 'q_y', 'q_w'], keep="first")

        # Format the input data - the user control signals and car state
        x = data[['throttle', 'brake', 'steering', 'x_vel', 'y_vel', 'z_vel',
                   'q_x_vel', 'q_y_vel', 'q_z_vel', 'x_acc', 'y_acc', 'z_acc']].to_numpy()

        timestamp = data[['timestamp']].to_numpy()
        dt = np.diff(timestamp, axis=0) * 10e-9

        # Format the output data - the translation of the car: x, y and yaw
        y = JetsonDataLoader.calc_translation_in_car_frame(data)
        y = np.hstack((y, x[1:, 3:]))  # Add the velocity and acceleration to the prediction

        x = np.hstack((dt, x[:-1, :]))  # Add dt to the input vector and remove the last row from the data

        y[:, 0:3] = y[:, 0:3] - JetsonDataLoader.calc_translations_by_linear_dynamic(x)

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
        frames = JetsonDataLoader.calc_car_frame(data)

        for i, frame in enumerate(frames[:-1]):
            translation = np.linalg.inv(frame) @ frames[i+1]
            translations.append(JetsonDataLoader.convert_frame_to_xy_yaw(translation))

        return np.array(translations)

    @staticmethod
    def calc_car_frame(data):
        frames = []
        for index, row in data.iterrows():
            frame = tf.transform_from_pq([row["x"], row["y"], row["z"], row["q_w"], row["q_x"], row["q_y"], row["q_z"]])
            frames.append(frame)
        return frames

    @staticmethod
    def convert_frame_to_xy_yaw(frame):
        pq = tf.pq_from_transform(frame)
        position, quaternion = pq[:3], pq[3:]
        roll_x, pitch_y, yaw_z = JetsonDataLoader.euler_from_quaternion(quaternion)
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

        training_loader = torch.utils.data.JetsonDataLoader(train_set, batch_size=batch_size, shuffle=True, num_workers=2)
        validation_loader = torch.utils.data.JetsonDataLoader(val_set, batch_size=batch_size, shuffle=False, num_workers=2)
        return training_loader, validation_loader


def main():
    # frames = JetsonDataLoader.load_data_as_frames("dataset/data_18_9_1.csv")
    # for frame in frames:
    #     FramePlotter.draw_frame(frame)
    data = JetsonDataLoader.load("dataset/data_18_9_5.csv")
    plt.plot(data["x"], data["y"])
    plt.show()

    # x, y = JetsonDataLoader.load_all()
    # dataset = CarStateDataset(x, y)
    # training_loader, validation_loader = JetsonDataLoader.split_train_test(dataset, 0.7)


if __name__ == "__main__":
    main()

