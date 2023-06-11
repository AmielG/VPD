import numpy as np
from glob import glob
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd


class CarLogPlotter:

    def __init__(self):
        self.data = []
        self.load_the_last_log()

    def load_the_last_log(self):
        logs = glob("logs/*.pkl")
        self.data = pd.read_pickle(logs[-1])
        self.data["timestamp"] = self.data["timestamp"] / 1000  # Convert timestamp to seconds

    def load_by_filename(self, filename):
        self.data = pd.read_pickle(filename)
        self.data["timestamp"] = self.data["timestamp"] / 1000  # Convert timestamp to seconds

    def plot_linear(self, to_show=False):
        marker_size = 17
        fig, axes = plt.subplots(3, 3)
        sns.scatterplot(ax=axes[0, 0], data=self.data, x='timestamp', y='x', s=marker_size)
        sns.scatterplot(ax=axes[0, 1], data=self.data, x='timestamp', y='y', s=marker_size)
        sns.scatterplot(ax=axes[0, 2], data=self.data, x='timestamp', y='z', s=marker_size)
        sns.scatterplot(ax=axes[1, 0], data=self.data, x='timestamp', y='x_vel', s=marker_size)
        sns.scatterplot(ax=axes[1, 1], data=self.data, x='timestamp', y='y_vel', s=marker_size)
        sns.scatterplot(ax=axes[1, 2], data=self.data, x='timestamp', y='z_vel', s=marker_size)
        sns.scatterplot(ax=axes[2, 0], data=self.data, x='timestamp', y='x_acc', s=marker_size)
        sns.scatterplot(ax=axes[2, 1], data=self.data, x='timestamp', y='y_acc', s=marker_size)
        sns.scatterplot(ax=axes[2, 2], data=self.data, x='timestamp', y='z_acc', s=marker_size)
        if to_show:
            plt.show()

    def plot_xy_path(self, to_show=False):
        sns.scatterplot(data=self.data, x='x', y='y')
        if to_show:
            plt.show()

    def plot_speed(self, to_show=False):
        vel = self.data[['x_vel', 'y_vel', 'z_vel']].to_numpy()
        speed = np.sqrt(np.sum(np.power(vel, 2), axis=1))
        plt.plot(speed)

    def plot_throttle(self, to_show=False):
        throttle = self.data[['throttle']].to_numpy()
        plt.plot(throttle)


if __name__ == "__main__":
    c = CarLogPlotter()
    logs = glob("logs/*.pkl")
    for log in logs:
        c.load_by_filename(log)
        c.plot_xy_path()
    plt.legend(labels=logs)
    plt.show()

