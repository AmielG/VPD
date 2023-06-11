import sys
import torch
import numpy as np
import time
sys.path.insert(1, 'C:/Users/Amiel/Projects/CarDNN')

from CarDNN import CarDNN
from DataLoader import DataLoader
from CarStateDataset import CarStateDataset


class DNNEstimator:

    def __init__(self, dataset):
        self.dataset = dataset
        self.model = CarDNN()
        self.model.load_the_last_model()
        self.model.eval()

    def estimate(self, z_nm1, u_nm2, u_nm1, dt):
        z_n_hat = self.dynamics(z_nm1, u_nm2, dt)
        z_np1_hat = self.dynamics(z_nm1, u_nm1, dt)
        return z_n_hat, z_np1_hat

    def dynamics(self, state, control, dt):
        x = np.hstack((control, state))
        x = np.hstack((dt, x))
        x_torch = torch.tensor(self.dataset.z_score_standardization_x(x)).float()
        x_torch = torch.atleast_2d(x_torch)
        y = self.model(x_torch).detach().numpy()
        y = self.dataset.undo_output_standardization(y)
        return y[0, :]


if __name__ == "__main__":
    x, y = DataLoader.load("data/car-data--02-06-2022--18-12-41-lite-sim.pkl")
    dataset = CarStateDataset(x, y)
    d = DNNEstimator(dataset)
