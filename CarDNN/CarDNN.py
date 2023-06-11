import torch
import torch.nn as nn
from glob import glob
import numpy as np
import os


class CarDNN(nn.Module):
    INPUT_SIZE = 13
    OUTPUT_SIZE = 3 + 9  # Predict the translation and the velocity and acceleration

    def __init__(self):
        super(CarDNN, self).__init__()
        self.fc1 = nn.Linear(self.INPUT_SIZE, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 256)
        self.fc4 = nn.Linear(256, 64)
        self.fc5 = nn.Linear(64, self.OUTPUT_SIZE - 3)

        self.prelu1 = nn.PReLU()
        self.prelu2 = nn.PReLU()
        self.prelu3 = nn.PReLU()
        self.prelu4 = nn.PReLU()

        self.fc1_2 = nn.Linear(10, 64)
        self.fc2_2 = nn.Linear(64, 128)
        self.fc3_2 = nn.Linear(128, 64)
        self.fc4_2 = nn.Linear(64, 3)

        self.prelu1_2 = nn.PReLU()
        self.prelu2_2 = nn.PReLU()
        self.prelu3_2 = nn.PReLU()

        self.dropout05 = nn.Dropout(p=0.5)
        self.dropout01 = nn.Dropout(p=0.1)

    def forward(self, x):
        x_1 = self.dropout01(x)
        x_1 = self.prelu1(self.fc1(x_1))
        x_1 = self.dropout05(x_1)
        x_1 = self.prelu2(self.fc2(x_1))
        x_1 = self.dropout05(x_1)
        x_1 = self.prelu3(self.fc3(x_1))
        x_1 = self.dropout05(x_1)
        x_1 = self.prelu4(self.fc4(x_1))
        x_1 = self.dropout01(x_1)
        x_1 = self.fc5(x_1)

        x_1[:, 3:] = x_1[:, 3:] + x[:, 4:10]  # Add the velocity from the previous step

        x_2 = self.dropout01(x[:, :10])
        x_2 = self.prelu1_2(self.fc1_2(x_2))
        x_2 = self.dropout05(x_2)
        x_2 = self.prelu2_2(self.fc2_2(x_2))
        x_2 = self.dropout05(x_2)
        x_2 = self.prelu3_2(self.fc3_2(x_2))
        x_2 = self.dropout01(x_2)
        x_2 = self.fc4_2(x_2)

        return torch.hstack((x_1, x_2))

    def load_the_last_model(self):
        models = sorted(glob("models/*"), key=os.path.getmtime)
        model_filename = models[-1]
        print(f"The last model {model_filename}")
        self.load_state_dict(torch.load(model_filename))

    def load_by_filename(self, filename):
        model_filename = "models/" + filename
        print(f"Load model {model_filename}")
        self.load_state_dict(torch.load(model_filename))

    @staticmethod
    def calc_translations_by_linear_dynamic(x, dt):
        dt = x[:, 0]
        velocity = x[:, 4:7]
        angular_velocity = x[:, 9]

        longitudinal = np.linalg.norm(np.atleast_2d(dt).T * velocity, axis=1)
        yaw = -np.atleast_2d(dt * angular_velocity)


 
  
        
        vx = x[:, 4] + x[:, 10] * dt
        vy = x[:, 5] + x[:, 11] * dt
        vz = x[:, 6] + x[:, 12] * dt
        # yaw_dot = angular_velocity + x[:, 15] * dt
        yaw_dot = angular_velocity

        x = np.cos(yaw) * longitudinal
        y = np.sin(yaw) * longitudinal

        return np.vstack((x, y, yaw, vx, vy, vz, yaw_dot*0, yaw_dot*0, yaw_dot)).T

