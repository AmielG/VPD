import torch
from torch.utils.data import Dataset


class CarStateDataset(Dataset):

    def __init__(self, x, y, transform=None, normalize_input=True, normalize_output=True):
        self.x = x
        self.y = y
        self.transform = transform

        self.x_mean = x.mean(axis=0)
        self.x_std = x.std(axis=0)
        self.y_mean = y.mean(axis=0)
        self.y_std = y.std(axis=0)

        if normalize_input:
            self.x = self.z_score_standardization_x(x)
        if normalize_output:
            self.y = self.z_score_standardization_y(y)

    def z_score_standardization_x(self, data):
        return (data - self.x_mean) / (self.x_std + 0.001)

    def z_score_standardization_y(self, data):
        return (data - self.y_mean) / self.y_std

    def undo_output_standardization(self, data):
        return data * self.y_std + self.y_mean

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        return {'x': self.x[idx], 'y': self.y[idx]}

