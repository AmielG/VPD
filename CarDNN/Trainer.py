import numpy as np
import torch
from CarDNN import CarDNN
from DataLoader import DataLoader
from JetsonDataLoader import JetsonDataLoader
from CarStateDataset import CarStateDataset
from torch.utils.tensorboard import SummaryWriter
from datetime import datetime
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class Trainer:

    def __init__(self, model, training_loader, validation_loader, lr, x_std):
        self.model = model
        self.training_loader = training_loader
        self.validation_loader = validation_loader
        self.optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-3)
        self.x_std = x_std

    @staticmethod
    def loss(y, y_prediction):
        error = torch.square(y-y_prediction)
        loss_weights = torch.tensor([10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1]).to(device)
        weight_error = error #* loss_weights
        return weight_error.mean()

    def add_noise_to_input(self, input):
        noise_scale = np.atleast_2d(self.x_std) / 100
        noise = np.random.normal(scale=np.tile(noise_scale, (input.shape[0], 1)))
        return input + noise

    def train_one_epoch(self, epoch_index, tb_writer):
        avg_loss = 0.
        loss_sum = 0.

        for i, data in enumerate(self.training_loader):
            x = self.add_noise_to_input(data["x"]).to(device)
            y = data["y"].to(device)

            # Zero the gradients for every batch!
            self.optimizer.zero_grad()

            # Make predictions for this batch
            y_prediction = self.model(x.float())

            # Compute the loss and its gradients
            loss = Trainer.loss(y_prediction, y.float())
            loss.backward()

            # Adjust learning weights
            self.optimizer.step()

            # Gather data and report
            loss_sum += loss.item()
            avg_loss = loss_sum / (i + 1)  # loss per batch
            tb_x = epoch_index * len(self.training_loader) + i + 1
            tb_writer.add_scalar('Loss/train', avg_loss, tb_x)

        return avg_loss

    def train(self, epochs=5):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        writer = SummaryWriter('logs/car_model_trainer_{}'.format(timestamp))

        avg_v_loss = 666
        best_v_loss = 1_000_000.
        best_epoch_index = 0

        for epoch in range(epochs):
            print('EPOCH {}:'.format(epoch))

            # Make sure gradient tracking is on, and do a pass over the data
            self.model.train(True)
            avg_loss = self.train_one_epoch(epoch, writer)

            # We don't need gradients on to do reporting
            self.model.train(False)

            running_loss = 0.0
            for i, v_data in enumerate(self.validation_loader):
                v_x = v_data["x"].to(device)
                v_y = v_data["y"].to(device)

                v_y_prediction = self.model(v_x.float())
                v_loss = Trainer.loss(v_y_prediction, v_y)
                running_loss += v_loss
                avg_v_loss = running_loss / (i + 1)

            print('LOSS train {} valid {}'.format(avg_loss, avg_v_loss))

            # Log the running loss averaged per batch, for both training and validation
            writer.add_scalars('Training vs. Validation Loss', {'Training': avg_loss, 'Validation': avg_v_loss}, epoch)
            writer.flush()

            # Track the best performance, and save the model's state
            if avg_v_loss < best_v_loss:
                print("The best results so far! Saving the model")
                best_v_loss = avg_v_loss
                best_epoch_index = epoch
                model_path = 'models/model_{}_e{}'.format(timestamp, epoch)
                torch.save(self.model.state_dict(), model_path)

            if epoch - best_epoch_index > 50:
                raise Exception('Validation loss did not improve in the last 50 epochs!')


def main():
    # Fix seed
    seed = 42
    torch.manual_seed(seed)
    np.random.seed(seed)

    # To start tensorboard run: tensorboard --logdir logs
    # x, y = DataLoader.load("data/car-data--22-08-2022--14-11-05.pkl")
    x, y = JetsonDataLoader.load_all()

    dataset = CarStateDataset(x, y, normalize_input=True, normalize_output=True)
    training_loader, validation_loader = DataLoader.split_train_test(dataset, 0.80, batch_size=128)

    car_model = CarDNN()
    car_model.to(device)

    trainer = Trainer(car_model, training_loader, validation_loader, 1e-04, dataset.x.std(axis=0))
    trainer.train(10000)

    print("Done training!")


if __name__ == "__main__":
    main()
