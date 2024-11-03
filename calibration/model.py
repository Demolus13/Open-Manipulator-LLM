import os
import pandas as pd

import torch
import torch.nn as nn
import torch.optim as optim

# Check if GPU is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Define the model
class CalibrationModel(nn.Module):
    def _init_(self):
        super(CalibrationModel, self)._init_()
        self.fc1 = nn.Linear(2, 128)
        self.fc2 = nn.Linear(128, 256)
        self.fc3 = nn.Linear(256, 128)
        self.fc4 = nn.Linear(128, 64)
        self.fc5 = nn.Linear(64, 4)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        x = torch.relu(self.fc4(x))
        x = self.fc5(x)
        return x

# Initialize the model, loss function, and optimizer
model = CalibrationModel().to(device)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Load data from .csv file
data_path = "calibration/datasets/DATASET_0.csv"
data = pd.read_csv(data_path)

# Split data into inputs and targets
inputs = torch.tensor(data.iloc[:, :2].values, dtype=torch.float32).to(device)
targets = torch.tensor(data.iloc[:, 2:].values, dtype=torch.float32).to(device)

# Train the model
num_epochs = 100
batch_size = 10

for epoch in range(num_epochs):
    for i in range(0, len(inputs), batch_size):
        # Get mini-batch of data
        batch_inputs = inputs[i:i + batch_size]
        batch_targets = targets[i:i + batch_size]

        # Forward pass
        outputs = model(batch_inputs)
        loss = criterion(outputs, batch_targets)

        # Backward pass and optimization
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    # Print the loss
    if (epoch + 1) % 10 == 0:
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')
        print('\n---------------------------------------------------------------\n')

# Save the trained model
os.makedirs("calibration/models", exist_ok=True)
model_path = "calibration/models/MODEL_0.pth"
torch.save(model.state_dict(), model_path)
print(f"Model saved to {model_path}")