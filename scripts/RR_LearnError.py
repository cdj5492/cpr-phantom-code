import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- Forward Kinematics Loss for a Single Sample ---
def solve_single(segments_c, thetas, errors, target, lambda_tangent=10.0):
    """
    Given a single sample (1D tensors for thetas, errors, and target as a 2D tensor),
    compute the loss as the distance from the final point to the target plus a penalty
    for the final tangent not being horizontal.
    """
    # Modified angles (using absolute value as in your original code)
    thetas_mod = torch.abs(thetas + errors)
    r = segments_c / thetas_mod
    segments_s = r * torch.sqrt(2 * (1 - torch.cos(thetas_mod)))
    
    N = thetas_mod.shape[0]
    phis = torch.empty_like(thetas_mod)
    phis[0] = thetas_mod[0] / 2
    for i in range(1, N):
        phis[i] = (thetas_mod[i - 1] + thetas_mod[i]) / 2

    # Forward kinematics to compute the final point.
    point = torch.zeros(2, device=segments_c.device)
    t = phis[0]
    for i in range(N):
        direction = torch.stack([torch.cos(t), torch.sin(t)])
        point = point + segments_s[i] * direction
        t = t + phis[i]  # update tangent

    # Loss term (a): Euclidean distance to target.
    point_loss = torch.norm(point - target)
    # Loss term (b): final tangent penalty (enforcing horizontal final tangent)
    tangent_penalty = lambda_tangent * (torch.sin(t - phis[-1]))**2

    loss = point_loss + tangent_penalty
    return loss

# --- Compute Curve Points (for visualization) ---
def compute_points(segments_c, thetas, errors):
    """
    Compute the list of 2D points along the kinematic chain.
    Returns a NumPy array of shape (N+1, 2).
    """
    thetas_mod = thetas + errors
    r = segments_c / thetas_mod
    segments_s = r * torch.sqrt(2 * (1 - torch.cos(thetas_mod)))
    
    N = thetas_mod.shape[0]
    phis = torch.empty_like(thetas_mod)
    phis[0] = thetas_mod[0] / 2
    for i in range(1, N):
        phis[i] = (thetas_mod[i - 1] + thetas_mod[i]) / 2

    points = [torch.zeros(2)]
    t = phis[0]
    for i in range(N):
        direction = torch.stack([torch.cos(t), torch.sin(t)])
        point = points[-1] + segments_s[i] * direction
        points.append(point)
        t = t + phis[i]
    
    return torch.stack(points).detach().cpu().numpy()

# --- Dataset Definition ---
class KinematicsDataset(torch.utils.data.Dataset):
    """
    Expects a CSV file with columns:
      theta1, theta2, theta3, theta4, theta5, theta6, target_x, target_y
    """
    def __init__(self, csv_file):
        self.data = pd.read_csv(csv_file)
        # Extract the 6 base angles and the 2 target coordinates
        self.thetas = self.data[['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']].values.astype(np.float32)
        self.targets = self.data[['target_x', 'target_y']].values.astype(np.float32)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        theta = self.thetas[idx]      # shape (6,)
        target = self.targets[idx]    # shape (2,)
        return theta, target

# --- Error Predictor Model ---
class ErrorPredictor(nn.Module):
    """
    A simple fully-connected network that takes as input a concatenation of
    the base angles (6 values) and target (2 values) and outputs error corrections (6 values).
    """
    def __init__(self, input_dim=8, hidden_dim=32, output_dim=6):
        super(ErrorPredictor, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, output_dim)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# --- Training Loop ---
def train_model(model, dataset, segments_c, n_epochs=1000, batch_size=16, learning_rate=0.001, lambda_reg=0.01, lambda_tangent=10.0):
    """
    Train the error predictor model over a dataset.
    For each sample, the loss is computed via the forward kinematics loss (solve_single)
    plus an L2 penalty on the predicted errors.
    """
    dataloader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    
    model.train()
    for epoch in range(n_epochs):
        epoch_loss = 0.0
        for thetas_batch, targets_batch in dataloader:
            # Convert to torch tensors
            thetas_batch = thetas_batch  # shape: (B, 6)
            targets_batch = targets_batch  # shape: (B, 2)
            # Concatenate thetas and targets to form the network input.
            inputs = torch.cat([thetas_batch, targets_batch], dim=1)  # shape: (B, 8)
            predicted_errors = model(inputs)  # shape: (B, 6)

            # Compute loss sample by sample (since our solve_single function works on single examples)
            batch_loss = 0.0
            batch_size_actual = predicted_errors.shape[0]
            for i in range(batch_size_actual):
                theta_sample = thetas_batch[i]
                target_sample = targets_batch[i]
                error_sample = predicted_errors[i]
                loss = solve_single(segments_c, theta_sample, error_sample, target_sample, lambda_tangent)
                # Add L2 regularization on the predicted error corrections
                loss = loss + lambda_reg * torch.norm(error_sample)**2
                batch_loss += loss
            batch_loss = batch_loss / batch_size_actual

            optimizer.zero_grad()
            batch_loss.backward()
            optimizer.step()

            epoch_loss += batch_loss.item()
        epoch_loss = epoch_loss / len(dataloader)
        if epoch % 100 == 0:
            print(f"Epoch {epoch}: Loss = {epoch_loss:.4f}")

# --- Main Function ---
def main():
    # The segments (chord lengths) remain constant.
    s_length = 711.2 / 6
    segments_c = torch.tensor([s_length] * 6, dtype=torch.float32)

    dataset = KinematicsDataset("data/thetas_targets.csv")
    
    # Initialize the error predictor model.
    model = ErrorPredictor()
    
    # Train the model.
    train_model(model, dataset, segments_c, n_epochs=1000, batch_size=16, learning_rate=0.001, lambda_reg=0.01, lambda_tangent=10.0)
    
    # --- Evaluation on a Sample ---
    model.eval()
    # Pick a sample from the dataset (here, the first sample)
    sample_theta, sample_target = dataset[0]
    sample_theta_tensor = torch.tensor(sample_theta, dtype=torch.float32)
    sample_target_tensor = torch.tensor(sample_target, dtype=torch.float32)
    with torch.no_grad():
        input_tensor = torch.cat([sample_theta_tensor, sample_target_tensor]).unsqueeze(0)  # shape (1,8)
        predicted_error = model(input_tensor).squeeze(0)  # shape (6,)
        print("\nEvaluation on Sample:")
        print("Base Thetas:", sample_theta)
        print("Target:", sample_target)
        print("Predicted Error Corrections:", predicted_error.numpy())
    
        # Compute the resulting curve with predicted corrections.
        points = compute_points(segments_c, sample_theta_tensor, predicted_error)
    
    # Plot the optimized curve.
    plt.figure(figsize=(6, 6))
    plt.plot(points[:, 0], points[:, 1], marker='o', label='Optimized Curve')
    plt.title("Optimized Kinematic Curve for Sample")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
