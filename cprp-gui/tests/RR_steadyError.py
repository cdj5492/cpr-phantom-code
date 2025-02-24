import torch
import numpy as np
import matplotlib.pyplot as plt

def solve(segments_c, thetas, errors, target, lambda_tangent=10.0):
    """
    Compute the loss given:
      - segments_c: 1D tensor of segment lengths (chord lengths)
      - thetas: 1D tensor of base angles for each segment
      - errors: 1D tensor of error corrections applied to the angles
      - target: 1D tensor (length 2) for the desired final point
      - lambda_tangent: weight for enforcing the final tangent is horizontal
      
    The loss is a combination of:
      (a) the Euclidean distance between the final point and the target, and
      (b) a penalty that enforces the final segment's tangent to be horizontal.
    """
    # Compute modified angles
    thetas_mod = torch.abs(thetas + errors)
    r = segments_c / thetas_mod
    segments_s = r * torch.sqrt(2 * (1 - torch.cos(thetas_mod)))
    
    # Compute 'phis'. By construction, the starting tangent is fixed via:
    #   phis[0] = theta_mod[0] / 2  (this enforces the (0,0) point to have horizontal tangent)
    N = thetas_mod.shape[0]
    phis = torch.empty_like(thetas_mod)
    phis[0] = thetas_mod[0] / 2
    for i in range(1, N):
        phis[i] = (thetas_mod[i-1] + thetas_mod[i]) / 2
    
    # Forward kinematics to compute the final point.
    # 't' accumulates the tangent direction.
    point = torch.zeros(2, device=segments_c.device)
    t = phis[0]
    for i in range(N):
        direction = torch.stack([torch.cos(t), torch.sin(t)])
        point = point + segments_s[i] * direction
        t = t + phis[i]  # update the tangent direction

    # Loss term (a): distance between final point and target
    point_loss = torch.norm(point - target)
    # Loss term (b): final tangent penalty; the tangent is horizontal if sin(t)==0.
    tangent_penalty = lambda_tangent * (torch.sin(t-phis[-1]))**2

    loss = point_loss + tangent_penalty
    return loss

def compute_points(segments_c, thetas, errors):
    """
    Compute the list of 2D points for the entire kinematic chain.
    Returns a NumPy array of shape (N+1, 2).
    """
    thetas_mod = thetas + errors
    r = segments_c / thetas_mod
    segments_s = r * torch.sqrt(2 * (1 - torch.cos(thetas_mod)))
    
    N = thetas_mod.shape[0]
    phis = torch.empty_like(thetas_mod)
    phis[0] = thetas_mod[0] / 2
    for i in range(1, N):
        phis[i] = (thetas_mod[i-1] + thetas_mod[i]) / 2

    points = []
    point = torch.zeros(2, device=segments_c.device)
    points.append(point.clone())
    t = phis[0]
    for i in range(N):
        direction = torch.stack([torch.cos(t), torch.sin(t)])
        point = point + segments_s[i] * direction
        points.append(point.clone())
        t = t + phis[i]
    
    points_np = torch.stack(points).detach().cpu().numpy()
    return points_np

def main():
    # Initial parameters: 6 segments
    # total 28 inches -> 711.2 mm
    s_length = 711.2/6
    s_list = [s_length] * 6
    theta_list = [0.5, 0.4, 0.8, 0.9, 0.3, 0.2]
    error_list = [0.0] * 6
    target = np.array([0.0, 293.1], dtype=np.float32)
    
    # Convert inputs to torch tensors.
    segments_c = torch.tensor(s_list, dtype=torch.float32)
    thetas = torch.tensor(theta_list, dtype=torch.float32)
    errors = torch.tensor(error_list, dtype=torch.float32)
    target_tensor = torch.tensor(target, dtype=torch.float32)
    
    # Compute the initial curve (with zero errors).
    with torch.no_grad():
        points_initial = compute_points(segments_c, thetas, errors)
    
    # --- Optimize Errors with L2 Regularization & Final Tangent Constraint ---
    # We'll optimize the error corrections directly.
    errors_opt = torch.tensor(error_list, dtype=torch.float32, requires_grad=True)
    optimizer = torch.optim.Adam([errors_opt], lr=0.01)
    
    lambda_reg = 0.01      # L2 regularization weight on the errors.
    lambda_tangent = 10.0  # Weight for final tangent (horizontal) constraint.
    
    n_epochs = 1000
    for epoch in range(n_epochs):
        optimizer.zero_grad()
        # rand_target = target_tensor + torch.randn(2) * 25
        loss_main = solve(segments_c, thetas, errors_opt, target_tensor, lambda_tangent=lambda_tangent)
        penalty = lambda_reg * torch.norm(errors_opt)**2
        loss = loss_main + penalty
        loss.backward()
        optimizer.step()
        
        if epoch % 100 == 0:
            print(f"Epoch {epoch:04d}: Total Loss = {loss.item():.4f}, "
                  f"Point Loss+Tangent Penalty = {loss_main.item():.4f}")
    
    # Compute the optimized curve.
    with torch.no_grad():
        points_optimized = compute_points(segments_c, thetas, errors_opt)
    
    # print the optimized errors
    print("Optimized Errors:")
    for i, error in enumerate(errors_opt):
        print(f"Segment {i+1}: {error.item():.4f}")
    
    # --- Plot both curves on the same graph ---
    plt.figure(figsize=(8, 8))
    plt.plot(points_initial[:, 0], points_initial[:, 1], 'o-', label='Before Optimization')
    plt.plot(points_optimized[:, 0], points_optimized[:, 1], 's-', label='After Optimization')
    plt.axis('equal')
    plt.title('Curve Before and After Error Optimization\n(with Final Tangent Constraint)')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
