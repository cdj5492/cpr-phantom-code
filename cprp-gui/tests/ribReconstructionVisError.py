import numpy as np
import matplotlib.pyplot as plt

def solve(segments_c, thetas, errors, target):
    # calculate secants of curved segments
    segments_s = []
    radii = []
    for i in range(len(segments_c)):
        thetas[i] = thetas[i] + errors[i]
        r = segments_c[i] / thetas[i]
        radii.append(r)
        segments_s.append(r*np.sqrt(2*(1-np.cos(thetas[i]))))
    
    # phi_i = (theta_{i-1} + theta_i) / 2
    phis = [thetas[0]/2]
    for i in range(1, len(segments_c)):
        phis.append((thetas[i-1] + thetas[i]) / 2)
    
    
    # psi_i = (theta_{i+1} + theta_i) / 2
    psis = [thetas[-1]/2]
    for i in range(len(segments_c) - 1):
        psis.append((thetas[i+1] + thetas[i]) / 2)

    points = [np.array([0, 0])]
    t = phis[0]

    # forward kinematics
    for i in range(len(segments_s)):
        point = points[-1] + segments_s[i] * np.array([np.cos(t), np.sin(t)])
        t += phis[i]
        points.append(point)
    
    loss = np.linalg.norm(points[-1] - target)
    
    return points, loss

def main():
    s_list = [55.0, 55.0, 55.0, 55.0, 55.0, 55.0]
    theta_list = [0.5, 0.3, 0.4, 0.5, 0.6, 0.7]
    error_list = [0.0, 0.7, 0.0, 1.0, 0.0, 0.0]
    target = np.array([0, 100])

    points, loss = solve(s_list, theta_list, error_list, target)
    
    print(f"Loss: {loss}")
    print(points)

    # Convert list of b-values to a NumPy array for easy plotting
    b_values = np.array(points)

    # add (0, 0) to the beginning of the array to make it easier to plot
    b_values = np.insert(b_values, 0, [0, 0], axis=0)


    # Step 3: Plot the b_i points
    plt.figure(figsize=(6,6))
    plt.plot(b_values[:, 0], b_values[:, 1], 'o-', label='b_i sequence')
    plt.axis('equal')
    plt.title('Plot of b_i values')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
