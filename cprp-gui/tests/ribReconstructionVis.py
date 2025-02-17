import numpy as np
import matplotlib.pyplot as plt

def main():
    # Example data (replace these with your actual s and theta values)
    s_list = [55.0, 55.0, 55.0, 55.0, 55.0, 55.0]
    theta_list = [0.5, 0.3, 0.4, 0.5, 0.6, 0.7]


    # Step 1: Compute all a_i vectors
    # a_i = [ r_i(1 - cos(theta_i)),  r_i sin(theta_i) ]
    # with r_i = s_i / theta_i
    a_vectors = []
    for s_i, th_i in zip(s_list, theta_list):
        r_i = s_i / th_i
        a_i = np.array([
            r_i * np.sin(th_i),
            r_i * (1.0 - np.cos(th_i))
        ])
        a_vectors.append(a_i)

    # Initial conditions
    b0 = a_vectors[0]
    phi0 = theta_list[0]

    # Step 2: Iteratively compute b_i and phi_i
    b_values = [b0]    # keep track of all b_i in a list
    phi_values = [phi0]

    for i in range(1, len(a_vectors)):
        b_prev = b_values[-1]
        phi_prev = phi_values[-1]
        a_i = a_vectors[i]
        th_i = theta_list[i]

        # Rotate a_i by phi_{i-1}
        # R(phi) = [[cos(phi), -sin(phi)],
        #           [sin(phi),  cos(phi)]]
        cos_p = np.cos(phi_prev)
        sin_p = np.sin(phi_prev)

        # Rotated version of a_i
        rotated_a_i = np.array([
            a_i[0] * cos_p - a_i[1] * sin_p,
            a_i[0] * sin_p + a_i[1] * cos_p
        ])

        # b_i = b_{i-1} + R_{phi_{i-1}}(a_i)
        b_i = b_prev + rotated_a_i

        # phi_i = phi_{i-1} + theta_i
        phi_i = phi_prev + th_i

        # Store them
        b_values.append(b_i)
        phi_values.append(phi_i)


    # print out b values
    for i in range(len(b_values)):
        print(f"b_{i+1}: {b_values[i]}")


    # Convert list of b-values to a NumPy array for easy plotting
    b_values = np.array(b_values)

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
