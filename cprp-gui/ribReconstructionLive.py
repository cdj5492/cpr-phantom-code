import socket
import json
import numpy as np
import pygame
import threading
import collections

# --- Configuration ---
UDP_IP = "127.0.0.1"
UDP_PORT = 9870
WINDOW_SIZE = 500  # Moving average filter size
SCREEN_SIZE = 600
s_list = [20.0, 15.0, 10.0]  # Fixed s values

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
pygame.display.set_caption("Real-Time Visualization")
clock = pygame.time.Clock()

# Data buffers
theta_buffer = collections.deque(maxlen=WINDOW_SIZE)  # Moving average buffer
theta_value = 0.5  # Default initial theta value


def moving_average(buffer):
    """Compute the moving average from the buffer."""
    return sum(buffer) / len(buffer) if buffer else 0


def udp_listener():
    """Receives UDP packets, extracts d[1], applies mapping, and stores in buffer."""
    global theta_value
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, _ = sock.recvfrom(1024)  # Receive UDP packet
        try:
            parsed = json.loads(data.decode('utf-8'))
            raw_value = parsed["d"][1]  # Extract only d[1]
            mapped_value = raw_value * (-0.00434451) + 4.85535  # Apply mapping function
            theta_buffer.append(mapped_value)  # Store in buffer
            theta_value = moving_average(theta_buffer)  # Smooth with moving average
        except (json.JSONDecodeError, IndexError, KeyError, ValueError):
            continue  # Ignore malformed packets


def compute_b_values(theta):
    """Computes b-values for visualization using given theta."""
    a_vectors = []
    for i, s_i in enumerate(s_list):
        r_i = s_i / theta
        a_i = np.array([
            r_i * np.sin(theta),
            r_i * (1.0 - np.cos(theta))
        ])
        a_vectors.append(a_i)

    # Initial values
    b_values = [a_vectors[0]]
    phi_values = [theta]

    for i in range(1, len(a_vectors)):
        b_prev = b_values[-1]
        phi_prev = phi_values[-1]
        a_i = a_vectors[i]

        # Rotate a_i by phi_{i-1}
        cos_p = np.cos(phi_prev)
        sin_p = np.sin(phi_prev)
        rotated_a_i = np.array([
            a_i[0] * cos_p - a_i[1] * sin_p,
            a_i[0] * sin_p + a_i[1] * cos_p
        ])

        # Compute new b_i and phi_i
        b_i = b_prev + rotated_a_i
        phi_i = phi_prev + theta

        # Store values
        b_values.append(b_i)
        phi_values.append(phi_i)

    return np.insert(np.array(b_values), 0, [0, 0], axis=0)  # Include origin


def draw():
    """Main pygame loop to visualize real-time updates."""
    global theta_value

    running = True
    while running:
        screen.fill((0, 0, 0))  # Clear screen
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Compute b-values for visualization
        b_values = compute_b_values(theta_value)

        # Transform to screen coordinates
        center_x, center_y = SCREEN_SIZE // 2, SCREEN_SIZE // 2
        scale = 5  # Scale factor for better visibility
        points = [(int(center_x + x * scale), int(center_y - y * scale)) for x, y in b_values]

        # Draw points and connecting lines
        if len(points) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, points, 2)
            for point in points:
                pygame.draw.circle(screen, (255, 0, 0), point, 5)

        pygame.display.flip()
        clock.tick(60)  # Limit FPS to 60

    pygame.quit()


# Start UDP listener in a separate thread
udp_thread = threading.Thread(target=udp_listener, daemon=True)
udp_thread.start()

# Start pygame visualization loop
draw()
