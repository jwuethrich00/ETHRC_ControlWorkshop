
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
L = 2.0        # distance sensor x-position
s = 0.1        # process noise bound
e = 0.05       # measurement noise bound
T = 100        # number of time steps

# Helper function
def wrap_angle(theta):
    return theta % (2 * np.pi)

# Dynamics and measurements
def f(x):
    b, theta = x
    v = np.random.uniform(-s, s)
    theta_new = wrap_angle(theta + b + v)
    return np.array([b, theta_new])

def h(x):
    _, theta = x
    x_obj = np.cos(theta)
    y_obj = np.sin(theta)
    dist = np.sqrt((L - x_obj)**2 + y_obj**2)
    return np.array([dist])

def F_jacobian(x):
    b, theta = x
    return np.array([
        [1.0, 0.0],
        [1.0, 1.0]
    ])  # approximate, inaccurate due to nonlinearity and angle wrap

def H_jacobian(x):
    _, theta = x
    x_obj = np.cos(theta)
    y_obj = np.sin(theta)
    dx = L - x_obj
    dy = -y_obj
    dist = np.sqrt(dx**2 + dy**2)
    dtheta = (dx * y_obj - dy * x_obj) / dist
    return np.array([[0.0, dtheta]])

# Simulation
x_true = np.zeros((2, T))
x_true[:, 0] = [np.random.uniform(-s, s), np.random.uniform(0, 2*np.pi)]
z = np.zeros(T)
for k in range(1, T):
    x_true[:, k] = f(x_true[:, k-1])
    z[k] = h(x_true[:, k]) + np.random.uniform(-e, e)

# EKF
x_ekf = np.zeros((2, T))
x_ekf[:, 0] = [0.0, 0.0]
P = 0.1 * np.eye(2)
Q = 0.01 * np.eye(2)
R = np.array([[e**2]])

for k in range(1, T):
    # Predict
    x_pred = f(x_ekf[:, k-1])
    F = F_jacobian(x_ekf[:, k-1])
    P = F @ P @ F.T + Q

    # Update
    H = H_jacobian(x_pred)
    y_pred = h(x_pred)
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    prediction_error = z[k] - y_pred
    x_ekf[:, k] = x_pred + (K @ prediction_error).flatten()
    x_ekf[1, k] = wrap_angle(x_ekf[1, k])
    P = (np.eye(2) - K @ H) @ P

# Animation
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-1.5, L + 0.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("EKF Evolution on Circular Motion")

# Constants
sensor_dot, = ax.plot(L, 0, 'gx', markersize=10, label='Sensor')
true_line, = ax.plot([], [], 'k-', lw=1.5, label='True trajectory')
ekf_line, = ax.plot([], [], 'r--', lw=1.5, label='EKF trajectory')
true_dot, = ax.plot([], [], 'ko', label='True position')
ekf_dot, = ax.plot([], [], 'ro', label='EKF estimate')
ax.legend(loc='upper right')

# Precompute positions
true_x = np.cos(x_true[1])
true_y = np.sin(x_true[1])
ekf_x = np.cos(x_ekf[1])
ekf_y = np.sin(x_ekf[1])

def init():
    true_line.set_data([], [])
    ekf_line.set_data([], [])
    true_dot.set_data([], [])
    ekf_dot.set_data([], [])
    return true_line, ekf_line, true_dot, ekf_dot

def update(frame):
    true_line.set_data(true_x[:frame], true_y[:frame])
    ekf_line.set_data(ekf_x[:frame], ekf_y[:frame])
    true_dot.set_data(true_x[frame], true_y[frame])
    ekf_dot.set_data(ekf_x[frame], ekf_y[frame])
    return true_line, ekf_line, true_dot, ekf_dot

ani = FuncAnimation(fig, update, frames=range(1, T), init_func=init, blit=True, interval=150)
ani.save('ekf_animation.gif', writer='pillow', fps=10)

plt.show()

