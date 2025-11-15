import numpy as np
import matplotlib.pyplot as plt
from numpy.random import multivariate_normal

# Strongly nonlinear true system
def f(x, u):
    return np.array([
        x[0] + np.sin(x[1]) + 0.05 * x[0]**2 + 0.1 * u,
        x[1] + 0.05 * np.cos(x[0]) + 0.1 * u
    ])

def h(x):
    return np.array([x[0]])  # Linear measurement

# Linear model assumed by KF
A_lin = np.array([[1.0, 1.0],
                  [0.0, 1.0]])
B_lin = np.array([[0.0],
                  [0.1]])
C_lin = np.array([[1.0, 0.0]])

Q = 0.01 * np.eye(2)
R = 0.1 * np.eye(1)

# Simulation parameters
T = 10
x_true = np.zeros((2, T))
x_true[:, 0] = [0.0, 2.0]
u = np.ones(T) * 0.2
y = np.zeros(T)

# KF initialization (assumes linear system)
x_kf = np.zeros((2, T))
x_kf[:, 0] = [0.0, 2.0]
P_kf = 0.1 * np.eye(2)
P_kf_traces = np.zeros(T)

for t in range(1, T):
    # True nonlinear dynamics
    w = multivariate_normal([0, 0], Q)
    x_true[:, t] = f(x_true[:, t-1], u[t-1]) + w
    v = multivariate_normal([0], R)
    y[t] = (h(x_true[:, t]) + v)[0]

    # KF predict (uses linear model)
    x_pred = A_lin @ x_kf[:, t-1] + B_lin.flatten() * u[t-1]
    P_kf = A_lin @ P_kf @ A_lin.T + Q

    # KF update (linear measurement)
    S = C_lin @ P_kf @ C_lin.T + R
    K = P_kf @ C_lin.T @ np.linalg.inv(S)
    x_kf[:, t] = x_pred + K.flatten() * (y[t] - C_lin @ x_pred)
    P_kf = (np.eye(2) - K @ C_lin) @ P_kf @ (np.eye(2) - K @ C_lin).T + K @ R @ K.T
    P_kf_traces[t] = np.trace(P_kf)

# Plot results
time = np.arange(T)
plt.figure(figsize=(10, 6))
plt.plot(time, x_true[0], 'k-', label='True Position (nonlinear)')
plt.plot(time, x_kf[0], 'r--', label='KF Estimate')
plt.fill_between(time, x_kf[0] - np.sqrt(P_kf_traces), x_kf[0] + np.sqrt(P_kf_traces), color='red', alpha=0.3, label='KF ±1σ')
plt.title("KF Failure on Strongly Nonlinear State Dynamics")
plt.xlabel("Time")
plt.ylabel("Position")
plt.legend()
plt.grid()
plt.show()
