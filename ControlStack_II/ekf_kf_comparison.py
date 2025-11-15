import numpy as np
import matplotlib.pyplot as plt
from numpy.random import multivariate_normal, seed

# Nonlinear system dynamics and measurement
def f(x, u):
    return np.array([
        x[0] + np.sin(x[1]) + 0.05 * x[0]**2 + 0.1 * u,
        x[1] + 0.05 * np.cos(x[0]) + 0.1 * u
    ])

def h(x):
    return np.array([x[0]])

# TO DO: Jacobians / Linearization Matrices as a function of the linearization point
def A_jacobian(x, u):
    return np.array(
    )

def H_jacobian(x):
    return np.array()

# System parameters
T = 10
x0 = np.array([0.0, 2.0])
u = np.ones(T) * 0.2

Q_low = 0.01 * np.eye(2)
R_low = 0.1 * np.eye(1)
Q_high = 0.5 * np.eye(2)
R_high = 0.75 * np.eye(1)

seed(99)

# Simulate and estimate with EKF and KF for low/high noise
def simulate_and_estimate(Q, R, label):

    x_true = np.zeros((2, T))
    x_true[:, 0] = x0

    x_kf = np.zeros((2, T))
    x_ekf = np.zeros((2, T))
    x_kf[:, 0] = [0.0, 0.0]
    P_kf = 0.1 * np.eye(2)

    # TO DO - initialize KF
    x_ekf[:, 0] = 
    P_ekf = 

    trace_ekf = np.zeros(T)
    trace_kf = np.zeros(T)

    y = np.zeros(T)

    # KF matrices
    A_lin = np.array([[1.0, 1.0],
                      [0.0, 1.0]])
    B_lin = np.array([[0.0],
                      [0.1]])
    C_lin = np.array([[1.0, 0.0]])

    for t in range(1, T):

        # True System
        w = multivariate_normal([0, 0], Q)
        v = multivariate_normal([0], R)
        x_true[:, t] = f(x_true[:, t-1], u[t-1]) + w
        y[t] = h(x_true[:, t])[0] + v

        # KF (assumes linear model)
        x_pred_kf = A_lin @ x_kf[:, t-1] + B_lin.flatten() * u[t-1]
        P_kf = A_lin @ P_kf @ A_lin.T + Q
        S_kf = C_lin @ P_kf @ C_lin.T + R
        K_kf = P_kf @ C_lin.T @ np.linalg.inv(S_kf)
        x_kf[:, t] = x_pred_kf + K_kf.flatten() * (y[t] - C_lin @ x_pred_kf)
        P_kf = (np.eye(2) - K_kf @ C_lin) @ P_kf
        trace_kf[t] = np.trace(P_kf)

        # TO DO EKF update equations
        x_pred_ekf =
        A_lin = 
        P_p_ekf = 
        H_lin = 
        y_pred = 
        K_ekf = 
        x_ekf[:, t] = 
        P_ekf 
        trace_ekf[t] = np.trace(P_ekf)

    return x_true, x_kf, x_ekf, trace_kf, trace_ekf, label

results = []
results.append(simulate_and_estimate(Q_low, R_low, "Low Noise"))
results.append(simulate_and_estimate(Q_high, R_high, "High Noise"))

# Plotting comparison
for x_true, x_kf, x_ekf, trace_kf, trace_ekf, label in results:
    print(f"Error KF: {np.mean(np.abs(x_true-x_kf))}")
    print(f"Error EKF: {np.mean(np.abs(x_true-x_ekf))}")

for x_true, x_kf, x_ekf, trace_kf, trace_ekf, label in results:
    time = np.arange(T)
    plt.figure(figsize=(10, 6))
    plt.plot(time, x_true[0], 'k-', label='True Position')
    plt.plot(time, x_kf[0], 'r--', label='KF')
    plt.plot(time, x_ekf[0], 'b-', label='EKF')
    plt.title(f"KF vs EKF - {label}")
    plt.xlabel("Time")
    plt.ylabel("Position")
    plt.legend()
    plt.grid()
    plt.show()
