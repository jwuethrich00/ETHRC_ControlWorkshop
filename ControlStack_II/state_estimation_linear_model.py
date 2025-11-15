
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from numpy.random import multivariate_normal

np.random.seed(42)

# System matrices (already discrete)
A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
C = np.array([[1.0, 0.0]])

# TO DO Luenberger-Matrices
Q_L = np.array()
R_L_slow = 
R_L_fast = 

# Observer gain via discrete-time algebraic Riccati equation
P_slow = 
L_slow_T = 

P_fast = 
L_fast_T = 

eig_vals_slow = np.linalg.eigvals(A-L_slow_T.T@C)
eig_vals_fast = np.linalg.eigvals(A-L_fast_T.T@C)
eig_vals_abs_slow = np.abs(eig_vals_slow)
eig_vals_abs_fast = np.abs(eig_vals_fast)
print(f"Eigenvalues of Slow Observer: {eig_vals_abs_slow}")
print(f"Eigenvalues of Fast Observer: {eig_vals_abs_fast}")

# Simulation parameters
T = 20
x_true = np.zeros((2, T))
x_true[:, 0] = [0.0, 1.0]

u = np.ones(T) * 0.2
y = np.zeros(T)
x_obs_slow = np.zeros((2, T))
x_obs_fast = np.zeros((2, T))

# TO DO Initial estimates
x_obs_slow[:, 0] = 
x_obs_fast[:, 0] = 

# Process and measurement noise
Q = np.array([[0.025, 0.05],
              [0.05, 0.1]])
R = 0.5 * np.eye(1)

for t in range(1, T):
    # True dynamics
    w = multivariate_normal([0, 0], Q)
    x_true[:, t] = A @ x_true[:, t-1] + B.flatten() * u[t-1] + w

    v = multivariate_normal([0], R)
    y[t] = (C @ x_true[:, t] + v)[0]

    # TO DO Luenberger (slow)
    y_hat_slow = 
    x_obs_slow[:, t] = 

    # TO DO Luenberger (fast)
    y_hat_fast = 
    x_obs_fast[:, t] =

# Plot observers
time = np.arange(T)
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(time, x_true[0], 'k-', label='True Position')
plt.plot(time, x_obs_slow[0], 'b--', label='Luenberger Slow')
plt.plot(time, x_obs_fast[0], 'r-.', label='Luenberger Fast')
plt.ylabel('Position')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, x_true[1], 'k-', label='True Velocity')
plt.plot(time, x_obs_slow[1], 'b--', label='Luenberger Slow')
plt.plot(time, x_obs_fast[1], 'r-.', label='Luenberger Fast')
plt.ylabel('Velocity')
plt.xlabel('Time')
plt.legend()
plt.suptitle("Slow vs Fast Luenberger Observer")
plt.grid()

"""
x_kf = np.zeros((2, T))
P_kf_traces = np.zeros(T)

# TO DO -> initial estimates
x_kf[:, 0] = 
P_kf = 
P_kf_traces[0] = np.trace(P_kf)

for t in range(1, T):

    # TO DO: prediction update KF predict
    x_pred = 
    P_p = 

    # TO DO: KF / Measurement update
    x_kf[:, t] = 
    P_kf =
    P_kf_traces[t] = np.trace(P_kf)

# Kalman Filter vs Ground Truth
plt.figure(figsize=(10, 6))
plt.plot(time, x_true[0], 'k-', label='True Position')
plt.plot(time, x_kf[0], 'g--', label='KF Estimate')
plt.fill_between(time, x_kf[0] - np.sqrt(P_kf_traces), x_kf[0] + np.sqrt(P_kf_traces), color='green', alpha=0.3, label='KF ±1σ')
plt.title("Kalman Filter Estimate with Uncertainty")
plt.xlabel("Time")
plt.ylabel("Position")
plt.legend()
plt.grid()

# Final comparison
plt.figure(figsize=(10, 6))
plt.plot(time, x_true[0], 'k-', label='True Position')
plt.plot(time, x_obs_slow[0], 'b--', label='Luenberger')
plt.plot(time, x_kf[0], 'g-', label='Kalman Filter')
plt.title("KF vs Luenberger Observer")
plt.xlabel("Time")
plt.ylabel("Position")
plt.legend()
plt.grid()

plt.show()
"""