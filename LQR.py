import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

def simulate_LQR(sys, T_sim, dt):

    # TO DO define discrete matrices
    Ad = 
    Bd = 

    # TO DO: Define LQR variables
    Q = 
    R = 
    P = 
    K = 

    # TO DO -> uncomment block, once we look at LQRI and define LQRI variables
    """
    # LQRI
    A_aug = 
    B_aug = 

    Q_aug = 
    R_aug = 

    P_aug = 
    K_aug = 
    Kx, Ki = K_aug[0, :4], K_aug[0, 4:]  # u = -Kx x - Ki z
    Kx = Kx.reshape(1,-1) # might need reshaping
    Ki = Ki.reshape(1,-1) # might need reshaping
    """

    # TO DO: first only look at a pulse disturbance, then uncomment and look at step disturbance
    # Disturbance is an additive input force 'd(t)' entering the SAME channel as control:
    # xdot = A x + B(u + d)
    def disturbance(t):
        # Sudden pulse: 0.5 N from t=1.0 to 1.2 s
        d_pulse = 0.5 if (1.0 <= t <= 1.2) else 0.0
        # Constant bias step: +0.2 N from t=2.0 s onward
        d_step  = 0.2 if (t >= 2.0) else 0.0
        return d_pulse #+ d_step 

    # Simulation helpers
    def sim_LQR(x0, T=6.0, dt=0.001):

        #Linear system with LQR: xdot = (A - B K)*x + B*d
        t = np.arange(0, T+dt, dt)
        x = x0.copy().reshape(-1,1)
        X = np.nan*np.ones((4, len(t)))
        U = np.nan*np.ones((1, len(t)-1))
        D = np.nan*np.ones((1, len(t)-1))
        for k in range(1, len(t)):
            d = disturbance(t[k-1])
            # TO DO, define system update and control law
            x = 
            u = 
            X[:,k] = x.flatten(); U[:,k-1] = u.flatten(); D[:,k-1] = d
        return t, X, U, D

    def sim_LQRI(x0, T=6.0, dt=0.001):
        #Augmented linear system with integral on x: [x; z]_dot = [A 0; -C 0][x; z] + [B; 0](u + d),  u = -Kx x - Ki z
        t = np.arange(0, T+dt, dt)
        X = np.nan*np.ones((4, len(t)))
        U = np.nan*np.ones((1, len(t)-1))
        Z = np.nan*np.ones((2, len(t)))
        D = np.nan*np.ones((1, len(t)-1))
        x = x0.copy().reshape(-1,1)
        z = np.array([[0.0],[0.0]])
        X[:,0] = x.copy().flatten(); Z[:,0] = z.flatten()

        for k in range(1, len(t)):
            d = disturbance(t[k-1])
            # TO DO, define system update and control law
            u = 
            x = 
            zdot =   # cart position error (ref=0)
            z = # update for z (don't forget, it's discrete)
            X[:,k] = x.flatten(); Z[:,k] = z.flatten(); U[:,k-1] = u.flatten(); D[:,k-1] = d

        return t, X, Z, U, D

    # Simulate
    x0 = np.array([0.0, 0.0, np.deg2rad(3.0), 0.0])  # small initial tip
    t, X_lqr, U_lqr, D_lqr = sim_LQR(x0, T_sim, dt)
    t, X_lqri, Z_lqri, U_lqri, D_lqri = sim_LQRI(x0, T_sim, dt)

    # Plot
    theta_lqr, x_lqr = X_lqr[2,:], X_lqr[0,:]
    theta_lqri, x_lqri = X_lqri[2,:], X_lqri[0,:]

    plt.figure(figsize=(10,8))

    plt.subplot(4,1,1)
    plt.plot(t, np.rad2deg(theta_lqr), label="θ (LQR)")
    plt.plot(t, np.rad2deg(theta_lqri), '--', label="θ (LQRI)")
    plt.ylabel("θ [rad]"); plt.title("Linear cart-pole: LQR vs LQRI with pulse + constant disturbance")
    plt.grid(True); plt.legend()

    plt.subplot(4,1,2)
    plt.plot(t, x_lqr, label="x (LQR)")
    plt.plot(t, x_lqri, '--', label="x (LQRI)")
    plt.ylabel("x [m]")
    plt.grid(True); plt.legend()

    plt.subplot(4,1,3)
    plt.plot(t[:-1], U_lqr.flatten(), label="u (LQR)")
    plt.plot(t[:-1], U_lqri.flatten(), '--', label="u (LQRI)")
    plt.ylabel("u [N]")
    plt.grid(True); plt.legend()

    plt.subplot(4,1,4)
    plt.plot(t[:-1], D_lqr.flatten(), 'k', label="disturbance d(t)")
    plt.ylabel("d [N]"); plt.xlabel("Time [s]")
    plt.grid(True); plt.legend()

    plt.tight_layout(); plt.show()

    # Print steady-state cart offsets
    def ss_mean(signal, t, t_ss=5.5):
        mask = t >= t_ss
        return np.mean(signal[mask])
    print("Steady-state x (LQR):  ", ss_mean(x_lqr, t))
    print("Steady-state x (LQRI): ", ss_mean(x_lqri, t))