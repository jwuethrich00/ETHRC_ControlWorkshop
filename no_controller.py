import numpy as np
import matplotlib.pyplot as plt

def simulate_no_controller(sys, T_sim, dt, model_type):

    x = sys.state
    pos_hist = []
    theta_hist = []
    U = []
    N = int(T_sim/dt)
    t = np.arange(0, T_sim + dt, dt)
    for _ in t:
        if len(x) == 2:
            theta, _ = x
        else:
            pos, _, theta, _ = x
        u = 0
        U.append(u)
        if model_type == "Linear":
            x_next = sys.step_linear(x, u, dt)
        elif model_type == "Nonlinear":   
            x_next = sys.step_nonlinear(x, u, dt)
        else:
            print("invalid model type")
        theta_hist.append(theta)
        if len(x) == 4:
            pos_hist.append(pos)
        x = x_next

    if model_type == "Nonlinear":
        theta_hist = np.asarray(theta_hist, dtype=np.float32)
        pos_hist = np.asarray(pos_hist, dtype=np.float32)
    u_hist = np.asarray(U, dtype=np.float32)

    if len(x) == 2:
        title = "Rotating Inverted Pendulum: θ(t)"
    else:
        title = "Cart Pole System: θ(t)"
        title2 = "Cart Pole System: x(t)"

    plt.figure()
    if len(x) == 4:
        plt.subplot(2,1,1)
    plt.plot(t, np.rad2deg(theta_hist))
    plt.title(title)
    if len(x) == 2:
        plt.xlabel("Time [s]")
    plt.ylabel("Angle [°]")
    plt.grid(True)

    if len(x) == 4:
        plt.subplot(2,1,2)
        plt.plot(t, pos_hist)
        plt.title(title2)
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")
        plt.grid(True)

    plt.figure()
    plt.plot(t, u_hist)
    plt.title("Control Torque u(t)")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [N·m]")
    plt.grid(True)
    plt.show()

    