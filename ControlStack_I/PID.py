import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

def simulate_pid(sys, T_sim, dt, model_type, sys_type):

    # ----- Actuator limits -----
    u_max = 0.5   # torque saturation [N·m]
    u_rate_max = 3  # torque rate limit [N·m/s]
    # ----- Filters & anti-windup -----
    tau_aw = 4.0          # anti-windup time constant
    k_aw = 1.0 / tau_aw

    if model_type == "Nonlinear":

        # ----- PID gains -----
        Kp = 25
        Kd = 15
        Ki = 0.1

        t = np.arange(0, T_sim + dt, dt)
        x = sys.state
        ei = 0.0
        u_prev = 0.0

        theta_hist, u_hist = [], []

        for _ in t:
            theta, theta_dot = x

            # PID control on θ (reference = 0)
            e = np.deg2rad(2)-theta
            ed = -theta_dot
            v = Kp*e + Kd*ed + Ki*ei
            u_unsat = sys.I*v + sys.b*theta_dot - sys.mass*sys.gravity*sys.length*np.sin(theta)

            # Apply saturation & rate limits
            u_cmd = np.clip(u_unsat, -u_max, u_max)
            du = np.clip(u_cmd - u_prev, -u_rate_max*dt, u_rate_max*dt)
            u = u_prev + du
            #u = u_cmd#u_unsat
            # Anti-windup
            ei += (e + k_aw * (u - u_unsat) / Ki) * dt #e

            x = sys.step_nonlinear(x,u,dt)

            theta_hist.append(theta)
            u_hist.append(u)
            u_prev = u

        # ----- Plot results -----
        plt.figure()
        plt.plot(t, np.rad2deg(theta_hist))
        plt.title("Rotating Inverted Pendulum: θ(t)")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [°]")
        plt.grid(True)

        plt.figure()
        plt.plot(t, u_hist)
        plt.title("Control Torque u(t)")
        plt.xlabel("Time [s]")
        plt.ylabel("Torque [N·m]")
        plt.grid(True)
        plt.show()

    elif model_type == "Linear" and sys_type == "pendulum":
        
        ss_sys = ctrl.ss(sys.A, sys.B, sys.C, sys.D)  # SISO

        Kp = 50
        Ki = 35
        Kd = 5

        # PID controller transfer function
        s = ctrl.tf('s')
        C_pid = Kp + Ki/s + Kd*s

        G = ctrl.ss2tf(ss_sys)
        print("Plant poles:", ctrl.pole(G))
        L = G*C_pid
        gm, pm, wg, wp = ctrl.margin(L)
        print(f"gm={gm:.2f}, pm={pm:.1f} deg, wg={wg:.2f}, wp={wp:.2f}")

        # Bode for intuition
        ctrl.bode_plot(L, omega_limits=(0.1, 100), dB=True, Hz=False, deg=True, plot=True)
        plt.show()

        # Close the loop and look at step response
        T = ctrl.feedback(L, 1)
        t = np.linspace(0, 5, 1000)
        t, y = ctrl.step_response(T, t)
        plt.plot(t, y); plt.grid(True)
        plt.title("Closed-loop step (linearized plant)")
        plt.xlabel("Time [s]"); plt.ylabel("y")
        plt.show()

        # Disturbance Analysis
        t = np.linspace(0, 10, 2000)
        Gdy = G / (1 + L)  

        # (a) Step disturbance (persistent)
        d_step = np.ones_like(t)
        t_step, y_step = ctrl.forced_response(Gdy, T=t, U=d_step)

        # (b) Impulse-like disturbance (short)
        d_impulse = np.zeros_like(t)
        pulse_start, pulse_end = 1.0, 1.05  # 0.05s pulse
        d_impulse[(t >= pulse_start) & (t <= pulse_end)] = 1.0
        t_imp, y_imp = ctrl.forced_response(Gdy, T=t, U=d_impulse)

        # Plot results
        plt.figure(figsize=(8,6))
        plt.subplot(2,1,1)
        plt.plot(t_step, y_step, 'b', lw=2)
        plt.title("Response to Step (Persistent) Disturbance")
        plt.xlabel("Time [s]"); plt.ylabel("Output y(t)")
        plt.grid(True)

        plt.subplot(2,1,2)
        plt.plot(t_imp, y_imp, 'r', lw=2)
        plt.title("Response to Pulse (Short) Disturbance")
        plt.xlabel("Time [s]"); plt.ylabel("Output y(t)")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    elif model_type == "Linear" and sys_type == "cart":
        
        # pip install slycot
        ss_sys = ctrl.ss(sys.A, sys.B, sys.C, sys.D)
        G = ctrl.ss2tf(ss_sys)

        Kp_1 = 30
        Ki_1 = 5
        Kd_1 = 10

        Kp_2 = 40
        Ki_2 = 2
        Kd_2 = 13

        s = ctrl.TransferFunction.s  # Laplace variable
        
        C_pid_1 = Kp_1 + Ki_1/s + Kd_1*s   # PID for first output
        C_pid_2 = Kp_2 + Ki_2/s + Kd_2*s   # PID for second output

        # Now combine them into a 1x2 controller
        C_pid_1 = ctrl.tf(C_pid_1)
        C_pid_2 = ctrl.tf(C_pid_2)
        L_11 = G[0,0]*C_pid_1
        L_12 = G[1,0]*C_pid_2

        T_11 = ctrl.feedback(L_11, 1)
        t = np.linspace(0, 5, 1000)
        t, y = ctrl.step_response(T_11, t)
        plt.plot(t, y); plt.grid(True)
        plt.title("Closed-loop step (linearized plant)")
        plt.xlabel("Time [s]"); plt.ylabel("y")
        plt.show()

        T_12 = ctrl.feedback(L_12, 1)
        t = np.linspace(0, 5, 1000)
        t, y = ctrl.step_response(T_12, t)
        plt.plot(t, y); plt.grid(True)
        plt.title("Closed-loop step (linearized plant)")
        plt.xlabel("Time [s]"); plt.ylabel("y")
        plt.show()


    else:
        print("invalid model type")