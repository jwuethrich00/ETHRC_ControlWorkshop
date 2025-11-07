import numpy as np
import matplotlib.pyplot as plt
import casadi as ca

def simulate_NonlinearMPC(sys, N_hor, T_sim, dt=0.01):
    # TO DO setup casadi update function
    x_k  = 
    u_k  = 
    x_kp = sys.step_nonlinear(x_k, u_k, dt)
    F_disc = 

    # ------------------ NMPC problem (multiple shooting) ------------------
    N = N_hor
    nx, nu = 4, 1

    # TO DO
    # Optimization variables
    X =  
    U = 

    # TO DO
    # Parameters
    x0_par  =        # current state
    xref_par=         # cart x reference over horizon

    # TO DO -> blindly apply MPC (don't need to make same considerations as for Linear MPC)
    # Weights
    Q =    # x, xdot, theta, thetadot
    R = 
    Qf =   # terminal weight

    # Constraints
    u_min, u_max = -10.0, 10.0
    x_min, x_max = -0.4, 0.4                # rail limits
    th_lim = np.deg2rad(20.0)               # keep small angles in NMPC

    # Build objective and constraints
    obj = 0
    g   = []        # equality/inequality constraints
    lbg = []        # lower bounds
    ubg = []        # upper bounds

    # TO DO -> fill out bracket
    # Initial state constraint
    g.append()
    lbg += 
    ubg += 

    # TO DO Define update equation constraint, state and input constraints
    for k in range(N):
        # dynamics constraint: X_{k+1} - F(X_k, U_k) = 0
        xk = 
        uk = 
        xk1 = # optimization variable
        xk1_pred = # model
        g.append() # fill out
        lbg += 
        ubg += 

        # Input bounds (inequalities)
        g.append()# fill out
        lbg += 
        ubg += 

        # State bounds on x and theta
        g.append()  # x, fill out
        lbg += 
        ubg += 
        g.append()  # theta, fill out
        lbg += 
        ubg += 

        # TO DO
        # Stage cost (track x_ref, stabilize theta and rates)
        e_k = # error
        obj += 

    # TO DO Terminal cost
    xN = #terminal state optimization variable
    eN = # error
    obj += 

    # Pack the NLP
    w  = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
    g  = ca.vertcat(*g)
    nlp = {'x': w, 'f': obj, 'g': g, 'p': ca.vertcat(x0_par, xref_par)}

    opts = {
        'ipopt.print_level': 0,
        'print_time': 0,
        'ipopt.max_iter': 100,
        'ipopt.tol': 1e-6,
        'ipopt.linear_solver': 'mumps'
    }
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    # Helpers to map between vector and matrices
    def split_w(w_vec):
        X_vec = w_vec[:nx*(N+1)]
        U_vec = w_vec[nx*(N+1):]
        X_mat = np.array(X_vec).reshape(nx, N+1)
        U_mat = np.array(U_vec).reshape(nu, N)
        return X_mat, U_mat

    def pack_w(X0, U0):
        return np.concatenate([X0.reshape(-1,1), U0.reshape(-1,1)], axis=0)

    # Reference
    def x_ref_of(t):
        return 1/2*np.sin(t)

    # ------------------ Closed-loop simulation ------------------
    Tsim = T_sim
    dt_sim = dt
    steps = int(Tsim/dt_sim)
    tgrid = np.arange(steps+1)*dt_sim

    # Initial state
    x_cl = np.array([0.0, 0.0, np.deg2rad(0.0), 0.0])
    X_hist = [x_cl.copy()]
    U_hist = []
    Xref_hist = []

    # Warm-start with zeros
    Xws = np.tile(x_cl.reshape(-1,1), (1, N+1))
    Uws = np.zeros((nu, N))

    for k in range(steps):
        # Build reference over horizon
        xr = np.array([x_ref_of((k+i)*dt) for i in range(N)])

        # Parameter vector
        p_val = np.concatenate([x_cl, xr])

        # Initial guess / warm start
        w0 = pack_w(Xws, Uws)

        # Solve NLP
        sol = solver(x0=w0, p=p_val, lbg=lbg, ubg=ubg)
        w_opt = sol['x']
        Xopt, Uopt = split_w(w_opt)

        # Apply first input
        u_k = float(Uopt[0,0])
        U_hist.append(u_k)
        Xref_hist.append(xr[0])

        # Propagate true (nonlinear) plant with RK4
        x_cl = np.array(F_disc(x_cl, np.array([u_k]))).squeeze()
        X_hist.append(x_cl.copy())

        # Shift warm start
        Xws = np.hstack([Xopt[:,1:], Xopt[:,-1:]])
        Uws = np.hstack([Uopt[:,1:], Uopt[:,-1:]])

    X_hist = np.array(X_hist)
    U_hist = np.array(U_hist)
    Xref_hist = np.array(Xref_hist)

    # ------------------ Plots ------------------
    plt.figure(figsize=(10,8))

    plt.subplot(3,1,1)
    plt.plot(tgrid, X_hist[:,0], label='x')
    plt.plot(tgrid[:-1], Xref_hist, 'k--', label='x_ref')
    plt.ylabel('Cart x [m]')
    plt.title('Nonlinear MPC tracking — cart-pole (upright)')
    plt.grid(True); plt.legend()

    plt.subplot(3,1,2)
    plt.plot(tgrid, X_hist[:,2])
    plt.ylabel('θ [rad]')
    plt.grid(True)

    plt.subplot(3,1,3)
    plt.step(tgrid[:-1], U_hist, where='post')
    plt.axhline(u_max, color='r', linestyle='--', alpha=0.5)
    plt.axhline(u_min, color='r', linestyle='--', alpha=0.5)
    plt.ylabel('u [N]'); plt.xlabel('Time [s]')
    plt.grid(True)

    plt.tight_layout(); plt.show()