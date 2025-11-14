# mpc_cartpole_tracking.py
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
import polytope as pc
#pip install cvxopt

def simulate_LinearMPC(sys,mode, N_hor=100, T_sim=10, dt=0.001):

    def pre_set(X, Acl):
        H_pre = X.A @ Acl
        h_pre = X.b
        return pc.Polytope(H_pre, h_pre)

    def compute_mpi_set(X, Acl, max_iter=100):
        Omega = X
        for i in range(max_iter):
            pre = pre_set(Omega, Acl)
            Omega_next = Omega.intersect(pre)
            # Stop if converged
            if Omega_next == Omega:
                print(f"Converged in {i+1} iterations.")
                return Omega_next
            Omega = Omega_next
        print("Warning: reached max_iter without full convergence.")
        return Omega

    if mode == "stabilize":

        # Discretize (simple forward Euler)
        Ad = np.eye(4) + sys.A*dt
        Bd = sys.B*dt

        # ----------------- MPC setup -----------------
        nx, nu = 4, 1
        N = N_hor  # horizon steps (N*dt seconds lookahead)

        # Costs
        Q = np.diag([500.0, 1.0, 100.0, 20.0])   # x, xdot, theta, thetadot
        R = np.array([[0.5]])
        # Terminal cost from LQR
        P = solve_discrete_are(Ad, Bd, Q, R)
        Qf = P
        K = - np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad) # u = K x

        # Constraints
        Fx = np.vstack((np.array([1,0, 0, 0]), -np.array([1,0, 0, 0])))
        fx = np.array([0.45, 0.45])
        Fu = np.array([[1], [-1]])
        fu = np.array([10, 10])

        # Combine constraints into state-only form (u = Kx)
        H = np.vstack((Fx, Fu @ K))
        h = np.hstack((fx, fu))

        # Build constraint set as a polytope
        X = pc.Polytope(H, h)

        # Compute max. invariant set
        A_cl = Ad + Bd@K
        Omega_inf = compute_mpi_set(X, A_cl)

        # Resulting MPI set
        print("Terminal set (H, h):")
        print(Omega_inf.A)
        print(Omega_inf.b)

        # setup parametrized optimization problem
        x = cp.Variable((nx, N+1))
        u = cp.Variable((nu, N))
        x0 = cp.Parameter(nx)

        cost = 0
        constraints = [x[:,0] == x0]
        for j in range(N):
            # dynamics
            constraints += [x[:,j+1] == Ad @ x[:,j] + Bd @ u[:,j]]
            # state and input constraints
            constraints += [Fx @ x[:,j] <= fx,
                            Fu @ u[:,j] <= fu]
            cost += cp.quad_form(x[:,j], Q) + cp.quad_form(u[:,j], R)

        # terminal constraints and terminal cost
        constraints += [Omega_inf.A @ x[:,N] <= Omega_inf.b]
        cost += cp.quad_form(x[:,N], Qf)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        # ----------------- Closed-loop simulation -----------------
        Tsim = T_sim
        steps = int(Tsim/dt)
        X = np.zeros((nx, steps+1))
        U = np.zeros((nu, steps))
        tgrid = np.arange(steps+1)*dt

        # initial condition: small angle perturbation
        X[:,0] = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])

        for k in range(steps):

            x0.value = X[:,k]
            prob.solve(solver = cp.OSQP, max_iter = 100000)

            if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                print(f"Warning: solver status at step {k}: {prob.status}")
                # fallback: zero input
                u_k = 0.0
            else:
                u_k = u.value[0,0]

            # apply first control move
            U[:,k] = u_k
            # advance linear model
            if k == int(steps/2):
                X[:,k+1] = Ad @ X[:,k] + Bd.flatten()*u_k + 0.1 # adding a disturbance
            else:
                X[:,k+1] = Ad @ X[:,k] + Bd.flatten()*u_k

        # ----------------- Plots -----------------
        theta = X[2,:]
        xpos  = X[0,:]

        plt.figure(figsize=(10,7))
        plt.subplot(3,1,1)
        plt.plot(tgrid, xpos, label="x")
        plt.ylabel("Cart x [m]"); plt.title("Linear MPC tracking on cart–pole (upright linearization)")
        plt.grid(True); plt.legend()

        plt.subplot(3,1,2)
        plt.plot(tgrid, np.rad2deg(theta))
        plt.ylabel("θ [°]")
        plt.grid(True)

        u_min, u_max = -10.0, 10.0
        plt.subplot(3,1,3)
        plt.step(tgrid[:-1], U[0], where='post')
        plt.axhline(u_max, color='r', linestyle='--', alpha=0.6)
        plt.axhline(u_min, color='r', linestyle='--', alpha=0.6)
        plt.ylabel("u [N]"); plt.xlabel("Time [s]")
        plt.grid(True)

        plt.tight_layout(); plt.show()

    elif mode == "reference":

        # Discretize (simple forward Euler is fine for small dt; use c2d if you prefer)
        Ad = np.eye(4) + sys.A*dt
        Bd = sys.B*dt

        # ----------------- MPC setup -----------------
        nx, nu = 4, 1
        N = N_hor  # horizon steps (N*dt seconds lookahead)

        # Costs
        Q = np.diag([500.0, 1.0, 100.0, 20.0])   # x, xdot, theta, thetadot
        R = np.array([[0.5]])
        # Terminal cost from LQR
        P = solve_discrete_are(Ad, Bd, Q, R)
        Qf = P
        K = - np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad) # u = K x

        # constraints
        Fx = np.vstack((np.array([1,0, 0, 0]), -np.array([1,0, 0, 0])))
        fx = np.array([[0.4], [0.4]])
        Fu = np.array([[1], [-1]])
        fu = np.array([[10], [10]])

        # Combine constraints into state-only form (u = Kx)
        H = np.vstack((Fx, Fu @ K))
        h = np.vstack((fx, fu))

        # Constraint set as polytope
        X = pc.Polytope(H, h)

        # Compute MPI set
        A_cl = Ad + Bd@K
        Omega_inf = compute_mpi_set(X, A_cl)

        # Resulting MPI set
        print("Terminal set (H, h):")
        print(Omega_inf.A)
        print(Omega_inf.b)

        # Optimization problem setup
        x = cp.Variable((nx, N+1))
        u = cp.Variable((nu, N))
        x0 = cp.Parameter(nx)
        xs_par = cp.Parameter(nx)
        us_par = cp.Parameter(nu)

        cost = 0
        constraints = [x[:,0] == x0]
        for j in range(N):
            # dynamics
            constraints += [x[:,j+1] == Ad @ x[:,j] + Bd @ u[:,j]]
            # state and input constraints, shifted
            constraints += [Fx @ x[:,j] <= fx - Fx@xs_par,
                            Fu @ u[:,j] <= fu - Fu@us_par]
            # cost
            cost += cp.quad_form(x[:,j], Q) + cp.quad_form(u[:,j], R)

        # terminal constraint and terminal cost
        constraints += [Omega_inf.A @ x[:,N] <= Omega_inf.b - Omega_inf.A@xs_par]
        cost += cp.quad_form(x[:,N], Qf)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        # ----------------- Closed-loop simulation -----------------
        Tsim = T_sim
        steps = int(Tsim/dt)
        X = np.zeros((nx, steps+1))
        U = np.zeros((nu, steps))
        tgrid = np.arange(steps+1)*dt

        # initial condition: small angle perturbation
        X[:,0] = np.array([0.0, 0.0, np.deg2rad(0.0), 0.0])
        r = np.nan*np.zeros((2,steps))

        for k in range(steps):
            
            """
            if k < int(steps/3):
                r[:,k] = np.array([0.1,0])
            elif k < int(2*steps/3):
                r[:,k] = np.array([0.6,0])
            else:
                r[:,k] = np.array([0.0,0])
            """
            r[:,k] = np.array([1/2*np.sin(k/250),0.0])

            # calculate steady state and input for given reference (trivial with given C matrix)
            M = np.block([[np.eye(nx) - Ad, -Bd],
                        [sys.C, np.zeros((sys.C.shape[0], nu))]])
            rhs = np.vstack((np.zeros((nx, 1)), r[:,k].reshape(-1,1)))
            sol = np.linalg.lstsq(M, rhs, rcond=None)[0]
            xs, us = sol[:nx], sol[nx:]
            xs, us = xs.flatten(), us.flatten()

            x0.value = X[:,k] - xs
            xs_par.value = xs
            us_par.value = us
            prob.solve(solver=cp.MOSEK, verbose=False)

            if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                print(f"Warning: solver status at step {k}: {prob.status}")
                # fallback: zero input
                u_k = 0.0
            else:
                u_k = u.value[0,0] + us

            # apply first control move
            U[:,k] = u_k
            # advance linear model
            X[:,k+1] = Ad @ X[:,k] + Bd.flatten()*u_k

        # ----------------- Plots -----------------
        theta = X[2,:]
        xpos  = X[0,:]

        plt.figure(figsize=(10,7))
        plt.subplot(3,1,1)
        plt.plot(tgrid, xpos, label="x")
        plt.plot(tgrid[:-1], r[0,:].flatten(), 'k--', label="x_ref")
        plt.ylabel("Cart x [m]"); plt.title("Linear MPC tracking on cart–pole (upright linearization)")
        plt.grid(True); plt.legend()

        plt.subplot(3,1,2)
        plt.plot(tgrid, np.rad2deg(theta))
        plt.ylabel("θ [°]")
        plt.grid(True)

        u_min, u_max = -10.0, 10.0
        plt.subplot(3,1,3)
        plt.step(tgrid[:-1], U[0], where='post')
        plt.axhline(u_max, color='r', linestyle='--', alpha=0.6)
        plt.axhline(u_min, color='r', linestyle='--', alpha=0.6)
        plt.ylabel("u [N]"); plt.xlabel("Time [s]")
        plt.grid(True)

        plt.tight_layout(); plt.show()