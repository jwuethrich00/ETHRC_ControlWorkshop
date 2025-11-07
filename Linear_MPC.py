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
        
        # TO DO define linear discretized system
        # Discretize (simple forward Euler)
        Ad = 
        Bd = 

        # ----------------- MPC setup -----------------
        nx, nu = 4, 1
        N = N_hor  # horizon steps (N*dt seconds lookahead)

        # TO DO Define MPC parameters
        # Costs
        Q =   # x, xdot, theta, thetadot
        R = 
        # Terminal cost from LQR
        P = 
        Qf = 
        K = 

        # TO DO: define constraints
        # Constraints
        Fx = 
        fx = 
        Fu = 
        fu = 

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

        # TO DO optimization variables + parameters 
        # setup parametrized optimization problem
        x = 
        u = 
        x0 = 

        # TO DO: define costs and constraints -> Hint: do initial and terminal constraints separate, for rest use for loop
        cost = 
        constraints =

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

        # TO DO: copy code from above up to H, h definitions

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

        # TO DO 
        # Optimization problem setup (think that steady state is a parameter as well)
        x = 
        u = 
        x0 = 
        xs_par = 
        us_par = 

        # TO DO: Implement cost and constraints, analogously to above but remember that constraints need to be shifted
        cost = 
        constraints = 

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
            
            # TO DO: try the two different references
            if k < int(steps/3):
                r[:,k] = np.array([0.1,0])
            elif k < int(2*steps/3):
                r[:,k] = np.array([0.6,0])
            else:
                r[:,k] = np.array([0.0,0])
            
            #r[:,k] = np.array([1/2*np.sin(k/250),0.0])

            # calculate steady state and input for given reference (trivial with given C matrix)
            # TO DO: define M matrix and RHS vector
            M = 
            rhs = 
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