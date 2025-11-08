from rotating_inverted_pendulum import rotating_inverted_pendulum
from cart_pole import cart_pole
from no_controller import simulate_no_controller
from PID import simulate_pid
from LQR import simulate_LQR
from Linear_MPC import simulate_LinearMPC
from Nonlinear_MPC import simulate_NonlinearMPC
import numpy as np

inverted_pendulum = rotating_inverted_pendulum()
cart_pole_sys = cart_pole()

controller_test = "No Control" # "No Control", "PID", "LQR", "Linear MPC", "Nonlinear MPC"

# simulation time
T_sim = 20.0

# update frequency
dt = 0.001

# initial condition
inverted_pendulum.state = np.array([np.deg2rad(2.0), 0.0])
cart_pole_sys.state = np.array([0, 0, np.deg2rad(2.0), 0.0])

if controller_test == "No Control":
    simulate_no_controller(inverted_pendulum, T_sim, dt, "Linear") #or cart_pole_sys, or "Linear"
elif controller_test == "PID":
    simulate_pid(inverted_pendulum, T_sim, dt, "Linear", "pendulum")
    #simulate_pid(cart_pole_sys, T_sim, dt, "Linear", "cart") # only use this with "Linear"
elif controller_test == "LQR":
    simulate_LQR(cart_pole_sys, T_sim, dt)
elif controller_test == "Linear MPC":
    simulate_LinearMPC(cart_pole_sys, "reference", N_hor=100, T_sim=10, dt=0.005) #5 or 15 -> approx. same for stabilize, see difference for reference
elif controller_test == "Nonlinear MPC":
    simulate_NonlinearMPC(cart_pole_sys, N_hor=100, T_sim = 10, dt = 0.01)
else:
    print("This controller method choice is not valid.") 
