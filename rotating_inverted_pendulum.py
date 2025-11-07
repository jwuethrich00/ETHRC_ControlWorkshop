import numpy as np

class rotating_inverted_pendulum:
    def __init__(self, mass=1, gravity=9.81, length = 1, b = 0.1, init_state = None):
        # Define parameters
        self.mass = mass
        self.gravity = gravity
        self.length = length
        self.b = b
        self.I = self.mass*self.length**2
        
        # Define state variables
        if init_state is None:
            self.state = np.array([0, 0])
        else:
            self.state = init_state
        self.time = 0.0

        self.A = np.array([[0, 1],[self.mass*self.gravity*self.length/self.I, -self.b/self.I]])
        self.B = np.array([[0],[1/self.I]])
        self.C = np.array([1, 0])
        self.D = np.array([0])

    def dynamics(self, x, u):
        theta, theta_dot = x
        theta_ddot = (u + self.mass*self.gravity*self.length*np.sin(theta) - self.b*theta_dot)/self.I
        return np.array([theta_dot, theta_ddot])
    
    def step_nonlinear(self, x, u, dt= 0.1):
        k1 = self.dynamics(x, u)
        k2 = self.dynamics(x + 0.5*dt*k1, u)
        k3 = self.dynamics(x + 0.5*dt*k2, u)
        k4 = self.dynamics(x + dt*k3, u)
        x += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return x

    def step_linear(self, x, u, dt = 0.1):
        Ad = np.eye(len(x)) + dt*self.A
        Bd = self.B*dt
        x_next = Ad @ x + Bd.flatten() * u
        return x_next
