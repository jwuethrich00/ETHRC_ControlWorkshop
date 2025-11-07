import numpy as np

class cart_pole:
    def __init__(self, mass=0.2, gravity=9.81, length = 0.3, b = 0.1, cart_mass = 0.5, init_state = None):
        # Define parameters
        self.mass = mass
        self.gravity = gravity
        self.length = length
        self.b = b
        self.cart_mass = cart_mass
        
        # Define state variables
        if init_state is None:
            self.state = np.array([0, 0, 0, 0])
        else:
            self.state = init_state
        self.time = 0.0

        self.A = np.array([[0,1,0,0],[0,0,-self.mass*self.gravity/self.cart_mass, 0],[0,0,0,1],[0,0,(self.mass+self.cart_mass)*self.gravity/(self.length*self.cart_mass), 0]])
        self.B = np.array([[0],[1/self.cart_mass],[0],[-1/(self.length*self.cart_mass)]])
        self.C = np.array([[1, 0, 0, 0],[0, 1, 0, 0]])
        self.D = np.array([[0],[0]])

    def dynamics(self, x, u):
        pos = x[0]
        vel = x[1]
        theta = x[2]
        theta_dot = x[3]
        #_, vel, theta, theta_dot = x
        accel = (u + self.mass*np.sin(theta)*(self.length*theta_dot**2-self.gravity*np.cos(theta)))/(self.cart_mass + self.mass*np.sin(theta)**2)
        theta_ddot = (-u*np.cos(theta)-self.mass*self.length*theta_dot**2*np.sin(theta)*np.cos(theta)+(self.mass+self.cart_mass)*self.gravity*np.sin(theta))/(self.length*(self.cart_mass+self.mass*np.sin(theta)**2))
        return np.array([vel, accel, theta_dot, theta_ddot])
    
    def step_nonlinear(self, x, u, dt):
        k1 = self.dynamics(x, u)
        k2 = self.dynamics(x + 0.5*dt*k1, u)
        k3 = self.dynamics(x + 0.5*dt*k2, u)
        k4 = self.dynamics(x + dt*k3, u)
        x += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return x

    def step_linear(self, x, u, dt = 0.1):
        Ad = np.eye(len(x)) + dt*self.A
        Bd = self.B*dt
        x_next = Ad @ x + Bd.flatten()* u
        return x_next
