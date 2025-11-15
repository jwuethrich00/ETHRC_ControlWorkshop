
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
import matplotlib.animation as animation

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.integral = np.zeros_like(Kp, dtype=float)
        self.prev_error = np.zeros_like(Kp, dtype=float)

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class JointDynamicsSimulator:
    def __init__(self, num_joints=2):
        self.num_joints = num_joints
        self.dt = 0.05
        self.joint_positions = np.zeros(num_joints)
        self.joint_velocities = np.zeros(num_joints)
        self.max_torque = 5.0

    def step(self, torques):
        torques = np.clip(torques, -self.max_torque, self.max_torque)
        acc = torques
        self.joint_velocities += acc * self.dt
        self.joint_positions += self.joint_velocities * self.dt
        return self.joint_positions.copy()

    def forward_kinematics(self, joints):
        l1, l2 = 1.0, 1.0
        x = l1 * np.cos(joints[0]) + l2 * np.cos(joints[0] + joints[1])
        y = l1 * np.sin(joints[0]) + l2 * np.sin(joints[0] + joints[1])
        return np.array([x, y])

# Load trained model
# My pretrained model
model = PPO.load("./best_model_pretrained/best_model_pretrained")
# Your trained model
#model = PPO.load("./best_model/best_model")

# Simulator and controller
sim = JointDynamicsSimulator()

# play around with PID gains
pid = PIDController(Kp=np.array([30.0, 30.0]), Ki=np.array([1.0, 1.0]), Kd=np.array([15.0, 15.0]), dt=sim.dt)

# Only trained on one given target
target = np.array([1.07890463, 0.5476279])

trajectory = []
ee_trajectory = []

for _ in range(200):
    joints = sim.joint_positions
    ee_pos = sim.forward_kinematics(joints)
    obs = np.concatenate([joints, ee_pos, target])
    target_joints = model.predict(obs, deterministic=True)[0]
    error = target_joints - joints
    torques = pid.compute(error)
    joints = sim.step(torques)
    ee_trajectory.append(sim.forward_kinematics(joints))
    trajectory.append(joints.copy())

trajectory = np.array(trajectory)
ee_trajectory = np.array(ee_trajectory)

# Animation Setup
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.grid()

# Plot target
ax.scatter(*target, marker='x', color='red', s=100, label="Target")

# Lines for arm segments
line1, = ax.plot([], [], 'o-', lw=4, color='blue')
line2, = ax.plot([], [], 'o-', lw=4, color='blue')

# End-effector trace
trace, = ax.plot([], [], '--', color='green', alpha=0.7)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    trace.set_data([], [])
    return line1, line2, trace

def animate(i):
    j1, j2 = trajectory[i]

    # Link lengths
    l1, l2 = 1.0, 1.0

    # Joint locations
    x0, y0 = 0, 0
    x1 = l1 * np.cos(j1)
    y1 = l1 * np.sin(j1)
    x2 = x1 + l2 * np.cos(j1 + j2)
    y2 = y1 + l2 * np.sin(j1 + j2)

    # Update arm segments
    line1.set_data([x0, x1], [y0, y1])
    line2.set_data([x1, x2], [y1, y2])

    # Update end effector path
    trace.set_data(ee_trajectory[:i, 0], ee_trajectory[:i, 1])

    return line1, line2, trace

ani = animation.FuncAnimation(
    fig, animate, frames=len(trajectory),
    init_func=init, interval=50, blit=True
)

plt.legend()
plt.title("2-Link Arm End Effector Animation")
plt.show()
