import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
L = 2.0        # distance sensor x-position
s = 0.025        # process noise bound
e = 0.05       # measurement noise bound
T = 100        # number of time steps
N = 2000       # number of particles

def wrap_angle(theta):
    return theta % (2 * np.pi)

# Dynamics and measurement models
def f(x):
    b, theta = x
    v = np.random.uniform(-s, s)
    theta_new = wrap_angle(theta + b + v)
    return np.array([b, theta_new])

def h(x):
    _, theta = x
    x_obj = np.cos(theta)
    y_obj = np.sin(theta)
    dist = np.sqrt((L - x_obj)**2 + y_obj**2)
    return np.array([dist])

def roughening(particles, K=0.05):
    """
    Apply roughening to a set of particles.
    particles: (N, d) array
    K: tuning parameter (typically 0.1–0.5)
    Returns: perturbed particles
    """
    N, d = particles.shape

    # Compute range E_i for each dimension
    E = np.max(particles, axis=0) - np.min(particles, axis=0)

    # Compute standard deviation for roughening noise per dimension
    sigma = K * E * N**(-1.0 / d)

    # Add Gaussian perturbation
    noise = np.random.randn(N, d) * sigma

    particles_rough = particles + noise
    return particles_rough

# Simulate true system
x_true = np.zeros((2, T))
z = np.zeros((2,T))
x_true[:, 0] = [np.random.uniform(-s, s), np.random.uniform(0, 2*np.pi)]

for k in range(1, T):
    x_true[:, k] = f(x_true[:, k-1])
    z[0,k] = h(x_true[:, k]) + np.random.uniform(-e, e)
    if k % 10:
        if x_true[1,k] >= 0 and x_true[1,k] < np.pi:
            z[1,k] = 1
        else:
            z[1,k] = -1
    else:
        z[1,k] = 0

# Particle Filter
particles = np.random.uniform(low=[-s, 0], high=[s, 2*np.pi], size=(N, 2))
weights = np.ones(N) / N
pf_estimates = np.zeros((2, T))
particle_history = []

for k in range(1, T):
    # Propagate
    particles = np.array([f(p) for p in particles])

    # Weight
    likelihoods = []
    diffs = np.array([z[0,k] - h(p) for p in particles]).reshape(N)
    for j in range(N):
        if np.abs(diffs[j]) <= e:
            if z[1,k] == 1 and particles[j,1] >= 0 and particles[j,1] < np.pi:
                likelihood = 1/(2*e)
            elif z[1,k] == -1 and particles[j,1] >= np.pi and particles[j,1] < 2*np.pi:
                likelihood = 1/(2*e)
            elif z[1,k] == 0:
                likelihood = 1/(2*e)
            else:
                likelihood = 0
        else:
            likelihood = 0
        likelihoods.append(likelihood)
    weights = np.array(likelihoods, dtype=np.float32) + 1e-12
    weights /= np.sum(weights)

    # Resample
    indices = np.random.choice(N, size=N, p=weights)
    particles = particles[indices]
    particles = roughening(particles)
    weights = np.ones(N) / N

    # Estimate
    angles = np.array([p[1] for p in particles])
    b_vals = np.array([p[0] for p in particles])
    pf_estimates[0, k] = np.mean(b_vals)
    pf_estimates[1, k] = np.mean(angles)

    particle_history.append(particles.copy())

# Animation
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-1.5, L + 0.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Particle Filter Evolution")

sensor_dot, = ax.plot(L, 0, 'rx', markersize=10, label='Sensor')
true_dot, = ax.plot([], [], 'ko', label='True position')
pf_dot, = ax.plot([], [], 'go', label='PF estimate')
particles_scatter = ax.scatter([], [], s=5, color='blue', alpha=0.4, label='Particles')

ax.legend(loc='upper right')

def init():
    true_dot.set_data([], [])
    pf_dot.set_data([], [])
    particles_scatter.set_offsets(np.empty((0, 2)))
    return true_dot, pf_dot, particles_scatter

def update(frame):
    # True position
    x_true_pos = np.cos(x_true[1, frame])
    y_true_pos = np.sin(x_true[1, frame])
    true_dot.set_data(x_true_pos, y_true_pos)

    # PF estimate
    x_est_pos = np.cos(pf_estimates[1, frame])
    y_est_pos = np.sin(pf_estimates[1, frame])
    pf_dot.set_data(x_est_pos, y_est_pos)

    # Particles
    p = particle_history[frame-1]
    x_part = np.cos(p[:, 1])
    y_part = np.sin(p[:, 1])
    particles_scatter.set_offsets(np.column_stack((x_part, y_part)))

    return true_dot, pf_dot, particles_scatter

ani = FuncAnimation(fig, update, frames=range(1, T), init_func=init, blit=True, interval=200)

plt.show()
