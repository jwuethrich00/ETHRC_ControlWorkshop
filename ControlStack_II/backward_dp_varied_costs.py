
import numpy as np
import matplotlib.pyplot as plt

# Grid world settings
grid_size = (8, 8)
goal = (7, 7)
obstacles = {(1, 1), (2, 2), (3, 3), (4, 1), (5, 5)}
T = 25  # finite horizon

# Terrain costs: dictionary with location as key and penalty as value
terrain_costs = {
    (0, 3): 2.0,
    (1, 3): 2.0,
    (2, 3): 2.0,
    (3, 0): 1.5,
    (4, 0): 1.5,
    (4, 4): 1.0,
    (6, 6): 0.5
}

# Action space
actions = {
    0: (1, 0),  # up
    1: (-1, 0),   # down
    2: (0, -1),  # left
    3: (0, 1),   # right
}
n_actions = len(actions)

# Reward function
def reward_fn(state):
    if state in obstacles:
        return 5.0
    elif state == goal:
        return 0.0
    elif state in terrain_costs:
        return terrain_costs[state]
    else:
        return 0.3

# Transition function (deterministic)
def transition(state, action):
    r, c = state
    dr, dc = actions[action]
    r_new = min(max(r + dr, 0), grid_size[0] - 1)
    c_new = min(max(c + dc, 0), grid_size[1] - 1)
    next_state = (r_new, c_new)
    return state if next_state in obstacles else next_state

# Backward DP

# Cost to go initialized
V = np.zeros((T + 1, *grid_size))
policy = np.zeros((T, *grid_size), dtype=int)

# TO DO Implement the backward DP algorithm
for t in reversed(range(T)):
    for r in range(grid_size[0]):
        for c in range(grid_size[1]):
            state = (r, c)
            if state == goal:
                V[t, r, c] = 0.0
                continue
            values = []
            for a in range(n_actions):
                next_state = 
                reward = 
                # TO DO append the term in the expectation of the DP recursion step
                values.append()
            # TO DO: use the values to do the recursion step and update the policy
            V[t, r, c] = 
            policy[t, r, c] = 

# Plotting value function and arrows
fig, ax = plt.subplots(figsize=(8, 8))
value_grid = V[0]
cmap = plt.cm.viridis
im = ax.imshow(value_grid, cmap=cmap, origin='upper')

# Arrows for policy
for r in range(grid_size[0]):
    for c in range(grid_size[1]):
        if (r, c) in obstacles or (r, c) == goal:
            continue
        a = policy[0, r, c]
        dr, dc = actions[a]
        ax.arrow(c, r, 0.3 * dc, 0.3 * dr, head_width=0.2, head_length=0.2, fc='k', ec='k')

# Highlight goal, obstacles, and terrain costs
for (r, c) in obstacles:
    ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color='black', alpha=0.8))
for (r, c), penalty in terrain_costs.items():
    ax.text(c, r, f"{penalty:.1f}", ha='center', va='center', color='white', fontsize=9, weight='bold')
ax.scatter(goal[1], goal[0], color='gold', s=100, marker='*', label='Goal')

ax.set_title('Cost To Go with Optimal Policy and Terrain Costs')
ax.set_xticks(np.arange(grid_size[1]))
ax.set_yticks(np.arange(grid_size[0]))
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_aspect('equal')
ax.legend()
fig.colorbar(im, ax=ax, label='Cost To Go')
plt.grid(True)
plt.tight_layout()
plt.show()
