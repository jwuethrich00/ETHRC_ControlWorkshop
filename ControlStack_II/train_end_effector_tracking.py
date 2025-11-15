import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

class EndEffectorTrackingEnv(gym.Env):

    def __init__(self):
        super().__init__()
        self.num_joints = 2
        self.link_lengths = [1.0, 1.0]
        self.dt = 0.1
        self.max_steps = 200
        self.action_space = spaces.Box(low=-np.pi, high=np.pi, shape=(self.num_joints,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.num_joints + 2 + 2,), dtype=np.float32
        )  # joint positions, ee_pos, goal
        self.reset()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.joint_positions = np.array([0.0, 0.0]) # to get a quickly trained NN, use always the same initial condition
        self.goal = np.array([1.07890463, 0.5476279]) # to get a quickly trained NN, use always the same target
        self.step_count = 0
        return self._get_obs(), {}

    def forward_kinematics(self, joints):
        x = self.link_lengths[0] * np.cos(joints[0]) + self.link_lengths[1] * np.cos(joints[0] + joints[1])
        y = self.link_lengths[0] * np.sin(joints[0]) + self.link_lengths[1] * np.sin(joints[0] + joints[1])
        return np.array([x, y])

    def _get_obs(self):
        ee_pos = self.forward_kinematics(self.joint_positions)
        return np.concatenate([self.joint_positions, ee_pos, self.goal])

    def step(self, action):
        
        # TO DO clip actions and assign to joint_positions
        action = 
        self.joint_positions = action

        # TO DO predict end effector position
        ee_pos = 

        # TO DO calculate error
        error = 
        dist = np.linalg.norm(error)

        # Play around with reward
        reward = -dist
        if dist < 0.1:
            reward += 5.0/(10*dist)  # bonus for getting close

        self.step_count += 1
        terminated = dist < 0.05
        truncated = self.step_count >= self.max_steps
        return self._get_obs(), reward, terminated, truncated, {}

    # output rendering
    def render(self):
        print(f"Joint pos: {self.joint_positions}, EE pos: {self.forward_kinematics(self.joint_positions)}")

# setup environment
env = EndEffectorTrackingEnv()

# callback to always check which is the best model and save the best model
eval_callback = EvalCallback(
    eval_env = EndEffectorTrackingEnv(),
    best_model_save_path="./best_model/",
    log_path="./logs/",
    eval_freq=5000,
    deterministic=True,
    render=False
)

# define agent
model = PPO("MlpPolicy", env, verbose=1)

# train
model.learn(total_timesteps=5e5, callback=eval_callback)
