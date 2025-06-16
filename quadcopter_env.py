import gym
from gym import spaces
import numpy as np
from quadcopter_sim.simulation import QuadcopterSimulation
from quadcopter_sim.main_trajectory import get_main_trajectory

class QuadcopterEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.sim = QuadcopterSimulation()
        self.sim.use_pid = False
        self.sim.waypoints = [np.array(wp) for wp in get_main_trajectory()]
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)  # [ax, ay, az] normalized
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        self.dt = 0.02
        self.reset()

    def reset(self):
        self.sim.reset()
        self.sim.state[:3] = np.array(self.sim.waypoints[0])
        self.sim.state[3:6] = 0
        return self.sim.state.copy()

    def step(self, action):
        # Map normalized action [-1,1] to reasonable accelerations
        max_acc = 5.0  # m/s^2, tune as needed
        acc_cmd = np.clip(action, -1, 1) * max_acc
        # Use LQR-style mapping: convert to roll/pitch/thrust
        u = np.zeros(6)
        u[0] = acc_cmd[0]  # tau_x or ax
        u[1] = acc_cmd[1]  # tau_y or ay
        u[2] = acc_cmd[2]  # thrust or az
        # You may want to use your controller here instead
        self.sim.physics_update(u, self.dt)
        obs = self.sim.state.copy()
        done = hasattr(self.sim, 'crashed') and self.sim.crashed
        reward = 1.0 if not done else -100.0
        return obs, reward, done, {}

    def render(self, mode='human'):
        pass  # Optional: implement visualization
