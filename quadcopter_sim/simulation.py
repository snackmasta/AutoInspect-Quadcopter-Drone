import numpy as np

class QuadcopterSimulation:
    def __init__(self):
        self.g = 9.81
        self.m = 0.5
        self.dt = 0.02
        self.L = 0.6
        self.state = np.zeros(6, dtype=float)
        self.state[2] = 0
        self.rotor_speeds = np.zeros(4)
        self.startup_rpm = 4000
        self.spinup_step = 100
        self.spinup_done = False
        self.blade_angles = [0.0] * 4
        self.trajectory = []
        self.wp_index = 0
        self._init_waypoints()

    def _init_waypoints(self):
        waypoints = [np.array([0, 0, 2]), np.array([2, 0, 2]), np.array([2, 2, 3]), np.array([0, 2, 2]), np.array([0, 0, 0])]
        all_waypoints = []
        for i in range(len(waypoints) - 1):
            for j in range(6):
                t = j / 5
                sub_wp = waypoints[i] * (1 - t) + waypoints[i + 1] * t
                all_waypoints.append(sub_wp)
        all_waypoints = [all_waypoints[0]] + [all_waypoints[i] for i in range(1, len(all_waypoints)) if not np.allclose(all_waypoints[i], all_waypoints[i-1])]
        self.waypoints = all_waypoints

    def control_input(self, target):
        pos, vel = self.state[:3], self.state[3:]
        kp, kd = 6.0, 4.0
        acc_des = kp * (target - pos) - kd * vel
        acc_des[2] += self.g
        thrust_total = self.m * acc_des[2]
        base_speed = np.clip(thrust_total * 1000, 3000, 6000)
        self.rotor_speeds[:] = base_speed + np.random.randn(4) * 100
        return self.m * acc_des

    def update_state(self, u, delta_time):
        pos, vel = self.state[:3], self.state[3:]
        acc = u / self.m
        acc[2] -= self.g
        vel += acc * delta_time
        pos += vel * delta_time
        self.state = np.hstack((pos, vel))

    def rotor_positions(self):
        offsets = np.array([[-self.L/2, self.L/2, 0], [self.L/2, self.L/2, 0], [self.L/2, -self.L/2, 0], [-self.L/2, -self.L/2, 0]])
        return self.state[:3] + offsets

    def reset(self):
        self.state[:] = 0
        self.state[2] = 0
        self.wp_index = 0
        self.rotor_speeds[:] = 0
        self.spinup_done = False
        self.trajectory.clear()
        self.blade_angles[:] = [0.0] * 4

    def step(self, delta_time):
        if not self.spinup_done:
            self.rotor_speeds[:] = np.minimum(self.rotor_speeds + self.spinup_step, self.startup_rpm)
            if np.all(self.rotor_speeds >= self.startup_rpm):
                self.spinup_done = True
        else:
            target = self.waypoints[self.wp_index]
            if np.linalg.norm(self.state[:3] - target) < 0.2 and self.wp_index < len(self.waypoints) - 1:
                self.wp_index += 1
            else:
                u = self.control_input(target)
                self.update_state(u, delta_time)
                self.trajectory.append(self.state[:3].copy())
        for i in range(4):
            self.blade_angles[i] = (self.blade_angles[i] + self.rotor_speeds[i] * 360.0 / 60.0 * delta_time) % 360
