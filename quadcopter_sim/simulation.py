import numpy as np

class QuadcopterSimulation:
    def __init__(self):
        # Physical parameters
        self.g = 9.81
        self.m = 5  # mass (kg)
        self.L = 0.6  # arm length (m)
        # More realistic inertia for a 5kg, 0.6m quadcopter
        # Ixx ≈ Iyy ≈ 0.6, Izz ≈ 0.3 (kg*m^2)
        self.I = np.diag([0.6, 0.6, 0.3])  # inertia matrix (kg*m^2)
        self.invI = np.linalg.inv(self.I)
        self.dt = 0.01
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        self.state = np.zeros(12)
        self.state[2] = 0  # start on ground
        self.rotor_speeds = np.zeros(4)
        self.max_rpm = 6000
        self.min_rpm = 0
        self.trajectory = []
        self.wp_index = 0
        self.spinup_done = True  # Always true for physics-based sim
        self.blade_angles = [0.0] * 4
        self.k_thrust = 1.5e-6  # thrust coefficient (N/(rad/s)^2), typical for small drones
        self.prev_thrust = 0.0
        self.thrust_alpha = 0.2  # smoothing factor for thrust (0=no smoothing, 1=full smoothing)
        self.manual_mode = False
        self.manual_rpms = np.zeros(4)
        self._init_waypoints()

    def _insert_hover_after_sharp_turns(self, waypoints, hover_steps=50, angle_threshold_deg=30):
        # Insert a hover (repeat waypoint) after sharp turns, and mark them for stabilization
        new_wps = []
        self.hover_indices = set()
        for i in range(len(waypoints)):
            new_wps.append(waypoints[i])
            if 0 < i < len(waypoints) - 1:
                v1 = waypoints[i] - waypoints[i-1]
                v2 = waypoints[i+1] - waypoints[i]
                v1[2] = 0
                v2[2] = 0
                norm1 = np.linalg.norm(v1)
                norm2 = np.linalg.norm(v2)
                if norm1 > 1e-3 and norm2 > 1e-3:
                    v1 /= norm1
                    v2 /= norm2
                    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
                    angle = np.arccos(dot) * 180 / np.pi
                    if angle > angle_threshold_deg:
                        # Insert hover after this waypoint and mark indices
                        for _ in range(hover_steps):
                            new_wps.append(waypoints[i].copy())
                            self.hover_indices.add(len(new_wps)-1)
        return new_wps

    def _init_waypoints(self):
        # Simple takeoff to (0,0,6) and hover, then move to a new point to test yaw/roll/pitch
        altitude = 6.0
        takeoff = [np.array([0, 0, z]) for z in np.linspace(0, altitude, 10)]
        hover = [np.array([0, 0, altitude])] * 100  # Shorter hover
        # New waypoint: move to (5, 5, 6) to test yaw/roll/pitch
        move1 = [np.array([x, x, altitude]) for x in np.linspace(0, 5, 30)]
        hover2 = [np.array([5, 5, altitude])] * 70
        # Move to (-5, 5, 6) to test more angular dynamics
        move2 = [np.array([x, 5, altitude]) for x in np.linspace(5, -5, 30)]
        hover3 = [np.array([-5, 5, altitude])] * 70
        # Return to center
        move3 = [np.array([x, y, altitude]) for x, y in zip(np.linspace(-5, 0, 30), np.linspace(5, 0, 30))]
        hover4 = [np.array([0, 0, altitude])] * 50
        wps = takeoff + hover + move1 + hover2 + move2 + hover3 + move3 + hover4
        self.waypoints = self._insert_hover_after_sharp_turns(wps, hover_steps=60, angle_threshold_deg=30)

    def get_hover_rpm(self):
        """Calculate the RPM needed for each rotor to hover, based on physical parameters."""
        hover_thrust = self.m * self.g
        thrust_per_motor = hover_thrust / 4
        # T = k_thrust * (omega^2), omega in rad/s
        # omega_hover = sqrt(T / k_thrust)
        omega_hover = np.sqrt(thrust_per_motor / self.k_thrust)
        rpm_hover = omega_hover * 60.0 / (2 * np.pi)
        return rpm_hover

    def set_manual_hover(self):
        """Set manual RPMs to hover value."""
        rpm_hover = self.get_hover_rpm()
        self.manual_rpms[:] = rpm_hover
        print(f"[Dynamic Physic] Calculated hover RPM per motor: {rpm_hover:.2f}")

    def step(self, delta_time):
        if self.manual_mode:
            # Optionally print hover RPM for user
            if not hasattr(self, '_hover_rpm_printed'):
                rpm_hover = self.get_hover_rpm()
                print(f"[Dynamic Physic] Calculated hover RPM per motor: {rpm_hover:.2f}")
                self._hover_rpm_printed = True
            
            # In manual mode, set rotor speeds directly
            self.rotor_speeds[:] = np.clip(self.manual_rpms, self.min_rpm, self.max_rpm)
            # For visualization, update blade angles
            for i in range(4):
                self.blade_angles[i] = (self.blade_angles[i] + 360.0 * delta_time * self.rotor_speeds[i] / 60.0) % 360            # Physics update: use the sum of manual thrusts and compute torques
            thrusts = self.k_thrust * (2 * np.pi * self.rotor_speeds / 60.0) ** 2
            total_thrust = np.sum(thrusts)
            
            # Store individual thrusts for visualization
            self.rotor_thrusts = thrusts
            
            # Torque calculation (X: roll, Y: pitch, Z: yaw)
            # Rotor order: [FL, FR, RR, RL] (assuming + configuration)
            # Roll: (right - left), Pitch: (front - rear), Yaw: (CW - CCW)
            L = self.L / 2
            # Original: tau_x = L * (thrusts[1] + thrusts[2] - thrusts[0] - thrusts[3])  # roll
            # Corrected for positive roll = right side down: (Left Thrust Sum - Right Thrust Sum)
            tau_x = L * (thrusts[0] + thrusts[3] - thrusts[1] - thrusts[2])  # roll
            tau_y = L * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3])  # pitch
            # Assume alternate rotors spin in opposite directions for yaw
            k_yaw = 1e-7  # drag torque coefficient (arbitrary small value)
            tau_z = k_yaw * ((self.rotor_speeds[0] + self.rotor_speeds[2]) - (self.rotor_speeds[1] + self.rotor_speeds[3]))
            u = np.array([tau_x, tau_y, total_thrust, 0, 0, tau_z])
            
            # Debug output for manual mode
            if hasattr(self, 'debug_counter'):
                self.debug_counter += 1
            else:
                self.debug_counter = 0
                
            if self.debug_counter % 50 == 0:
                print(f"MANUAL MODE")
                print(f"Pos: [{self.state[0]:.1f}, {self.state[1]:.1f}, {self.state[2]:.1f}]")
                print(f"Manual RPMs: {[int(rpm) for rpm in self.rotor_speeds]}")
                print(f"Control: Roll={u[0]:.2f}, Pitch={u[1]:.2f}, Thrust={u[2]:.1f}, Yaw={u[5]:.2f}")
                print(f"Rotor Thrusts: T1={thrusts[0]:.1f}, T2={thrusts[1]:.1f}, "
                      f"T3={thrusts[2]:.1f}, T4={thrusts[3]:.1f}")
                print(f"Angles: Roll={np.degrees(self.state[6]):.1f}°, Pitch={np.degrees(self.state[7]):.1f}°, "
                      f"Yaw={np.degrees(self.state[8]):.1f}°")
                print("---")
            
            self.physics_update(u, delta_time)
            self.trajectory.append(self.state[:3].copy())
            return

        # Simple PID to follow waypoints
        # Increase threshold for switching waypoints to avoid getting stuck
        if self.wp_index < len(self.waypoints) - 1 and np.linalg.norm(self.state[:3] - self.waypoints[self.wp_index]) < 0.7:
            self.wp_index += 1
        target = self.waypoints[self.wp_index]
        u = self.position_controller(target)
        
        # Calculate individual rotor thrusts for debug
        rotor_thrusts = self.calculate_rotor_thrusts(u)
        self.rotor_thrusts = rotor_thrusts  # Store for debug access
        
        # Debug output every 50 steps (reduce spam)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 50 == 0:
            print(f"WP: {self.wp_index}/{len(self.waypoints)-1} | "
                  f"Pos: [{self.state[0]:.1f}, {self.state[1]:.1f}, {self.state[2]:.1f}] | "
                  f"Target: [{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}]")
            print(f"Control: Roll={u[0]:.2f}, Pitch={u[1]:.2f}, Thrust={u[2]:.1f}, Yaw={u[5]:.2f}")
            print(f"Rotor Thrusts: T1={rotor_thrusts[0]:.1f}, T2={rotor_thrusts[1]:.1f}, "
                  f"T3={rotor_thrusts[2]:.1f}, T4={rotor_thrusts[3]:.1f}")
            print(f"Angles: Roll={np.degrees(self.state[6]):.1f}°, Pitch={np.degrees(self.state[7]):.1f}°, "
                  f"Yaw={np.degrees(self.state[8]):.1f}°")
            print("---")
        
        self.physics_update(u, delta_time)
        self.trajectory.append(self.state[:3].copy())        # Animate blade angles for visualization
        for i in range(4):
            self.blade_angles[i] = (self.blade_angles[i] + 360.0 * delta_time * 20) % 360
        
        # Update rotor_speeds based on actual individual thrusts
        rpm_smoothing = 0.2  # Smoothing factor (0 = no smoothing, 1 = instant)
        min_thrust = 1e-3    # Minimum thrust to avoid 0 RPM
        for i in range(4):
            thrust = max(self.rotor_thrusts[i], min_thrust)
            omega = np.sqrt(thrust / self.k_thrust)  # rad/s
            rpm = omega * 60.0 / (2 * np.pi)
            rpm = np.clip(rpm, self.min_rpm, self.max_rpm)
            # Smooth RPM update
            self.rotor_speeds[i] = (1 - rpm_smoothing) * self.rotor_speeds[i] + rpm_smoothing * rpm

    def position_controller(self, target):
        # PID for position -> desired acceleration
        pos, vel = self.state[:3], self.state[3:6]
        roll, pitch, yaw = self.state[6:9]
        wx, wy, wz = self.state[9:12]
        kp, kd = 3.0, 2.0
        acc_des = kp * (target - pos) - kd * vel
        acc_des[2] += self.g
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        horizontal_distance = np.sqrt(dx**2 + dy**2)

        # --- Lookahead Yaw Logic with Stabilize-on-Hover ---
        desired_yaw = None
        if hasattr(self, 'hover_indices') and self.wp_index in self.hover_indices:
            # During hover after sharp turn, hold current yaw
            desired_yaw = yaw
        else:
            if hasattr(self, 'waypoints') and hasattr(self, 'wp_index'):
                if self.wp_index < len(self.waypoints) - 1:
                    next_wp = self.waypoints[self.wp_index + 1]
                    lookahead_dx = next_wp[0] - target[0]
                    lookahead_dy = next_wp[1] - target[1]
                    if abs(lookahead_dx) > 1e-3 or abs(lookahead_dy) > 1e-3:
                        desired_yaw = np.arctan2(lookahead_dy, lookahead_dx)
        if desired_yaw is None:
            desired_yaw = np.arctan2(dy, dx)

        # Transform desired acceleration from world to body frame using current yaw
        acc_des_xy = acc_des[:2]
        c_yaw = np.cos(yaw)
        s_yaw = np.sin(yaw)
        acc_body_x =  c_yaw * acc_des[0] + s_yaw * acc_des[1]
        acc_body_y = -s_yaw * acc_des[0] + c_yaw * acc_des[1]
        pitch_des = acc_body_x / self.g
        roll_des  = -acc_body_y / self.g
        max_angle = np.pi / 6  # 30 degrees
        pitch_des = np.clip(pitch_des, -max_angle, max_angle)
        roll_des = np.clip(roll_des, -max_angle, max_angle)
        kp_att, kd_att = 8.0, 2.0
        tau_x = kp_att * (roll_des - roll) - kd_att * wx
        tau_y = kp_att * (pitch_des - pitch) - kd_att * wy
        # Yaw control: always lead with the next segment
        yaw_error = (desired_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        heading_vec = np.array([np.cos(yaw), np.sin(yaw)])
        target_vec = np.array([np.cos(desired_yaw), np.sin(desired_yaw)])
        cross = heading_vec[0]*target_vec[1] - heading_vec[1]*target_vec[0]
        if cross < 0 and yaw_error > 0:
            yaw_error -= 2 * np.pi
        elif cross > 0 and yaw_error < 0:
            yaw_error += 2 * np.pi
        kp_yaw = 4.0
        kd_yaw = 1.0
        tau_z = kp_yaw * yaw_error - kd_yaw * wz
        
        # Total thrust (vertical component)
        thrust = self.m * acc_des[2]
        thrust = np.clip(thrust, 0, 4 * self.max_rpm)
        
        # Apply smoothing to thrust to reduce jitter
        thrust = self.thrust_alpha * thrust + (1 - self.thrust_alpha) * self.prev_thrust
        self.prev_thrust = thrust
        
        return np.array([tau_x, tau_y, thrust, 0, 0, tau_z])

    def physics_update(self, u, dt):
        # u: [tau_x, tau_y, thrust, tau_roll, tau_pitch, tau_yaw]
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = self.state
        Fz = u[2]
        tau_x, tau_y, _, _, _, tau_z = u
        # Rotation matrix for body to world
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        # Thrust in body frame (0, 0, Fz), convert to world frame
        thrust_body = np.array([0, 0, Fz])
        thrust_world = R @ thrust_body
        # Linear acceleration
        a = thrust_world / self.m - np.array([0, 0, self.g])
        vx += a[0] * dt
        vy += a[1] * dt
        vz += a[2] * dt
        x += vx * dt
        y += vy * dt
        z = max(0, z + vz * dt)  # don't go below ground        # Angular acceleration (Euler's equation)
        omega = np.array([wx, wy, wz])
        tau = np.array([tau_x, tau_y, tau_z])
        omega_dot = self.invI @ (tau - np.cross(omega, self.I @ omega))
        wx += omega_dot[0] * dt
        wy += omega_dot[1] * dt
        wz += omega_dot[2] * dt
        
        # Proper Euler angle integration using kinematic equations
        # Convert body angular velocities to Euler angle rates
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        tp = np.tan(pitch)
        
        # Avoid singularity at pitch = ±90 degrees
        if abs(cp) < 1e-6:
            cp = 1e-6 * np.sign(cp)
            tp = sp / cp
        
        # Kinematic equations: [roll_dot, pitch_dot, yaw_dot] = T * [wx, wy, wz]
        T = np.array([
            [1, sr * tp, cr * tp],
            [0, cr, -sr],
            [0, sr / cp, cr / cp]
        ])
        
        euler_rates = T @ omega
        roll += euler_rates[0] * dt
        pitch += euler_rates[1] * dt
        yaw += euler_rates[2] * dt
        # Keep yaw in [-pi, pi]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        self.state = np.array([x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz])

    def reset(self):
        self.state[:] = 0
        self.state[2] = 0
        self.wp_index = 0
        self.trajectory.clear()
        self.prev_thrust = 0.0  # reset thrust smoothing
        self.manual_mode = False
        self.manual_rpms[:] = 0

    def rotor_positions(self):
        # Return positions of the 4 rotors based on current state (x, y, z, roll, pitch, yaw)
        x, y, z, _, _, _, roll, pitch, yaw, _, _, _ = self.state
        # Rotor offsets in body frame
        offsets = np.array([
            [-self.L/2, self.L/2, 0],
            [self.L/2, self.L/2, 0],
            [self.L/2, -self.L/2, 0],
            [-self.L/2, -self.L/2, 0]
        ])
        # Rotation matrix from body to world
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        world_offsets = offsets @ R.T
        return np.array([ [x, y, z] + world_offsets[i] for i in range(4)])

    def feet_positions(self):
        # Define 4 feet, each offset from the center (body frame)
        # Place feet slightly inward from rotors, and below the body
        x, y, z, _, _, _, roll, pitch, yaw, _, _, _ = self.state
        # Feet offsets in body frame (x, y, z)
        foot_z = -0.25  # feet extend below drone body
        foot_offset = self.L * 0.35  # closer to center than rotors
        offsets = np.array([
            [-foot_offset, foot_offset, foot_z],
            [foot_offset, foot_offset, foot_z],
            [foot_offset, -foot_offset, foot_z],
            [-foot_offset, -foot_offset, foot_z]
        ])
        # Rotation matrix from body to world
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        world_offsets = offsets @ R.T
        return np.array([[x, y, z] + world_offsets[i] for i in range(4)])

    def feet_hitboxes(self, radius=0.06):
        # Returns a list of (center, radius) for each foot
        return [(pos, radius) for pos in self.feet_positions()]

    def get_camera_image(self, environment, fov=60, res=32, offset=0.2):
        """
        Simulate a downward-facing camera at the bottom of the drone.
        Returns a heightmap (grayscale image) of the terrain below.
        """
        pos = self.state[:3].copy()
        pos[2] -= offset  # camera is below the drone
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * pos[2]
        xs = np.linspace(pos[0] - size, pos[0] + size, res)
        ys = np.linspace(pos[1] - size, pos[1] + size, res)
        img = np.zeros((res, res), dtype=np.float32)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                img[j, i] = environment.contour_height(x, y)
        return img

    def calculate_rotor_thrusts(self, u):
        """
        Convert control inputs [tau_x, tau_y, total_thrust, 0, 0, tau_z] 
        to individual rotor thrusts [T1, T2, T3, T4]
        
        Rotor layout (+ configuration):
        T1 (FL) --- T2 (FR)
           |         |
           |    +    |
           |         |
        T4 (RL) --- T3 (RR)
        """
        tau_x, tau_y, total_thrust, _, _, tau_z = u
        L = self.L / 2  # half arm length
        T_base = total_thrust / 4.0
        k_yaw = 1e-7  # drag torque coefficient
        d_roll = tau_x / (2 * L) if L > 0 else 0
        d_pitch = tau_y / (2 * L) if L > 0 else 0
        d_yaw = -tau_z / (4 * k_yaw) if k_yaw > 0 else 0
        T1 = T_base + d_roll + d_pitch + d_yaw  # Front Left
        T2 = T_base + d_roll - d_pitch - d_yaw  # Front Right
        T3 = T_base - d_roll - d_pitch + d_yaw  # Rear Right
        T4 = T_base - d_roll + d_pitch - d_yaw  # Rear Left
        thrusts = np.array([T1, T2, T3, T4])
        # --- Fine control: scale torques if any thrust < min_thrust ---
        min_thrust = 1e-3
        min_val = np.min(thrusts)
        if min_val < min_thrust:
            # Compute scaling factor for torques
            # Only scale torques, not total thrust
            scale = (T_base - min_thrust) / (T_base - min_val) if T_base != min_val else 0.0
            d_roll *= scale
            d_pitch *= scale
            d_yaw *= scale
            T1 = T_base + d_roll + d_pitch + d_yaw
            T2 = T_base + d_roll - d_pitch - d_yaw
            T3 = T_base - d_roll - d_pitch + d_yaw
            T4 = T_base - d_roll + d_pitch - d_yaw
            thrusts = np.array([T1, T2, T3, T4])
        thrusts = np.clip(thrusts, min_thrust, None)
        return thrusts
