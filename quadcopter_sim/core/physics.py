"""
Physics engine for quadcopter simulation.
Handles state updates, force calculations, and ground interactions.
"""
import numpy as np
from ..drone_body_box import get_body_box_corners, body_box_terrain_forces
import debug_config


class PhysicsEngine:
    """Handles all physics calculations for the quadcopter."""
    
    def __init__(self, mass, gravity, inertia_matrix, atmosphere_density, k_thrust):
        self.mass = mass
        self.gravity = gravity
        self.I = inertia_matrix
        self.invI = np.linalg.inv(inertia_matrix)
        self.atmosphere_density = atmosphere_density
        self.k_thrust = k_thrust
    
    def update_state(self, state, rotor_speeds, control_input, environment, dt):
        """
        Update the quadcopter state based on physics.
        
        Args:
            state: Current state [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
            rotor_speeds: Array of 4 rotor speeds (RPM)
            control_input: Control input [tau_x, tau_y, thrust, tau_roll, tau_pitch, tau_z]
            environment: Environment object for ground collision
            dt: Time step
            
        Returns:
            Updated state array
        """
        # Unpack state
        x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = state
        
        # Compute actual total thrust from current rotor_speeds
        omega = 2 * np.pi * rotor_speeds / 60.0
        thrusts = self.k_thrust * self.atmosphere_density * (omega ** 2)
        Fz = np.sum(thrusts)
        
        tau_x, tau_y, _, _, _, tau_z = control_input
        
        # Rotation matrix for body to world
        R = self._rotation_matrix(roll, pitch, yaw)
        
        # Thrust in body frame (0, 0, Fz), convert to world frame
        thrust_body = np.array([0, 0, Fz])
        thrust_world = R @ thrust_body
        
        # Linear acceleration
        a = thrust_world / self.mass - np.array([0, 0, self.gravity])
        
        # Update velocities and positions
        vx += a[0] * dt
        vy += a[1] * dt
        vz += a[2] * dt
        x += vx * dt
        y += vy * dt
        z += vz * dt
        
        # Ground collision handling
        x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = self._handle_ground_collision(
            x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz, environment
        )
        
        # Angular dynamics
        tau = np.array([tau_x, tau_y, tau_z])
        
        # Add ground torque
        center = [x, y, z]
        force_ground, torque_ground = body_box_terrain_forces(
            roll, pitch, yaw, center, environment, self.mass, self.gravity, vx, vy, wx, wy, wz
        )
        tau += torque_ground
        
        # Air drag torque
        omega_body = np.array([wx, wy, wz])
        k_drag = 0.3
        tau_drag = -k_drag * omega_body
        tau += tau_drag
        
        # Angular acceleration
        omega_dot = self.invI @ (tau - np.cross(omega_body, self.I @ omega_body))
        wx += omega_dot[0] * dt
        wy += omega_dot[1] * dt
        wz += omega_dot[2] * dt
        
        # Euler angle integration
        roll, pitch, yaw = self._update_euler_angles(roll, pitch, yaw, wx, wy, wz, dt)
        
        # Clamp velocities
        vx, vy, vz = self._clamp_velocities(vx, vy, vz)
        wx, wy, wz = self._clamp_angular_velocities(wx, wy, wz)
        
        return np.array([x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz])
    
    def _rotation_matrix(self, roll, pitch, yaw):
        """Calculate rotation matrix from body to world frame."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
    
    def _handle_ground_collision(self, x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz, environment):
        """Handle ground collision and enforce solid ground."""
        corners = get_body_box_corners(roll, pitch, yaw, [x, y, z])
        ground_heights = np.array([environment.contour_height(cx, cy) for cx, cy, _ in corners])
        penetrations = ground_heights - np.array([cz for _, _, cz in corners])
        max_penetration = np.max(penetrations)
        
        if max_penetration > 0:
            # Adjust position and orientation to prevent ground penetration
            min_ground = np.min(ground_heights)
            min_corner_z = np.min([cz for _, _, cz in corners])
            z += min_ground - min_corner_z
            
            # Flatten drone to ground plane
            w, d, h = 0.8, 0.8, 0.2
            local_corners = np.array([
                [-w/2, -d/2, -h/2], [w/2, -d/2, -h/2],
                [w/2, d/2, -h/2], [-w/2, d/2, -h/2],
                [-w/2, -d/2, h/2], [w/2, -d/2, h/2],
                [w/2, d/2, h/2], [-w/2, d/2, h/2],
            ])
            
            bottom_indices = [0, 1, 2, 3]
            bottom_local = local_corners[bottom_indices, :2]
            bottom_ground = ground_heights[bottom_indices]
            
            # Fit plane to ground
            A = np.c_[bottom_local, np.ones(4)]
            coeffs, _, _, _ = np.linalg.lstsq(A, bottom_ground, rcond=None)
            a, b, c = coeffs
            
            # Set pitch and roll to match ground plane
            pitch = -np.arctan(a)
            roll = np.arctan(b)
            
            # Zero velocities on ground contact
            vz = 0
            wx = wy = wz = 0
            vx *= 0.2  # Apply friction
            vy *= 0.2
        
        return x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz
    
    def _update_euler_angles(self, roll, pitch, yaw, wx, wy, wz, dt):
        """Update Euler angles using proper kinematic equations."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        tp = np.tan(pitch)
        
        # Avoid singularity at pitch = ±90 degrees
        if abs(cp) < 1e-6:
            cp = 1e-6 * np.sign(cp)
            tp = sp / cp
        
        # Kinematic equations
        T = np.array([
            [1, sr * tp, cr * tp],
            [0, cr, -sr],
            [0, sr / cp, cr / cp]
        ])
        
        omega_body = np.array([wx, wy, wz])
        euler_rates = T @ omega_body
        
        roll += euler_rates[0] * dt
        pitch += euler_rates[1] * dt
        yaw += euler_rates[2] * dt
        
        # Keep yaw in [-pi, pi]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        
        return roll, pitch, yaw
    
    def _clamp_velocities(self, vx, vy, vz, max_vel=20.0):
        """Clamp linear velocities to prevent overflow."""
        return (
            np.clip(vx, -max_vel, max_vel),
            np.clip(vy, -max_vel, max_vel),
            np.clip(vz, -max_vel, max_vel)
        )
    
    def _clamp_angular_velocities(self, wx, wy, wz, max_omega=10.0):
        """Clamp angular velocities to prevent overflow."""
        return (
            np.clip(wx, -max_omega, max_omega),
            np.clip(wy, -max_omega, max_omega),
            np.clip(wz, -max_omega, max_omega)
        )
