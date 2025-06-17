"""
thrust.py
Module for calculating and visualizing thrust forces for a quadcopter simulation.
"""

import numpy as np
from OpenGL.GL import *

class Thrust:
    def __init__(self, thrust_coefficient: float, atmosphere_density: float = 1.225, num_rotors: int = 4):
        """
        Initialize the Thrust model.
        :param thrust_coefficient: Coefficient relating rotor speed squared to thrust (N/(rad/s)^2)
        :param atmosphere_density: Air density (kg/m^3)
        :param num_rotors: Number of rotors (default: 4 for quadcopter)
        """
        self.thrust_coefficient = thrust_coefficient
        self.atmosphere_density = atmosphere_density
        self.num_rotors = num_rotors

    def compute_thrusts(self, rotor_speeds_rpm: np.ndarray) -> np.ndarray:
        """
        Compute the thrust produced by each rotor given RPM.
        :param rotor_speeds_rpm: Array of rotor speeds (RPM), shape (num_rotors,)
        :return: Array of thrusts (N), shape (num_rotors,)
        """
        omega = 2 * np.pi * rotor_speeds_rpm / 60.0  # rad/s
        return self.thrust_coefficient * self.atmosphere_density * np.square(omega)

    def rpm_from_thrust(self, thrust: float) -> float:
        """
        Compute the required RPM for a given thrust (N).
        :param thrust: Desired thrust (N)
        :return: Required RPM
        """
        omega = np.sqrt(thrust / (self.thrust_coefficient * self.atmosphere_density))
        rpm = omega * 60.0 / (2 * np.pi)
        return rpm

    def total_thrust(self, rotor_speeds_rpm: np.ndarray) -> float:
        """
        Compute the total thrust produced by all rotors.
        :param rotor_speeds_rpm: Array of rotor speeds (RPM), shape (num_rotors,)
        :return: Total thrust (N)
        """
        return np.sum(self.compute_thrusts(rotor_speeds_rpm))    @staticmethod
    def draw_thrust_arrows(rotor_positions, rotor_speeds_rpm, min_rpm=0, max_rpm=12000, thrust_coefficient=1.0, atmosphere_density=1.225, drone_orientation=None):
        """
        Draw arrows showing the thrust force from each rotor based on actual thrust direction.
        The arrow length is scaled based on the thrust corresponding to min_rpm and max_rpm (default 0-12000 RPM).
        :param rotor_positions: List/array of rotor positions (shape: [4, 3])
        :param rotor_speeds_rpm: List/array of rotor speeds (RPM) for each rotor
        :param min_rpm: Minimum RPM for scaling (default 0)
        :param max_rpm: Maximum RPM for scaling (default 12000)
        :param thrust_coefficient: Thrust coefficient (N/(rad/s)^2)
        :param atmosphere_density: Air density (kg/m^3)
        :param drone_orientation: [roll, pitch, yaw] in radians for proper thrust direction
        """
        if rotor_speeds_rpm is None or rotor_positions is None:
            return
        # Compute thrusts for min and max RPM
        omega_min = 2 * np.pi * min_rpm / 60.0
        omega_max = 2 * np.pi * max_rpm / 60.0
        min_thrust = thrust_coefficient * atmosphere_density * omega_min ** 2
        max_thrust = thrust_coefficient * atmosphere_density * omega_max ** 2
        # Compute actual thrusts
        omega = 2 * np.pi * np.array(rotor_speeds_rpm) / 60.0
        rotor_thrusts = thrust_coefficient * atmosphere_density * np.square(omega)
        
        # Calculate thrust direction based on drone orientation
        if drone_orientation is not None:
            roll, pitch, yaw = drone_orientation
            # Rotation matrix from body to world frame
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            R = np.array([
                [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],                [-sp, cp*sr, cp*cr]
            ])
            # Thrust direction in body frame (pointing down - direction air is pushed)
            thrust_body = np.array([0, 0, -1])
            # Transform to world frame
            thrust_world = R @ thrust_body
        else:
            # Fallback to downward direction if no orientation provided
            thrust_world = np.array([0, 0, -1])
        
        glLineWidth(4)
        for i, (rotor_pos, thrust) in enumerate(zip(rotor_positions, rotor_thrusts)):
            if thrust <= 0:
                continue
            # Scale arrow length between min_thrust and max_thrust
            arrow_length = 0.1 + ((thrust - min_thrust) / (max_thrust - min_thrust)) * 0.9
            arrow_length = max(0.1, min(arrow_length, 1.0))
            intensity = (thrust - min_thrust) / (max_thrust - min_thrust)
            intensity = max(0.0, min(intensity, 1.0))
            glColor3f(1.0, 1.0 - intensity, 0.0)  # Yellow to Red gradient
            start_pos = np.array(rotor_pos)
            # Calculate end position based on actual thrust direction
            end_pos = start_pos + thrust_world * arrow_length
            glBegin(GL_LINES)
            glVertex3f(*start_pos)
            glVertex3f(*end_pos)
            glEnd()
            # Arrowhead - create a cone pointing in thrust direction
            arrow_tip = end_pos
            arrow_size = 0.05
            
            # Create arrowhead perpendicular vectors
            if abs(thrust_world[2]) < 0.9:  # Not nearly vertical
                perp1 = np.cross(thrust_world, np.array([0, 0, 1]))
                perp1 = perp1 / np.linalg.norm(perp1)
            else:  # Nearly vertical, use different reference
                perp1 = np.cross(thrust_world, np.array([1, 0, 0]))
                perp1 = perp1 / np.linalg.norm(perp1)
            
            perp2 = np.cross(thrust_world, perp1)
            perp2 = perp2 / np.linalg.norm(perp2)
            
            # Create arrowhead base
            base_center = arrow_tip - thrust_world * arrow_size
            
            glBegin(GL_TRIANGLES)
            # Create 4 triangular faces for the arrowhead cone
            for angle in [0, np.pi/2, np.pi, 3*np.pi/2]:
                edge_point = base_center + arrow_size * (np.cos(angle) * perp1 + np.sin(angle) * perp2)
                next_angle = angle + np.pi/2
                next_edge = base_center + arrow_size * (np.cos(next_angle) * perp1 + np.sin(next_angle) * perp2)
                
                # Triangle from tip to two edge points
                glVertex3f(*arrow_tip)
                glVertex3f(*edge_point)
                glVertex3f(*next_edge)
            glEnd()
    @staticmethod
    def calculate_rotor_thrusts(u, L, min_thrust=1e-3):
        """
        Convert control inputs [tau_x, tau_y, total_thrust, 0, 0, tau_z]
        to individual rotor thrusts [T1, T2, T3, T4]
        Rotor layout (+ configuration):
        T1 (FL) --- T2 (FR)
           |         |
           |    +    |
           |         |
        T4 (RL) --- T3 (RR)
        :param u: control input vector [tau_x, tau_y, total_thrust, 0, 0, tau_z]
        :param L: arm length (meters)
        :param min_thrust: minimum thrust per rotor (N)
        :return: np.array([T1, T2, T3, T4])
        """
        tau_x, tau_y, total_thrust, _, _, tau_z = u
        L = L / 2  # half arm length
        T_base = total_thrust / 4.0
        k_yaw = 1e-7  # drag torque coefficient
        d_roll = tau_x / (2 * L) if L > 0 else 0
        d_pitch = tau_y / (2 * L) if L > 0 else 0
        d_yaw = 0.0  # Remove yaw differential
        T1 = T_base + d_roll + d_pitch + d_yaw  # Front Left
        T2 = T_base + d_roll - d_pitch - d_yaw  # Front Right
        T3 = T_base - d_roll - d_pitch + d_yaw  # Rear Right
        T4 = T_base - d_roll + d_pitch - d_yaw  # Rear Left
        thrusts = np.array([T1, T2, T3, T4])
        min_val = np.min(thrusts)
        if min_val < min_thrust:
            diff = min_thrust - min_val
            thrusts += diff
            total = np.sum(thrusts)
            if total > total_thrust:
                thrusts = thrusts * (total_thrust / total)
        max_diff = 0.6 * T_base
        max_val = np.max(thrusts)
        if max_val - np.min(thrusts) > max_diff:
            mean_val = np.mean(thrusts)
            thrusts = np.clip(thrusts, mean_val - max_diff/2, mean_val + max_diff/2)
            thrusts = thrusts * (total_thrust / np.sum(thrusts))
        thrusts = np.clip(thrusts, min_thrust, None)
        return thrusts
