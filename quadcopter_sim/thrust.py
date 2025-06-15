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
        return np.sum(self.compute_thrusts(rotor_speeds_rpm))

    @staticmethod
    def draw_thrust_arrows(rotor_positions, rotor_speeds_rpm, min_rpm=0, max_rpm=12000, thrust_coefficient=1.0, atmosphere_density=1.225):
        """
        Draw arrows showing the thrust force from each rotor.
        The arrow length is scaled based on the thrust corresponding to min_rpm and max_rpm (default 0-12000 RPM).
        :param rotor_positions: List/array of rotor positions (shape: [4, 3])
        :param rotor_speeds_rpm: List/array of rotor speeds (RPM) for each rotor
        :param min_rpm: Minimum RPM for scaling (default 0)
        :param max_rpm: Maximum RPM for scaling (default 12000)
        :param thrust_coefficient: Thrust coefficient (N/(rad/s)^2)
        :param atmosphere_density: Air density (kg/m^3)
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
            end_pos = start_pos - np.array([0, 0, arrow_length])
            glBegin(GL_LINES)
            glVertex3f(*start_pos)
            glVertex3f(*end_pos)
            glEnd()
            # Arrowhead
            arrow_tip = end_pos
            arrow_size = 0.05
            glBegin(GL_TRIANGLES)
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1], arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1], arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1], arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1], arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glEnd()
