import numpy as np
from .controllers import position_controller

def hover_control(state, mass=3.0, gravity=9.81, k_thrust=1.0, atmosphere_density=1.225):
    """
    Use the position controller to generate control inputs to stay at the current position and altitude.
    Returns the control output: [tau_x, tau_y, thrust, 0, 0, tau_z]
    """
    pos = state[:3]
    target = pos.copy()  # Stay at current position/altitude
    # Call the position controller with the current state and target
    control = position_controller(state, target, yaw_control_enabled=True, g=gravity, mass=mass)
    return control
