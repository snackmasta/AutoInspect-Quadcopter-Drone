"""
State manager for quadcopter simulation.
Handles state initialization, updates, and validation.
"""
import numpy as np


class StateManager:
    """Manages the quadcopter state and related parameters."""
    
    def __init__(self):
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        self.state = np.zeros(12)
        self.state[2] = 0  # start on ground
        
        # Rotor state
        self.rotor_speeds = np.zeros(4)
        self.blade_angles = [0.0] * 4
        self.rotor_thrusts = np.zeros(4)
        
        # Trajectory tracking
        self.trajectory = []
        self.wp_index = 0
        
        # Control state
        self.manual_mode = False
        self.manual_rpms = np.zeros(4)
        self.manual_yaw = 0.0
        
        # Spinup state
        self.spinup_done = False
        self.spinup_time = 2.0
        self.spinup_timer = 0.0
        
        # Flight modes
        self.is_taking_off = False
        self.is_landing = False
        self.takeoff_target = None
        self.landing_target = None
        self.landing_spindown = False
        
        # Safety state
        self.crashed = False
        self.recovery_mode = False
        
        # Hover state
        self._waypoint_paused = False
        self._hover_target = None
        
        # Debug
        self.debug_counter = 0
        self._hover_rpm_printed = False
        self._vz_history = []
        self._vz_pid_integral = 0.0
        self._vz_pid_prev_error = 0.0
    
    def reset(self):
        """Reset all state to initial conditions."""
        self.state[:] = 0
        self.state[2] = 0
        self.wp_index = 0
        self.trajectory.clear()
        self.manual_mode = False
        self.manual_rpms[:] = 0
        self.spinup_done = False
        self.spinup_timer = 0.0
        self.rotor_speeds[:] = 0
        self.crashed = False
        self.recovery_mode = False
        self._waypoint_paused = False
        if hasattr(self, '_hover_target'):
            del self._hover_target
        self.debug_counter = 0
        self._hover_rpm_printed = False
        self._vz_history.clear()
        self._vz_pid_integral = 0.0
        self._vz_pid_prev_error = 0.0
    
    def update_blade_angles(self, delta_time, speeds=None):
        """Update blade rotation angles for visualization."""
        if speeds is None:
            speeds = self.rotor_speeds
        for i in range(4):
            direction = -1 if i in [0, 3] else 1
            self.blade_angles[i] = (self.blade_angles[i] + direction * 360.0 * delta_time * speeds[i] / 60.0) % 360
    
    def get_position(self):
        """Get current position [x, y, z]."""
        return self.state[:3].copy()
    
    def get_velocity(self):
        """Get current velocity [vx, vy, vz]."""
        return self.state[3:6].copy()
    
    def get_orientation(self):
        """Get current orientation [roll, pitch, yaw]."""
        return self.state[6:9].copy()
    
    def get_angular_velocity(self):
        """Get current angular velocity [wx, wy, wz]."""
        return self.state[9:12].copy()
    
    def set_position(self, position):
        """Set position [x, y, z]."""
        self.state[:3] = position
    
    def set_orientation(self, orientation):
        """Set orientation [roll, pitch, yaw]."""
        self.state[6:9] = orientation
    
    def add_trajectory_point(self):
        """Add current position to trajectory."""
        self.trajectory.append(self.state[:3].copy())
    
    def is_manual_mode(self):
        """Check if in manual control mode."""
        return self.manual_mode
    
    def set_manual_mode(self, enabled):
        """Enable/disable manual control mode."""
        self.manual_mode = enabled
    
    def hover(self):
        """Pause waypoint progression and freeze at current position."""
        self._waypoint_paused = True
        self._hover_target = self.state[:3].copy()
    
    def resume_waypoint_progression(self):
        """Resume waypoint progression and clear hover target."""
        self._waypoint_paused = False
        if hasattr(self, '_hover_target'):
            del self._hover_target
    
    def is_hovering(self):
        """Check if currently in hover mode."""
        return self._waypoint_paused
    
    def get_hover_target(self):
        """Get the hover target position."""
        return getattr(self, '_hover_target', None)
