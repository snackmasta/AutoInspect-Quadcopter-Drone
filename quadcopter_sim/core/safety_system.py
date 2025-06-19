"""
Safety system for quadcopter simulation.
Handles crash detection, recovery, and emergency procedures.
"""
import numpy as np
import sys
import os

# Import debug configuration using centralized utility
from ..debug_utils import debug_config


class SafetySystem:
    """Handles safety monitoring and emergency procedures."""
    
    def __init__(self):
        self.crash_threshold = 0.05  # meters
        self.fall_threshold = -0.8   # m/s
        self.fall_window = 10        # steps
    
    def check_crash_or_low_altitude(self, state_manager):
        """Detect crash or dangerously low altitude."""
        # Skip safety checks if disabled
        if not state_manager.safety_system_enabled:
            return
            
        z = state_manager.state[2]
        vz = state_manager.state[5]
        
        if state_manager.crashed:
            return
        
        if z <= self.crash_threshold and vz < -0.1:
            state_manager.crashed = True
            # Removed automatic rotor stopping to allow simulation to continue
            # state_manager.rotor_speeds[:] = 0
            print("[CRASH] Drone has crashed or hit dangerously low altitude! (Simulation continues)")
    
    def check_recovery(self, state_manager):
        """Detect persistent falling and trigger recovery mode."""
        # Skip safety checks if disabled
        if not state_manager.safety_system_enabled:
            return
            
        if not hasattr(state_manager, '_vz_history'):
            state_manager._vz_history = []
        
        state_manager._vz_history.append(state_manager.state[5])
        if len(state_manager._vz_history) > self.fall_window:
            state_manager._vz_history.pop(0)
        
        # Trigger recovery if falling persistently
        if not state_manager.recovery_mode:
            falling_count = sum(1 for vz in state_manager._vz_history 
                              if vz < self.fall_threshold)
            if falling_count > self.fall_window // 2:
                state_manager.recovery_mode = True
                print("[RECOVERY] Persistent fall detected! Triggering recovery mode.")
        
        # Exit recovery if ascending
        if state_manager.recovery_mode and state_manager.state[5] > 0.1:
            state_manager.recovery_mode = False
            print("[RECOVERY] Recovery complete. Drone stabilized.")
    
    def vertical_speed_pid(self, state_manager, target_vz, dt, kp=2.0, ki=0.5, kd=0.8):
        """PID controller for vertical speed."""
        vz = state_manager.state[5]
        error = target_vz - vz
        
        state_manager._vz_pid_integral += error * dt
        derivative = (error - state_manager._vz_pid_prev_error) / dt
        state_manager._vz_pid_prev_error = error
        
        return kp * error + ki * state_manager._vz_pid_integral + kd * derivative
    
    def toggle_safety_system(self, state_manager):
        """Toggle safety system on/off."""
        state_manager.safety_system_enabled = not state_manager.safety_system_enabled
        status = "ENABLED" if state_manager.safety_system_enabled else "DISABLED"
        print(f"[SAFETY] Safety system {status}")
        return state_manager.safety_system_enabled

    def reset_safety_state(self, state_manager):
        """Reset all safety-related state."""
        state_manager.crashed = False
        state_manager.recovery_mode = False
        # Preserve safety_system_enabled state during reset
        state_manager._vz_history = []
        state_manager._vz_pid_integral = 0.0
        state_manager._vz_pid_prev_error = 0.0
