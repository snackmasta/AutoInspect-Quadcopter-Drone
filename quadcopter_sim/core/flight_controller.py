"""
Flight control system for quadcopter simulation.
Handles different flight modes and control logic.
"""
import numpy as np
import sys
import os
from ..controllers import lqr_position_attitude_controller, position_controller
from ..takeoff_landing import takeoff as takeoff_fn, land as land_fn
from ..thrust import Thrust

# Add debug directory to path to import debug_config
debug_path = os.path.join(os.path.dirname(__file__), '..', '..', 'debug')
if debug_path not in sys.path:
    sys.path.append(debug_path)
import debug_config


class FlightController:
    """Handles flight control logic and mode switching."""
    
    def __init__(self, mass, gravity, arm_length, k_thrust, atmosphere_density, 
                 max_rpm, min_rpm):
        self.mass = mass
        self.gravity = gravity
        self.arm_length = arm_length
        self.k_thrust = k_thrust
        self.atmosphere_density = atmosphere_density
        self.max_rpm = max_rpm
        self.min_rpm = min_rpm
        
        # Control parameters
        self.use_pid = False  # Toggle for PID (True) or LQR (False)
        self.target_speed = 1.0
        self.yaw_control_enabled = True
          # Thrust model
        self.thrust_model = Thrust(k_thrust, atmosphere_density)
    
    def compute_control(self, state_manager, waypoints, environment, delta_time):
        """
        Compute control inputs based on current flight mode.
        
        Returns:
            control_input: [tau_x, tau_y, thrust, tau_roll, tau_pitch, tau_z]
            rotor_thrusts: Individual rotor thrust values
        """
        state = state_manager.state
        
        # Check for recovery mode (still allow control when crashed)
        # if state_manager.crashed:
        #     return np.zeros(6), np.zeros(4)
        
        if state_manager.recovery_mode:
            return self._recovery_control(state_manager)
        
        # Handle different flight modes
        if not state_manager.spinup_done:
            return self._spinup_control(state_manager, delta_time)
        
        if state_manager.manual_mode:
            return self._manual_control(state_manager)
        
        if state_manager.is_taking_off:
            return self._takeoff_control(state_manager)
        
        if state_manager.is_landing:
            return self._landing_control(state_manager)
        
        # Normal waypoint following
        return self._waypoint_control(state_manager, waypoints)
    
    def _spinup_control(self, state_manager, delta_time):
        """Handle rotor spinup phase."""
        state_manager.spinup_timer += delta_time
        ramp_ratio = min(state_manager.spinup_timer / state_manager.spinup_time, 1.0)
        hover_rpm = self._get_hover_rpm()
        state_manager.rotor_speeds[:] = hover_rpm * ramp_ratio
        
        if ramp_ratio >= 1.0:
            state_manager.spinup_done = True
        
        return np.zeros(6), np.zeros(4)
    
    def _manual_control(self, state_manager):
        """Handle manual control mode."""
        if not state_manager._hover_rpm_printed:
            rpm_hover = self._get_hover_rpm()
            print(f"[Dynamic Physic] Calculated hover RPM per motor: {rpm_hover:.2f}")
            state_manager._hover_rpm_printed = True
        
        # Set rotor speeds directly from manual input
        state_manager.rotor_speeds[:] = np.clip(state_manager.manual_rpms, 
                                               self.min_rpm, self.max_rpm)
        
        # Handle manual yaw control
        if hasattr(state_manager, 'manual_yaw'):
            state_manager.state[8] = state_manager.manual_yaw
        
        # Compute thrusts and torques
        thrusts = self.thrust_model.compute_thrusts(state_manager.rotor_speeds)
        total_thrust = np.sum(thrusts)
        state_manager.rotor_thrusts = thrusts
        
        # Torque calculation
        L = self.arm_length / 2
        tau_x = L * (thrusts[0] + thrusts[3] - thrusts[1] - thrusts[2])  # roll
        tau_y = L * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3])  # pitch
        k_yaw = 1e-7
        tau_z = k_yaw * ((state_manager.rotor_speeds[0] + state_manager.rotor_speeds[2]) - 
                        (state_manager.rotor_speeds[1] + state_manager.rotor_speeds[3]))
        
        u = np.array([tau_x, tau_y, total_thrust, 0, 0, tau_z])
        
        # Debug output
        self._debug_manual_control(state_manager, u, thrusts)
        
        return u, thrusts
    
    def _takeoff_control(self, state_manager):
        """Handle takeoff mode."""
        target = state_manager.takeoff_target
        u = self._position_controller(state_manager, target)
        rotor_thrusts = self._calculate_rotor_thrusts(u)
        state_manager.rotor_speeds = rotor_thrusts
        
        # Check if reached target altitude
        if abs(state_manager.state[2] - target[2]) < 0.15:
            state_manager.is_taking_off = False
        
        return u, rotor_thrusts
    
    def _landing_control(self, state_manager):
        """Handle landing mode."""
        # Simple landing: just exit landing mode
        state_manager.is_landing = False
        return np.zeros(6), np.zeros(4)
    
    def _waypoint_control(self, state_manager, waypoints):
        """Handle autonomous waypoint following."""
        # Update waypoint index
        if (not state_manager._waypoint_paused and 
            state_manager.wp_index < len(waypoints) - 1 and 
            np.linalg.norm(state_manager.state[:3] - waypoints[state_manager.wp_index]) < 0.7):
            print(f"[DEBUG] Advancing to next waypoint: {state_manager.wp_index+1}/{len(waypoints)}")
            state_manager.wp_index += 1
        
        # Determine target position
        if state_manager._waypoint_paused:
            target_pos = state_manager._hover_target if state_manager._hover_target is not None else state_manager.state[:3].copy()
        else:
            target_pos = waypoints[state_manager.wp_index]        # Estimate desired velocity
        if (state_manager.wp_index < len(waypoints) - 1 and 
            (not state_manager._waypoint_paused)):
            next_pos = waypoints[state_manager.wp_index + 1]
            desired_vel = (next_pos - target_pos) / 2.0  # Assume 2 seconds to next waypoint
            max_speed = 3.0  # Increased max speed
            speed = np.linalg.norm(desired_vel)
            if speed > max_speed:
                desired_vel = desired_vel * (max_speed / speed)
        else:
            desired_vel = np.zeros(3)
            
        # Choose control method
        if self.use_pid:
            u = self._position_controller(state_manager, target_pos, 
                                        force_target_override=state_manager._waypoint_paused)
        else:
            # LQR controller - pass position and velocity target
            lqr_target = np.hstack((target_pos, desired_vel))
            
            # Debug: print what we're sending to LQR
            if state_manager.debug_counter % 60 == 0:
                print(f"[FLIGHT CTRL] Sending to LQR - Target pos: {target_pos}, Target vel: {desired_vel}")
                print(f"[FLIGHT CTRL] Current waypoint index: {state_manager.wp_index}/{len(waypoints)-1}")
                if hasattr(state_manager, '_waypoint_paused'):
                    print(f"[FLIGHT CTRL] Waypoint paused: {state_manager._waypoint_paused}")
            
            u = lqr_position_attitude_controller(
                state_manager.state, 
                lqr_target, 
                g=self.gravity, 
                m=self.mass,
                max_thrust=60.0,
                max_tau=2.0,
                yaw_control=self.yaw_control_enabled
            )
        
        # Calculate rotor thrusts
        rotor_thrusts = self._calculate_rotor_thrusts(u)
        state_manager.rotor_thrusts = rotor_thrusts
        
        # Update rotor speeds based on thrusts
        self._update_rotor_speeds_from_thrusts(state_manager, rotor_thrusts)
        
        # Debug output
        self._debug_waypoint_control(state_manager, target_pos, u, rotor_thrusts)
        
        return u, rotor_thrusts
    
    def _recovery_control(self, state_manager):
        """Handle recovery mode (max thrust)."""
        state_manager.rotor_speeds[:] = self.max_rpm
        thrusts = self.thrust_model.compute_thrusts(state_manager.rotor_speeds)
        total_thrust = np.sum(thrusts)
        u = np.array([0, 0, total_thrust, 0, 0, 0])
        return u, thrusts
    
    def _position_controller(self, state_manager, target, force_target_override=False):
        """Call the position controller with appropriate parameters."""
        return position_controller(
            state_manager.state,
            target,
            hover_indices=getattr(state_manager, 'hover_indices', None),
            wp_index=getattr(state_manager, 'wp_index', None),
            waypoints=getattr(state_manager, 'waypoints', None),
            yaw_control_enabled=self.yaw_control_enabled,
            g=self.gravity,
            mass=self.mass,
            target_speed=self.target_speed,
            force_target_override=force_target_override
        )
    
    def _calculate_rotor_thrusts(self, u):
        """Convert control inputs to individual rotor thrusts."""
        return Thrust.calculate_rotor_thrusts(u, self.arm_length)
    
    def _update_rotor_speeds_from_thrusts(self, state_manager, rotor_thrusts):
        """Update rotor speeds based on thrust values."""
        min_thrust = 1e-3
        max_omega = 2 * np.pi * self.max_rpm / 60.0
        max_thrust = self.k_thrust * self.atmosphere_density * (max_omega ** 2)
        
        for i in range(4):
            thrust = max(rotor_thrusts[i], min_thrust)
            thrust = min(thrust, max_thrust)
            omega = np.sqrt(thrust / (self.k_thrust * self.atmosphere_density))
            rpm = omega * 60.0 / (2 * np.pi)
            rpm = np.clip(rpm, self.min_rpm, self.max_rpm)
            state_manager.rotor_speeds[i] = rpm
    
    def _get_hover_rpm(self):
        """Calculate RPM needed for hovering."""
        hover_thrust = self.mass * self.gravity
        thrust_per_motor = hover_thrust / 4
        omega_hover = np.sqrt(thrust_per_motor / (self.k_thrust * self.atmosphere_density))
        rpm_hover = omega_hover * 60.0 / (2 * np.pi)
        return rpm_hover
    
    def _debug_manual_control(self, state_manager, u, thrusts):
        """Debug output for manual control."""
        if debug_config.DEBUG_MANUAL_STATUS and state_manager.debug_counter % 50 == 0:
            state = state_manager.state
            print(f"MANUAL MODE")
            print(f"Pos: [{state[0]:.1f}, {state[1]:.1f}, {state[2]:.1f}]")
            print(f"Manual RPMs: {[int(rpm) for rpm in state_manager.rotor_speeds]}")
            print(f"Control: Roll={u[0]:.2f}, Pitch={u[1]:.2f}, Thrust={u[2]:.1f}, Yaw={u[5]:.2f}")
            print(f"Rotor Thrusts: T1={thrusts[0]:.1f}, T2={thrusts[1]:.1f}, "
                  f"T3={thrusts[2]:.1f}, T4={thrusts[3]:.1f}")
            print(f"Angles: Roll={np.degrees(state[6]):.1f}°, Pitch={np.degrees(state[7]):.1f}°, "
                  f"Yaw={np.degrees(state[8]):.1f}°")
            print("---")
    
    def _debug_waypoint_control(self, state_manager, target_pos, u, rotor_thrusts):
        """Debug output for waypoint control."""
        if state_manager.debug_counter % 50 == 0:
            state = state_manager.state
            print(f"WP: {state_manager.wp_index}/{len(getattr(state_manager, 'waypoints', [])) - 1} | "
                  f"Pos: [{state[0]:.1f}, {state[1]:.1f}, {state[2]:.1f}] | "
                  f"Target: [{target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f}]")
            print(f"Control: Roll={u[0]:.2f}, Pitch={u[1]:.2f}, Thrust={u[2]:.1f}, Yaw={u[5]:.2f}")
            print(f"Rotor Thrusts: T1={rotor_thrusts[0]:.1f}, T2={rotor_thrusts[1]:.1f}, "
                  f"T3={rotor_thrusts[2]:.1f}, T4={rotor_thrusts[3]:.1f}")
            print(f"Angles: Roll={np.degrees(state[6]):.1f}°, Pitch={np.degrees(state[7]):.1f}°, "
                  f"Yaw={np.degrees(state[8]):.1f}°")
            print("---")
    
    def set_target_speed(self, speed):
        """Set target speed for waypoint following."""
        self.target_speed = speed
    
    def set_pid_mode(self, use_pid):
        """Toggle between PID and LQR control."""
        self.use_pid = use_pid
    
    def test_lqr_controller(self, state_manager):
        """Test the LQR controller with a simple hover target."""
        print("Testing LQR Controller...")
        
        # Simple hover test
        hover_target = np.array([0, 0, 5, 0, 0, 0])  # hover at 5m altitude
        
        try:
            u = lqr_position_attitude_controller(
                state_manager.state,
                hover_target,
                g=self.gravity,
                m=self.mass,
                max_thrust=60.0,
                max_tau=2.0,
                yaw_control=True
            )
            
            print(f"LQR Test Successful!")
            print(f"Current state: {state_manager.state[:6]}")
            print(f"Target: {hover_target}")
            print(f"Control output: {u}")
            return True
            
        except Exception as e:
            print(f"LQR Test Failed: {e}")
            return False
