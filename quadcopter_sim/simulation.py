"""
Quadcopter simulation entry point.
Provides a clean interface for quadcopter simulation with physics, control, and waypoint following.
"""
import numpy as np
import sys
import os
from .main_trajectory import get_main_trajectory
from .takeoff_landing import takeoff as takeoff_fn, land as land_fn
from .environment import Environment
from .core import PhysicsEngine, StateManager, FlightController, SafetySystem
from .utils import get_rotor_positions, get_camera_image

# Import debug configuration using centralized utility
from .debug_utils import debug_config


class QuadcopterSimulation:
    """
    Main simulation class that orchestrates all subsystems.
    Serves as the entry point for quadcopter simulation.
    """
    
    def __init__(self):
        # Physical parameters
        self.gravity = 9.81
        self.mass = 3  # kg
        self.arm_length = 0.6  # m
        self.atmosphere_density = 1.225  # kg/m^3
        self.dt = 0.002
        self.max_rpm = 12000
        self.min_rpm = 0
        
        # Calculate inertia matrix
        w, d, h = 0.8, 0.8, 0.2  # meters (body box dimensions)
        Ixx = (1/12) * self.mass * (h**2 + d**2)
        Iyy = (1/12) * self.mass * (h**2 + w**2)
        Izz = (1/12) * self.mass * (w**2 + d**2)
        self.I = np.diag([Ixx, Iyy, Izz])
        
        # Calculate thrust coefficient
        desired_hover_rpm = 6000
        omega_hover = 2 * np.pi * desired_hover_rpm / 60.0
        hover_thrust_per_motor = (self.mass * self.gravity) / 4
        self.k_thrust = hover_thrust_per_motor / (self.atmosphere_density * omega_hover ** 2)
        
        # Initialize subsystems
        self.state_manager = StateManager()
        self.physics_engine = PhysicsEngine(
            self.mass, self.gravity, self.I, self.atmosphere_density, self.k_thrust
        )
        self.flight_controller = FlightController(
            self.mass, self.gravity, self.arm_length, self.k_thrust,
            self.atmosphere_density, self.max_rpm, self.min_rpm
        )
        self.safety_system = SafetySystem()
        
        # Environment and waypoints
        self.environment = Environment(size=3, step=0.5)
        self._init_waypoints()
    
    def _init_waypoints(self):
        """Initialize waypoints for trajectory following."""
        wps = get_main_trajectory()
        self.waypoints = wps
        self.hover_indices = set()
    
    # Properties for backward compatibility
    @property
    def state(self):
        """Get current state vector."""
        return self.state_manager.state
    
    @state.setter 
    def state(self, value):
        """Set state vector."""
        self.state_manager.state = value
    
    @property
    def rotor_speeds(self):
        """Get current rotor speeds."""
        return self.state_manager.rotor_speeds
    
    @rotor_speeds.setter
    def rotor_speeds(self, value):
        """Set rotor speeds."""
        self.state_manager.rotor_speeds = value
    
    @property
    def trajectory(self):
        """Get trajectory history."""
        return self.state_manager.trajectory
    
    @property
    def wp_index(self):
        """Get current waypoint index."""
        return self.state_manager.wp_index
    
    @wp_index.setter
    def wp_index(self, value):
        """Set waypoint index."""
        self.state_manager.wp_index = value
    
    @property
    def blade_angles(self):
        """Get blade angles for visualization."""
        return self.state_manager.blade_angles
    
    @property
    def rotor_thrusts(self):
        """Get individual rotor thrusts."""
        return self.state_manager.rotor_thrusts
    
    @property
    def manual_mode(self):
        """Check if in manual mode."""
        return self.state_manager.manual_mode
    
    @manual_mode.setter
    def manual_mode(self, value):
        """Set manual mode."""
        self.state_manager.manual_mode = value
    
    @property
    def manual_rpms(self):
        """Get manual RPM settings."""
        return self.state_manager.manual_rpms
    
    @manual_rpms.setter
    def manual_rpms(self, value):
        """Set manual RPM settings."""
        self.state_manager.manual_rpms = value
    
    @property
    def spinup_done(self):
        """Check if spinup is complete."""
        return self.state_manager.spinup_done
    
    @spinup_done.setter
    def spinup_done(self, value):
        """Set spinup done status."""
        self.state_manager.spinup_done = value
    
    @property
    def target_speed(self):
        """Get target speed."""
        return self.flight_controller.target_speed

    @target_speed.setter
    def target_speed(self, value):
        """Set target speed."""
        self.flight_controller.target_speed = value
    
    @property
    def safety_system_enabled(self):
        """Check if safety system is enabled."""
        return self.state_manager.safety_system_enabled
    
    @safety_system_enabled.setter 
    def safety_system_enabled(self, value):
        """Enable or disable safety system."""
        self.state_manager.safety_system_enabled = value
        if value:
            print("[SAFETY] Safety system enabled")
        else:
            print("[SAFETY] Safety system disabled")
    
    # Configuration methods
    def set_target_speed(self, value):
        """Set target speed for waypoint following."""
        self.flight_controller.set_target_speed(value)
    
    def toggle_safety_system(self):
        """Toggle safety system on/off."""
        return self.safety_system.toggle_safety_system(self.state_manager)
    
    def get_hover_rpm(self):
        """Calculate RPM needed for hovering."""
        return self.flight_controller._get_hover_rpm()
    
    def set_manual_hover(self, target_altitude=None):
        """Set manual RPMs to hover at current or specified altitude."""
        if target_altitude is not None:
            # Only set altitude if explicitly specified
            self.state_manager.state[2] = target_altitude
            print(f"[DEBUG] Setting manual hover at altitude {target_altitude}m")
        else:
            # Preserve current altitude when toggling to manual mode
            current_altitude = self.state_manager.state[2]
            print(f"[DEBUG] Preserving current altitude {current_altitude:.2f}m in manual mode")
        
        rpm_hover = self.get_hover_rpm()
        self.state_manager.manual_rpms[:] = rpm_hover
        print(f"[Dynamic Physic] Calculated hover RPM per motor: {rpm_hover:.2f}")
    
    def step(self, delta_time):
        """
        Main simulation step.
        Updates physics, control, and safety systems.
        """
        # Prevent invalid time steps
        if delta_time is None or delta_time <= 1e-6:
            return
          # Safety checks
        self.safety_system.check_crash_or_low_altitude(self.state_manager)
        self.safety_system.check_recovery(self.state_manager)
        
        # Continue simulation even when crashed (removed early return)
        # if self.state_manager.crashed:
        #     return
        
        # Compute control inputs
        control_input, rotor_thrusts = self.flight_controller.compute_control(
            self.state_manager, self.waypoints, self.environment, delta_time
        )
        
        # Update physics
        if self.state_manager.spinup_done:
            self.state_manager.state = self.physics_engine.update_state(
                self.state_manager.state,
                self.state_manager.rotor_speeds,
                control_input,
                self.environment,
                delta_time
            )
        
        # Update visualization state
        self.state_manager.update_blade_angles(delta_time)
        self.state_manager.add_trajectory_point()
        
        # Update debug counter
        self.state_manager.debug_counter += 1
    
    def reset(self):
        """Reset simulation to initial state."""
        print("[DEBUG] Simulation reset: state, wp_index, and trajectory cleared.")
        self.state_manager.reset()
        self.safety_system.reset_safety_state(self.state_manager)
    
    def takeoff(self, target_altitude=3.0):
        """Initiate takeoff to specified altitude."""
        self.state_manager.takeoff_target = takeoff_fn(self.state_manager.state, target_altitude)
        self.state_manager.is_taking_off = True
        self.state_manager.is_landing = False
        self.state_manager.landing_spindown = False
    
    def land(self):
        """Initiate landing sequence."""
        self.state_manager.landing_target, self.state_manager.landing_spindown = land_fn(self.state_manager.state)
        self.state_manager.is_landing = True
        self.state_manager.is_taking_off = False
    
    def hover(self):
        """Pause waypoint progression and hover at current position."""
        self.state_manager.hover()
    
    def resume_waypoint_progression(self):
        """Resume waypoint following."""
        self.state_manager.resume_waypoint_progression()
    
    def rotor_positions(self):
        """Get world positions of all four rotors."""
        return get_rotor_positions(self.state_manager.state, self.arm_length)
    
    def get_camera_image(self, environment=None, fov=60, res=32, offset=0.2):
        """Get simulated camera image of terrain below drone."""
        # Use provided environment or default to self.environment
        env = environment if environment is not None else self.environment
        return get_camera_image(self.state_manager.state, env, fov, res, offset)
