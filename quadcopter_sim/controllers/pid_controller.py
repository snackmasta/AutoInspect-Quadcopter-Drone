"""
Basic PID Controller for quadcopter simulation.
Provides simple PID control for position and attitude.
"""
import numpy as np


class PIDController:
    """Simple PID controller for single axis control."""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.last_error = 0.0
        self.integral = 0.0
        
    def update(self, error, dt):
        """Update PID controller with current error and time step."""
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        if self.output_limits is not None:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Save error for next iteration
        self.last_error = error
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.last_error = 0.0
        self.integral = 0.0


class QuadcopterPIDController:
    """PID-based quadcopter controller using multiple PID controllers."""
    
    def __init__(self):
        # Position controllers (P and D only for stability)
        self.pid_x = PIDController(kp=1.0, ki=0.0, kd=2.0, output_limits=(-0.5, 0.5))
        self.pid_y = PIDController(kp=1.0, ki=0.0, kd=2.0, output_limits=(-0.5, 0.5))
        self.pid_z = PIDController(kp=3.0, ki=0.1, kd=4.0, output_limits=(0, 60))
        
        # Attitude controllers
        self.pid_roll = PIDController(kp=4.0, ki=0.0, kd=2.0, output_limits=(-5.0, 5.0))
        self.pid_pitch = PIDController(kp=4.0, ki=0.0, kd=2.0, output_limits=(-5.0, 5.0))
        self.pid_yaw = PIDController(kp=2.0, ki=0.0, kd=1.0, output_limits=(-2.0, 2.0))
    
    def control(self, state, target, dt=0.01, mass=3.0, g=9.81):
        """
        Generate control signals for quadcopter.
        
        Args:
            state: Current state [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
            target: Target state [x, y, z, yaw]
            dt: Time step
            mass: Quadcopter mass
            g: Gravity
            
        Returns:
            control: [roll_cmd, pitch_cmd, thrust_cmd, yaw_cmd]
        """
        pos = state[:3]
        vel = state[3:6]
        euler = state[6:9]
        
        target_pos = target[:3]
        target_yaw = target[3] if len(target) > 3 else 0.0
        
        # Position errors
        pos_error = target_pos - pos
        
        # Position control -> desired tilts
        desired_pitch = -self.pid_x.update(pos_error[0], dt)  # Negative for NED convention
        desired_roll = self.pid_y.update(pos_error[1], dt)
        
        # Altitude control -> thrust
        thrust = mass * g + self.pid_z.update(pos_error[2], dt)
        
        # Attitude control
        roll_error = desired_roll - euler[0]
        pitch_error = desired_pitch - euler[1]
        yaw_error = target_yaw - euler[2]
        
        # Wrap yaw error to [-π, π]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        
        roll_cmd = self.pid_roll.update(roll_error, dt)
        pitch_cmd = self.pid_pitch.update(pitch_error, dt)
        yaw_cmd = self.pid_yaw.update(yaw_error, dt)
        
        return np.array([roll_cmd, pitch_cmd, thrust, yaw_cmd])
    
    def reset(self):
        """Reset all PID controllers."""
        for pid in [self.pid_x, self.pid_y, self.pid_z, 
                   self.pid_roll, self.pid_pitch, self.pid_yaw]:
            pid.reset()


def simple_pid_controller(state, target, dt=0.01, mass=3.0, g=9.81):
    """
    Simple PID controller function for compatibility with existing code.
    
    Args:
        state: Current state [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        target: Target position [x, y, z] or [x, y, z, yaw]
        dt: Time step
        mass: Quadcopter mass  
        g: Gravity
        
    Returns:
        control: [tau_x, tau_y, thrust, tau_z] (torques and thrust)
    """
    # Create a simple controller instance
    if not hasattr(simple_pid_controller, 'controller'):
        simple_pid_controller.controller = QuadcopterPIDController()
    
    controller = simple_pid_controller.controller
    
    # Get control output
    control_out = controller.control(state, target, dt, mass, g)
    
    # Convert to expected format: [tau_x, tau_y, thrust, tau_z]
    # Note: This is a simplified conversion - in practice you'd need proper 
    # moment calculations based on quadcopter geometry
    tau_x = control_out[0]  # Roll moment
    tau_y = control_out[1]  # Pitch moment  
    thrust = control_out[2]  # Thrust
    tau_z = control_out[3]  # Yaw moment
    
    return np.array([tau_x, tau_y, thrust, 0.0, 0.0, tau_z])
