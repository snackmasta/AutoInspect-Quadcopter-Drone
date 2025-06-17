import numpy as np
from scipy.linalg import solve_continuous_are

# LQR for position and attitude control

def lqr(A, B, Q, R):
    # Solve Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    # Compute LQR gain
    K = np.linalg.inv(R) @ B.T @ P
    return K

def lqr_position_attitude_controller(state, target, g=9.81, m=1.0, max_thrust=60.0, min_thrust=0.0, max_tau=0.5, yaw_control=False):
    """
    LQR controller for quadcopter position control.
    
    State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
    Target: [x, y, z, vx, vy, vz] or just [x, y, z] 
    Control: [tau_x, tau_y, thrust, tau_roll, tau_pitch, tau_z]
    """
    
    # Handle different target formats
    if len(target) == 3:
        target_pos = target
        target_vel = np.zeros(3)
    else:
        target_pos = target[:3]
        target_vel = target[3:6] if len(target) >= 6 else np.zeros(3)
    
    # Current state
    pos = state[:3]
    vel = state[3:6]
    angles = state[6:9]  # roll, pitch, yaw
    
    # Position control using simplified LQR approach
    # State vector: [x, y, z, vx, vy, vz]
    A = np.zeros((6, 6))
    A[0, 3] = 1  # x_dot = vx
    A[1, 4] = 1  # y_dot = vy  
    A[2, 5] = 1  # z_dot = vz
    
    # Control matrix: [ax, ay, az] (accelerations)
    B = np.zeros((6, 3))
    B[3, 0] = 1  # vx_dot = ax
    B[4, 1] = 1  # vy_dot = ay
    B[5, 2] = 1  # vz_dot = az
      # Cost matrices - more aggressive position tracking
    Q = np.diag([50, 50, 100, 10, 10, 20])  # [x, y, z, vx, vy, vz] - increased gains
    R = np.diag([0.5, 0.5, 0.05])  # [ax, ay, az] - reduced control penalty
    
    # Solve LQR
    K = lqr(A, B, Q, R)
    
    # Build error vector
    pos_error = target_pos - pos
    vel_error = target_vel - vel
    error_state = np.hstack([pos_error, vel_error])
    
    # LQR control law gives desired accelerations
    desired_accel = K @ error_state
    
    # Debug output every few calls to see what's happening
    debug_counter = getattr(lqr_position_attitude_controller, 'debug_counter', 0)
    lqr_position_attitude_controller.debug_counter = debug_counter + 1
    
    if debug_counter % 60 == 0:  # Print every 60 calls (~1 second at 60Hz)
        print(f"[LQR DEBUG] Current pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"[LQR DEBUG] Target pos:  [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")
        print(f"[LQR DEBUG] Pos error:   [{pos_error[0]:.2f}, {pos_error[1]:.2f}, {pos_error[2]:.2f}]")
        print(f"[LQR DEBUG] Desired accel: [{desired_accel[0]:.2f}, {desired_accel[1]:.2f}, {desired_accel[2]:.2f}]")
        print(f"[LQR DEBUG] Distance to target: {np.linalg.norm(pos_error):.2f}m")
    
    # Convert accelerations to control inputs
    # For small angles: ax ≈ g*theta_y, ay ≈ -g*theta_x
    desired_roll = -np.arcsin(np.clip(desired_accel[1] / g, -0.8, 0.8))  # Increased max angle
    desired_pitch = np.arcsin(np.clip(desired_accel[0] / g, -0.8, 0.8))   # Increased max angle
    
    # Attitude errors
    roll_error = desired_roll - angles[0]
    pitch_error = desired_pitch - angles[1]
      # Attitude control gains - more aggressive
    kp_att = 8.0   # Increased from 3.0
    kd_att = 1.5   # Increased from 0.5
    
    tau_x = kp_att * roll_error - kd_att * state[9]   # roll torque
    tau_y = kp_att * pitch_error - kd_att * state[10] # pitch torque
    
    # Debug attitude commands
    if debug_counter % 60 == 0:
        print(f"[LQR DEBUG] Desired roll: {np.degrees(desired_roll):.1f}°, pitch: {np.degrees(desired_pitch):.1f}°")
        print(f"[LQR DEBUG] Current roll: {np.degrees(angles[0]):.1f}°, pitch: {np.degrees(angles[1]):.1f}°")
        print(f"[LQR DEBUG] Torques: tau_x={tau_x:.3f}, tau_y={tau_y:.3f}")
        print("---")
    
    # Thrust control
    thrust = m * (g + desired_accel[2])
    
    # Yaw control
    if yaw_control and len(target) >= 7:
        target_yaw = target[6] if len(target) > 6 else 0.0
        yaw_error = target_yaw - angles[2]
        # Normalize to [-pi, pi]
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
        tau_z = 2.0 * yaw_error - 0.5 * state[11]
    else:
        tau_z = 0.0
    
    # Clamp values
    tau_x = np.clip(tau_x, -max_tau, max_tau)
    tau_y = np.clip(tau_y, -max_tau, max_tau) 
    tau_z = np.clip(tau_z, -max_tau, max_tau)
    thrust = np.clip(thrust, min_thrust, max_thrust)
    
    # Return control vector: [tau_x, tau_y, thrust, tau_roll, tau_pitch, tau_z]
    return np.array([tau_x, tau_y, thrust, 0, 0, tau_z])
