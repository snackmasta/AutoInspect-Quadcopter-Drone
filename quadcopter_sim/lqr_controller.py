import numpy as np
from scipy.linalg import solve_continuous_are

# LQR for position and attitude control

def lqr(A, B, Q, R):
    # Solve Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    # Compute LQR gain
    K = np.linalg.inv(R) @ B.T @ P
    return K

def lqr_position_attitude_controller(
    state, target, g=9.81, m=1.0, max_thrust=60.0, min_thrust=0.0, max_tau=0.5, yaw_control=False,
    Q=None, R=None, debug=False
):
    # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
    # Target: [x, y, z, vx, vy, vz, yaw, wz]
    # Linearized model around hover
    # Position control (outer loop)
    A = np.zeros((6,6))
    A[0,3] = 1
    A[1,4] = 1
    A[2,5] = 1
    B = np.zeros((6,3))  # Only control ax, ay, az
    B[3,0] = 1  # ax
    B[4,1] = 1  # ay
    B[5,2] = 1  # az
    if Q is None:
        Q = np.diag([
            80.67158375172214,
            22.426305037285562,
            400.86095012274626,
            12.340153000983255,
            9.470822304218235,
            28.995234005420553
        ])
    if R is None:
        R = np.diag([
            0.9239052950120759,
            0.6740801361666535,
            0.1514381497427214
        ])
    K = lqr(A, B, Q, R)
    pos = state[:3]
    vel = state[3:6]
    roll, pitch, yaw = state[6:9]
    wx, wy, wz = state[9:12]
    target_pos = target[:3]
    target_vel = target[3:6]
    # Safety: if on ground, force hover
    if pos[2] < 0.1:
        target_pos[2] = 0.5
        target_vel[2] = 0.0
    err = np.hstack((target_pos - pos, target_vel - vel))
    # Desired accelerations in world frame
    acc_cmd = K @ err
    ax_des, ay_des, az_des = acc_cmd
    # Convert desired accelerations to desired roll and pitch
    phi_des = (1/g) * (ax_des * np.sin(yaw) - ay_des * np.cos(yaw))
    theta_des = (1/g) * (ax_des * np.cos(yaw) + ay_des * np.sin(yaw))
    # Attitude control (inner loop, simple PD)
    k_p_att = 4.0
    k_d_att = 1.0
    tau_x = np.clip(k_p_att * (phi_des - roll) - k_d_att * wx, -max_tau, max_tau)
    tau_y = np.clip(k_p_att * (theta_des - pitch) - k_d_att * wy, -max_tau, max_tau)
    # Thrust (compensate for tilt)
    thrust = np.clip(m * (g + az_des) / (np.cos(roll) * np.cos(pitch)), min_thrust, max_thrust)
    # Optional debug output
    if debug:
        print(f"[LQR] z={state[2]:.2f}, vz={state[5]:.2f}, err_z={target_pos[2]-pos[2]:.2f}, az_des={az_des:.2f}, thrust={thrust:.2f}")
        print(f"[LQR DEBUG] Target pos: {target_pos}, Current pos: {pos}")
        print(f"[LQR DEBUG] Target vel: {target_vel}, Current vel: {vel}")
        print(f"[LQR DEBUG] Error: {err}")
        print(f"[LQR DEBUG] acc_cmd: {acc_cmd}, phi_des: {phi_des:.3f}, theta_des: {theta_des:.3f}")
    # Optional: yaw control
    if yaw_control and len(target) >= 8:
        target_yaw = target[6]
        target_wz = target[7]
        yaw_err = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        wz_err = target_wz - wz
        tau_z = np.clip(2.0 * yaw_err + 0.5 * wz_err, -max_tau, max_tau)
    else:
        tau_z = 0.0
    if debug:
        print(f"[LQR DEBUG] Control: tau_x={tau_x:.3f}, tau_y={tau_y:.3f}, thrust={thrust:.3f}, tau_z={tau_z:.3f}")
    # Return control vector (pad to 6 for compatibility)
    return np.array([tau_x, tau_y, thrust, 0, 0, tau_z])
