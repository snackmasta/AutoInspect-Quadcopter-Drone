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
    A = np.zeros((6,6))
    A[0,3] = 1
    A[1,4] = 1
    A[2,5] = 1
    # B matrix: control input to acceleration
    B = np.zeros((6,4))
    B[3,0] = 1  # tau_x to wx_dot
    B[4,1] = 1  # tau_y to wy_dot
    B[5,2] = 1/m  # thrust to vz_dot
    # Default Q and R if not provided
    if Q is None:
        Q = np.diag([20, 20, 200, 5, 5, 100])
    if R is None:
        R = np.diag([1, 1, 0.1, 1])
    K = lqr(A, B, Q, R)
    pos = state[:3]
    vel = state[3:6]
    target_pos = target[:3]
    target_vel = target[3:6]
    # Safety: if on ground, force hover
    if pos[2] < 0.1:
        target_pos[2] = 0.5
        target_vel[2] = 0.0
    err = np.hstack((target_pos - pos, target_vel - vel))
    u = K @ err
    tau_x = np.clip(u[0], -max_tau, max_tau)
    tau_y = np.clip(u[1], -max_tau, max_tau)
    thrust = np.clip(m * (g + u[2]), min_thrust, max_thrust)
    # Optional debug output
    if debug:
        print(f"[LQR] z={state[2]:.2f}, vz={state[5]:.2f}, err_z={target_pos[2]-pos[2]:.2f}, u2={u[2]:.2f}, thrust={thrust:.2f}")
        print(f"[LQR DEBUG] Target pos: {target_pos}, Current pos: {pos}")
        print(f"[LQR DEBUG] Target vel: {target_vel}, Current vel: {vel}")
        print(f"[LQR DEBUG] Error: {err}")
    # Optional: yaw control
    if yaw_control and len(target) >= 8:
        yaw = state[8]
        wz = state[11]
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
