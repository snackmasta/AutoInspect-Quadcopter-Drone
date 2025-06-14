import numpy as np

# Reduced gains to minimize oscillation
# Position controller gains
KP_POS = 1.5  # was 3.0
KD_POS = 3.0  # was 2.0
# Attitude controller gains
KP_ATT = 5.0  # was 8.0
KD_ATT = 3.0  # was 2.0
# Yaw controller gains
KP_YAW = 2.0  # was 4.0
KD_YAW = 1.5  # was 1.0

def position_controller(state, target, hover_indices=None, wp_index=None, waypoints=None, yaw_control_enabled=True, g=9.81, m=1.0):
    pos, vel = state[:3], state[3:6]
    roll, pitch, yaw = state[6:9]
    wx, wy, wz = state[9:12]
    # Use new gains
    kp, kd = KP_POS, KD_POS
    acc_des = kp * (target - pos) - kd * vel
    acc_des[2] += g
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    horizontal_distance = np.sqrt(dx**2 + dy**2)
    acc_des_xy = acc_des[:2]
    c_yaw = np.cos(yaw)
    s_yaw = np.sin(yaw)
    acc_body_x =  c_yaw * acc_des[0] + s_yaw * acc_des[1]
    acc_body_y = -s_yaw * acc_des[0] + c_yaw * acc_des[1]
    pitch_des = acc_body_x / g
    roll_des  = -acc_body_y / g
    max_angle = np.pi / 6  # 30 degrees
    pitch_des = np.clip(pitch_des, -max_angle, max_angle)
    roll_des = np.clip(roll_des, -max_angle, max_angle)
    # Use new attitude gains
    kp_att, kd_att = KP_ATT, KD_ATT
    tau_x = kp_att * (roll_des - roll) - kd_att * wx
    tau_y = kp_att * (pitch_des - pitch) - kd_att * wy
    # Yaw control toggle
    if yaw_control_enabled:
        desired_yaw = None
        if hover_indices is not None and wp_index is not None and wp_index in hover_indices:
            desired_yaw = yaw
        elif waypoints is not None and wp_index is not None and wp_index < len(waypoints) - 1:
            next_wp = waypoints[wp_index + 1]
            lookahead_dx = next_wp[0] - target[0]
            lookahead_dy = next_wp[1] - target[1]
            if abs(lookahead_dx) > 1e-3 or abs(lookahead_dy) > 1e-3:
                desired_yaw = np.arctan2(lookahead_dy, lookahead_dx)
        if desired_yaw is None:
            desired_yaw = np.arctan2(dy, dx)
        heading_vec = np.array([np.cos(yaw), np.sin(yaw)])
        target_vec = np.array([np.cos(desired_yaw), np.sin(desired_yaw)])
        yaw_error = desired_yaw - yaw
        cross = heading_vec[0]*target_vec[1] - heading_vec[1]*target_vec[0]
        if cross < 0 and yaw_error > 0:
            yaw_error -= 2 * np.pi
        elif cross > 0 and yaw_error < 0:
            yaw_error += 2 * np.pi
        # Use new yaw gains
        kp_yaw = KP_YAW
        kd_yaw = KD_YAW
        tau_z = kp_yaw * yaw_error - kd_yaw * wz
    else:
        tau_z = 0.0
    thrust = m * acc_des[2]
    return np.array([tau_x, tau_y, thrust, 0, 0, tau_z])
