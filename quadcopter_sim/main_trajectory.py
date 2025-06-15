import numpy as np

def get_main_trajectory():
    altitude = 6.0
    takeoff = [np.array([0, 0, z]) for z in np.linspace(0, altitude, 8)]
    # Segitiga: tiga titik utama
    p1 = np.array([0, 0, altitude])
    p2 = np.array([20, 0, altitude])
    p3 = np.array([10, 17.32, altitude])  # 17.32 ≈ 20 * sin(60°)
    # Trajectory: p1 -> p2 -> p3 -> p1
    n_points = 40
    leg1 = [p1 + (p2 - p1) * t for t in np.linspace(0, 1, n_points)]
    leg2 = [p2 + (p3 - p2) * t for t in np.linspace(0, 1, n_points)]
    leg3 = [p3 + (p1 - p3) * t for t in np.linspace(0, 1, n_points)]
    wps = takeoff + leg1 + leg2 + leg3
    # Kembali ke home
    wps.append(np.array([0, 0, altitude]))
    return wps

def get_lookahead_target(pos, waypoints, lookahead_dist=2.0):
    """
    Given the current position and a list of waypoints, find a target point
    that is lookahead_dist ahead along the path.
    """
    if len(waypoints) < 2:
        return waypoints[0] if waypoints else pos
    # Find closest segment
    min_dist = float('inf')
    closest_idx = 0
    for i in range(len(waypoints) - 1):
        seg_start = waypoints[i]
        seg_end = waypoints[i+1]
        seg_vec = seg_end - seg_start
        seg_len = np.linalg.norm(seg_vec)
        if seg_len < 1e-6:
            continue
        proj = np.dot(pos - seg_start, seg_vec) / seg_len
        proj = np.clip(proj, 0, seg_len)
        closest_point = seg_start + seg_vec * (proj / seg_len)
        dist = np.linalg.norm(pos - closest_point)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
            closest_proj = proj
    # Move lookahead_dist along the path from the closest point
    seg_start = waypoints[closest_idx]
    seg_end = waypoints[closest_idx+1]
    seg_vec = seg_end - seg_start
    seg_len = np.linalg.norm(seg_vec)
    lookahead_proj = closest_proj + lookahead_dist
    if lookahead_proj <= seg_len:
        target = seg_start + seg_vec * (lookahead_proj / seg_len)
    else:
        # Move to next segments if needed
        remaining = lookahead_proj - seg_len
        idx = closest_idx + 1
        while remaining > 0 and idx < len(waypoints) - 1:
            next_seg = waypoints[idx+1] - waypoints[idx]
            next_len = np.linalg.norm(next_seg)
            if remaining <= next_len:
                target = waypoints[idx] + next_seg * (remaining / next_len)
                break
            remaining -= next_len
            idx += 1
        else:
            target = waypoints[-1]
    return target
