import numpy as np

def takeoff(state, waypoints, wp_index, target_altitude=3.0):
    current_pos = state[:3].copy()
    # Start takeoff trajectory slightly above ground to avoid friction lock
    start_z = max(current_pos[2], 0.05)
    takeoff_traj = [np.array([current_pos[0], current_pos[1], z]) for z in np.linspace(start_z, target_altitude, 20)]
    hover = [np.array([current_pos[0], current_pos[1], target_altitude])] * 50
    # Resume from the last waypoint index after takeoff
    new_waypoints = takeoff_traj + hover + list(waypoints[wp_index:])
    # Set wp_index to the first mission waypoint after takeoff+hover
    new_wp_index = len(takeoff_traj) + len(hover)
    return new_waypoints, new_wp_index

def land(state, dt=0.01):
    current_pos = state[:3].copy()
    z0 = current_pos[2]
    # 1. Maintain stable orientation for 7 seconds (longer to damp oscillation)
    stable_hover = [np.array([current_pos[0], current_pos[1], z0])] * int(7.0 / dt)
    # 2. Descend very slowly to ground (more steps for smoother descent)
    descend_steps = 200
    descend = [np.array([current_pos[0], current_pos[1], z]) for z in np.linspace(z0, 0, descend_steps)]
    # 3. Touchdown and spin to 0 RPM (simulate by holding at ground for 2 seconds)
    ground_wait = [np.array([current_pos[0], current_pos[1], 0])] * int(2.0 / dt)
    # 4. Wait for takeoff command (hold indefinitely at ground)
    # (The main loop should check for takeoff command to proceed)
    landing_traj = stable_hover + descend + ground_wait
    return landing_traj

# Note: The main route (waypoints) is not erased during landing. The main loop should switch between the landing trajectory and the main route as needed.
