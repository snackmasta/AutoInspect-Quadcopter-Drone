import numpy as np

# Takeoff: set lookahead/target to (current_x, current_y, target_altitude)
def takeoff(state, target_altitude=3.0):
    current_pos = state[:3].copy()
    target = np.array([current_pos[0], current_pos[1], target_altitude])
    return target

# Land: hover at low altitude, then spin down after stabilization
# Returns (target, spindown_flag)
def land(state, hover_altitude=3, hover_time=2.0, dt=0.02):
    current_pos = state[:3].copy()
    # Use persistent state to track landing phase
    if not hasattr(land, "_timer"):
        land._timer = 0.0
        land._phase = "hover"
    if land._phase == "hover":
        target = np.array([current_pos[0], current_pos[1], hover_altitude])
        land._timer += dt
        spindown = False
        if land._timer >= hover_time:
            land._phase = "spindown"
    if land._phase == "spindown":
        target = np.array([current_pos[0], current_pos[1], 0.0])
        spindown = True
    return target, spindown

# Note: The main simulation loop should call land() every step during landing.
# To reset the landing sequence, you may want to clear land._timer and land._phase when not landing.
