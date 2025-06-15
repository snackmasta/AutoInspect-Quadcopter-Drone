import numpy as np

# Takeoff: set lookahead/target to (current_x, current_y, target_altitude)
def takeoff(state, target_altitude=3.0):
    current_pos = state[:3].copy()
    target = np.array([current_pos[0], current_pos[1], target_altitude])
    return target

# Land: set lookahead/target to (current_x, current_y, 0) and signal spindown
# Returns (target, spindown_flag)
def land(state):
    current_pos = state[:3].copy()
    target = np.array([current_pos[0], current_pos[1], 0.0])
    spindown = True
    return target, spindown

# Note: The main simulation loop should use these targets directly and handle spindown logic.
