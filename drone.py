import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import logging

# GPU acceleration setup
try:
    import cupy as cp
    print("Using GPU acceleration with CuPy")
    xp = cp  # Use CuPy for array operations
    USE_GPU = True
except ImportError:
    print("CuPy not available, falling back to NumPy (CPU)")
    xp = np  # Use NumPy for array operations
    USE_GPU = False

# Constants
g, m, dt, L = 9.81, 0.5, 0.02, 0.6
state = xp.array([0, 0, 0, 0, 0, 0], dtype=float)
waypoints = [xp.array([0, 0, 2]), xp.array([2, 0, 2]), xp.array([2, 2, 3]), xp.array([0, 2, 2]), xp.array([0, 0, 0])]

# Generate sub-waypoints (6 stops per segment, including start and end)
all_waypoints = []
for i in range(len(waypoints) - 1):
    for j in range(6):  # Change from 5 to 6
        t = j / 5  # 0, 0.2, 0.4, 0.6, 0.8, 1
        sub_wp = waypoints[i] * (1 - t) + waypoints[i + 1] * t
        all_waypoints.append(sub_wp)
# Remove duplicates (the last of one segment is the first of the next)
all_waypoints = [all_waypoints[0]] + [all_waypoints[i] for i in range(1, len(all_waypoints)) if not xp.allclose(all_waypoints[i], all_waypoints[i-1])]

waypoints = all_waypoints  # Use sub-waypoints for navigation
wp_index = 0
rotor_speeds = xp.array([0.0] * 4)
speed_history, time_vals = [], []

# Spin-up variables
startup_rpm = 4000
spinup_step = 100  # RPM per frame
spinup_done = False

# Logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)
file_handler = logging.FileHandler('waypoint_rotor_speeds.log')
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logger.addHandler(file_handler)
console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logger.addHandler(console_handler)

def control_input(state, target):
    pos, vel = state[:3], state[3:]
    kp, kd = 6.0, 4.0
    acc_des = kp * (target - pos) - kd * vel
    acc_des[2] += g
    thrust_total = m * acc_des[2]
    base_speed = xp.clip(thrust_total * 1000, 3000, 6000)
    
    if USE_GPU:
        # Generate random noise on GPU
        rotor_speeds[:] = base_speed + cp.random.randn(4) * 100
    else:
        rotor_speeds[:] = base_speed + np.random.randn(4) * 100
    
    return m * acc_des

def update_state(state, u):
    pos, vel = state[:3], state[3:]
    acc = u / m
    acc[2] -= g
    vel += acc * dt
    pos += vel * dt
    return xp.hstack((pos, vel))

def rotor_positions(center):
    offsets = xp.array([[-L/2, L/2, 0], [L/2, L/2, 0], [L/2, -L/2, 0], [-L/2, -L/2, 0]])
    return center + offsets

def to_cpu(arr):
    """Convert GPU array to CPU array for matplotlib compatibility"""
    if USE_GPU and hasattr(arr, 'get'):
        return arr.get()  # CuPy to NumPy
    return arr

# --- Combined Visualization Setup ---
fig = plt.figure("Quadcopter Simulation", figsize=(16, 8))

# 3D Visualization subplot
ax = fig.add_subplot(121, projection='3d')
ax.set_xlim(-1, 3)
ax.set_ylim(-1, 3)
ax.set_zlim(0, 4)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Quadcopter in 3D")

# Plot yellow markers for all sub-waypoints
wps = np.array([to_cpu(wp) for wp in waypoints])
ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2], c='yellow', s=60, marker='o', label='Sub-waypoints')

# Plot blue markers for the 5 main waypoints
main_waypoints = np.array([
    [0, 0, 2],
    [2, 0, 2],
    [2, 2, 3],
    [0, 2, 2],
    [0, 0, 0]
])
ax.scatter(main_waypoints[:, 0], main_waypoints[:, 1], main_waypoints[:, 2], c='blue', s=80, marker='o', label='Main waypoints')

trajectory = []
traj_line, = ax.plot([], [], [], 'b')
rotor_lines = [ax.plot([], [], [], 'k-')[0] for _ in range(4)]
rotor_dots = ax.scatter([], [], [], c='r', s=50)
center_dot = ax.scatter([], [], [], c='green', s=80)
target_dot = ax.scatter([], [], [], c='green', marker='x', s=80)
rpm_texts = [ax.text(0, 0, 0, '') for _ in range(4)]
rotor_number_texts = [ax.text(0, 0, 0, f'{i+1}', color='red') for i in range(4)]
status_text = ax.text2D(0.05, 0.95, "Status: Spinning Up", transform=ax.transAxes, fontsize=12, color='blue')

# --- Rotor Speed Plot subplot ---
ax2 = fig.add_subplot(122)
speed_lines = [ax2.plot([], [], label=f'Rotor {i+1}')[0] for i in range(4)]
ax2.set_xlim(0, 20)
ax2.set_ylim(-7000, 7000)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("RPM")
ax2.set_title("Rotor Speeds Over Time")
ax2.legend()

def moving_average(data, window_size=5):
    if len(data) < window_size:
        return data
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

waypoint_reach_times = []
plotted_waypoint_times = set()

def animate(i):
    global state, wp_index, rotor_speeds, spinup_done

    # Persistent time accumulator
    if not hasattr(animate, "t"):
        animate.t = 0
    else:
        animate.t += dt
    t = animate.t    # Persistent stop timer for sub-waypoint pausing
    if not hasattr(animate, "pause_until"):
        animate.pause_until = None

    if not spinup_done:
        rotor_speeds[:] = xp.minimum(rotor_speeds + spinup_step, startup_rpm)
        if xp.all(rotor_speeds >= startup_rpm):
            spinup_done = True
            status_text.set_text("Status: Navigating")
        else:
            status_text.set_text("Status: Spinning Up")
    else:
        target = waypoints[wp_index]
        # Pause at each sub-waypoint for 0.1s
        if animate.pause_until is not None:
            if t < animate.pause_until:
                # Hold position, do not update state
                pass
            else:
                animate.pause_until = None
        elif xp.linalg.norm(state[:3] - target) < 0.2 and wp_index < len(waypoints) - 1:
            wp_index += 1
            waypoint_reach_times.append(t)
            rotor_speeds_cpu = to_cpu(rotor_speeds)
            state_cpu = to_cpu(state)
            formatted_speeds = [f"R{j+1} = {rotor_speeds_cpu[j]:.3f} RPM" for j in range(4)]
            pos_str = f"Pos = ({state_cpu[0]:.3f}, {state_cpu[1]:.3f}, {state_cpu[2]:.3f})"
            log_message = f"Waypoint {wp_index} reached at time {t:.2f}s. {pos_str}. Rotor Speeds: {', '.join(formatted_speeds)}"
            logger.info(log_message)
            animate.pause_until = t + 0.1  # Pause for 0.1 seconds
        elif wp_index == len(waypoints) - 1 and xp.linalg.norm(state[:3] - target) < 0.2 and state[2] <= 0.1:
            rotor_speeds[:] = xp.maximum(rotor_speeds - 50, 0)
            rotor_speeds_cpu = to_cpu(rotor_speeds)
            for j in range(4):
                rpm_texts[j].set_text(f'{rotor_speeds_cpu[j]:.0f} RPM')
            if xp.all(rotor_speeds == 0):
                status_text.set_text("Status: Landed")
                return
        else:
            u = control_input(state, target)
            state[:] = update_state(state, u)
            trajectory.append(to_cpu(state[:3].copy()))

    # --- Update 3D Elements ---
    if trajectory:
        traj = np.array(trajectory)
        traj_line.set_data(traj[:, 0], traj[:, 1])
        traj_line.set_3d_properties(traj[:, 2])

    center = to_cpu(state[:3])
    rotors = to_cpu(rotor_positions(state[:3]))

    for j in range(4):
        p1, p2 = rotors[j], rotors[(j+1)%4]
        rotor_lines[j].set_data([p1[0], p2[0]], [p1[1], p2[1]])
        rotor_lines[j].set_3d_properties([p1[2], p2[2]])

        rpm_texts[j].set_position((rotors[j][0], rotors[j][1]))
        rpm_texts[j].set_3d_properties(rotors[j][2] + 0.1)
        rpm_texts[j].set_text(f'{to_cpu(rotor_speeds)[j]:.0f} RPM')

        rotor_number_texts[j].set_position((rotors[j][0], rotors[j][1]))
        rotor_number_texts[j].set_3d_properties(rotors[j][2] - 0.2)

    rotor_dots._offsets3d = (rotors[:, 0], rotors[:, 1], rotors[:, 2])
    center_dot._offsets3d = ([center[0]], [center[1]], [center[2]])
    if spinup_done:
        target = to_cpu(waypoints[wp_index])
        target_dot._offsets3d = ([target[0]], [target[1]], [target[2]])    # --- Rotor Speeds Plot ---
    time_vals.append(t)
    speed_history.append(to_cpu(rotor_speeds.copy()))

    speeds = np.array(speed_history)
    times_trimmed = time_vals[-len(speeds):]  # Ensure length match

    # Apply moving average safely
    window_size = 5
    if len(speeds) >= window_size:
        filtered_speeds = np.array([moving_average(speeds[:, j], window_size) for j in range(4)])
        trimmed_time_vals = time_vals[window_size - 1:]
    else:
        filtered_speeds = speeds.T
        trimmed_time_vals = time_vals

    # Update each rotor line
    for j in range(4):
        speed_lines[j].set_data(trimmed_time_vals, filtered_speeds[j])

    ax2.set_xlim(trimmed_time_vals[0] if trimmed_time_vals else 0, t + 1)

    # Scale Y-axis with buffer
    min_rpm = np.min(filtered_speeds) if filtered_speeds.size else -7000
    max_rpm = np.max(filtered_speeds) if filtered_speeds.size else 7000
    ax2.set_ylim(min_rpm - 500, max_rpm + 500)

    # Mark waypoint reach times with vertical lines (only once per time)
    for waypoint_time in waypoint_reach_times:
        if waypoint_time not in plotted_waypoint_times:
            ax2.axvline(x=waypoint_time, color='red', linestyle='--')
            plotted_waypoint_times.add(waypoint_time)

ani = animation.FuncAnimation(fig, animate, frames=500, interval=10, blit=False)
plt.show()
