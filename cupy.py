try:
    import cupy as cp  # GPU-accelerated NumPy alternative
    xp = cp
    print("Using CuPy for GPU acceleration.")
except ImportError:
    import numpy as np
    xp = np
    print("CuPy not installed. Falling back to NumPy.")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import logging

# Constants
g, m, dt, L = 9.81, 0.5, 0.02, 0.6
state = xp.array([0, 0, 0, 0, 0, 0], dtype=float)
waypoints = [xp.array([0, 0, 2]), xp.array([2, 0, 2]), xp.array([2, 2, 3]), xp.array([0, 2, 2]), xp.array([0, 0, 0])]
wp_index = 0
rotor_speeds = xp.array([0.0] * 4)
speed_history, time_vals = [], []

# Spin-up variables
startup_rpm = 4000
spinup_step = 100  # RPM per frame
spinup_done = False

# Set up logging to both file and console
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# File handler
file_handler = logging.FileHandler('waypoint_rotor_speeds.log')
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logger.addHandler(file_handler)

# Console handler
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
    rotor_speeds[:] = base_speed + xp.random.randn(4) * 100
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

# --- 3D Visualization Setup ---
fig = plt.figure("3D Quadcopter")
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1, 3)
ax.set_ylim(-1, 3)
ax.set_zlim(0, 4)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Quadcopter in 3D")

trajectory = []
traj_line, = ax.plot([], [], [], 'b')
rotor_lines = [ax.plot([], [], [], 'k-')[0] for _ in range(4)]
rotor_dots = ax.scatter([], [], [], c='r', s=50)
center_dot = ax.scatter([], [], [], c='green', s=80)
target_dot = ax.scatter([], [], [], c='green', marker='x', s=80)
rpm_texts = [ax.text(0, 0, 0, '') for _ in range(4)]
rotor_number_texts = [ax.text(0, 0, 0, f'{i+1}', color='red') for i in range(4)]
status_text = ax.text2D(0.05, 0.95, "Status: Spinning Up", transform=ax.transAxes, fontsize=12, color='blue')

# --- Rotor Speed Plot ---
fig2, ax2 = plt.subplots()
speed_lines = [ax2.plot([], [], label=f'Rotor {i+1}')[0] for i in range(4)]
ax2.set_xlim(0, 10)
ax2.set_ylim(-7000, 7000)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("RPM")
ax2.set_title("Rotor Speeds Over Time")
ax2.legend()

def moving_average(data, window_size=5):
    if len(data) < window_size:
        return data
    return xp.convolve(data, xp.ones(window_size)/window_size, mode='valid')

# Add a list to store waypoint reach times
waypoint_reach_times = []
# Add a set to track already plotted waypoint times
plotted_waypoint_times = set()

def animate(i):
    global state, wp_index, rotor_speeds, spinup_done

    t = i * dt

    if not spinup_done:
        # Spin-up phase
        rotor_speeds[:] = xp.minimum(rotor_speeds + spinup_step, startup_rpm)
        if xp.all(rotor_speeds >= startup_rpm):
            spinup_done = True
            status_text.set_text("Status: Navigating")
        else:
            status_text.set_text("Status: Spinning Up")
    else:
        target = waypoints[wp_index]

        if wp_index == len(waypoints) - 1 and xp.linalg.norm(state[:3] - target) < 0.2 and state[2] <= 0.1:
            rotor_speeds[:] = xp.maximum(rotor_speeds - 50, 0)
            for j in range(4):
                rpm_texts[j].set_text(f'{rotor_speeds[j]:.0f} RPM')
            if xp.all(rotor_speeds == 0):
                status_text.set_text("Status: Landed")
                return
        else:
            u = control_input(state, target)
            state[:] = update_state(state, u)
            trajectory.append(state[:3].copy())

            if xp.linalg.norm(state[:3] - target) < 0.2 and wp_index < len(waypoints) - 1:
                wp_index += 1
                waypoint_reach_times.append(t)  # Record the time when the waypoint is reached

                # Format rotor speeds in list style
                formatted_speeds = [f"R{j+1} = {rotor_speeds[j]:.3f} RPM" for j in range(4)]
                formatted_speeds_str = ", ".join(formatted_speeds)

                # Log and print rotor speeds
                log_message = f"Waypoint {wp_index} reached at time {t:.2f}s. Rotor Speeds: {formatted_speeds_str}"
                logger.info(log_message)
                print(log_message)

    # --- Update 3D elements ---
    if trajectory:
        traj = xp.array(trajectory)
        traj_line.set_data(traj[:, 0], traj[:, 1])
        traj_line.set_3d_properties(traj[:, 2])

    center = state[:3]
    rotors = rotor_positions(center)

    for j in range(4):
        p1, p2 = rotors[j], rotors[(j+1)%4]
        rotor_lines[j].set_data([p1[0], p2[0]], [p1[1], p2[1]])
        rotor_lines[j].set_3d_properties([p1[2], p2[2]])

        rpm_texts[j].set_position((rotors[j][0], rotors[j][1]))
        rpm_texts[j].set_3d_properties(rotors[j][2]+0.1)
        rpm_texts[j].set_text(f'{rotor_speeds[j]:.0f} RPM')

        rotor_number_texts[j].set_position((rotors[j][0], rotors[j][1]))
        rotor_number_texts[j].set_3d_properties(rotors[j][2] - 0.2)

    rotor_dots._offsets3d = (rotors[:, 0], rotors[:, 1], rotors[:, 2])
    center_dot._offsets3d = ([center[0]], [center[1]], [center[2]])
    if spinup_done:
        target = waypoints[wp_index]
        target_dot._offsets3d = ([target[0]], [target[1]], [target[2]])

    # --- Rotor Speeds Plot ---
    time_vals.append(t)
    speed_history.append(rotor_speeds.copy())
    speeds = xp.array(speed_history)

    filtered_speeds = xp.array([moving_average(speeds[:, j]) for j in range(4)]).T

    if t > ax2.get_xlim()[1]:
        ax2.set_xlim(0, t + 5)
    for j in range(4):
        speed_lines[j].set_data(time_vals[:len(filtered_speeds)], filtered_speeds[:, j])

    # Add vertical lines for waypoint reach times (only once per waypoint)
    for waypoint_time in waypoint_reach_times:
        if waypoint_time not in plotted_waypoint_times:
            ax2.axvline(x=waypoint_time, color='red', linestyle='--', label='Waypoint Reached')
            plotted_waypoint_times.add(waypoint_time)

    ax2.relim()
    ax2.autoscale_view()

    fig2.canvas.draw()
    fig2.canvas.flush_events()

ani = animation.FuncAnimation(fig, animate, frames=500, interval=10, blit=False)
plt.show()
