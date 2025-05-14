import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
g, m, dt, L = 9.81, 0.5, 0.02, 0.6
state = np.array([0, 0, 0, 0, 0, 0], dtype=float)
waypoints = [np.array([0, 0, 2]), np.array([2, 0, 2]), np.array([2, 2, 3]), np.array([0, 2, 2]), np.array([0, 0, 0])]
wp_index = 0
rotor_speeds = np.array([4000.0] * 4)
speed_history, time_vals = [], []

def control_input(state, target):
    pos, vel = state[:3], state[3:]
    kp, kd = 12.0, 8.0
    acc_des = kp * (target - pos) - kd * vel
    acc_des[2] += g
    thrust_total = m * acc_des[2]
    base_speed = np.clip(thrust_total * 1000, 3000, 6000)
    rotor_speeds[:] = base_speed + np.random.randn(4) * 100
    return m * acc_des

def update_state(state, u):
    pos, vel = state[:3], state[3:]
    acc = u / m
    acc[2] -= g
    vel += acc * dt
    pos += vel * dt
    return np.hstack((pos, vel))

def rotor_positions(center):
    offsets = np.array([[-L/2, L/2, 0], [L/2, L/2, 0], [L/2, -L/2, 0], [-L/2, -L/2, 0]])
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
target_dot = ax.scatter([], [], [], c='orange', marker='x', s=80)
rpm_texts = [ax.text(0, 0, 0, '') for _ in range(4)]

# --- Rotor Speed Plot ---
fig2, ax2 = plt.subplots()
speed_lines = [ax2.plot([], [], label=f'Rotor {i+1}')[0] for i in range(4)]
ax2.set_xlim(0, 10)
ax2.set_ylim(3000, 7000)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("RPM")
ax2.set_title("Rotor Speeds Over Time")
ax2.legend()

def animate(i):
    global state, wp_index, rotor_speeds

    t = i * dt
    target = waypoints[wp_index]

    # Check if the drone is near the last waypoint and on the ground
    if wp_index == len(waypoints) - 1 and np.linalg.norm(state[:3] - target) < 0.2 and state[2] <= 0.1:
        rotor_speeds[:] = 0  # Stop the rotors

        # Update rotor speed text to show 0 RPM
        for j in range(4):
            rpm_texts[j].set_text(f'{rotor_speeds[j]:.0f} RPM')

        return  # Stop updating the state

    u = control_input(state, target)
    state[:] = update_state(state, u)
    trajectory.append(state[:3].copy())

    if np.linalg.norm(state[:3] - target) < 0.2 and wp_index < len(waypoints) - 1:
        wp_index += 1

    # --- Update 3D elements ---
    traj = np.array(trajectory)
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

    rotor_dots._offsets3d = (rotors[:, 0], rotors[:, 1], rotors[:, 2])
    center_dot._offsets3d = ([center[0]], [center[1]], [center[2]])
    target_dot._offsets3d = ([target[0]], [target[1]], [target[2]])

    # --- Rotor Speeds ---
    time_vals.append(t)
    speed_history.append(rotor_speeds.copy())
    speeds = np.array(speed_history)
    if t > ax2.get_xlim()[1]:
        ax2.set_xlim(0, t + 5)
    for j in range(4):
        speed_lines[j].set_data(time_vals, speeds[:, j])
    ax2.relim()
    ax2.autoscale_view()

    fig2.canvas.draw()
    fig2.canvas.flush_events()

ani = animation.FuncAnimation(fig, animate, frames=500, interval=10, blit=False)
plt.show()
