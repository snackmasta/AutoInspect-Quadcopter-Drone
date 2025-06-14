import numpy as np
import sys
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import threading
import tkinter as tk
from queue import SimpleQueue

# Simulation constants
g, m, dt, L = 9.81, 0.5, 0.02, 0.6
state = np.array([0, 0, 0, 0, 0, 0], dtype=float)
waypoints = [np.array([0, 0, 2]), np.array([2, 0, 2]), np.array([2, 2, 3]), np.array([0, 2, 2]), np.array([0, 0, 0])]

# Generate sub-waypoints (6 stops per segment, including start and end)
all_waypoints = []
for i in range(len(waypoints) - 1):
    for j in range(6):
        t = j / 5
        sub_wp = waypoints[i] * (1 - t) + waypoints[i + 1] * t
        all_waypoints.append(sub_wp)
all_waypoints = [all_waypoints[0]] + [all_waypoints[i] for i in range(1, len(all_waypoints)) if not np.allclose(all_waypoints[i], all_waypoints[i-1])]
waypoints = all_waypoints
wp_index = 0
rotor_speeds = np.array([0.0] * 4)
speed_history, time_vals = [], []

startup_rpm = 4000
spinup_step = 100
spinup_done = False

trajectory = []

# OpenGL state
window_width, window_height = 1200, 800
# Set a top-down, slightly tilted camera
angle_x, angle_y, angle_z = -73, 0, 17  # Set default camera angles to match the screenshot
zoom = 1.2

# Thread-safe queue for redisplay requests
redisplay_queue = SimpleQueue()

# Control and update functions (copied from original)
def control_input(state, target):
    pos, vel = state[:3], state[3:]
    kp, kd = 6.0, 4.0
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

def draw_text(x, y, text, font=GLUT_BITMAP_HELVETICA_18):
    glWindowPos2f(x, y)
    for ch in text:
        glutBitmapCharacter(font, ord(ch))

def draw_ground_grid(size=3, step=0.5):
    glColor3f(0.7, 0.7, 0.7)
    glLineWidth(1)
    glBegin(GL_LINES)
    for x in np.arange(-1, size+step, step):
        glVertex3f(x, -1, 0)
        glVertex3f(x, size, 0)
    for y in np.arange(-1, size+step, step):
        glVertex3f(-1, y, 0)
        glVertex3f(size, y, 0)
    glEnd()

def draw_scene():
    global state, wp_index, rotor_speeds, spinup_done, trajectory
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslatef(0, 0, -10 * zoom)
    glRotatef(angle_x, 1, 0, 0)
    glRotatef(angle_y, 0, 1, 0)
    glRotatef(angle_z, 0, 0, 1)  # Add Z rotation

    # Draw ground grid for reference
    draw_ground_grid()

    # Draw waypoints
    glColor3f(1, 1, 0)
    glPointSize(8)
    glBegin(GL_POINTS)
    for wp in waypoints:
        glVertex3f(*wp)
    glEnd()

    # Draw trajectory
    glColor3f(0, 0, 1)
    glBegin(GL_LINE_STRIP)
    for p in trajectory:
        glVertex3f(*p)
    glEnd()

    # Draw quadcopter arms
    center = state[:3]
    rotors = rotor_positions(center)
    glColor3f(0, 0, 0)
    glLineWidth(3)
    glBegin(GL_LINES)
    for i in range(4):
        glVertex3f(*center)
        glVertex3f(*rotors[i])
    glEnd()

    # Draw rotors
    glColor3f(1, 0, 0)
    glPointSize(12)
    glBegin(GL_POINTS)
    for r in rotors:
        glVertex3f(*r)
    glEnd()

    # Draw current target
    glColor3f(0, 1, 0)
    glPointSize(14)
    glBegin(GL_POINTS)
    glVertex3f(*waypoints[wp_index])
    glEnd()

    glutSwapBuffers()

def update(value):
    global state, wp_index, rotor_speeds, spinup_done, trajectory
    # Check for redisplay requests from Tkinter
    while not redisplay_queue.empty():
        redisplay_queue.get()
        glutPostRedisplay()
    if not spinup_done:
        rotor_speeds[:] = np.minimum(rotor_speeds + spinup_step, startup_rpm)
        if np.all(rotor_speeds >= startup_rpm):
            spinup_done = True
    else:
        target = waypoints[wp_index]
        if np.linalg.norm(state[:3] - target) < 0.2 and wp_index < len(waypoints) - 1:
            wp_index += 1
        else:
            u = control_input(state, target)
            state[:] = update_state(state, u)
            trajectory.append(state[:3].copy())
    glutPostRedisplay()
    glutTimerFunc(int(dt * 1000), update, 0)

def reshape(width, height):
    global window_width, window_height
    window_width, window_height = width, height
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width / float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

def keyboard(key, x, y):
    global zoom
    if key == b'+':
        zoom /= 1.1
    elif key == b'-':
        zoom *= 1.1
    elif key == b'q':
        sys.exit()
    glutPostRedisplay()

def special_keys(key, x, y):
    global angle_x, angle_y
    if key == GLUT_KEY_LEFT:
        angle_y -= 5
    elif key == GLUT_KEY_RIGHT:
        angle_y += 5
    elif key == GLUT_KEY_UP:
        angle_x -= 5
    elif key == GLUT_KEY_DOWN:
        angle_x += 5
    glutPostRedisplay()

def start_tkinter_controls():
    def on_angle_x(val):
        global angle_x
        angle_x = float(val)
        redisplay_queue.put(1)

    def on_angle_y(val):
        global angle_y
        angle_y = float(val)
        redisplay_queue.put(1)

    def on_angle_z(val):
        global angle_z
        angle_z = float(val)
        redisplay_queue.put(1)

    root = tk.Tk()
    root.title("Camera Angle Controls")
    tk.Label(root, text="Angle X (tilt)").pack()
    sx = tk.Scale(root, from_=-90, to=90, orient=tk.HORIZONTAL, length=300, command=on_angle_x)
    sx.set(angle_x)
    sx.pack()
    tk.Label(root, text="Angle Y (pan)").pack()
    sy = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=on_angle_y)
    sy.set(angle_y)
    sy.pack()
    tk.Label(root, text="Angle Z (roll)").pack()
    sz = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=on_angle_z)
    sz.set(angle_z)
    sz.pack()
    root.mainloop()

# --- Tkinter drone topdown UI ---
# Add RPM history buffer for trend graph
rpm_history_len = 100
rpm_history = [ [0]*rpm_history_len for _ in range(4) ]

def start_drone_topdown_ui():
    import math
    def rpm_color(val):
        if val < 3500:
            return 'yellow'
        elif val > 5500:
            return 'red'
        else:
            return 'green'
    def update_canvas():
        canvas.delete('all')
        # Draw main border and title
        canvas.create_rectangle(10, 10, 230, 200, outline='black', width=3)
        canvas.create_text(120, 20, text='DRONE SCADA HMI', font=('Arial', 12, 'bold'))
        cx, cy = 120, 100
        arm = 60
        # Draw arms
        for angle in [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]:
            x2 = cx + arm * math.cos(angle)
            y2 = cy + arm * math.sin(angle)
            canvas.create_line(cx, cy, x2, y2, width=6, fill='#222')
        # Draw rotors, RPM, and digital readout
        for i, angle in enumerate([math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]):
            x = cx + arm * math.cos(angle)
            y = cy + arm * math.sin(angle)
            color = rpm_color(rotor_speeds[i])
            canvas.create_oval(x-18, y-18, x+18, y+18, fill=color, outline='black', width=2)
            canvas.create_text(x, y, text=f"{int(rotor_speeds[i])}", font=('Arial', 11, 'bold'), fill='white')
            canvas.create_text(x, y+25, text=f"RPM", font=('Arial', 8, 'bold'), fill='black')
        # Update RPM history
        for i in range(4):
            rpm_history[i].append(rotor_speeds[i])
            if len(rpm_history[i]) > rpm_history_len:
                rpm_history[i].pop(0)
        # Draw trend graphs
        graph_w, graph_h = 80, 40
        for i in range(4):
            gx = 20 + (i%2)*110
            gy = 210 + (i//2)*60
            canvas.create_rectangle(gx, gy, gx+graph_w, gy+graph_h, outline='gray')
            if len(rpm_history[i]) > 1:
                min_rpm = min(rpm_history[i])
                max_rpm = max(rpm_history[i])
                rng = max(1, max_rpm - min_rpm)
                points = []
                for j, rpm in enumerate(rpm_history[i]):
                    px = gx + j * graph_w / (rpm_history_len-1)
                    py = gy + graph_h - ((rpm-min_rpm)/rng)*graph_h
                    points.append((px, py))
                for j in range(1, len(points)):
                    canvas.create_line(points[j-1][0], points[j-1][1], points[j][0], points[j][1], fill='blue', width=2)
            canvas.create_text(gx+graph_w/2, gy+graph_h+8, text=f"Rotor {i+1} Trend", font=('Arial', 8))
        # Status bar
        canvas.create_rectangle(0, 280, 240, 300, fill='#222', outline='black')
        canvas.create_text(120, 290, text='System: RUNNING', font=('Arial', 10, 'bold'), fill='white')
        root.after(100, update_canvas)
    root = tk.Tk()
    root.title("Drone SCADA HMI")
    canvas = tk.Canvas(root, width=240, height=300, bg='white')
    canvas.pack()
    update_canvas()
    root.mainloop()

def main():
    # Start Tkinter controls in a separate thread
    threading.Thread(target=start_tkinter_controls, daemon=True).start()
    # Start drone topdown UI in a separate thread
    threading.Thread(target=start_drone_topdown_ui, daemon=True).start()

    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(window_width, window_height)
    glutCreateWindow(b"Quadcopter PyOpenGL Simulation")
    glEnable(GL_DEPTH_TEST)
    glClearColor(1, 1, 1, 1)
    glutDisplayFunc(draw_scene)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutSpecialFunc(special_keys)
    glutTimerFunc(10, update, 0)
    glutMainLoop()

if __name__ == "__main__":
    main()
