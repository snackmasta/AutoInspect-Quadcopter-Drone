import numpy as np
import sys
from OpenGL.GL import *
from OpenGL.GLU import *
from queue import SimpleQueue
import imgui
from imgui.integrations.opengl import ProgrammablePipelineRenderer
import glfw
from imgui.integrations.glfw import GlfwRenderer

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

# For spinning blades
blade_angles = [0.0, 0.0, 0.0, 0.0]

imgui_renderer = None  # Will hold the ImGui OpenGL renderer

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
    global state, wp_index, rotor_speeds, spinup_done, trajectory, blade_angles, angle_x, angle_y, angle_z, zoom
    io = imgui.get_io()
    io.display_size = (window_width, window_height)
    # Start new ImGui frame is now handled in main loop

    # ImGui window for camera controls
    imgui.begin("Camera Controls")
    changed_x, angle_x = imgui.slider_float("Angle X", angle_x, -90.0, 90.0)
    changed_y, angle_y = imgui.slider_float("Angle Y", angle_y, -180.0, 180.0)
    changed_z, angle_z = imgui.slider_float("Angle Z", angle_z, -180.0, 180.0)
    changed_zoom, zoom = imgui.slider_float("Zoom", zoom, 0.2, 3.0)
    imgui.end()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslatef(0, 0, -10 * zoom)
    glRotatef(angle_x, 1, 0, 0)
    glRotatef(angle_y, 0, 1, 0)
    glRotatef(angle_z, 0, 0, 1)

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
    # Draw rotors (hubs)
    glColor3f(1, 0, 0)
    glPointSize(12)
    glBegin(GL_POINTS)
    for r in rotors:
        glVertex3f(*r)
    glEnd()
    # Draw spinning blades
    for i, r in enumerate(rotors):
        glPushMatrix()
        glTranslatef(r[0], r[1], r[2])
        glRotatef(blade_angles[i], 0, 0, 1)
        glColor3f(0.2, 0.2, 0.2)
        glLineWidth(4)
        glBegin(GL_LINES)
        glVertex3f(-0.18, 0, 0)
        glVertex3f(0.18, 0, 0)
        glVertex3f(0, -0.06, 0)
        glVertex3f(0, 0.06, 0)
        glEnd()
        glPopMatrix()

    # Draw current target
    glColor3f(0, 1, 0)
    glPointSize(14)
    glBegin(GL_POINTS)
    glVertex3f(*waypoints[wp_index])
    glEnd()

    # No glutSwapBuffers needed, GLFW handles this in main loop

def update(_):
    global state, wp_index, rotor_speeds, spinup_done, trajectory, blade_angles
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
    # Update blade angles based on RPM
    for i in range(4):
        blade_angles[i] = (blade_angles[i] + rotor_speeds[i] * 360.0 / 60.0 * dt) % 360

def reshape(width, height):
    global window_width, window_height
    window_width, window_height = width, height
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width / float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

def main():
    global window_width, window_height, angle_x, angle_y, angle_z, zoom
    if not glfw.init():
        print("Could not initialize GLFW")
        return
    window = glfw.create_window(window_width, window_height, "Quadcopter PyOpenGL Simulation", None, None)
    if not window:
        glfw.terminate()
        print("Could not create GLFW window")
        return
    glfw.make_context_current(window)
    glEnable(GL_DEPTH_TEST)
    glClearColor(1, 1, 1, 1)
    imgui.create_context()
    impl = GlfwRenderer(window)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        width, height = glfw.get_framebuffer_size(window)
        window_width, window_height = width, height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / float(height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        imgui.new_frame()  # Start new ImGui frame here
        update(0)  # Call update logic
        draw_scene()
        imgui.render()  # End ImGui frame here
        draw_data = imgui.get_draw_data()
        if draw_data is not None:
            impl.render(draw_data)
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    main()
