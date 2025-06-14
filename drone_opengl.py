import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutInit, glutSolidSphere
import time

# Drone parameters
g, m, dt, L = 9.81, 0.5, 0.02, 0.6
state = np.array([0, 0, 0, 0, 0, 0], dtype=float)
waypoints = [np.array([0, 0, 2]), np.array([2, 0, 2]), np.array([2, 2, 3]), np.array([0, 2, 2]), np.array([0, 0, 0])]

# Generate sub-waypoints
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

# Drone control
startup_rpm = 4000
spinup_step = 100
spinup_done = False

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

def draw_sphere(radius, slices=8, stacks=8):
    for i in range(stacks):
        lat0 = np.pi * (-0.5 + float(i) / stacks)
        z0 = np.sin(lat0)
        zr0 = np.cos(lat0)

        lat1 = np.pi * (-0.5 + float(i + 1) / stacks)
        z1 = np.sin(lat1)
        zr1 = np.cos(lat1)

        glBegin(GL_QUAD_STRIP)
        for j in range(slices + 1):
            lng = 2 * np.pi * float(j) / slices
            x = np.cos(lng)
            y = np.sin(lng)
            glNormal3f(x * zr0, y * zr0, z0)
            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0)
            glNormal3f(x * zr1, y * zr1, z1)
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1)
        glEnd()

def draw_drone(center):
    rotors = rotor_positions(center)
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINE_LOOP)
    for p in rotors:
        glVertex3fv(p)
    glEnd()
    glColor3f(1.0, 0.0, 0.0)
    for p in rotors:
        glPushMatrix()
        glTranslatef(*p)
        draw_sphere(0.05, 10, 10)
        glPopMatrix()
    glColor3f(0.0, 1.0, 0.0)
    glPushMatrix()
    glTranslatef(*center)
    draw_sphere(0.08, 12, 12)
    glPopMatrix()

def draw_waypoints(wps):
    glColor3f(1.0, 1.0, 0.0)
    for wp in wps:
        glPushMatrix()
        glTranslatef(*wp)
        draw_sphere(0.04, 8, 8)
        glPopMatrix()

def main():
    global state, wp_index, spinup_done
    pygame.init()
    display = (900, 700)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glutInit()  # <-- Move here, after OpenGL context is created
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, -1.5, -8)
    glEnable(GL_DEPTH_TEST)
    running = True
    t = 0
    pause_until = None
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # Draw waypoints
        draw_waypoints(waypoints)
        # Draw drone
        draw_drone(state[:3])
        # Camera controls (optional)
        # Update drone state
        if not spinup_done:
            rotor_speeds[:] = np.minimum(rotor_speeds + spinup_step, startup_rpm)
            if np.all(rotor_speeds >= startup_rpm):
                spinup_done = True
        else:
            target = waypoints[wp_index]
            if pause_until is not None:
                if t < pause_until:
                    pass
                else:
                    pause_until = None
            elif np.linalg.norm(state[:3] - target) < 0.2 and wp_index < len(waypoints) - 1:
                wp_index += 1
                pause_until = t + 0.1
            elif wp_index == len(waypoints) - 1 and np.linalg.norm(state[:3] - target) < 0.2 and state[2] <= 0.1:
                rotor_speeds[:] = np.maximum(rotor_speeds - 50, 0)
                if np.all(rotor_speeds == 0):
                    running = False
            else:
                u = control_input(state, target)
                state[:] = update_state(state, u)
        pygame.display.flip()
        pygame.time.wait(int(dt * 1000))
        t += dt
    pygame.quit()

if __name__ == "__main__":
    main()
