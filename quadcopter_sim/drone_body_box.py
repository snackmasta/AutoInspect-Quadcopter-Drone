import numpy as np
from OpenGL.GL import *

def draw_drone_body_box(roll, pitch, yaw, center, size=(0.8, 0.8, 0.2)):
    """
    Draws a blue wireframe box representing the drone body, oriented by roll (deg, X), pitch (deg, Y), yaw (deg, Z), centered at center (x, y, z).
    size: (width, depth, height)
    """
    w, d, h = size
    glPushMatrix()
    glTranslatef(center[0], center[1], center[2])
    # Apply yaw (Z), then pitch (Y), then roll (X)
    glRotatef(yaw, 0, 0, 1)    # Yaw (Z)
    glRotatef(pitch, 0, 1, 0)  # Pitch (Y)
    glRotatef(roll, 1, 0, 0)   # Roll (X)
    glColor3f(0, 0, 1)  # Blue
    glLineWidth(2)
    # Vertices of the box
    x0, x1 = -w/2, w/2
    y0, y1 = -d/2, d/2
    z0, z1 = -h/2, h/2
    # Bottom face
    glBegin(GL_LINE_LOOP)
    glVertex3f(x0, y0, z0)
    glVertex3f(x1, y0, z0)
    glVertex3f(x1, y1, z0)
    glVertex3f(x0, y1, z0)
    glEnd()
    # Top face
    glBegin(GL_LINE_LOOP)
    glVertex3f(x0, y0, z1)
    glVertex3f(x1, y0, z1)
    glVertex3f(x1, y1, z1)
    glVertex3f(x0, y1, z1)
    glEnd()
    # Vertical edges
    glBegin(GL_LINES)
    glVertex3f(x0, y0, z0); glVertex3f(x0, y0, z1)
    glVertex3f(x1, y0, z0); glVertex3f(x1, y0, z1)
    glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1)
    glVertex3f(x0, y1, z0); glVertex3f(x0, y1, z1)
    glEnd()
    glPopMatrix()

def get_body_box_corners(roll, pitch, yaw, center, size=(0.8, 0.8, 0.2)):
    """
    Returns the 8 world coordinates of the drone's main body box corners.
    The box matches the one drawn in draw_drone_body_box.
    Args:
        roll, pitch, yaw: orientation in radians
        center: (x, y, z) position of the box center
        size: (width, depth, height)
    Returns:
        corners: (8, 3) array of world coordinates
    """
    w, d, h = size
    # Box corners in local body frame
    corners = np.array([
        [-w/2, -d/2, -h/2],
        [ w/2, -d/2, -h/2],
        [ w/2,  d/2, -h/2],
        [-w/2,  d/2, -h/2],
        [-w/2, -d/2,  h/2],
        [ w/2, -d/2,  h/2],
        [ w/2,  d/2,  h/2],
        [-w/2,  d/2,  h/2],
    ])
    # Rotation matrix (same as in draw_drone_body_box, but using radians)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    world_corners = (R @ corners.T).T + np.array(center)
    return world_corners

def body_box_terrain_penetration(roll, pitch, yaw, center, environment, size=(0.8, 0.8, 0.2)):
    """
    Returns the maximum penetration depth of the body box below the terrain.
    If > 0, the box is colliding with the terrain.
    """
    corners = get_body_box_corners(roll, pitch, yaw, center, size)
    ground_heights = np.array([environment.contour_height(x, y) for x, y, _ in corners])
    penetrations = ground_heights - corners[:, 2]
    max_penetration = np.max(penetrations)
    return max_penetration

def body_box_terrain_forces(roll, pitch, yaw, center, environment, m, g, vx, vy, wx, wy, wz, size=(0.8, 0.8, 0.2), penetration_tol=0.01, velocity_deadzone=0.5):
    """
    Compute normal force and torque from terrain collision for the body box.
    Returns (force, torque) in world frame.
    Implements a hard, non-penetrable ground: forcibly corrects any penetration by projecting the drone body up to the ground surface.
    """
    corners = get_body_box_corners(roll, pitch, yaw, center, size)
    valid_corners = [c for c in corners if np.all(np.isfinite(c))]
    if len(valid_corners) < len(corners):
        return np.zeros(3), np.zeros(3)
    ground_heights = np.array([environment.contour_height(x, y) for x, y, _ in valid_corners])
    penetrations = ground_heights - np.array([c[2] for c in valid_corners])
    max_penetration = np.max(penetrations)
    # If any part is below ground, forcibly move the drone up
    if max_penetration > penetration_tol:
        # Project the center up by the maximum penetration
        center[2] += max_penetration + penetration_tol
        # Zero vertical velocity and angular velocity
        vz = 0
        wx = 0
        wy = 0
        wz = 0
        # Apply only gravity support force
        force = np.array([0, 0, m * g])
        torque = np.zeros(3)
        return force, torque
    # Otherwise, no correction needed
    return np.zeros(3), np.zeros(3)
