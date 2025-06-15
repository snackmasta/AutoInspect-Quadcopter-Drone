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
    Includes damping, penetration tolerance, and velocity deadzone to prevent jitter.
    """
    corners = get_body_box_corners(roll, pitch, yaw, center, size)
    # Filter out corners with NaN or inf values
    valid_corners = [c for c in corners if np.all(np.isfinite(c))]
    if len(valid_corners) < len(corners):
        # If any corner is invalid, skip terrain force (return zero force/torque)
        return np.zeros(3), np.zeros(3)
    ground_heights = np.array([environment.contour_height(x, y) for x, y, _ in valid_corners])
    penetrations = ground_heights - np.array([c[2] for c in valid_corners])
    force = np.zeros(3)
    torque = np.zeros(3)
    k_ground = 150  # N/m, spring constant
    k_friction = 10.0   # friction coefficient
    d_ground = 10.0    # Damping coefficient for normal direction
    spring_scale = 0.5  # Scale down the spring force output
    for i, penetration in enumerate(penetrations):
        if penetration > penetration_tol:
            r = corners[i] - np.array(center)
            v_contact = np.array([vx, vy, 0]) + np.cross([wx, wy, wz], r)
            v_n = v_contact[2]
            # Improved deadzone for vertical velocity
            if abs(v_n) < velocity_deadzone:
                v_n = 0.0
            # If penetration is significant and v_n is very small, clamp to zero to avoid jitter
            if abs(v_n) < velocity_deadzone and penetration > 2 * penetration_tol:
                v_n = 0.0
            f_n = np.array([0, 0, spring_scale * (k_ground * (penetration - penetration_tol) - d_ground * v_n)])
            # Friction force (opposes velocity at contact point)
            v_xy = v_contact[:2]
            if np.linalg.norm(v_xy) < velocity_deadzone:
                v_xy = np.zeros(2)
            f_friction = -k_friction * v_xy
            f_friction = np.append(f_friction, 0)
            f_total = f_n + f_friction
            # Clamp very small forces to zero
            if np.linalg.norm(f_total) < 1e-3:
                f_total = np.zeros(3)
            force += f_total
            torque += np.cross(r, f_total)
    return force, torque
