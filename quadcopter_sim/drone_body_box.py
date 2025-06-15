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
