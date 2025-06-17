"""3D scene rendering components for the quadcopter simulation."""

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutInit, glutBitmapCharacter, GLUT_BITMAP_HELVETICA_18, GLUT_BITMAP_HELVETICA_12
from ..thrust import Thrust


class SceneRenderer:
    """Handles 3D scene rendering for the quadcopter simulation."""
    
    def __init__(self, sim):
        self.sim = sim
        self.thrust_visual = Thrust(sim.k_thrust, sim.atmosphere_density)
        self._glut_initialized = False
    
    def _ensure_glut_init(self):
        """Ensure GLUT is initialized for text rendering."""
        if not self._glut_initialized:
            glutInit()
            self._glut_initialized = True
    
    def clear_scene(self):
        """Clear the OpenGL buffers."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
    
    def draw_environment(self, environment):
        """Draw the terrain environment."""
        # Always render unscanned terrain as grey wireframe
        glLineWidth(1.5)
        environment.draw(wireframe=True)
        glLineWidth(1.0)  # Restore default
    
    def draw_waypoints_and_trajectory(self, sim):
        """Draw waypoints and drone trajectory."""
        # Draw waypoints as blue points
        glColor3f(0, 0, 1)
        glPointSize(8)
        glBegin(GL_POINTS)
        for wp in sim.waypoints:
            glVertex3f(*wp)
        glEnd()
        
        # Draw trajectory as blue line
        glColor3f(0, 0, 1)
        glBegin(GL_LINE_STRIP)
        for p in sim.trajectory:
            glVertex3f(*p)
        glEnd()
    
    def draw_drone_structure(self, sim):
        """Draw the basic drone structure (arms and rotors)."""
        center = sim.state[:3]
        rotors = sim.rotor_positions()
        
        # Draw arms connecting center to rotors
        glColor3f(0, 0, 0)
        glLineWidth(3)
        glBegin(GL_LINES)
        for i in range(4):
            glVertex3f(*center)
            glVertex3f(*rotors[i])
        glEnd()
        
        # Draw rotor positions as red points
        glColor3f(1, 0, 0)
        glPointSize(12)
        glBegin(GL_POINTS)
        for r in rotors:
            glVertex3f(*r)
        glEnd()
    
    def draw_rotor_blades(self, sim):
        """Draw animated rotor blades."""
        center = sim.state[:3]
        rotors = sim.rotor_positions()
        
        for i, r in enumerate(rotors):
            glPushMatrix()
            glTranslatef(r[0], r[1], r[2])
            
            # Apply drone orientation (roll, pitch, yaw)
            roll, pitch, yaw = np.degrees(sim.state[6:9])
            glRotatef(yaw, 0, 0, 1)
            glRotatef(pitch, 1, 0, 0)
            glRotatef(roll, 0, 1, 0)
            
            # Apply blade rotation around local Z
            glRotatef(sim.blade_angles[i], 0, 0, 1)
            
            # Draw blade
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(4)
            glBegin(GL_LINES)
            glVertex3f(-0.18, 0, 0)
            glVertex3f(0.18, 0, 0)
            glVertex3f(0.18, 0, 0)
            glVertex3f(0, -0.06, 0)
            glVertex3f(0, 0.06, 0)
            glEnd()
            glPopMatrix()
    
    def draw_rotor_numbers(self, sim):
        """Draw rotor numbers above each rotor."""
        self._ensure_glut_init()
        rotors = sim.rotor_positions()
        
        glColor3f(0, 0, 0)
        for i, r in enumerate(rotors):
            # Offset above rotor (z+0.12)
            glRasterPos3f(r[0], r[1], r[2] + 0.12)
            for c in str(i+1):
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(c))
    
    def draw_drone_body_box(self, sim):
        """Draw the drone body box with orientation."""
        from ..drone_body_box import draw_drone_body_box
        
        center = sim.state[:3]
        roll, pitch, yaw = np.degrees(sim.state[6:9])
        draw_drone_body_box(roll, pitch, yaw, center)
    
    def draw_body_corner_numbers(self, sim, environment):
        """Draw corner numbers for the drone body box."""
        from ..drone_body_box import get_body_box_corners
        
        self._ensure_glut_init()
        
        # Get corners
        roll_rad, pitch_rad, yaw_rad = sim.state[6:9]
        center = sim.state[:3]
        size = (0.8, 0.8, 0.2)
        corners = get_body_box_corners(roll_rad, pitch_rad, yaw_rad, center, size)
        
        glColor3f(0, 0, 0)
        for i, (x, y, z) in enumerate(corners):
            glRasterPos3f(x, y, z + 0.05)  # Slightly above the corner
            for c in str(i+1):
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, ord(c))
    
    def draw_thrust_arrows(self, sim):
        """Draw thrust visualization arrows for each rotor."""
        rotors = sim.rotor_positions()
        
        Thrust.draw_thrust_arrows(
            rotors,
            sim.rotor_speeds,
            min_rpm=0,
            max_rpm=12000,
            thrust_coefficient=self.thrust_visual.thrust_coefficient,
            atmosphere_density=self.thrust_visual.atmosphere_density
        )
    
    def draw_camera_fov(self, sim, environment):
        """Draw camera field of view visualization."""
        pos = sim.state[:3].copy()
        pos[2] -= 0.2
        fov = 60
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * pos[2]
        
        corners = [
            [pos[0] - size, pos[1] - size, environment.contour_height(pos[0] - size, pos[1] - size)],
            [pos[0] + size, pos[1] - size, environment.contour_height(pos[0] + size, pos[1] - size)],
            [pos[0] + size, pos[1] + size, environment.contour_height(pos[0] + size, pos[1] + size)],
            [pos[0] - size, pos[1] + size, environment.contour_height(pos[0] - size, pos[1] + size)],
        ]
        
        glColor3f(0.2, 0.8, 1.0)
        glLineWidth(2)
        glBegin(GL_LINE_LOOP)
        for c in corners:
            glVertex3f(*c)
        glEnd()
    
    def draw_lookahead_target(self, sim):
        """Draw the lookahead target for path following."""
        try:
            from ..main_trajectory import get_lookahead_target
            lookahead_target = get_lookahead_target(sim.state[:3], sim.waypoints, lookahead_dist=2.0)
            glColor3f(1, 0, 1)  # Magenta for lookahead
            glPointSize(18)
            glBegin(GL_POINTS)
            glVertex3f(*lookahead_target)
            glEnd()
        except Exception:
            pass  # If import fails, skip lookahead marker
    
    def draw_camera_chunks(self, camera_chunks):
        """Render accumulated camera chunks as wireframe grids."""
        for img, pos in camera_chunks:
            res = img.shape[0]
            fov = 60
            half_fov = np.radians(fov / 2)
            size = np.tan(half_fov) * (pos[2] - 0.2)
            xs = np.linspace(pos[0] - size, pos[0] + size, res)
            ys = np.linspace(pos[1] - size, pos[1] + size, res)
            
            glColor3f(0.2, 0.8, 0.2)
            glLineWidth(1.2)
            
            # Draw horizontal grid lines
            for i in range(res):
                glBegin(GL_LINE_STRIP)
                for j in range(res):
                    if not np.isnan(img[i, j]):  # Skip NaN values from masking
                        glVertex3f(xs[j], ys[i], img[i, j])
                glEnd()
            
            # Draw vertical grid lines
            for j in range(res):
                glBegin(GL_LINE_STRIP)
                for i in range(res):
                    if not np.isnan(img[i, j]):  # Skip NaN values from masking
                        glVertex3f(xs[j], ys[i], img[i, j])
                glEnd()
