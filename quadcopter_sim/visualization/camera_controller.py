"""Camera controller for 3D scene navigation."""

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *


class CameraController:
    """Handles 3D camera control and mouse interactions."""
    
    def __init__(self):
        self.angle_x, self.angle_y, self.angle_z = 17, 0, 90
        self.zoom = 1.2
        self._dragging = False
        self._last_mouse = (0, 0)
    
    def handle_mouse(self, window):
        """Handle mouse input for camera controls."""
        import glfw
        import imgui
        
        # Check if ImGui wants to capture mouse (cursor is over UI elements)
        io = imgui.get_io()
        if io.want_capture_mouse:
            # ImGui is handling the mouse, don't move camera
            self._dragging = False
            return
        
        # Mouse drag to orbit (left-right controls angle_z, up-down controls angle_x)
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            x, y = glfw.get_cursor_pos(window)
            if not self._dragging:
                self._dragging = True
                self._last_mouse = (x, y)
            else:
                dx = x - self._last_mouse[0]
                dy = y - self._last_mouse[1]
                self.angle_z += dx * 0.3  # left-right controls yaw (z)
                self.angle_x += dy * 0.3  # up-down controls pitch (x)
                self.angle_x = max(-89, min(89, self.angle_x))
                self._last_mouse = (x, y)
        else:
            self._dragging = False
            
        # Mouse wheel to zoom
        def scroll_callback(window, xoffset, yoffset):
            # Only zoom if ImGui doesn't want to capture mouse
            io = imgui.get_io()
            if not io.want_capture_mouse:
                self.zoom *= 0.95 ** yoffset
                self.zoom = max(0.2, min(3.0, self.zoom))
        glfw.set_scroll_callback(window, scroll_callback)
    
    def setup_camera_view(self, center):
        """Set up the 3D camera view based on current angles and zoom."""
        # Camera orbit: calculate camera position in spherical coordinates around the drone
        r = 10 * self.zoom
        pitch = np.radians(self.angle_x)
        yaw = np.radians(self.angle_z)
        cam_x = center[0] + r * np.cos(pitch) * np.sin(yaw)
        cam_y = center[1] + r * np.cos(pitch) * np.cos(yaw)
        cam_z = center[2] + r * np.sin(pitch)
        gluLookAt(cam_x, cam_y, cam_z, center[0], center[1], center[2], 0, 0, 1)
    
    def setup_projection(self, width, height):
        """Set up the OpenGL projection matrix."""
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / float(height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
