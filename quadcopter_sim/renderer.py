import os
# Force use of high-performance GPU on NVIDIA/AMD systems (must be set before OpenGL context creation)
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
os.environ["QT_DEVICE_PIXEL_RATIO"] = "auto"
# For NVIDIA Optimus (laptops):
os.environ["CUDA_FORCE_PTX_JIT"] = "1"
os.environ["NVIDIA_OPTIMUS_ENABLE_NVML"] = "1"
os.environ["NV_OPTIMUS_ENABLE"] = "1"
# For AMD switchable graphics:
os.environ["AMD_POWERXPRESS_REQUEST_HIGH_PERFORMANCE"] = "1"

import numpy as np
from OpenGL.GL import *
from OpenGL.arrays import vbo
from OpenGL.GLU import *
import imgui
from quadcopter_sim.environment import Environment

class Renderer:
    def __init__(self, sim):
        self.sim = sim
        self.window_width, self.window_height = 1200, 800
        self.angle_x, self.angle_y, self.angle_z = -73, 0, 17
        self.zoom = 1.2
        # Camera interaction state
        self._dragging = False
        self._last_mouse = (0, 0)
        self.environment = Environment(size=3, step=0.5)
        self.show_lidar = False  # Toggle for LiDAR visualization (default OFF)
        self.scanned_points = []  # Accumulated scanned terrain points
        self.vbo = None  # Vertex Buffer Object for scanned points
        self.vbo_needs_update = True
        self.show_camera = False  # Camera visualization toggle, default off

    def handle_mouse(self, window):
        import glfw
        # Mouse drag to orbit (now left-right controls angle_z, up-down controls angle_x)
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
            self.zoom *= 0.95 ** yoffset
            self.zoom = max(0.2, min(3.0, self.zoom))
        glfw.set_scroll_callback(window, scroll_callback)

    def draw_ground_grid(self, size=3, step=0.5):
        self.environment.draw()

    def draw_scene(self):
        sim = self.sim
        io = imgui.get_io()
        io.display_size = (self.window_width, self.window_height)
        imgui.set_next_window_position(10, 10)
        imgui.set_next_window_size(350, 500)
        imgui.begin("Mission Control", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        if imgui.button("Pause"): pass
        imgui.same_line()
        if imgui.button("Resume"): pass
        imgui.same_line()
        if imgui.button("Reset"): sim.reset()
        imgui.separator()
        imgui.text("Telemetry")
        pos, vel = sim.state[:3], sim.state[3:]
        imgui.text(f"Position: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
        imgui.text(f"Velocity: x={vel[0]:.2f}, y={vel[1]:.2f}, z={vel[2]:.2f}")
        imgui.text(f"Current Waypoint: {sim.wp_index+1}/{len(sim.waypoints)}")
        imgui.text(f"Rotor Speeds: {', '.join(f'{rpm:.0f}' for rpm in sim.rotor_speeds)} RPM")
        # Additional situational awareness telemetry
        distance_to_wp = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        imgui.text(f"Distance to Waypoint: {distance_to_wp:.2f} m")
        if sim.wp_index < len(sim.waypoints) - 1:
            next_wp = sim.waypoints[sim.wp_index + 1]
            imgui.text(f"Next Waypoint: x={next_wp[0]:.2f}, y={next_wp[1]:.2f}, z={next_wp[2]:.2f}")
        imgui.text(f"Spinup Done: {sim.spinup_done}")
        imgui.text(f"Trajectory Points: {len(sim.trajectory)}")
        imgui.text(f"Altitude: {pos[2]:.2f} m")
        imgui.text(f"Ground Speed: {np.linalg.norm(vel[:2]):.2f} m/s")
        imgui.text(f"Vertical Speed: {vel[2]:.2f} m/s")
        imgui.separator()
        imgui.text("Camera Controls")
        _, self.angle_x = imgui.slider_float("Angle X", self.angle_x, -90.0, 90.0)
        _, self.angle_y = imgui.slider_float("Angle Y", self.angle_y, -180.0, 180.0)
        _, self.angle_z = imgui.slider_float("Angle Z", self.angle_z, -180.0, 180.0)
        _, self.zoom = imgui.slider_float("Zoom", self.zoom, 0.2, 3.0)
        imgui.separator()
        # Manual control toggle and sliders
        changed, self.sim.manual_mode = imgui.checkbox("Manual Mode", self.sim.manual_mode)
        if self.sim.manual_mode:
            imgui.text("Manual Propeller RPM Control")
            for i in range(4):
                _, rpm = imgui.slider_float(f"Propeller {i+1} RPM", float(self.sim.manual_rpms[i]), self.sim.min_rpm, self.sim.max_rpm)
                self.sim.manual_rpms[i] = rpm
        imgui.separator()
        if len(sim.trajectory) > 1:
            altitudes = np.array([p[2] for p in sim.trajectory[-100:]], dtype=np.float32)
            imgui.plot_lines("Altitude (last 100 steps)", altitudes, graph_size=(300, 80))
        imgui.end()
        # Visual, at-a-glance situational awareness panel
        imgui.set_next_window_position(self.window_width - 370, 10)
        imgui.set_next_window_size(360, 0)
        imgui.push_style_var(imgui.STYLE_WINDOW_ROUNDING, 8)
        imgui.push_style_var(imgui.STYLE_WINDOW_BORDERSIZE, 1)
        imgui.push_style_var(imgui.STYLE_WINDOW_PADDING, (10, 8))
        imgui.push_style_var(imgui.STYLE_ALPHA, 0.92)
        imgui.begin("Situational Awareness", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_ALWAYS_AUTO_RESIZE | imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_NO_TITLE_BAR)
        pos, vel = sim.state[:3], sim.state[3:]
        # Altitude gauge
        imgui.text("Altitude")
        imgui.progress_bar(pos[2] / 5.0, size=(320, 18), overlay=f"{pos[2]:.2f} m")
        # Ground speed gauge
        imgui.text("Ground Speed")
        ground_speed = np.linalg.norm(vel[:2])
        imgui.progress_bar(min(ground_speed / 5.0, 1.0), size=(320, 18), overlay=f"{ground_speed:.2f} m/s")
        # Vertical speed bar (centered at 0)
        imgui.text("Vertical Speed")
        vs = vel[2]
        imgui.progress_bar((vs + 2.5) / 5.0, size=(320, 18), overlay=f"{vs:+.2f} m/s")
        # Distance to waypoint as a radial progress
        imgui.text("Distance to Waypoint")
        dist = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        imgui.progress_bar(min(dist / 5.0, 1.0), size=(320, 18), overlay=f"{dist:.2f} m")
        # Rotor RPM as mini bar graphs
        imgui.text("Rotor RPM")
        for i, rpm in enumerate(sim.rotor_speeds):
            imgui.progress_bar(min(rpm / 6000.0, 1.0), size=(75, 12), overlay=f"{int(rpm)}")
            if i < 3:
                imgui.same_line()
        imgui.new_line()
        # Waypoint progress as a horizontal bar
        imgui.text("Mission Progress")
        imgui.progress_bar((sim.wp_index + 1) / len(sim.waypoints), size=(320, 18), overlay=f"{sim.wp_index+1}/{len(sim.waypoints)}")
        # Altitude plot at the bottom
        if len(sim.trajectory) > 1:
            altitudes = np.array([p[2] for p in sim.trajectory[-100:]], dtype=np.float32)
            imgui.text("Recent Altitude")
            imgui.plot_lines("", altitudes, graph_size=(320, 60))
        imgui.pop_style_var(4)
        imgui.end()
        # 3D Scene
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # Camera orbit: calculate camera position in spherical coordinates around the drone
        center = sim.state[:3]
        r = 10 * self.zoom
        pitch = np.radians(self.angle_x)
        yaw = np.radians(self.angle_z)
        cam_x = center[0] + r * np.cos(pitch) * np.sin(yaw)
        cam_y = center[1] + r * np.cos(pitch) * np.cos(yaw)
        cam_z = center[2] + r * np.sin(pitch)
        gluLookAt(cam_x, cam_y, cam_z, center[0], center[1], center[2], 0, 0, 1)
        self.draw_ground_grid()
        glColor3f(1, 1, 0)
        glPointSize(8)
        glBegin(GL_POINTS)
        for wp in sim.waypoints:
            glVertex3f(*wp)
        glEnd()
        glColor3f(0, 0, 1)
        glBegin(GL_LINE_STRIP)
        for p in sim.trajectory:
            glVertex3f(*p)
        glEnd()
        center = sim.state[:3]
        rotors = sim.rotor_positions()
        glColor3f(0, 0, 0)
        glLineWidth(3)
        glBegin(GL_LINES)
        for i in range(4):
            glVertex3f(*center)
            glVertex3f(*rotors[i])
        glEnd()
        glColor3f(1, 0, 0)
        glPointSize(12)
        glBegin(GL_POINTS)
        for r in rotors:
            glVertex3f(*r)
        glEnd()
        for i, r in enumerate(rotors):
            glPushMatrix()
            glTranslatef(r[0], r[1], r[2])
            glRotatef(sim.blade_angles[i], 0, 0, 1)
            glColor3f(0.2, 0.2, 0.2)
            glLineWidth(4)
            glBegin(GL_LINES)
            glVertex3f(-0.18, 0, 0)
            glVertex3f(0.18, 0, 0)
            glVertex3f(0, -0.06, 0)
            glVertex3f(0, 0.06, 0)
            glEnd()
            glPopMatrix()
        glColor3f(0, 1, 0)
        glPointSize(14)
        glBegin(GL_POINTS)
        glVertex3f(*sim.waypoints[sim.wp_index])
        glEnd()
        # Draw drone feet
        feet = sim.feet_positions()
        glColor3f(0.4, 0.2, 0.1)  # brown feet
        glPointSize(10)
        glBegin(GL_POINTS)
        for f in feet:
            glVertex3f(*f)
        glEnd()
        # Draw hitboxes (as circles in XY plane)
        for f in feet:
            glPushMatrix()
            glTranslatef(f[0], f[1], f[2])
            glColor3f(1, 0, 1)
            glBegin(GL_LINE_LOOP)
            for i in range(20):
                theta = 2 * np.pi * i / 20
                x = 0.06 * np.cos(theta)
                y = 0.06 * np.sin(theta)
                glVertex3f(x, y, 0)
            glEnd()
            glPopMatrix()
        # Draw drone sensors (camera FOV and LiDAR rays)
        sim = self.sim
        # Camera FOV visualization
        pos = sim.state[:3].copy()
        pos[2] -= 0.2
        fov = 60
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * pos[2]
        corners = [
            [pos[0] - size, pos[1] - size, self.environment.contour_height(pos[0] - size, pos[1] - size)],
            [pos[0] + size, pos[1] - size, self.environment.contour_height(pos[0] + size, pos[1] - size)],
            [pos[0] + size, pos[1] + size, self.environment.contour_height(pos[0] + size, pos[1] + size)],
            [pos[0] - size, pos[1] + size, self.environment.contour_height(pos[0] - size, pos[1] + size)],
        ]
        glColor3f(0.2, 0.8, 1.0)
        glLineWidth(2)
        glBegin(GL_LINE_LOOP)
        for c in corners:
            glVertex3f(*c)
        glEnd()
        # Show camera and LiDAR data in ImGui
        imgui.separator()
        if self.show_camera:
            imgui.text("Camera (heightmap, center)")
            cam_img = sim.get_camera_image(self.environment, fov=60, res=16)
            imgui.plot_lines("", cam_img[cam_img.shape[0]//2], graph_size=(300, 60))
            imgui.separator()
        if imgui.button("Toggle LiDAR"):
            self.show_lidar = not self.show_lidar
        imgui.same_line()
        imgui.text(f"LiDAR: {'ON' if self.show_lidar else 'OFF'}")
        # Remove LiDAR scan and plot, bind vertex accumulation/rendering to LiDAR toggle
        if self.show_lidar:
            # Simulate LiDAR-like accumulation: only add a few points per frame in a circular scan
            if not hasattr(self, 'lidar_angle'):
                self.lidar_angle = 0.0
            num_rays = 32  # number of rays per scan
            scan_radius = 1.0  # scan radius in meters
            pos = sim.state[:3].copy()
            pos[2] -= 0.2
            angles = np.linspace(self.lidar_angle, self.lidar_angle + 2 * np.pi, num_rays, endpoint=False)
            for angle in angles:
                x = pos[0] + scan_radius * np.cos(angle)
                y = pos[1] + scan_radius * np.sin(angle)
                z = self.environment.contour_height(x, y)
                key = (round(x, 2), round(y, 2))
                if not hasattr(self, 'scanned_points_set'):
                    self.scanned_points_set = set()
                if key not in self.scanned_points_set:
                    self.scanned_points_set.add(key)
                    if not hasattr(self, 'scanned_points'):
                        self.scanned_points = []
                    self.scanned_points.append((x, y, z))
            self.lidar_angle = (self.lidar_angle + np.pi / 32) % (2 * np.pi)
            # Prepare VBO data if needed
            arr = np.array(self.scanned_points, dtype=np.float32)
            if arr.size > 0:
                if self.vbo is not None:
                    self.vbo.delete()
                self.vbo = vbo.VBO(arr)
                self.vbo_needs_update = False
            # Render all accumulated scanned points as GL_POINTS using VBO
            if self.vbo is not None:
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                glPointSize(3)
                self.vbo.bind()
                glEnableClientState(GL_VERTEX_ARRAY)
                glVertexPointer(3, GL_FLOAT, 0, self.vbo)
                glColor4f(0.2, 0.8, 0.2, 0.5)
                glDrawArrays(GL_POINTS, 0, len(self.scanned_points))
                glDisableClientState(GL_VERTEX_ARRAY)
                self.vbo.unbind()
                glDisable(GL_BLEND)
        imgui.separator()
        self.draw_thrust_arrows(sim)

    def draw_thrust_arrows(self, sim):
        """Draw arrows showing the thrust force from each rotor"""
        if not hasattr(sim, 'rotor_thrusts'):
            return
            
        rotors = sim.rotor_positions()
        max_thrust = max(sim.rotor_thrusts) if max(sim.rotor_thrusts) > 0 else 1.0
        
        # Draw thrust arrows downward from each rotor
        glLineWidth(4)
        for i, (rotor_pos, thrust) in enumerate(zip(rotors, sim.rotor_thrusts)):
            if thrust <= 0:
                continue
                
            # Normalize thrust to arrow length (0.1 to 1.0 meters)
            arrow_length = 0.1 + (thrust / max_thrust) * 0.9
            
            # Arrow color based on thrust intensity
            intensity = thrust / max_thrust
            glColor3f(1.0, 1.0 - intensity, 0.0)  # Yellow to Red gradient
            
            # Draw main arrow shaft (downward)
            start_pos = np.array(rotor_pos)
            end_pos = start_pos - np.array([0, 0, arrow_length])
            
            glBegin(GL_LINES)
            glVertex3f(*start_pos)
            glVertex3f(*end_pos)
            glEnd()
            
            # Draw arrowhead
            arrow_tip = end_pos
            arrow_size = 0.05
            
            glBegin(GL_TRIANGLES)
            # Arrowhead pointing down
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1] + arrow_size, arrow_tip[2] + arrow_size)
            
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] + arrow_size, arrow_tip[1] + arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1] + arrow_size, arrow_tip[2] + arrow_size)
            
            glVertex3f(arrow_tip[0], arrow_tip[1], arrow_tip[2])
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1] + arrow_size, arrow_tip[2] + arrow_size)
            glVertex3f(arrow_tip[0] - arrow_size, arrow_tip[1] - arrow_size, arrow_tip[2] + arrow_size)
            glEnd()
            
            # Draw thrust value as text near the arrow
            # Note: This is a simplified text rendering - in a real app you'd use a proper text renderer
            glPushMatrix()
            glTranslatef(rotor_pos[0] + 0.1, rotor_pos[1] + 0.1, rotor_pos[2])
            glColor3f(1, 1, 1)
            # We'll just draw the rotor number for now since OpenGL text rendering is complex
            # You could integrate a text rendering library here
            glPopMatrix()

    # ...existing code...

    def reshape(self, width, height):
        self.window_width, self.window_height = width, height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / float(height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
