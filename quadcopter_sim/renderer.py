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
from .drone_body_box import draw_drone_body_box
from .thrust import Thrust

class Renderer:
    def __init__(self, sim):
        self.sim = sim
        self.window_width, self.window_height = 1200, 800
        self.angle_x, self.angle_y, self.angle_z = 17, 0, 90
        self.zoom = 1.2
        # Camera interaction state
        self._dragging = False
        self._last_mouse = (0, 0)
        self.environment = Environment(size=20, step=2.0)  # Increased step for performance
        self.show_lidar = False  # Toggle for LiDAR visualization (default OFF)
        self.scanned_points = []  # Accumulated scanned terrain points
        self.vbo = None  # Vertex Buffer Object for scanned points
        self.vbo_needs_update = True
        self.show_camera = False  # Camera visualization toggle, default off
        self.thrust_visual = Thrust(sim.k_thrust, sim.atmosphere_density)

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
        if imgui.button("Take Off"):
            sim.takeoff(target_altitude=3.0)
        imgui.same_line()
        if imgui.button("Land"):
            sim.land()
        imgui.separator()
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
        changed, new_manual_mode = imgui.checkbox("Manual Mode", self.sim.manual_mode)
        if changed:
            # When changing mode, keep the last state of the drone
            if new_manual_mode:
                # Switching to manual: set manual_rpms to current rotor_speeds
                self.sim.manual_rpms[:] = self.sim.rotor_speeds
            else:
                # Switching to auto: nothing to do, auto mode uses state as is
                pass
            self.sim.manual_mode = new_manual_mode
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
            imgui.progress_bar(min(rpm / 12000.0, 1.0), size=(75, 12), overlay=f"{int(rpm)}")
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

        # --- Always render unscanned terrain as grey wireframe ---
        glLineWidth(1.5)
        self.environment.draw(wireframe=True)
        glLineWidth(1.0)  # Restore default

        glColor3f(0, 0, 1)  # Changed to blue
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
            # Apply drone orientation (roll, pitch, yaw)
            roll, pitch, yaw = np.degrees(sim.state[6:9])
            glRotatef(yaw, 0, 0, 1)
            glRotatef(pitch, 1, 0, 0)
            glRotatef(roll, 0, 1, 0)
            # Then apply blade rotation around local Z
            glRotatef(sim.blade_angles[i], 0, 0, 1)
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
        glColor3f(0, 1, 0)
        glPointSize(14)
        glBegin(GL_POINTS)
        glVertex3f(*sim.waypoints[sim.wp_index])
        glEnd()
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
        # --- Accumulate camera chunks every 3 seconds with partial culling ---
        if not hasattr(self, 'camera_chunks'):
            self.camera_chunks = []  # List of (img, pos)
            self.camera_chunk_counter = 0
        self.camera_chunk_counter += 1
        # Assuming ~30 FPS, 3 seconds = 90 frames
        if self.camera_chunk_counter >= 90:
            img = sim.get_camera_image(self.environment, fov=60, res=12)
            pos = sim.state[:3].copy()
            self.add_camera_chunk_with_partial_culling(img, pos, fov=60)
            self.camera_chunk_counter = 0
        # Render all accumulated camera chunks as wireframe grids
        for img, pos in self.camera_chunks:
            res = img.shape[0]
            fov = 60
            half_fov = np.radians(fov / 2)
            size = np.tan(half_fov) * (pos[2] - 0.2)
            xs = np.linspace(pos[0] - size, pos[0] + size, res)
            ys = np.linspace(pos[1] - size, pos[1] + size, res)
            glColor3f(0.2, 0.8, 0.2)
            glLineWidth(1.2)
            for i in range(res):
                glBegin(GL_LINE_STRIP)
                for j in range(res):
                    glVertex3f(xs[j], ys[i], img[i, j])
                glEnd()
            for j in range(res):
                glBegin(GL_LINE_STRIP)
                for i in range(res):
                    glVertex3f(xs[j], ys[i], img[i, j])
                glEnd()
        imgui.separator()
        # Draw blue box outline for drone body, with orientation from module
        roll, pitch, yaw = np.degrees(sim.state[6:9])
        draw_drone_body_box(roll, pitch, yaw, center)

        # Draw rotor numbers above each rotor
        from OpenGL.GLUT import glutInit, glutBitmapCharacter, GLUT_BITMAP_HELVETICA_18
        glutInit()
        for i, r in enumerate(rotors):
            glColor3f(0, 0, 0)
            # Offset above rotor (z+0.12)
            glRasterPos3f(r[0], r[1], r[2] + 0.12)
            for c in str(i+1):
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(c))

        # Draw thrust arrows for each rotor (scaled 0-12000 RPM)
        Thrust.draw_thrust_arrows(
            rotors,
            sim.rotor_speeds,
            min_rpm=0,
            max_rpm=12000,
            thrust_coefficient=self.thrust_visual.thrust_coefficient,
            atmosphere_density=self.thrust_visual.atmosphere_density
        )

    def is_chunk_overlapping(self, pos1, pos2, fov=60, altitude=None):
        """Return True if two camera chunks overlap based on their positions and FOV coverage."""
        if altitude is None:
            altitude = pos1[2]
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * (altitude - 0.2)
        # Use a square region for simplicity
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        return dx < 2 * size and dy < 2 * size

    def mask_overlapping_area(self, img, pos, existing_chunks, fov=60):
        """Mask out overlapping areas in img based on existing chunks."""
        res = img.shape[0]
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * (pos[2] - 0.2)
        xs = np.linspace(pos[0] - size, pos[0] + size, res)
        ys = np.linspace(pos[1] - size, pos[1] + size, res)
        mask = np.ones_like(img, dtype=bool)
        for _, old_pos in existing_chunks:
            old_size = np.tan(half_fov) * (old_pos[2] - 0.2)
            x_min, x_max = old_pos[0] - old_size, old_pos[0] + old_size
            y_min, y_max = old_pos[1] - old_size, old_pos[1] + old_size
            # Mask out overlapping region
            x_overlap = (xs >= x_min) & (xs <= x_max)
            y_overlap = (ys >= y_min) & (ys <= y_max)
            for i in range(res):
                for j in range(res):
                    if x_overlap[j] and y_overlap[i]:
                        mask[i, j] = False
        return np.where(mask, img, np.nan), mask

    def add_camera_chunk_with_partial_culling(self, img, pos, fov=60):
        """Add only the non-overlapping part of a camera chunk."""
        if not hasattr(self, 'camera_chunks'):
            self.camera_chunks = []
        masked_img, mask = self.mask_overlapping_area(img, pos, self.camera_chunks, fov)
        if np.any(mask):
            self.camera_chunks.append((masked_img, pos))

    def add_camera_chunk_with_culling(self, img, pos, fov=60):
        """Add a camera chunk, replacing any overlapping chunk to save memory."""
        new_chunks = []
        replaced = False
        for old_img, old_pos in self.camera_chunks:
            if self.is_chunk_overlapping(pos, old_pos, fov, altitude=pos[2]):
                # Replace the old chunk with the new one
                if not replaced:
                    new_chunks.append((img, pos))
                    replaced = True
                # else: skip adding the old chunk
            else:
                new_chunks.append((old_img, old_pos))
        if not replaced:
            new_chunks.append((img, pos))
        self.camera_chunks = new_chunks

    def reshape(self, width, height):
        self.window_width, self.window_height = width, height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / float(height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
