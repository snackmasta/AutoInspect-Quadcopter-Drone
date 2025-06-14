from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import imgui

class Renderer:
    def __init__(self, sim):
        self.sim = sim
        self.window_width, self.window_height = 1200, 800
        self.angle_x, self.angle_y, self.angle_z = -73, 0, 17
        self.zoom = 1.2
        # Camera interaction state
        self._dragging = False
        self._last_mouse = (0, 0)

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
        # Camera now orbits around the drone's current position
        center = sim.state[:3]
        glTranslatef(-center[0], -center[1], -center[2])
        glTranslatef(0, 0, -10 * self.zoom)
        glRotatef(self.angle_x, 1, 0, 0)  # pitch (x)
        glRotatef(self.angle_z, 0, 0, 1)  # yaw (z)
        # Removed yaw (angle_y)
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

    def reshape(self, width, height):
        self.window_width, self.window_height = width, height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / float(height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
