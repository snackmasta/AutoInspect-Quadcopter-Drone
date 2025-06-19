"""ImGui UI panels for drone control and telemetry."""

import numpy as np
import imgui


class UIPanels:
    """Manages all ImGui UI panels for the simulation."""
    
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
        
        # SCADA Color Scheme
        self.colors = {
            'bg_dark': (0.12, 0.12, 0.15, 1.0),       # Dark background
            'bg_panel': (0.18, 0.18, 0.22, 0.95),     # Panel background
            'bg_header': (0.25, 0.25, 0.30, 1.0),     # Header background
            'text_primary': (0.95, 0.95, 0.95, 1.0),  # Primary text
            'text_secondary': (0.8, 0.8, 0.8, 1.0),   # Secondary text
            'accent_blue': (0.2, 0.6, 1.0, 1.0),      # System blue
            'accent_cyan': (0.0, 0.8, 0.9, 1.0),      # Cyan accents
            'status_good': (0.2, 0.8, 0.2, 1.0),      # Green for good status
            'status_warn': (1.0, 0.8, 0.0, 1.0),      # Yellow for warnings
            'status_alarm': (1.0, 0.3, 0.3, 1.0),     # Red for alarms
            'button_normal': (0.3, 0.3, 0.4, 1.0),    # Normal button
            'button_hover': (0.4, 0.4, 0.5, 1.0),     # Hover button
            'button_active': (0.2, 0.6, 1.0, 1.0),    # Active button
            'button_emergency': (0.8, 0.2, 0.2, 1.0), # Emergency button
            'gauge_bg': (0.15, 0.15, 0.2, 1.0),       # Gauge background
            'gauge_fill': (0.2, 0.6, 1.0, 1.0),       # Gauge fill
        }
        
        # Button dimensions for consistency
        self.button_size = {
            'small': (80, 30),
            'medium': (120, 35),
            'large': (160, 40),
            'emergency': (140, 50),
        }
    
    def _apply_scada_theme(self):
        """Apply SCADA industrial theme to ImGui."""
        style = imgui.get_style()
        
        # Window styling
        style.window_rounding = 2.0
        style.window_border_size = 1.0
        style.window_padding = (12, 8)
        style.frame_rounding = 3.0
        style.frame_border_size = 1.0
        style.frame_padding = (8, 4)
        style.item_spacing = (8, 6)
        style.item_inner_spacing = (6, 4)
        style.indent_spacing = 20.0
        style.scrollbar_size = 16.0
        style.scrollbar_rounding = 3.0
        style.grab_min_size = 12.0
        style.grab_rounding = 3.0
        style.button_text_align = (0.5, 0.5)
        
        # Colors
        colors = style.colors
        colors[imgui.COLOR_WINDOW_BACKGROUND] = self.colors['bg_panel']
        colors[imgui.COLOR_CHILD_BACKGROUND] = self.colors['bg_dark']
        colors[imgui.COLOR_POPUP_BACKGROUND] = self.colors['bg_panel']
        colors[imgui.COLOR_FRAME_BACKGROUND] = self.colors['gauge_bg']
        colors[imgui.COLOR_FRAME_BACKGROUND_HOVERED] = self.colors['button_hover']
        colors[imgui.COLOR_FRAME_BACKGROUND_ACTIVE] = self.colors['button_active']
        colors[imgui.COLOR_TITLE_BACKGROUND] = self.colors['bg_header']
        colors[imgui.COLOR_TITLE_BACKGROUND_ACTIVE] = self.colors['accent_blue']
        colors[imgui.COLOR_TITLE_BACKGROUND_COLLAPSED] = self.colors['bg_header']
        colors[imgui.COLOR_BUTTON] = self.colors['button_normal']
        colors[imgui.COLOR_BUTTON_HOVERED] = self.colors['button_hover']
        colors[imgui.COLOR_BUTTON_ACTIVE] = self.colors['button_active']
        colors[imgui.COLOR_HEADER] = self.colors['accent_blue']
        colors[imgui.COLOR_HEADER_HOVERED] = (0.3, 0.7, 1.0, 0.8)
        colors[imgui.COLOR_HEADER_ACTIVE] = (0.2, 0.6, 1.0, 1.0)
        colors[imgui.COLOR_TEXT] = self.colors['text_primary']
        colors[imgui.COLOR_TEXT_DISABLED] = self.colors['text_secondary']
        colors[imgui.COLOR_BORDER] = self.colors['accent_cyan']
        colors[imgui.COLOR_BORDER_SHADOW] = (0.0, 0.0, 0.0, 0.5)
        colors[imgui.COLOR_PLOT_LINES] = self.colors['accent_cyan']
        colors[imgui.COLOR_PLOT_HISTOGRAM] = self.colors['accent_blue']
        colors[imgui.COLOR_SLIDER_GRAB] = self.colors['accent_blue']
        colors[imgui.COLOR_SLIDER_GRAB_ACTIVE] = self.colors['accent_cyan']
        colors[imgui.COLOR_CHECK_MARK] = self.colors['status_good']
    
    def _draw_status_indicator(self, label, status, size=(20, 20)):
        """Draw a circular status indicator light."""
        if status == 'good':
            color = self.colors['status_good']
        elif status == 'warn':
            color = self.colors['status_warn']
        elif status == 'alarm':
            color = self.colors['status_alarm']
        else:
            color = self.colors['text_secondary']
        
        # Draw colored circle
        draw_list = imgui.get_window_draw_list()
        pos = imgui.get_cursor_screen_pos()
        center = (pos[0] + size[0]/2, pos[1] + size[1]/2)
        radius = min(size[0], size[1]) / 2 - 2
        
        # Convert color to ImU32
        color_u32 = imgui.get_color_u32_rgba(*color)
        draw_list.add_circle_filled(center[0], center[1], radius, color_u32)
        draw_list.add_circle(center[0], center[1], radius, imgui.get_color_u32_rgba(1,1,1,0.3), 0, 1.0)
        
        imgui.dummy(size[0], size[1])
        imgui.same_line()
        imgui.text(label)
    
    def _draw_large_button(self, label, size=None, color_scheme='normal'):
        """Draw a large SCADA-style button."""
        if size is None:
            size = self.button_size['medium']
            
        if color_scheme == 'emergency':
            imgui.push_style_color(imgui.COLOR_BUTTON, *self.colors['button_emergency'])
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.4, 0.4, 1.0)
            imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.9, 0.1, 0.1, 1.0)
        elif color_scheme == 'active':
            imgui.push_style_color(imgui.COLOR_BUTTON, *self.colors['button_active'])
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.3, 0.7, 1.0, 1.0)
            imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.1, 0.5, 0.9, 1.0)        
        result = imgui.button(label, *size)
        
        if color_scheme in ['emergency', 'active']:
            imgui.pop_style_color(3)
            
        return result
    
    def _draw_section_header(self, title):
        """Draw a section header with SCADA styling."""
        imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['accent_cyan'])
        imgui.text(f"■ {title.upper()}")
        imgui.pop_style_color()
        imgui.separator()
        imgui.spacing()
    
    def _draw_value_display(self, label, value, unit="", status='normal', width=120):
        """Draw a value display with label and status coloring."""
        if status == 'good':
            color = self.colors['status_good']
        elif status == 'warn':
            color = self.colors['status_warn']
        elif status == 'alarm':
            color = self.colors['status_alarm']
        else:
            color = self.colors['text_primary']
        
        imgui.text(f"{label}:")
        imgui.same_line(width)
        imgui.push_style_color(imgui.COLOR_TEXT, *color)
        imgui.text(f"{value} {unit}")
        imgui.pop_style_color()
    
    def draw_mission_control_panel(self, sim, camera_controller):
        """Draw the main mission control panel with SCADA styling."""
        self._apply_scada_theme()
        
        io = imgui.get_io()
        io.display_size = (self.window_width, self.window_height)
        
        # Main control panel - larger and more prominent
        imgui.set_next_window_position(10, 10)
        imgui.set_next_window_size(420, 650)
        imgui.begin("■ DRONE CONTROL SYSTEM", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        
        # System status header
        self._draw_section_header("SYSTEM STATUS")
        
        # Flight mode and safety status indicators
        flight_status = 'good' if not (hasattr(sim.state_manager, 'crashed') and sim.state_manager.crashed) else 'alarm'
        safety_status = 'good' if sim.safety_system_enabled else 'warn'
        manual_status = 'warn' if sim.manual_mode else 'good'
        
        self._draw_status_indicator("FLIGHT SYSTEM", flight_status)
        self._draw_status_indicator("SAFETY SYSTEM", safety_status)
        self._draw_status_indicator("AUTO MODE", manual_status)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Flight control section
        self._draw_section_header("FLIGHT OPERATIONS")
        
        # Primary flight controls - arranged in a grid
        if self._draw_large_button("TAKE OFF", self.button_size['large'], 'active'):
            sim.takeoff(target_altitude=3.0)
        imgui.same_line()
        if self._draw_large_button("LAND", self.button_size['large'], 'emergency'):
            sim.land()
        
        if self._draw_large_button("HOVER", self.button_size['large']):
            sim.hover()
        imgui.same_line()
        if self._draw_large_button("RESUME WP", self.button_size['large']):
            sim.resume_waypoint_progression()
        
        imgui.spacing()
        
        # Secondary controls
        if self._draw_large_button("PAUSE", self.button_size['medium']):
            pass
        imgui.same_line()
        if self._draw_large_button("RESUME", self.button_size['medium']):
            pass
        imgui.same_line()
        if self._draw_large_button("RESET", self.button_size['medium'], 'emergency'):
            sim.reset()
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Flight parameters section
        self._draw_section_header("FLIGHT PARAMETERS")
        
        # Target speed control with SCADA styling
        imgui.text("TARGET SPEED:")
        imgui.push_item_width(300)
        changed, new_speed = imgui.slider_float("##target_speed", sim.target_speed, 0.5, 10.0, "%.1f m/s")
        imgui.pop_item_width()
        if changed:
            sim.set_target_speed(new_speed)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Telemetry section
        self._draw_section_header("TELEMETRY DATA")
        self._draw_telemetry(sim)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Manual control section
        self._draw_section_header("MANUAL CONTROL")
        self._draw_manual_controls(sim)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Camera control section
        self._draw_section_header("CAMERA CONTROL")
        self._draw_camera_controls(camera_controller)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Safety system section
        self._draw_section_header("SAFETY SYSTEMS")
        changed, new_safety_enabled = imgui.checkbox("ENABLE SAFETY SYSTEM", sim.safety_system_enabled)
        if changed:
            sim.safety_system_enabled = new_safety_enabled
        
        imgui.spacing()
        
        # System information
        if imgui.collapsing_header("SYSTEM HELP"):
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['text_secondary'])
            imgui.text("KEYBOARD CONTROLS:")
            imgui.text("• M - Toggle Manual Mode")
            imgui.text("• Ctrl+S - Toggle Safety System")
            imgui.text("MANUAL MODE CONTROLS:")
            imgui.text("• W/S - Pitch Forward/Backward")
            imgui.text("• A/D - Roll Left/Right") 
            imgui.text("• Q/E - Yaw Left/Right")
            imgui.text("• R/F - Increase/Decrease Throttle")
            imgui.pop_style_color()
        
        imgui.end()
    
    def draw_situational_awareness_panel(self, sim):
        """Draw the situational awareness panel with SCADA styling."""
        self._apply_scada_theme()
        
        # Position on the right side, larger
        imgui.set_next_window_position(self.window_width - 450, 10)
        imgui.set_next_window_size(440, 500)
        
        imgui.begin("■ SITUATIONAL AWARENESS", 
                   flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | 
                         imgui.WINDOW_NO_COLLAPSE)
        
        pos, vel = sim.state[:3], sim.state[3:]
        
        # Flight status section
        self._draw_section_header("FLIGHT STATUS")
        
        # Key flight parameters with status coloring
        altitude_status = 'good' if 1.0 <= pos[2] <= 10.0 else 'warn' if pos[2] > 0.5 else 'alarm'
        ground_speed = np.linalg.norm(vel[:2])
        speed_status = 'good' if ground_speed <= 5.0 else 'warn' if ground_speed <= 8.0 else 'alarm'
        vs = vel[2]
        vs_status = 'good' if abs(vs) <= 1.0 else 'warn' if abs(vs) <= 2.0 else 'alarm'
        
        self._draw_value_display("ALTITUDE", f"{pos[2]:.2f}", "m", altitude_status, 140)
        self._draw_value_display("GROUND SPEED", f"{ground_speed:.2f}", "m/s", speed_status, 140)
        self._draw_value_display("VERTICAL SPEED", f"{vs:+.2f}", "m/s", vs_status, 140)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Navigation section
        self._draw_section_header("NAVIGATION")
        
        dist_to_wp = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        nav_status = 'good' if dist_to_wp <= 2.0 else 'warn'
        
        self._draw_value_display("WAYPOINT", f"{sim.wp_index+1}/{len(sim.waypoints)}", "", 'good', 140)
        self._draw_value_display("DISTANCE TO WP", f"{dist_to_wp:.2f}", "m", nav_status, 140)
        
        # Mission progress bar
        imgui.spacing()
        imgui.text("MISSION PROGRESS:")
        self._draw_mission_progress(sim)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Propulsion system section
        self._draw_section_header("PROPULSION SYSTEM")
        
        imgui.text("ROTOR RPM:")
        imgui.spacing()
        
        # RPM bars in a 2x2 grid
        for i in range(4):
            rpm = sim.rotor_speeds[i]
            rpm_ratio = min(rpm / 12000.0, 1.0)
            rpm_status = 'good' if 6000 <= rpm <= 11000 else 'warn' if rpm > 0 else 'alarm'
            
            # Color the progress bar based on status
            if rpm_status == 'good':
                bar_color = self.colors['status_good']
            elif rpm_status == 'warn':
                bar_color = self.colors['status_warn']
            else:
                bar_color = self.colors['status_alarm']
            
            imgui.push_style_color(imgui.COLOR_PLOT_HISTOGRAM, *bar_color)
            imgui.progress_bar(rpm_ratio, size=(100, 20), overlay=f"M{i+1}: {int(rpm)}")
            imgui.pop_style_color()
            
            if i == 1:  # After second motor, go to next line
                imgui.new_line()
            elif i < 3:
                imgui.same_line()
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # System status section
        self._draw_section_header("SYSTEM STATUS")
        
        # System status indicators
        if hasattr(sim.state_manager, 'crashed') and sim.state_manager.crashed:
            self._draw_status_indicator("FLIGHT SYSTEM", 'alarm')
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_alarm'])
            imgui.text("■ SYSTEM CRASHED")
            imgui.pop_style_color()
        elif hasattr(sim.state_manager, 'recovery_mode') and sim.state_manager.recovery_mode:
            self._draw_status_indicator("FLIGHT SYSTEM", 'warn')
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_warn'])
            imgui.text("■ RECOVERY MODE ACTIVE")
            imgui.pop_style_color()
        else:
            self._draw_status_indicator("FLIGHT SYSTEM", 'good')
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_good'])
            imgui.text("■ SYSTEM OPERATIONAL")
            imgui.pop_style_color()
        
        safety_status = 'good' if sim.safety_system_enabled else 'warn'
        manual_status = 'warn' if sim.manual_mode else 'good'
        
        self._draw_status_indicator("SAFETY SYSTEM", safety_status)
        self._draw_status_indicator("AUTO CONTROL", manual_status)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Flight data visualization
        self._draw_section_header("FLIGHT DATA")
        
        # Recent altitude plot
        if len(sim.trajectory) > 1:
            altitudes = np.array([p[2] for p in sim.trajectory[-50:]], dtype=np.float32)
            imgui.text("ALTITUDE TREND (50 pts):")
            imgui.push_style_color(imgui.COLOR_PLOT_LINES, *self.colors['accent_cyan'])
            imgui.plot_lines("", altitudes, graph_size=(400, 80))
            imgui.pop_style_color()
        
        imgui.end()
    
    def draw_terrain_telemetry(self, sim, environment):
        """Draw terrain distance telemetry with SCADA styling."""
        self._apply_scada_theme()
        
        # Position at bottom left
        imgui.set_next_window_position(10, self.window_height - 250)
        imgui.set_next_window_size(420, 240)
        
        imgui.begin("■ TERRAIN TELEMETRY", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        
        from ..drone_body_box import get_body_box_corners
        
        # Get orientation in radians for get_body_box_corners
        roll_rad, pitch_rad, yaw_rad = sim.state[6:9]
        center = sim.state[:3]
        size = (0.8, 0.8, 0.2)  # Default size
        corners = get_body_box_corners(roll_rad, pitch_rad, yaw_rad, center, size)
        
        distances = []
        for i, (x, y, z) in enumerate(corners):
            terrain_z = environment.contour_height(x, y)
            dist = z - terrain_z
            distances.append(dist)
        
        self._draw_section_header("TERRAIN CLEARANCE")
        
        min_dist = min(distances)
        clearance_status = 'good' if min_dist > 2.0 else 'warn' if min_dist > 1.0 else 'alarm'
        
        self._draw_value_display("MIN CLEARANCE", f"{min_dist:.3f}", "m", clearance_status, 140)
        
        imgui.spacing()
        imgui.text("CORNER DISTANCES:")
        for i, dist in enumerate(distances):
            corner_status = 'good' if dist > 1.5 else 'warn' if dist > 0.8 else 'alarm'
            self._draw_value_display(f"CORNER {i+1}", f"{dist:.3f}", "m", corner_status, 100)
        
        imgui.end()
    
    def _draw_telemetry(self, sim):
        """Draw basic telemetry information with SCADA styling."""
        pos, vel = sim.state[:3], sim.state[3:]
        
        # Position data
        self._draw_value_display("POS X", f"{pos[0]:.2f}", "m", 'good', 100)
        self._draw_value_display("POS Y", f"{pos[1]:.2f}", "m", 'good', 100)
        self._draw_value_display("POS Z", f"{pos[2]:.2f}", "m", 'good', 100)
        
        imgui.spacing()
        
        # Velocity data
        self._draw_value_display("VEL X", f"{vel[0]:.2f}", "m/s", 'good', 100)
        self._draw_value_display("VEL Y", f"{vel[1]:.2f}", "m/s", 'good', 100)
        self._draw_value_display("VEL Z", f"{vel[2]:.2f}", "m/s", 'good', 100)
        
        imgui.spacing()
        
        # Angle telemetry
        roll, pitch, yaw = np.degrees(sim.state[6:9])
        self._draw_value_display("ROLL", f"{roll:.1f}", "°", 'good', 100)
        self._draw_value_display("PITCH", f"{pitch:.1f}", "°", 'good', 100)
        self._draw_value_display("YAW", f"{yaw:.1f}", "°", 'good', 100)
        
        imgui.spacing()
        
        # Waypoint info
        distance_to_wp = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        wp_status = 'good' if distance_to_wp <= 2.0 else 'warn'
        self._draw_value_display("WP DISTANCE", f"{distance_to_wp:.2f}", "m", wp_status, 120)
        
        # Mission status
        mission_status = 'good' if not (hasattr(sim.state_manager, 'crashed') and sim.state_manager.crashed) else 'alarm'
        if hasattr(sim.state_manager, 'recovery_mode') and sim.state_manager.recovery_mode:
            mission_status = 'warn'
        
        imgui.spacing()
        if mission_status == 'good':
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_good'])
            imgui.text("■ MISSION: OPERATIONAL")
        elif mission_status == 'warn':
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_warn'])
            imgui.text("■ MISSION: RECOVERY MODE")
        else:
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_alarm'])
            imgui.text("■ MISSION: CRITICAL")
        imgui.pop_style_color()
    
    def _draw_camera_controls(self, camera_controller):
        """Draw camera control sliders with SCADA styling."""
        imgui.text("CAMERA ANGLES:")
        imgui.push_item_width(280)
        _, camera_controller.angle_x = imgui.slider_float("X ANGLE", camera_controller.angle_x, -90.0, 90.0, "%.0f°")
        _, camera_controller.angle_y = imgui.slider_float("Y ANGLE", camera_controller.angle_y, -180.0, 180.0, "%.0f°")
        _, camera_controller.angle_z = imgui.slider_float("Z ANGLE", camera_controller.angle_z, -180.0, 180.0, "%.0f°")
        _, camera_controller.zoom = imgui.slider_float("ZOOM", camera_controller.zoom, 0.2, 3.0, "%.1fx")
        imgui.pop_item_width()
    
    def _draw_manual_controls(self, sim):
        """Draw manual control interface with SCADA styling."""
        changed, new_manual_mode = imgui.checkbox("ENABLE MANUAL MODE", sim.manual_mode)
        if changed:
            if new_manual_mode:
                # Switching to manual: set manual_rpms to current rotor_speeds
                sim.manual_rpms[:] = sim.rotor_speeds
                sim.manual_mode = True
            else:
                # Switching to auto: reset manual_rpms and manual-specific state
                sim.manual_mode = False
                sim.manual_rpms[:] = 0
                if hasattr(sim, 'manual_yaw'):
                    del sim.manual_yaw
        
        if sim.manual_mode:
            imgui.spacing()
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_warn'])
            imgui.text("■ MANUAL CONTROL ACTIVE")
            imgui.pop_style_color()
            imgui.spacing()
            
            imgui.text("PROPELLER RPM CONTROL:")
            imgui.push_item_width(250)
            for i in range(4):
                _, rpm = imgui.slider_float(f"PROP {i+1} RPM", 
                                          float(sim.manual_rpms[i]), 
                                          sim.min_rpm, sim.max_rpm, "%.0f RPM")
                sim.manual_rpms[i] = rpm
            imgui.pop_item_width()
    
    def _draw_mission_progress(self, sim):
        """Draw mission progress with SCADA styling."""
        try:
            from ..main_trajectory import get_lookahead_target
            pos = sim.state[:3]
            waypoints = sim.waypoints
            
            # Compute total path length
            total_length = sum(np.linalg.norm(waypoints[i+1] - waypoints[i]) 
                             for i in range(len(waypoints)-1))
            
            # Find closest segment and progress along path
            min_dist = float('inf')
            progress_length = 0.0
            
            for i in range(len(waypoints) - 1):
                seg_start = waypoints[i]
                seg_end = waypoints[i+1]
                seg_vec = seg_end - seg_start
                seg_len = np.linalg.norm(seg_vec)
                if seg_len < 1e-6:
                    continue
                    
                proj = np.dot(pos - seg_start, seg_vec) / seg_len
                proj = np.clip(proj, 0, seg_len)
                closest_point = seg_start + seg_vec * (proj / seg_len)
                dist = np.linalg.norm(pos - closest_point)
                
                if dist < min_dist:
                    min_dist = dist
                    progress_length = sum(np.linalg.norm(waypoints[j+1] - waypoints[j]) 
                                        for j in range(i)) + proj
            
            progress_ratio = min(progress_length / total_length, 1.0)
        except Exception:
            # Fallback to simple waypoint-based progress
            progress_ratio = sim.wp_index / max(len(sim.waypoints) - 1, 1)
        
        # Draw progress bar with SCADA styling
        imgui.text("MISSION PROGRESS:")
        
        # Color the progress bar based on completion
        if progress_ratio >= 0.9:
            bar_color = self.colors['status_good']
        elif progress_ratio >= 0.5:
            bar_color = self.colors['accent_blue']
        else:
            bar_color = self.colors['status_warn']
        
        imgui.push_style_color(imgui.COLOR_PLOT_HISTOGRAM, *bar_color)
        imgui.progress_bar(progress_ratio, size=(400, 25), overlay=f"{int(progress_ratio*100)}%")
        imgui.pop_style_color()
    
    def update_window_size(self, width, height):
        """Update the window dimensions."""
        self.window_width = width
        self.window_height = height
    
    def draw_debug_panel(self, debug_config=None):
        """Draw debug control panel with SCADA styling."""
        if debug_config is None:
            return
            
        self._apply_scada_theme()
        
        # Position at bottom right
        imgui.set_next_window_position(self.window_width - 450, self.window_height - 300)
        imgui.set_next_window_size(440, 290)
        
        imgui.begin("■ SYSTEM DIAGNOSTICS", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        
        self._draw_section_header("DEBUG CONTROLS")
        
        # Key event debugging
        imgui.text("INPUT SYSTEM:")
        changed, debug_config.DEBUG_KEY_EVENT = imgui.checkbox("KEY EVENT DEBUG", debug_config.DEBUG_KEY_EVENT)
        if changed:
            status = "ENABLED" if debug_config.DEBUG_KEY_EVENT else "DISABLED"
            print(f"[DEBUG] Key Event Debug {status}")
        
        changed, debug_config.DEBUG_IMGUI_CAPTURE = imgui.checkbox("IMGUI CAPTURE DEBUG", debug_config.DEBUG_IMGUI_CAPTURE)
        changed, debug_config.DEBUG_ACTION_IGNORE = imgui.checkbox("ACTION IGNORE DEBUG", debug_config.DEBUG_ACTION_IGNORE)
        
        imgui.spacing()
        imgui.text("FLIGHT CONTROLS:")
        
        # Manual mode debugging
        changed, debug_config.DEBUG_MANUAL_MODE_OFF = imgui.checkbox("MANUAL MODE OFF DEBUG", debug_config.DEBUG_MANUAL_MODE_OFF)
        
        # Individual key debugging - arranged in a grid
        imgui.text("KEY DEBUG:")
        imgui.columns(4, "key_debug")
        _, debug_config.DEBUG_KEY_W = imgui.checkbox("W", debug_config.DEBUG_KEY_W)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_S = imgui.checkbox("S", debug_config.DEBUG_KEY_S)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_A = imgui.checkbox("A", debug_config.DEBUG_KEY_A)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_D = imgui.checkbox("D", debug_config.DEBUG_KEY_D)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_Q = imgui.checkbox("Q", debug_config.DEBUG_KEY_Q)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_E = imgui.checkbox("E", debug_config.DEBUG_KEY_E)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_R = imgui.checkbox("R", debug_config.DEBUG_KEY_R)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_F = imgui.checkbox("F", debug_config.DEBUG_KEY_F)
        imgui.columns(1)
        
        imgui.spacing()
        imgui.text("SYSTEM DEBUG:")
        _, debug_config.DEBUG_MANUAL_RPMS = imgui.checkbox("MANUAL RPMS DEBUG", debug_config.DEBUG_MANUAL_RPMS)
        _, debug_config.DEBUG_PHYSICS = imgui.checkbox("PHYSICS DEBUG", debug_config.DEBUG_PHYSICS)
        _, debug_config.DEBUG_MANUAL_STATUS = imgui.checkbox("MANUAL STATUS DEBUG", debug_config.DEBUG_MANUAL_STATUS)
        
        # Debug status indicator
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        debug_active = any([debug_config.DEBUG_KEY_EVENT, debug_config.DEBUG_IMGUI_CAPTURE,
                           debug_config.DEBUG_ACTION_IGNORE, debug_config.DEBUG_MANUAL_RPMS,
                           debug_config.DEBUG_PHYSICS, debug_config.DEBUG_MANUAL_STATUS])
        
        debug_status = 'warn' if debug_active else 'good'
        self._draw_status_indicator("DEBUG MODE", debug_status)
        
        if debug_active:
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_warn'])
            imgui.text("■ DEBUG LOGGING ACTIVE")
            imgui.pop_style_color()
        else:
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_good'])
            imgui.text("■ NORMAL OPERATION")
            imgui.pop_style_color()
        
        imgui.end()
