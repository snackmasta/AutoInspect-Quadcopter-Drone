"""Mission control panel for flight operations and telemetry."""

import numpy as np
import imgui
from .base_panel import BasePanel


class MissionControlPanel(BasePanel):
    """Mission control panel with flight operations and telemetry."""
    
    def draw(self, sim, camera_controller):
        """Draw the mission control panel with SCADA styling."""
        self.apply_scada_theme()
        
        io = imgui.get_io()
        io.display_size = (self.window_width, self.window_height)
        
        # Main control panel - larger and more prominent
        imgui.set_next_window_position(10, 10)
        imgui.set_next_window_size(420, 650)
        imgui.begin("■ DRONE CONTROL SYSTEM", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        
        # System status header
        self.draw_section_header("SYSTEM STATUS")
        
        # Flight mode and safety status indicators
        flight_status = 'good' if not (hasattr(sim.state_manager, 'crashed') and sim.state_manager.crashed) else 'alarm'
        safety_status = 'good' if sim.safety_system_enabled else 'warn'
        manual_status = 'warn' if sim.manual_mode else 'good'
        
        self.draw_status_indicator("FLIGHT SYSTEM", flight_status)
        self.draw_status_indicator("SAFETY SYSTEM", safety_status)
        self.draw_status_indicator("AUTO MODE", manual_status)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Flight control section
        self.draw_section_header("FLIGHT OPERATIONS")
        
        # Primary flight controls - arranged in a grid
        if self.draw_large_button("TAKE OFF", self.button_size['large'], 'active'):
            sim.takeoff(target_altitude=3.0)
        imgui.same_line()
        if self.draw_large_button("LAND", self.button_size['large'], 'emergency'):
            sim.land()
        
        if self.draw_large_button("HOVER", self.button_size['large']):
            sim.hover()
        imgui.same_line()
        if self.draw_large_button("RESUME WP", self.button_size['large']):
            sim.resume_waypoint_progression()
        
        imgui.spacing()
        
        # Secondary controls
        if self.draw_large_button("PAUSE", self.button_size['medium']):
            pass
        imgui.same_line()
        if self.draw_large_button("RESUME", self.button_size['medium']):
            pass
        imgui.same_line()
        if self.draw_large_button("RESET", self.button_size['medium'], 'emergency'):
            sim.reset()
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Flight parameters section
        self.draw_section_header("FLIGHT PARAMETERS")
        
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
        self.draw_section_header("TELEMETRY DATA")
        self._draw_telemetry(sim)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Manual control section
        self.draw_section_header("MANUAL CONTROL")
        self._draw_manual_controls(sim)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Camera control section
        self.draw_section_header("CAMERA CONTROL")
        self._draw_camera_controls(camera_controller)
        
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        # Safety system section
        self.draw_section_header("SAFETY SYSTEMS")
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
    
    def _draw_telemetry(self, sim):
        """Draw basic telemetry information with SCADA styling."""
        pos, vel = sim.state[:3], sim.state[3:]
        
        # Position data
        self.draw_value_display("POS X", f"{pos[0]:.2f}", "m", 'good', 100)
        self.draw_value_display("POS Y", f"{pos[1]:.2f}", "m", 'good', 100)
        self.draw_value_display("POS Z", f"{pos[2]:.2f}", "m", 'good', 100)
        
        imgui.spacing()
        
        # Velocity data
        self.draw_value_display("VEL X", f"{vel[0]:.2f}", "m/s", 'good', 100)
        self.draw_value_display("VEL Y", f"{vel[1]:.2f}", "m/s", 'good', 100)
        self.draw_value_display("VEL Z", f"{vel[2]:.2f}", "m/s", 'good', 100)
        
        imgui.spacing()
        
        # Angle telemetry
        roll, pitch, yaw = np.degrees(sim.state[6:9])
        self.draw_value_display("ROLL", f"{roll:.1f}", "°", 'good', 100)
        self.draw_value_display("PITCH", f"{pitch:.1f}", "°", 'good', 100)
        self.draw_value_display("YAW", f"{yaw:.1f}", "°", 'good', 100)
        
        imgui.spacing()
        
        # Waypoint info
        distance_to_wp = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        wp_status = 'good' if distance_to_wp <= 2.0 else 'warn'
        self.draw_value_display("WP DISTANCE", f"{distance_to_wp:.2f}", "m", wp_status, 120)
        
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
