"""Situational awareness panel for real-time status monitoring."""

import numpy as np
import imgui
from .base_panel import BasePanel


class SituationalAwarenessPanel(BasePanel):
    """Situational awareness panel with real-time monitoring."""
    
    def draw(self, sim):
        """Draw the situational awareness panel with SCADA styling."""
        self.apply_scada_theme()        # Position on the right side, auto-size to fit content
        imgui.set_next_window_position(self.window_width - 430, 10)
        imgui.begin("■ SITUATIONAL AWARENESS", 
                   flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | 
                         imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_ALWAYS_AUTO_RESIZE)
        
        pos, vel = sim.state[:3], sim.state[3:]
        
        # Flight status section
        self.draw_section_header("FLIGHT STATUS")
        
        # Key flight parameters with status coloring
        altitude_status = 'good' if 1.0 <= pos[2] <= 10.0 else 'warn' if pos[2] > 0.5 else 'alarm'
        ground_speed = np.linalg.norm(vel[:2])
        speed_status = 'good' if ground_speed <= 5.0 else 'warn' if ground_speed <= 8.0 else 'alarm'
        vs = vel[2]
        vs_status = 'good' if abs(vs) <= 1.0 else 'warn' if abs(vs) <= 2.0 else 'alarm'
        
        self.draw_value_display("ALTITUDE", f"{pos[2]:.2f}", "m", altitude_status, 140)
        self.draw_value_display("GROUND SPEED", f"{ground_speed:.2f}", "m/s", speed_status, 140)
        self.draw_value_display("VERTICAL SPEED", f"{vs:+.2f}", "m/s", vs_status, 140)
        
        imgui.separator()
        
        # Navigation section
        self.draw_section_header("NAVIGATION")
        
        dist_to_wp = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        nav_status = 'good' if dist_to_wp <= 2.0 else 'warn'
        
        self.draw_value_display("WAYPOINT", f"{sim.wp_index+1}/{len(sim.waypoints)}", "", 'good', 140)
        self.draw_value_display("DISTANCE TO WP", f"{dist_to_wp:.2f}", "m", nav_status, 140)
        
        # Mission progress bar
        imgui.text("PROGRESS:")
        self._draw_mission_progress(sim)
        
        imgui.separator()
        
        # Propulsion system section
        self.draw_section_header("PROPULSION")
        
        # RPM bars in a 2x2 grid - more compact
        imgui.columns(2, "rpm_grid")
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
            imgui.progress_bar(rpm_ratio, size=(80, 15), overlay=f"M{i+1}: {int(rpm)}")
            imgui.pop_style_color()
            
            if i % 2 == 1:  # After every second motor, go to next column
                imgui.next_column()
        imgui.columns(1)
        
        imgui.separator()
        
        # System status section
        self.draw_section_header("SYSTEM STATUS")
        
        # System status indicators
        if hasattr(sim.state_manager, 'crashed') and sim.state_manager.crashed:
            self.draw_status_indicator("FLIGHT SYSTEM", 'alarm')
        else:
            self.draw_status_indicator("FLIGHT SYSTEM", 'good')
        
        safety_status = 'good' if sim.safety_system_enabled else 'warn'
        manual_status = 'warn' if sim.manual_mode else 'good'
        
        self.draw_status_indicator("SAFETY SYSTEM", safety_status)
        self.draw_status_indicator("AUTO CONTROL", manual_status)
        
        imgui.end()
    
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
