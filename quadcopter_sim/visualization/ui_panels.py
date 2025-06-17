"""ImGui UI panels for drone control and telemetry."""

import numpy as np
import imgui


class UIPanels:
    """Manages all ImGui UI panels for the simulation."""
    
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
    
    def draw_mission_control_panel(self, sim, camera_controller):
        """Draw the main mission control panel."""
        io = imgui.get_io()
        io.display_size = (self.window_width, self.window_height)
        
        imgui.set_next_window_position(10, 10)
        imgui.set_next_window_size(350, 500)
        imgui.begin("Mission Control", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        
        # Control buttons
        if imgui.button("Take Off"):
            sim.takeoff(target_altitude=3.0)
        imgui.same_line()
        if imgui.button("Land"):
            sim.land()
        imgui.same_line()
        if imgui.button("Hover"):
            sim.hover()
        imgui.same_line()
        if imgui.button("Resume WP"):
            sim.resume_waypoint_progression()
        
        imgui.separator()
        
        if imgui.button("Pause"): 
            pass
        imgui.same_line()
        if imgui.button("Resume"): 
            pass
        imgui.same_line()
        if imgui.button("Reset"): 
            sim.reset()
        
        imgui.separator()
        
        # Telemetry
        self._draw_telemetry(sim)
        
        imgui.separator()
        
        # Camera controls
        self._draw_camera_controls(camera_controller)
        
        imgui.separator()
        
        # Target speed control
        changed, new_speed = imgui.slider_float("Target Speed (m/s)", sim.target_speed, 0.5, 10.0)
        if changed:
            sim.set_target_speed(new_speed)
        
        imgui.separator()
        
        # Manual control
        self._draw_manual_controls(sim)
        
        imgui.separator()
        
        # Altitude plot
        if len(sim.trajectory) > 1:
            altitudes = np.array([p[2] for p in sim.trajectory[-100:]], dtype=np.float32)
            imgui.plot_lines("Altitude (last 100 steps)", altitudes, graph_size=(300, 80))
        
        imgui.end()
    
    def draw_situational_awareness_panel(self, sim):
        """Draw the situational awareness panel."""
        imgui.set_next_window_position(self.window_width - 370, 10)
        imgui.set_next_window_size(360, 0)
        imgui.push_style_var(imgui.STYLE_WINDOW_ROUNDING, 8)
        imgui.push_style_var(imgui.STYLE_WINDOW_BORDERSIZE, 1)
        imgui.push_style_var(imgui.STYLE_WINDOW_PADDING, (10, 8))
        imgui.push_style_var(imgui.STYLE_ALPHA, 0.92)
        
        imgui.begin("Situational Awareness", 
                   flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | 
                         imgui.WINDOW_ALWAYS_AUTO_RESIZE | imgui.WINDOW_NO_COLLAPSE | 
                         imgui.WINDOW_NO_TITLE_BAR)
        
        pos, vel = sim.state[:3], sim.state[3:]
        
        # Altitude gauge
        imgui.text("Altitude")
        imgui.progress_bar(pos[2] / 5.0, size=(320, 18), overlay=f"{pos[2]:.2f} m")
        
        # Ground speed gauge
        imgui.text("Ground Speed")
        ground_speed = np.linalg.norm(vel[:2])
        imgui.progress_bar(min(ground_speed / 5.0, 1.0), size=(320, 18), 
                          overlay=f"{ground_speed:.2f} m/s")
        
        # Vertical speed bar (centered at 0)
        imgui.text("Vertical Speed")
        vs = vel[2]
        imgui.progress_bar((vs + 2.5) / 5.0, size=(320, 18), overlay=f"{vs:+.2f} m/s")
        
        # Distance to waypoint
        imgui.text("Distance to Waypoint")
        dist = np.linalg.norm(pos - sim.waypoints[sim.wp_index])
        imgui.progress_bar(min(dist / 5.0, 1.0), size=(320, 18), overlay=f"{dist:.2f} m")
        
        # Rotor RPM bars
        imgui.text("Rotor RPM")
        for i, rpm in enumerate(sim.rotor_speeds):
            imgui.progress_bar(min(rpm / 12000.0, 1.0), size=(75, 12), overlay=f"{int(rpm)}")
            if i < 3:
                imgui.same_line()
        imgui.new_line()
        
        # Mission progress
        self._draw_mission_progress(sim)
        
        # Recent altitude plot
        if len(sim.trajectory) > 1:
            altitudes = np.array([p[2] for p in sim.trajectory[-100:]], dtype=np.float32)
            imgui.text("Recent Altitude")
            imgui.plot_lines("", altitudes, graph_size=(320, 60))
        
        imgui.pop_style_var(4)
        imgui.end()
    
    def draw_terrain_telemetry(self, sim, environment):
        """Draw terrain distance telemetry."""
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
        
        imgui.separator()
        imgui.text("Box Corner Distances to Terrain (m):")
        for i, dist in enumerate(distances):
            imgui.text(f"Corner {i+1}: {dist:.3f} m")
        min_dist = min(distances)
        imgui.text(f"Closest Corner to Terrain: {min_dist:.3f} m")
        imgui.separator()
    
    def _draw_telemetry(self, sim):
        """Draw basic telemetry information."""
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
        
        # Angle telemetry
        roll, pitch, yaw = np.degrees(sim.state[6:9])
        imgui.text(f"Angle (deg): Roll={roll:.1f}, Pitch={pitch:.1f}, Yaw={yaw:.1f}")
    
    def _draw_camera_controls(self, camera_controller):
        """Draw camera control sliders."""
        imgui.text("Camera Controls")
        _, camera_controller.angle_x = imgui.slider_float("Angle X", camera_controller.angle_x, -90.0, 90.0)
        _, camera_controller.angle_y = imgui.slider_float("Angle Y", camera_controller.angle_y, -180.0, 180.0)
        _, camera_controller.angle_z = imgui.slider_float("Angle Z", camera_controller.angle_z, -180.0, 180.0)
        _, camera_controller.zoom = imgui.slider_float("Zoom", camera_controller.zoom, 0.2, 3.0)
    
    def _draw_manual_controls(self, sim):
        """Draw manual control interface."""
        changed, new_manual_mode = imgui.checkbox("Manual Mode", sim.manual_mode)
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
            imgui.text("Manual Propeller RPM Control")
            for i in range(4):
                _, rpm = imgui.slider_float(f"Propeller {i+1} RPM", 
                                          float(sim.manual_rpms[i]), 
                                          sim.min_rpm, sim.max_rpm)
                sim.manual_rpms[i] = rpm
    
    def _draw_mission_progress(self, sim):
        """Draw mission progress based on path distance."""
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
            imgui.text("Mission Progress")
            imgui.progress_bar(progress_ratio, size=(320, 18), 
                             overlay=f"{int(progress_ratio*100)}%")
        except Exception:
            # Fallback to simple waypoint-based progress
            progress_ratio = sim.wp_index / max(len(sim.waypoints) - 1, 1)
            imgui.text("Mission Progress")
            imgui.progress_bar(progress_ratio, size=(320, 18), 
                             overlay=f"{int(progress_ratio*100)}%")
    
    def update_window_size(self, width, height):
        """Update the window dimensions."""
        self.window_width = width
        self.window_height = height
