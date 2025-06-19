"""Terrain telemetry panel for ground clearance monitoring."""

import imgui
from .base_panel import BasePanel


class TerrainTelemetryPanel(BasePanel):
    """Terrain telemetry panel for ground clearance and distance monitoring."""
    
    def draw(self, sim, environment, layout_manager=None):
        """Draw terrain distance telemetry with SCADA styling."""
        self.apply_scada_theme()
        
        # Position is handled by layout manager if provided
        if layout_manager is None:
            # Fallback to old positioning
            imgui.set_next_window_position(10, 500)
        
        imgui.begin("■ TERRAIN TELEMETRY")
        
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
        
        self.draw_section_header("TERRAIN CLEARANCE")
        
        min_dist = min(distances)
        clearance_status = 'good' if min_dist > 2.0 else 'warn' if min_dist > 1.0 else 'alarm'
        
        self.draw_value_display("MIN CLEARANCE", f"{min_dist:.3f}", "m", clearance_status, 140)
        
        imgui.text("CORNER DISTANCES:")
        # Display corners in 2x2 grid for compactness
        imgui.columns(2, "corner_grid")
        for i, dist in enumerate(distances):
            corner_status = 'good' if dist > 1.5 else 'warn' if dist > 0.8 else 'alarm'
            self.draw_value_display(f"C{i+1}", f"{dist:.3f}", "m", corner_status, 60)
            if i % 2 == 1:  # After every second corner, go to next column
                imgui.next_column()
        imgui.columns(1)
        
        imgui.end()
