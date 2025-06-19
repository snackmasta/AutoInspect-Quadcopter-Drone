"""ImGui UI panels for drone control and telemetry - Modular SCADA Interface."""

import imgui
from .base_panel import BasePanel
from .mission_control_panel import MissionControlPanel
from .situational_awareness_panel import SituationalAwarenessPanel
from .terrain_telemetry_panel import TerrainTelemetryPanel
from .debug_panel import DebugPanel
from .panel_layout_manager import PanelLayoutManager


class UIPanels:
    """Manages all ImGui UI panels for the simulation using modular components."""
    
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
          # Initialize layout manager
        self.layout_manager = PanelLayoutManager(window_width, window_height)
        
        # Register panels with layout manager
        self.layout_manager.register_panel('mission_control', 'left', (420, 400))
        self.layout_manager.register_panel('terrain_telemetry', 'left', (420, 200))
        self.layout_manager.register_panel('situational_awareness', 'right', (420, 350))
        self.layout_manager.register_panel('debug_panel', 'right', (420, 200))
        
        # Initialize modular panels
        self.mission_control = MissionControlPanel(window_width, window_height)
        self.situational_awareness = SituationalAwarenessPanel(window_width, window_height)
        self.terrain_telemetry = TerrainTelemetryPanel(window_width, window_height)
        self.debug_panel = DebugPanel(window_width, window_height)
          # Update initial layout
        self.layout_manager.update_layout()
    
    def draw_mission_control_panel(self, sim, camera_controller):
        """Draw the mission control panel."""
        self.layout_manager.begin_panel('mission_control')
        self.mission_control.draw(sim, camera_controller, self.layout_manager)
        self.layout_manager.capture_panel_size('mission_control')
    
    def draw_situational_awareness_panel(self, sim):
        """Draw the situational awareness panel."""
        self.layout_manager.begin_panel('situational_awareness')
        self.situational_awareness.draw(sim, self.layout_manager)
        self.layout_manager.capture_panel_size('situational_awareness')
    
    def draw_terrain_telemetry(self, sim, environment):
        """Draw terrain distance telemetry."""
        self.layout_manager.begin_panel('terrain_telemetry')
        self.terrain_telemetry.draw(sim, environment, self.layout_manager)
        self.layout_manager.capture_panel_size('terrain_telemetry')
    
    def draw_debug_panel(self, debug_config=None):
        """Draw debug control panel."""
        self.layout_manager.begin_panel('debug_panel')
        self.debug_panel.draw(debug_config, self.layout_manager)
        self.layout_manager.capture_panel_size('debug_panel')
    
    def finalize_frame(self):
        """Call this after all panels have been drawn to update layout for next frame."""        # Update layout after all panels are rendered
        self.layout_manager.update_layout()
    
    def draw_all_panels(self, sim, camera_controller, environment, debug_config=None, terrain_scanner=None):
        """Draw all UI panels in proper sequence and finalize layout."""
        self.draw_mission_control_panel(sim, camera_controller)
        self.draw_situational_awareness_panel(sim)
        
        # Camera display in ImGui (if terrain scanner provided)
        if terrain_scanner:
            imgui.separator()
            if terrain_scanner.show_camera:
                imgui.text("Camera (heightmap, center)")
                cam_data = terrain_scanner.get_camera_display_data(sim, environment)
                if cam_data is not None:
                    imgui.plot_lines("", cam_data, graph_size=(300, 60))
                imgui.separator()
        
        self.draw_terrain_telemetry(sim, environment)
        self.draw_debug_panel(debug_config)
        
        # Finalize layout after all panels are rendered
        self.finalize_frame()
    
    def update_window_size(self, width, height):
        """Update the window dimensions for all panels."""
        self.window_width = width
        self.window_height = height
        
        # Update layout manager
        self.layout_manager.update_window_size(width, height)
        
        # Update all panels
        self.mission_control.update_window_size(width, height)
        self.situational_awareness.update_window_size(width, height)
        self.terrain_telemetry.update_window_size(width, height)
        self.debug_panel.update_window_size(width, height)