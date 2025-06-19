"""ImGui UI panels for drone control and telemetry - Modular SCADA Interface."""

from .base_panel import BasePanel
from .mission_control_panel import MissionControlPanel
from .situational_awareness_panel import SituationalAwarenessPanel
from .terrain_telemetry_panel import TerrainTelemetryPanel
from .debug_panel import DebugPanel


class UIPanels:
    """Manages all ImGui UI panels for the simulation using modular components."""
    
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
        
        # Initialize modular panels
        self.mission_control = MissionControlPanel(window_width, window_height)
        self.situational_awareness = SituationalAwarenessPanel(window_width, window_height)
        self.terrain_telemetry = TerrainTelemetryPanel(window_width, window_height)
        self.debug_panel = DebugPanel(window_width, window_height)
    
    def draw_mission_control_panel(self, sim, camera_controller):
        """Draw the mission control panel."""
        self.mission_control.draw(sim, camera_controller)
    
    def draw_situational_awareness_panel(self, sim):
        """Draw the situational awareness panel."""
        self.situational_awareness.draw(sim)
    
    def draw_terrain_telemetry(self, sim, environment):
        """Draw terrain distance telemetry."""
        self.terrain_telemetry.draw(sim, environment)
    
    def draw_debug_panel(self, debug_config=None):
        """Draw debug control panel."""
        self.debug_panel.draw(debug_config)
    
    def update_window_size(self, width, height):
        """Update the window dimensions for all panels."""
        self.window_width = width
        self.window_height = height
        
        # Update all panels
        self.mission_control.update_window_size(width, height)
        self.situational_awareness.update_window_size(width, height)
        self.terrain_telemetry.update_window_size(width, height)
        self.debug_panel.update_window_size(width, height)
  
