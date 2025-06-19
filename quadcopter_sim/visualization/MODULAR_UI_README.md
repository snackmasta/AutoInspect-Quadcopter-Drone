# Modular UI Panel System

This directory contains a modular SCADA-style UI system for the quadcopter simulation.

## Overview

The UI system has been refactored from a monolithic design to a modular architecture for better maintainability and code organization.

## Architecture

### Base Components

- **`base_panel.py`**: Common SCADA styling, theme management, and utility methods shared by all panels.

### Specialized Panels

- **`mission_control_panel.py`**: Flight operations, telemetry data, manual controls, and camera controls.
- **`situational_awareness_panel.py`**: Real-time flight status, navigation data, propulsion system monitoring, and flight data visualization.
- **`terrain_telemetry_panel.py`**: Terrain clearance monitoring and distance measurements.
- **`debug_panel.py`**: System diagnostics and debug controls for development.

### Main Interface

- **`ui_panels.py`**: Thin wrapper that coordinates all modular panels and provides a unified interface.

## SCADA Theme Features

- **Industrial Color Scheme**: Dark backgrounds with bright status indicators
- **Status Indicators**: Color-coded circular indicators (green/yellow/red)
- **Large Buttons**: Touch-friendly controls with emergency/active color schemes  
- **Grid Layout**: Organized control groups and information displays
- **Typography**: Bold, uppercase headers with clear visual hierarchy

## Usage

```python
from quadcopter_sim.visualization.ui_panels import UIPanels

# Initialize the UI system
ui_panels = UIPanels(window_width=1200, window_height=800)

# Draw panels (called from main render loop)
ui_panels.draw_mission_control_panel(sim, camera_controller)
ui_panels.draw_situational_awareness_panel(sim)
ui_panels.draw_terrain_telemetry(sim, environment)
ui_panels.draw_debug_panel(debug_config)
```

## Benefits of Modular Design

1. **Maintainability**: Each panel is self-contained and easy to modify
2. **Reusability**: Panels can be used independently or in different combinations
3. **Testing**: Individual panels can be tested in isolation
4. **Performance**: Only load/update panels that are actually used
5. **Collaboration**: Multiple developers can work on different panels simultaneously

## File Structure

```text
visualization/
├── base_panel.py              # Common SCADA theme and utilities
├── mission_control_panel.py   # Flight operations panel
├── situational_awareness_panel.py  # Status monitoring panel
├── terrain_telemetry_panel.py # Terrain clearance panel
├── debug_panel.py             # Debug/diagnostics panel
├── ui_panels.py               # Main UI coordinator (50 lines vs 681 before)
└── MODULAR_UI_README.md       # This documentation
```

## Color Scheme

The SCADA theme uses an industrial color palette:

- **Backgrounds**: Dark grays (#1a1a1a, #2a2a2a)
- **Status Good**: Green (#00ff88)
- **Status Warning**: Yellow (#ffaa00)  
- **Status Alarm**: Red (#ff4444)
- **Accents**: Cyan (#00aaff) and Blue (#0088ff)
- **Text**: Light gray (#e0e0e0) primary, medium gray (#a0a0a0) secondary

## Migration Notes

The refactoring removed ~631 lines of duplicated code from the main `ui_panels.py` file while maintaining full backward compatibility with existing renderer integration.
