# Visualization Module

This module contains the refactored visualization components for the quadcopter simulation, breaking down the monolithic `renderer.py` into smaller, more manageable components.

## Architecture

The visualization system is now split into four main components:

### 1. `CameraController` (`camera_controller.py`)
- Handles 3D camera control and mouse interactions
- Manages camera angles (x, y, z) and zoom level
- Provides orbital camera movement around the drone
- Sets up OpenGL camera view and projection matrices

### 2. `UIPanels` (`ui_panels.py`)
- Manages all ImGui user interface panels
- **Mission Control Panel**: Flight controls, telemetry, manual mode
- **Situational Awareness Panel**: Progress bars, gauges, mission progress
- **Terrain Telemetry**: Distance measurements to terrain
- Modular panel system for easy extension

### 3. `SceneRenderer` (`scene_renderer.py`)
- Handles all 3D OpenGL scene rendering
- Renders drone components (structure, rotors, blades, body box)
- Draws environment, waypoints, and trajectory
- Visualizes sensors (camera FOV, thrust arrows)
- Manages text rendering and visual effects

### 4. `TerrainScanner` (`terrain_scanner.py`)
- Manages camera chunks for terrain mapping
- Handles overlapping chunk detection and culling
- Provides terrain scanning visualization
- Accumulates scanned terrain data over time

## Main Renderer

The main `Renderer` class in `renderer.py` now acts as an orchestrator that:
- Initializes and coordinates all visualization components
- Maintains legacy API compatibility
- Provides a clean interface for the simulation

## Benefits of Refactoring

1. **Separation of Concerns**: Each component has a single, well-defined responsibility
2. **Maintainability**: Easier to modify individual components without affecting others
3. **Testability**: Components can be tested in isolation
4. **Extensibility**: New visualization features can be added as separate components
5. **Code Reusability**: Components can be reused in different contexts
6. **Legacy Compatibility**: Existing code continues to work without modifications

## Usage

The refactored renderer maintains the same API as the original:

```python
from quadcopter_sim.renderer import Renderer

renderer = Renderer(sim)
renderer.handle_mouse(window)
renderer.draw_scene()
renderer.reshape(width, height)
```

## Component Dependencies

```
Renderer
├── CameraController (camera control)
├── UIPanels (UI interface)
│   └── requires sim, environment, camera_controller
├── SceneRenderer (3D rendering)
│   └── requires sim, thrust visualization
└── TerrainScanner (terrain mapping)
    └── requires sim, environment
```

## Future Enhancements

The modular structure makes it easy to add:
- New UI panels for different drone modes
- Additional 3D visualization components
- Enhanced terrain analysis tools
- Performance profiling panels
- Debug visualization overlays
