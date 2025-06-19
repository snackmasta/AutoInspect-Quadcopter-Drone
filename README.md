# Quadcopter Simulation Project

A comprehensive quadcopter simulation system with realistic physics, multiple control algorithms, 3D visualization, and terrain mapping capabilities. Features modular architecture for research and development in drone control algorithms.

## Project Structure

```text
├── quadcopter_sim/          # Main simulation package
│   ├── controllers/         # Control algorithms (PID, LQR, position control)
│   ├── core/               # Core simulation components
│   │   ├── physics.py      # Physics engine and state updates
│   │   ├── state_manager.py # State management and tracking
│   │   ├── flight_controller.py # Flight control logic and modes
│   │   └── safety_system.py # Safety monitoring and recovery
│   ├── utils/              # Utility functions and helpers
│   ├── visualization/      # Modular visualization components
│   │   ├── camera_controller.py # 3D camera control
│   │   ├── ui_panels.py    # ImGui interface panels
│   │   ├── scene_renderer.py # 3D OpenGL rendering
│   │   └── terrain_scanner.py # Terrain mapping visualization
│   ├── simulation.py       # Main simulation entry point
│   ├── renderer.py         # Graphics renderer
│   ├── environment.py      # Terrain generation with Perlin noise
│   └── main_trajectory.py  # Predefined flight trajectories
├── tools/                  # Development and tuning tools
│   ├── calibrate_drone.py  # Drone parameter calibration
│   ├── pid_tuner.py        # PID controller optimization
│   └── reduce_oscillation.py # Oscillation reduction tuning
├── examples/               # Example scripts and demos
│   ├── drone_opengl.py     # Main 3D simulation with GUI
│   └── glfw_key_test.py    # Input testing utility
├── debug/                  # Debug utilities and configuration
│   ├── debug_config.py     # Debug toggles and settings
│   └── debug_physics.py    # Physics debugging tool
├── config/                 # Configuration files
├── docs/                   # Documentation
├── slides/                 # Presentation materials
└── tests/                  # Test files
```

## Features

### Core Simulation
- **Realistic Physics**: 6-DOF quadcopter dynamics with ground collision detection
- **Multiple Flight Modes**: Manual control, autonomous waypoint following, takeoff/landing, hover
- **Safety Systems**: Crash detection, recovery modes, emergency procedures
- **Terrain Environment**: Procedural terrain generation using Perlin noise

### Control Algorithms
- **PID Controllers**: Position and attitude control with tunable parameters
- **LQR Control**: Linear Quadratic Regulator for optimal control
- **Lookahead Control**: Advanced path following with predictive targeting

### Visualization & UI
- **3D OpenGL Rendering**: Real-time drone visualization with environment
- **Interactive GUI**: ImGui-based control panels and telemetry display
- **Camera System**: Orbital camera with mouse controls and zoom
- **Real-time Plotting**: Trajectory tracking, altitude plots, mission progress

### Development Tools
- **Parameter Tuning**: Automated PID gain optimization tools
- **Calibration Utilities**: Drone parameter calibration and testing
- **Debug Systems**: Comprehensive debugging with configurable output

## Dependencies

Install the required dependencies:

```bash
pip install numpy scipy OpenGL glfw imgui[opengl] perlin-noise scikit-optimize
```

### Core Dependencies
- `numpy` - Numerical computations and linear algebra
- `scipy` - Scientific computing (LQR controller)
- `OpenGL` - 3D graphics rendering
- `glfw` - Window management and input handling
- `imgui[opengl]` - Immediate mode GUI
- `perlin-noise` - Terrain generation
- `scikit-optimize` - Parameter optimization (optional)

## Quick Start

### 1. Run the Main 3D Simulation

```bash
python examples/drone_opengl.py
```

This launches the full 3D simulation with:
- Interactive 3D visualization
- Real-time control panels
- Manual and automatic flight modes
- Terrain mapping visualization

### 2. Run Physics Debug Mode

```bash
python debug/debug_physics.py
```

Monitor detailed physics state without visualization.

### 3. Parameter Tuning

```bash
python tools/pid_tuner.py        # Optimize PID gains
python tools/calibrate_drone.py  # Calibrate drone parameters
python tools/reduce_oscillation.py # Reduce control oscillations
```

## Usage

### Basic Simulation Setup

```python
from quadcopter_sim.simulation import QuadcopterSimulation

# Create simulation instance
sim = QuadcopterSimulation()

# Run simulation steps
for i in range(1000):
    sim.step(0.01)  # 10ms time step
    
    # Access state
    position = sim.state[:3]
    velocity = sim.state[3:6]
    orientation = sim.state[6:9]
```

### Flight Control

```python
# Takeoff to 3 meters
sim.takeoff(target_altitude=3.0)

# Hover at current position
sim.hover()

# Manual control mode
sim.manual_mode = True
sim.set_manual_hover()

# Landing sequence
sim.land()
```

### Accessing Sensor Data

```python
# Get rotor positions in world coordinates
rotor_positions = sim.rotor_positions()

# Get simulated camera image of terrain
camera_image = sim.get_camera_image(fov=60, res=32)

# Access trajectory history
trajectory = sim.trajectory
```

## Controls (in Manual Mode)

When running `examples/drone_opengl.py`:

- **M** - Toggle manual/automatic mode
- **W/S** - Pitch forward/backward
- **A/D** - Roll left/right  
- **Q/E** - Yaw left/right
- **R/F** - Increase/decrease throttle
- **Space** - Emergency stop (all rotors to 0)
- **V** - Reset velocities
- **Mouse** - Camera control (drag to orbit)
- **Scroll** - Zoom in/out

## Development

### Extending the Simulation

The modular architecture makes it easy to:

1. **Add New Controllers**: Implement in `quadcopter_sim/controllers/`
2. **Modify Physics**: Edit `quadcopter_sim/core/physics.py`
3. **Create Flight Modes**: Extend `quadcopter_sim/core/flight_controller.py`
4. **Add Visualizations**: Implement in `quadcopter_sim/visualization/`

### Debug Configuration

Configure debug output in `debug/debug_config.py`:

```python
DEBUG_PHYSICS = True        # Enable physics debugging
DEBUG_KEY_EVENT = True      # Debug keyboard input
DEBUG_MANUAL_RPMS = True    # Show rotor RPM values
```

### Tools and Utilities

- **Calibration**: Use `tools/calibrate_drone.py` to find optimal parameters
- **PID Tuning**: Run `tools/pid_tuner.py` for automated gain optimization
- **Oscillation Reduction**: Use `tools/reduce_oscillation.py` for stability
- **Input Testing**: Use `examples/glfw_key_test.py` to test keyboard input

## Architecture

The simulation follows a modular design:

- **`QuadcopterSimulation`**: Main orchestrator class
- **`PhysicsEngine`**: Handles dynamics and state updates
- **`FlightController`**: Manages flight modes and control logic
- **`StateManager`**: Tracks drone state and trajectory
- **`SafetySystem`**: Monitors safety and handles emergencies
- **`Renderer`**: Orchestrates all visualization components

## Documentation

- See `docs/` for detailed technical documentation
- Check `slides/` for presentation materials and project overview
- Refer to `REFACTORING_SUMMARY.md` for architecture details
- View `RENDERER_REFACTORING_SUMMARY.md` for visualization system

## Research Applications

This simulator is designed for:

- **Control Algorithm Development**: Test PID, LQR, and custom controllers
- **Mission Planning**: Waypoint navigation and trajectory optimization  
- **Sensor Simulation**: Camera-based terrain mapping and SLAM
- **Safety System Testing**: Crash detection and recovery algorithms
- **Performance Analysis**: Real-time telemetry and trajectory analysis

## License

[not yet]

## Build Guide

### Building a Standalone Windows Executable

You can build a standalone `.exe` for Windows using PyInstaller. This will bundle all dependencies, custom modules, and debug configuration for portable distribution.

#### 1. Install Requirements

Make sure you have all dependencies installed in your Python environment:

```bash
pip install -r requirements.txt
# Or manually:
pip install numpy scipy OpenGL glfw imgui[opengl] perlin-noise scikit-optimize pyinstaller
```

#### 2. Build the Executable

From the project root or `examples/` directory, run:

```powershell
# In PowerShell (Windows)
cd examples
pyinstaller QuadcopterSimulation.spec
```

- The build process uses the `QuacopterSimulation.spec` file to ensure all custom modules, DLLs, and debug configuration are included.
- The output executable will be in `examples/dist/QuadcopterSimulation.exe`.

#### 3. Run the Executable

```powershell
cd dist
./QuadcopterSimulation.exe
```

#### 4. Troubleshooting
- If you see missing DLL or import errors, ensure all dependencies are installed and the `debug/` directory is present.
- The build system uses a robust fallback for debug configuration, so the executable should run even if `debug_config.py` is missing.
- For advanced debugging, see `quadcopter_sim/debug_utils.py` and `quadcopter_sim/debug_fallback.py`.

#### 5. Customizing the Build
- Edit `QuacopterSimulation.spec` to add or exclude data files, DLLs, or hidden imports as needed.
- You can add runtime hooks for custom initialization if your environment changes.

### Step-by-Step Build Commands (Windows)

1. **Open PowerShell** and navigate to your project directory:

```powershell
cd E:\Obsidian\shigoto\Robotika\scara
```

2. **(Optional) Activate your virtual environment:**

```powershell
.\venv\Scripts\Activate.ps1
```

3. **Install all required dependencies:**

```powershell
pip install -r requirements.txt
# Or, if you don't have requirements.txt:
pip install numpy scipy OpenGL glfw imgui[opengl] perlin-noise scikit-optimize pyinstaller
```

4. **Build the executable using PyInstaller:**

```powershell
cd examples
pyinstaller QuadcopterSimulation.spec
```

5. **Run the generated executable:**

```powershell
cd dist
./QuadcopterSimulation.exe
```

6. **(Optional) Validate debug configuration:**

```powershell
cd ..\..\quadcopter_sim
python validate_debug.py
```

---
