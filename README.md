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
