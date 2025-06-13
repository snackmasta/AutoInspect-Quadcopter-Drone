# Drone Quadcopter Simulation

A modular Python simulation of a quadcopter drone navigating 3D waypoints with real-time 3D visualization and rotor speed plots.

## Features

- Modular codebase (core, visualization, waypoints, logging)
- 3D animation of drone and waypoints
- Rotor speed plotting
- Logging of waypoint events

## Requirements

To run this project, you need the following Python libraries:

- `numpy`
- `matplotlib`

You can install the required libraries using pip:

```sh
pip install numpy matplotlib
```

## How to Run

1. Clone or download this repository.
2. Open a terminal and navigate to the project directory.
3. Run the simulation:

   ```sh
   python main.py
   ```

4. The simulation will display two windows:
   - A 3D visualization of the quadcopter's movement.
   - A 2D plot showing the rotor speeds over time.

## File Structure

- `drone/` - Main package
  - `core.py` - Drone physics and state
  - `visualization.py` - 3D/2D plotting and animation
  - `waypoints.py` - Waypoint generation
  - `logging_utils.py` - Logging setup
- `main.py` - Entry point

## Simulation Details

- **Constants**:
  - `g`: Gravitational acceleration (9.81 m/s²).
  - `m`: Mass of the quadcopter (0.5 kg).
  - `dt`: Time step for simulation (0.02 s).
  - `L`: Distance between rotors (0.6 m).

- **Waypoints**:
  The quadcopter navigates through the following waypoints:
  - `[0, 0, 2]`
  - `[2, 0, 2]`
  - `[2, 2, 3]`
  - `[0, 2, 2]`
  - `[0, 0, 0]`

- **Logging**:
  Rotor speeds and waypoint reach times are logged to `waypoint_rotor_speeds.log`.

## Visualization

- **3D Visualization**:
  - Blue line: Quadcopter trajectory.
  - Red dots: Rotor positions.
  - Green dot: Quadcopter center.
  - Green "X": Current target waypoint.

- **Rotor Speed Plot**:
  - Displays the rotor speeds over time.
  - Vertical red dashed lines indicate when waypoints are reached.