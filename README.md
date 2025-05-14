# Quadcopter Simulation

This project simulates the behavior of a quadcopter navigating through a series of waypoints in 3D space. The simulation includes real-time visualization of the quadcopter's trajectory, rotor speeds, and waypoint navigation.

## Features

- **3D Visualization**: Displays the quadcopter's position, trajectory, and rotor positions in a 3D plot.
- **Waypoint Navigation**: The quadcopter autonomously navigates through predefined waypoints.
- **Rotor Speed Monitoring**: Plots the rotor speeds over time with real-time updates.
- **Spin-Up Phase**: Simulates the rotor spin-up process before navigation begins.
- **Logging**: Logs rotor speeds and waypoint reach times to a file and console.

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
   python drone.py
   ```

4. The simulation will display two windows:
   - A 3D visualization of the quadcopter's movement.
   - A 2D plot showing the rotor speeds over time.

## File Structure

- `drone.py`: The main script that contains the simulation logic, visualization setup, and animation.

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