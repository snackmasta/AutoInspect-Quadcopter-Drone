# Controllers Module

This module contains various control algorithms for the quadcopter simulation.

## Available Controllers

### 1. Position Controller (`position_controller.py`)
- **Main function**: `position_controller(state, target, ...)`
- **Type**: PID-based position and attitude controller
- **Features**: 
  - Lookahead target selection for trajectory following
  - Tuned PID gains for minimal oscillation
  - Yaw control support
  - Target speed control
- **Usage**: Primary controller used by the FlightController in simulation

### 2. LQR Controller (`lqr_controller.py`) 
- **Main function**: `lqr_position_attitude_controller(state, target, ...)`
- **Type**: Linear Quadratic Regulator for optimal control
- **Features**:
  - Linearized model around hover point
  - Optimal control based on quadratic cost function
  - Separate gains for position and velocity errors
- **Usage**: Alternative controller for more advanced control scenarios

### 3. PID Controller (`pid_controller.py`)
- **Classes**: `PIDController`, `QuadcopterPIDController`
- **Function**: `simple_pid_controller(state, target, ...)`
- **Type**: Modular PID controllers for individual axes
- **Features**:
  - Individual PID controllers for each axis
  - Configurable gains and output limits
  - Reset functionality
  - Simple interface compatible with existing code

## Integration

The controllers are integrated into the simulation through the `FlightController` class in `core/flight_controller.py`. The `position_controller` is the primary controller used for:

- Waypoint following
- Takeoff and landing
- Hover control
- Manual flight assistance

## Usage Examples

```python
from quadcopter_sim.controllers import position_controller, PIDController

# Using the position controller
control_output = position_controller(state, target_position)

# Using the PID controller
pid = PIDController(kp=1.0, ki=0.1, kd=0.5)
error = target - current
control_signal = pid.update(error, dt)
```

## Configuration

Controller parameters can be modified by importing the modules and changing the global variables:

```python
import quadcopter_sim.controllers.position_controller as pc
pc.KP_POS = 1.2  # Adjust position gain
pc.KD_POS = 3.0  # Adjust damping
```
