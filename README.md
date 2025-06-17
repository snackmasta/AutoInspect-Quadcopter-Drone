# Quadcopter Simulation Project

A comprehensive quadcopter simulation system with physics, control, and visualization components.

## Project Structure

```text
├── quadcopter_sim/          # Main simulation package
│   ├── controllers/         # Control algorithms (PID, LQR, etc.)
│   ├── core/               # Core simulation components
│   ├── utils/              # Utility functions
│   └── visualization/      # Visualization components
├── tools/                  # Development and tuning tools
├── examples/               # Example scripts and demos
├── debug/                  # Debug utilities and scripts
├── config/                 # Configuration files
├── docs/                   # Documentation
├── tests/                  # Test files
├── slides/                 # Presentation materials
└── apps/                   # Application entry points
```

## Quick Start

1. Install dependencies (if using virtual environment):

   ```bash
   pip install -r requirements.txt
   ```

2. Run the main simulation:

   ```bash
   python -m quadcopter_sim.main
   ```

## Development

- Use tools in the `tools/` directory for calibration and tuning
- Examples and demos are available in the `examples/` directory
- Debug utilities can be found in the `debug/` directory

## Documentation

See the `docs/` directory for detailed documentation and the `slides/` directory for presentation materials.
