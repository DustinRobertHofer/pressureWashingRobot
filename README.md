# Pressure Washing Robot

A complete simulation-based project for an autonomous pressure washing robot that can efficiently clean large surfaces while following optimized cleaning paths.

![Pressure Washing Robot CAD Model](assets/robotCAD.png)

## Project Overview

This project implements an autonomous robot system designed for pressure washing large surfaces such as driveways, patios, or parking lots. The robot:

1. Navigates through predefined areas using optimized cleaning patterns
2. Avoids obstacles using distance sensors
3. Efficiently covers surfaces with minimal overlap between cleaning passes
4. Provides detailed simulation through Webots robotics simulator

![Pressure Washing Robot Simulation](worlds/.Rectangle_Arena.jpg)

## Project Structure

```
├── assets/                     # Project assets
│   └── robotCAD.png            # Robot CAD model image
├── controllers/                # Robot control logic
│   └── robotControl/           # Main robot controller
│       ├── actuators/          # Motor and actuator control modules
│       │   └── motor.py        # Motor control interface
│       ├── config/             # Configuration settings
│       │   └── robot_config.py # Robot parameters and cleaning areas
│       ├── navigation/         # Path navigation logic
│       │   └── navigator.py    # Navigation control system (4.0KB)
│       ├── sensors/            # Sensor input processing
│       │   ├── distanceSensor.py  # Proximity detection
│       │   ├── laserRange.py      # Laser rangefinder interface
│       │   ├── wheelSensors.py    # Wheel encoder interface
│       │   └── digitalCompass.py  # Orientation detection
│       ├── utils/              # Utility modules
│       │   ├── motionController.py # Movement control system
│       │   ├── path_planner.py     # Path planning utilities
│       │   ├── sensorManager.py    # Sensor data aggregation
│       │   └── state.py            # Robot state tracking
│       └── robotControl.py     # Main controller entry point (3.3KB)
├── scripts/                    # Standalone scripts
│   └── pathGeneration/         # Path generation tools and algorithms
│       ├── PathGenerator.py    # Current production path generator (25KB)
│       ├── PathGeneratorV1.py  # Initial path generator implementation
│       ├── PathGeneratorV2.py  # Improved path generator
│       └── PathGeneratorV3.py  # Enhanced path generator with obstacle avoidance
├── worlds/                     # Webots simulation worlds
│   └── Rectangle_Arena.wbt     # Main simulation environment
└── .gitignore                  # Git ignore file
```

## Features

- **Optimized Cleaning Path Generation**: Creates efficient cleaning paths to cover different surface shapes (rectangle, L-shape, etc.)
- **Obstacle Detection and Avoidance**: Uses distance sensors to detect and navigate around obstacles
- **Configurable Cleaning Parameters**: 
  - Surface cleaner diameter
  - Path overlap percentage
  - Edge buffer distance
- **Motion Control System**: Precise control of robot movement and cleaning mechanisms
- **Simulation Environment**: Complete Webots simulation for testing and visualization

## Requirements

- Python 3.8+
- Webots R2023a or newer
- Python packages:
  - numpy
  - opencv-python
  - controller (Webots Python API)

## Installation

1. Install Webots from [cyberbotics.com](https://cyberbotics.com/)
2. Clone this repository:
   ```
   git clone https://github.com/yourusername/pressureWashingRobot.git
   cd pressureWashingRobot
   ```
3. Install required Python packages:
   ```
   pip install numpy opencv-python
   ```

## Usage

### Running the Simulation

1. Open Webots and load the world file:
   ```
   File > Open World > /path/to/pressureWashingRobot/worlds/Rectangle_Arena.wbt
   ```
2. Click the "Play" button to start the simulation

### Configuring Cleaning Areas

Cleaning areas can be configured in the `controllers/robotControl/config/robot_config.py` file:

```python
CLEANING_AREAS = {
    'rectangle': [
        (0, 0),
        (9, 0),
        (9, 9),
        (0, 9)
    ],
    'L_shape': [
        # Define L-shape coordinates here
    ]
}
```

### Adjusting Cleaning Parameters

Path generation parameters can be adjusted in the path generator:

- Surface cleaner diameter (in inches)
- Path overlap (in inches)
- Edge buffer radius (in inches)

## Path Generation

The project includes a sophisticated path generation algorithm that:

1. Takes boundary points as input
2. Calculates an efficient coverage pattern
3. Ensures proper overlap between passes
4. Provides buffer distance from edges
5. Optimizes total cleaning time


## Acknowledgments

- Webots robot simulator by Cyberbotics
- OpenCV for image processing functionality 