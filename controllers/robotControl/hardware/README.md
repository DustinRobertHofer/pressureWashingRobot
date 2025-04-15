# Pressure Washing Robot - Raspberry Pi Implementation

This directory contains the hardware implementation of the pressure washing robot control system designed to run on a Raspberry Pi 4B.

## Hardware Requirements

- Raspberry Pi 4B
- L298N or similar motor driver module
- 2 DC motors with encoders
- Laser range finder (TFmini, VL53L0X, or similar)
- HMC5883L digital compass for heading tracking
- Power supply for Raspberry Pi and motors
- Pressure washer attachment (hardware specific)

## Pin Connections

The default pin connections are defined in `pi_config.py` and can be modified as needed:

### Motor Connections:
- Left Motor:
  - PWM: GPIO 12
  - Direction 1: GPIO 16
  - Direction 2: GPIO 18
  - Encoder A: GPIO 23
  - Encoder B: GPIO 24

- Right Motor:
  - PWM: GPIO 13
  - Direction 1: GPIO 19
  - Direction 2: GPIO 26
  - Encoder A: GPIO 5
  - Encoder B: GPIO 6

### Sensor Connections:
- Laser Range Finder (Choose one interface method):
  - I2C:
    - SDA: GPIO 2
    - SCL: GPIO 3
  - UART:
    - Use hardware UART: /dev/ttyAMA0
  - Software Serial:
    - RX: GPIO 20
    - TX: GPIO 21

- Digital Compass HMC5883L:
  - SDA: GPIO 2 (I2C1)
  - SCL: GPIO 3 (I2C1)
  - Address: 0x1E (default for HMC5883L)

## Installation

1. Clone the repository onto your Raspberry Pi:
   ```
   git clone https://github.com/yourusername/pressure_washing_robot.git
   cd pressure_washing_robot
   ```

2. Install the required dependencies:
   ```
   sudo bash controllers/robotControl/hardware/install_dependencies.sh
   ```

3. Reboot the Raspberry Pi to apply I2C, UART, and PWM settings:
   ```
   sudo reboot
   ```

## Configuration

Modify the configuration parameters in `pi_config.py` to match your robot's physical properties and performance requirements:

- Motor pins
- Sensor pins (adjust for your specific laser range finder interface)
- Robot dimensions
- Speed limits
- Path planning parameters

## Running the Robot

1. Manual start:
   ```
   cd pressure_washing_robot
   python3 controllers/robotControl/hardware/pi_robot_controller.py
   ```

2. Automatic start as a service:
   ```
   sudo systemctl enable pressure_washing_robot.service
   sudo systemctl start pressure_washing_robot.service
   ```

## System Architecture

The system consists of the following components:

- `pi_robot_controller.py`: Main controller that coordinates all subsystems
- `pi_motion_controller.py`: Handles motor control for robot movement
- `pi_sensor_manager.py`: Manages all sensor readings
- `pi_motor.py`: Implements motor control via GPIO and PWM
- `pi_distance_sensor.py`: Interfaces with the laser range finder
- `pi_compass.py`: Handles heading using HMC5883L digital compass
- `pi_path_planner.py`: Generates cleaning paths based on boundary points
- `pi_config.py`: Configuration parameters for all components

## Troubleshooting

- **Motors not moving**: Check motor driver connections and power supply
- **Inaccurate navigation**: Run the compass calibration procedure
- **Laser range finder errors**: Verify connection type (I2C/UART) and power supply
- **System crashes**: Check the system logs with `journalctl -u pressure_washing_robot.service`

## Extending the System

To add additional features:
- Add new sensor implementations in the `hardware` directory
- Modify the path planning algorithm in `pi_path_planner.py`
- Improve obstacle avoidance by extending the `_check_obstacles` method

## License

This project is licensed under the MIT License - see the LICENSE file for details. 