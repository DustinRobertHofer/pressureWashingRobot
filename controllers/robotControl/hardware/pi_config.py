# Raspberry Pi Hardware Configuration for Robot Control

# Motor pins configuration
MOTOR_PINS = {
    'left_motor': {
        'pwm_pin': 12,      # PWM pin for left motor speed control
        'dir1_pin': 16,     # Direction pin 1 for left motor
        'dir2_pin': 18,     # Direction pin 2 for left motor
        'encoder_a': 23,    # Encoder A pin for left motor
        'encoder_b': 24     # Encoder B pin for left motor
    },
    'right_motor': {
        'pwm_pin': 13,      # PWM pin for right motor speed control
        'dir1_pin': 19,     # Direction pin 1 for right motor
        'dir2_pin': 26,     # Direction pin 2 for right motor
        'encoder_a': 5,     # Encoder A pin for right motor
        'encoder_b': 6      # Encoder B pin for right motor
    }
}

# Sensor pins configuration
SENSOR_PINS = {
    'laser_range': {
        'rx_pin': 20,       # Serial RX pin for laser range finder (or use None for I2C/hardware UART)
        'tx_pin': 21,       # Serial TX pin for laser range finder (or use None for I2C/hardware UART)
        'uart_port': '/dev/ttyAMA0',  # Hardware UART port (alternative to GPIO pins)
        'i2c_address': 0x62   # I2C address if using I2C interface
    },
    'compass': {
        'scl_pin': 3,       # I2C SCL pin for digital compass
        'sda_pin': 2,       # I2C SDA pin for digital compass
        'address': 0x1E     # I2C address for HMC5883L digital compass
    }
}

# Robot physical parameters
ROBOT_PARAMS = {
    'wheel_radius': 0.08,         # Wheel radius in meters
    'wheel_distance': 0.40244,    # Distance between wheels in meters
    'encoder_ticks_per_rev': 20,  # Encoder ticks per wheel revolution
    'cleaning_unit_diameter': 0.3048,  # meters (12 inches)
    'path_overlap': 0.1016,       # meters (4 inches)
}

# Motion parameters
MOTION_PARAMS = {
    'max_linear_speed': 0.6,      # meters/second
    'max_angular_speed': 0.5,     # radians/second
    'pwm_frequency': 1000,        # PWM frequency in Hz
    'min_duty_cycle': 20,         # Minimum PWM duty cycle (to overcome friction)
    'max_duty_cycle': 100         # Maximum PWM duty cycle
}

# Navigation parameters
NAVIGATION_PARAMS = {
    'waypoint_threshold': 0.1,    # meters (distance to consider waypoint reached)
    'heading_threshold': 0.05,    # radians (angle to consider heading aligned)
    'turn_threshold': 0.1,        # radians (angle to consider turn in place)
    'safe_distance': 0.3          # meters (minimum safe distance to obstacles)
}

# System parameters
SYSTEM_PARAMS = {
    'control_loop_interval': 0.05,  # seconds (50ms)
    'sensor_update_interval': 0.05  # seconds (50ms)
}

# Default cleaning area (in meters) - rectangular area
DEFAULT_CLEANING_AREA = [
    {'x': 0.0, 'y': 0.0},    # Starting point
    {'x': 2.8, 'y': 0.0},    # Right edge
    {'x': 2.8, 'y': 3.0},    # Top-right corner
    {'x': 0.0, 'y': 3.0},    # Top-left corner
] 