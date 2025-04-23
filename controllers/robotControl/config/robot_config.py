# Operation Mode
OPERATION_MODE = {
    'use_ui': False,  # Set to True to use UI for area selection, False to use predefined cleaning areas
    'default_area': 'rectangle'  # Default cleaning area to use when use_ui is False
}

# Physics Simulation Parameters
SIMULATION_PARAMS = {
    'basic_time_step': 32,  # milliseconds (increased from default for better stability)
}

# Robot Physical Parameters
ROBOT_PARAMS = {
    'sensor_height' : 0.17145, #meters
    'sensor_pitch' : 0.637549, #radians
    'sensor_yaw' : 0.7368, #radians
    'sensor_offset' : 0.09652, #meters
    'left_motor_name': 'left_wheel',
    'right_motor_name': 'right_wheel',
    'wheel_radius': 0.08,  # meters
    'wheel_distance': 0.40244,  # meters
    'surface_cleaner_diameter': 12,  # inches
    'path_overlap': 4,  # inches (increased from 3 for better coverage)
    'edge_buffer': 6,  # inches (reduced from 8 to clean closer to edges)
}

# Motion Parameters
MOTION_PARAMS = {
    'max_linear_speed': 0.6,  # meters/second (slightly reduced for better accuracy)
    'max_angular_speed': 0.5,  # radians/second
    
}

# Navigation Parameters
NAVIGATION_PARAMS = {
    'waypoint_threshold': 0.01,  # meters (distance to consider waypoint reached)
    'heading_threshold': 0.01,  # radians (angle to consider heading aligned)
    'turn_threshold': 0.01,  # radians (angle to consider turn in place)
    'avoidance_threshold': 0.5,  # meters (distance to consider obstacle)
    'safe_distance': 0.075,  # meters (distance to maintain while avoiding)
}

# Visualization Parameters
VISUALIZATION_PARAMS = {
    'enable': True,
    'window_name': 'Robot Path Visualization',
    'width': 500,
    'height': 500,
    'update_interval': 3,  # Update visualization more frequently (was 5)
}

# Cleaning Area Parameters
CLEANING_AREAS = {
    'rectangle': [
        {'x': 0.0, 'y': 0.0},     # Starting point
        {'x': 2.0, 'y': 0.0},     # Right edge
        {'x': 2.0, 'y': 2.0},     # Top-right corner
        {'x': 0.0, 'y': 2.0},     # Top-left corner
    ],
    'L_shape': [
        {'x': 0.0, 'y': 0.0},     # Starting point
        {'x': 4.8, 'y': 0.0},     # Right edge of top
        {'x': 4.8, 'y': 1.5},     # Top-right inner corner
        {'x': 1.5, 'y': 1.5},     # Bottom-right inner corner
        {'x': 1.5, 'y': 4.3},     # Top-right outer corner
        {'x': 0.0, 'y': 4.3}      # Top-left corner
    ]
} 