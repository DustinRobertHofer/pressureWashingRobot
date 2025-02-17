# Physics Simulation Parameters
SIMULATION_PARAMS = {
    'basic_time_step': 32,  # milliseconds (increased from default for better stability)
}

# Robot Physical Parameters
ROBOT_PARAMS = {
    'wheel_radius': 0.08,  # meters
    'wheel_distance': 0.40244,  # meters
    'surface_cleaner_diameter': 12,  # inches
    'path_overlap': 3,  # inches
    'edge_buffer': 8,  # inches
}

# Motion Parameters
MOTION_PARAMS = {
    'max_linear_speed': 0.5,  # meters/second
    'max_angular_speed': 1.0,  # radians/second
}

# Navigation Parameters
NAVIGATION_PARAMS = {
    'waypoint_threshold': 0.1,  # meters (distance to consider waypoint reached)
    'heading_threshold': 0.1,  # radians (angle to consider heading aligned)
}

# Cleaning Area Parameters
CLEANING_AREAS = {
    'rectangle': [
        {'x': 0.0, 'y': 0.0},     # Starting point
        {'x': 2.8, 'y': 0.0},     # Right edge
        {'x': 2.8, 'y': 3.0},     # Top-right corner
        {'x': 0.0, 'y': 3.0},     # Top-left corner
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