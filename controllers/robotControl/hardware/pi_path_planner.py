import math
from hardware.pi_config import ROBOT_PARAMS

def generate_cleaning_path(boundary_points, path_overlap=None):
    """
    Generate a cleaning path based on boundary points.
    
    Args:
        boundary_points: List of dictionary points with 'x' and 'y' coordinates
        path_overlap: Overlap between cleaning paths in meters
        
    Returns:
        List of waypoints as dictionaries with 'x' and 'y' coordinates
    """
    if path_overlap is None:
        path_overlap = ROBOT_PARAMS['path_overlap']
    
    # Find bounds of the cleaning area
    min_x = min(point['x'] for point in boundary_points)
    max_x = max(point['x'] for point in boundary_points)
    min_y = min(point['y'] for point in boundary_points)
    max_y = max(point['y'] for point in boundary_points)
    
    # Calculate step size based on cleaning unit diameter and overlap
    cleaning_width = ROBOT_PARAMS['cleaning_unit_diameter'] - path_overlap
    step_size = max(0.1, cleaning_width)  # Ensure step is at least 10cm
    
    # Generate a simple zig-zag path
    path = []
    y = min_y
    going_right = True
    
    # Add starting point
    path.append({'x': min_x, 'y': min_y})
    
    # Generate zig-zag pattern
    while y <= max_y:
        if going_right:
            path.append({'x': max_x, 'y': y})
        else:
            path.append({'x': min_x, 'y': y})
        
        # Move to next row
        y += step_size
        if y <= max_y:
            if going_right:
                path.append({'x': max_x, 'y': y})
            else:
                path.append({'x': min_x, 'y': y})
        
        going_right = not going_right
    
    return path

def calculate_path_length(path):
    """
    Calculate the total length of a path in meters.
    
    Args:
        path: List of waypoints as dictionaries with 'x' and 'y' coordinates
        
    Returns:
        Total path length in meters
    """
    if not path or len(path) < 2:
        return 0
    
    total_length = 0
    for i in range(1, len(path)):
        dx = path[i]['x'] - path[i-1]['x']
        dy = path[i]['y'] - path[i-1]['y']
        segment_length = math.sqrt(dx*dx + dy*dy)
        total_length += segment_length
    
    return total_length

def estimate_cleaning_time(path, speed=0.5):
    """
    Estimate the time to complete a cleaning path.
    
    Args:
        path: List of waypoints as dictionaries with 'x' and 'y' coordinates
        speed: Average robot speed in meters per second
        
    Returns:
        Estimated time in seconds
    """
    path_length = calculate_path_length(path)
    
    # Add turning time - estimate 2 seconds per waypoint for turns
    turn_time = (len(path) - 1) * 2
    
    # Calculate travel time
    travel_time = path_length / speed
    
    return travel_time + turn_time 