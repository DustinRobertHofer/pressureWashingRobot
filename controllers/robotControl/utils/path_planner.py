import numpy as np
import cv2
from config.robot_config import ROBOT_PARAMS

def generate_cleaning_path(boundary_points):
    """Generate a cleaning path from boundary points
    Args:
        boundary_points: List of dictionaries with 'x' and 'y' coordinates marking the boundary (in meters)
    Returns:
        List of waypoints for the robot to follow
    """
    surface_cleaner_diameter = ROBOT_PARAMS['surface_cleaner_diameter']
    path_overlap = ROBOT_PARAMS['path_overlap']
    edge_buffer = ROBOT_PARAMS['edge_buffer']

    # Convert points to numpy arrays and from meters to feet
    np_points = []
    origin_x = boundary_points[0]['x']
    origin_y = boundary_points[0]['y']
    
    for point in boundary_points:
        # Convert to feet (1 meter = ~3.28084 feet)
        x_ft = (point['x'] - origin_x) * 3.28084
        y_ft = (point['y'] - origin_y) * 3.28084
        np_points.append(np.array([x_ft, y_ft]))

    # Add a virtual water supply point (slightly behind start point)
    water_supply = np.array([np_points[0][0], np_points[0][1] - 2])
    points = np_points + [water_supply]
    
    # Convert points to inches
    points_inches = np.array(points) * 12

    # Calculate path using rectilinear path generator
    path_coordinates = _calculate_rectilinear_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer)
    
    # Convert path coordinates back to meters and to waypoint format
    waypoints = []
    for point in path_coordinates:
        # Convert from inches to meters (1 inch = 0.0254 meters)
        x_meters = point['x'] * 0.0254 + origin_x
        y_meters = point['y'] * 0.0254 + origin_y
        waypoints.append({'x': x_meters, 'y': y_meters})
    
    return waypoints

def _calculate_rectilinear_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer):
    """Calculate a rectilinear path that ensures complete coverage
    Returns coordinates in inches relative to the origin point
    """
    # Extract points
    water_supply = points_inches[-1]
    boundary = points_inches[:-1]
    
    # Create a mask image for the boundary
    scale = 10  # Scale factor to convert inches to pixels
    padding = 100  # Padding around the boundary in pixels
    
    # Scale the boundary points to image coordinates
    min_x = np.min(boundary[:,0])
    max_x = np.max(boundary[:,0])
    min_y = np.min(boundary[:,1])
    max_y = np.max(boundary[:,1])
    
    width = int((max_x - min_x) * scale) + 2 * padding
    height = int((max_y - min_y) * scale) + 2 * padding
    
    # Create mask image
    mask = np.zeros((height, width), dtype=np.uint8)
    
    # Convert boundary points to image coordinates
    image_points = []
    for point in boundary:
        x = int((point[0] - min_x) * scale) + padding
        y = int((point[1] - min_y) * scale) + padding
        image_points.append([x, y])
    
    # Draw filled polygon
    cv2.fillPoly(mask, [np.array(image_points)], 255)
    
    # Erode the mask by edge_buffer to ensure the robot stays away from edges
    kernel_size = int(edge_buffer * scale / 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded_mask = cv2.erode(mask, kernel)
    
    # Calculate spacing between passes in pixels
    effective_width = surface_cleaner_diameter - path_overlap
    spacing_pixels = int(effective_width * scale)
    
    # Generate rectilinear path coordinates
    path_coords = []
    
    # Find the leftmost, rightmost, top and bottom points within the eroded mask
    y_indices, x_indices = np.where(eroded_mask > 0)
    if len(y_indices) == 0 or len(x_indices) == 0:
        # No valid area to clean
        return []
        
    min_pixel_x = np.min(x_indices)
    max_pixel_x = np.max(x_indices)
    min_pixel_y = np.min(y_indices)
    max_pixel_y = np.max(y_indices)
    
    # Start from the bottom-left corner and move in a rectilinear pattern
    current_y = min_pixel_y
    moving_right = True
    
    while current_y <= max_pixel_y:
        # Find the valid x-coordinates for this row
        row = eroded_mask[current_y, :]
        valid_x = np.where(row > 0)[0]
        
        if len(valid_x) > 0:
            row_min_x = valid_x[0]
            row_max_x = valid_x[-1]
            
            # Convert to inches for the path
            min_inch_x = (row_min_x - padding) / scale + min_x
            min_inch_y = (current_y - padding) / scale + min_y
            max_inch_x = (row_max_x - padding) / scale + min_x
            max_inch_y = (current_y - padding) / scale + min_y
            
            # Add path points for this row
            if moving_right:
                # Move to the leftmost point if this is the first row or we need to move up
                if len(path_coords) == 0 or path_coords[-1]['y'] != min_inch_y:
                    path_coords.append({'x': min_inch_x, 'y': min_inch_y})
                # Move right
                path_coords.append({'x': max_inch_x, 'y': max_inch_y})
            else:
                # Move to the rightmost point if we need to move up
                if path_coords[-1]['y'] != min_inch_y:
                    path_coords.append({'x': max_inch_x, 'y': max_inch_y})
                # Move left
                path_coords.append({'x': min_inch_x, 'y': min_inch_y})
        
        # Move to the next row
        current_y += spacing_pixels
        
        if current_y <= max_pixel_y:
            # If there's another row, prepare to move to it
            new_row = eroded_mask[current_y, :]
            new_valid_x = np.where(new_row > 0)[0]
            
            if len(new_valid_x) > 0:
                new_row_min_x = new_valid_x[0]
                new_row_max_x = new_valid_x[-1]
                
                # Convert to inches
                new_min_inch_x = (new_row_min_x - padding) / scale + min_x
                new_max_inch_x = (new_row_max_x - padding) / scale + min_x
                new_inch_y = (current_y - padding) / scale + min_y
                
                # Create vertical segment to connect to the next row
                if moving_right:
                    # We're at the rightmost point, so add a point at the same x but next row y
                    path_coords.append({'x': max_inch_x, 'y': new_inch_y})
                else:
                    # We're at the leftmost point, so add a point at the same x but next row y
                    path_coords.append({'x': min_inch_x, 'y': new_inch_y})
                
                # Switch direction
                moving_right = not moving_right
    
    return path_coords 