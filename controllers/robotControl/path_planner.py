import numpy as np
import cv2

def generate_cleaning_path(boundary_points, surface_cleaner_diameter=12, path_overlap=3, edge_buffer=8):
    """Generate a cleaning path from boundary points
    Args:
        boundary_points: List of dictionaries with 'x' and 'y' coordinates marking the boundary (in meters)
        surface_cleaner_diameter: Diameter of cleaning head in inches
        path_overlap: Overlap between passes in inches
        edge_buffer: Buffer from edges in inches
    Returns:
        List of waypoints for the robot to follow
    """
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

    # Calculate path using similar logic to the original PathGenerator
    path_coordinates = _calculate_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer)
    
    # Convert path coordinates back to meters and to waypoint format
    waypoints = []
    for i in range(0, len(path_coordinates), 2):
        # Convert from inches to meters (1 inch = 0.0254 meters)
        x_meters = path_coordinates[i]['x'] * 0.0254 + origin_x
        y_meters = path_coordinates[i]['y'] * 0.0254 + origin_y
        waypoints.append({'x': x_meters, 'y': y_meters})
    
    return waypoints

def _calculate_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer):
    """Internal function to calculate the cleaning path
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
    
    # Erode the mask by edge_buffer
    kernel_size = int(edge_buffer * scale / 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded_mask = cv2.erode(mask, kernel)
    
    # Calculate spacing between passes in pixels
    spacing_pixels = int((surface_cleaner_diameter - path_overlap) * scale)
    
    # Generate path coordinates
    path_coords = []
    y = padding
    going_right = True
    
    while y < height - padding:
        # Find the start and end points of this row
        row = eroded_mask[y,:]
        if np.any(row):  # If there are any white pixels in this row
            x_coords = np.where(row > 0)[0]
            start_x = x_coords[0]
            end_x = x_coords[-1]
            
            # Convert back to inches and add to path
            if going_right:
                path_coords.extend([
                    {'x': (start_x - padding) / scale + min_x, 'y': (y - padding) / scale + min_y},
                    {'x': (end_x - padding) / scale + min_x, 'y': (y - padding) / scale + min_y}
                ])
            else:
                path_coords.extend([
                    {'x': (end_x - padding) / scale + min_x, 'y': (y - padding) / scale + min_y},
                    {'x': (start_x - padding) / scale + min_x, 'y': (y - padding) / scale + min_y}
                ])
        
        y += spacing_pixels
        going_right = not going_right
    
    return path_coords 