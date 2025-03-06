import numpy as np
import cv2
from typing import List, Dict, Any, Tuple, Optional, Union
from config.robot_config import ROBOT_PARAMS

def generate_cleaning_path(boundary_points: List[Dict[str, float]]) -> List[Dict[str, float]]:
    """Generate an optimal cleaning path from boundary points.
    
    Args:
        boundary_points: List of dictionaries with 'x' and 'y' coordinates marking the boundary perimeter (in meters)
        
    Returns:
        List of waypoints (as dicts with 'x' and 'y' keys) for the robot to follow (in meters)
    """
    # Get robot configuration parameters
    surface_cleaner_diameter = ROBOT_PARAMS['surface_cleaner_diameter']  # inches
    path_overlap = ROBOT_PARAMS['path_overlap']  # inches
    edge_buffer = ROBOT_PARAMS['edge_buffer']  # inches

    # Set the origin to the first boundary point for relative coordinates
    origin_x = boundary_points[0]['x']
    origin_y = boundary_points[0]['y']
    
    # Convert boundary points to numpy arrays and from meters to feet
    boundary_points_ft = convert_boundary_to_feet(boundary_points, origin_x, origin_y)
    
    # Add a virtual water supply point (slightly behind start point)
    water_supply = np.array([boundary_points_ft[0][0], boundary_points_ft[0][1] - 2])
    points_ft = boundary_points_ft + [water_supply]
    
    # Convert points to inches for internal calculations
    points_inches = np.array(points_ft) * 12

    # Calculate path using rectilinear path generator
    path_coordinates = calculate_rectilinear_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer)
    
    # Convert path coordinates back to meters and to waypoint format
    waypoints = convert_path_to_meters(path_coordinates, origin_x, origin_y)
    
    return waypoints

def convert_boundary_to_feet(boundary_points: List[Dict[str, float]], 
                             origin_x: float, 
                             origin_y: float) -> List[np.ndarray]:
    """Convert boundary points from meters to feet relative to origin point.
    
    Args:
        boundary_points: List of point dictionaries in meters
        origin_x: Origin x-coordinate in meters
        origin_y: Origin y-coordinate in meters
        
    Returns:
        List of numpy arrays with coordinates in feet
    """
    feet_per_meter = 3.28084
    points_ft = []
    
    for point in boundary_points:
        # Convert to feet relative to the origin
        x_ft = (point['x'] - origin_x) * feet_per_meter
        y_ft = (point['y'] - origin_y) * feet_per_meter
        points_ft.append(np.array([x_ft, y_ft]))
        
    return points_ft

def convert_path_to_meters(path_inches: List[Dict[str, float]],
                           origin_x: float,
                           origin_y: float) -> List[Dict[str, float]]:
    """Convert path coordinates from inches back to meters.
    
    Args:
        path_inches: List of point dictionaries in inches
        origin_x: Origin x-coordinate in meters
        origin_y: Origin y-coordinate in meters
        
    Returns:
        List of waypoints in meters
    """
    meters_per_inch = 0.0254
    waypoints = []
    
    for point in path_inches:
        # Convert from inches to meters and add origin offset
        x_meters = point['x'] * meters_per_inch + origin_x
        y_meters = point['y'] * meters_per_inch + origin_y
        waypoints.append({'x': x_meters, 'y': y_meters})
    
    return waypoints

def calculate_rectilinear_path(points_inches: np.ndarray, 
                              surface_cleaner_diameter: float, 
                              path_overlap: float, 
                              edge_buffer: float) -> List[Dict[str, float]]:
    """Calculate a rectilinear (back-and-forth) path that ensures complete coverage.
    
    This function creates a path with parallel passes that ensures the entire area is cleaned.
    
    Args:
        points_inches: Array of points in inches, including boundary and water supply
        surface_cleaner_diameter: Diameter of the surface cleaner in inches
        path_overlap: Overlap between adjacent cleaning passes in inches
        edge_buffer: Safety margin from the boundary edges in inches
        
    Returns:
        List of coordinates (as dicts with 'x' and 'y' keys) in inches relative to the origin
    """
    # Extract water supply point and boundary points
    water_supply = points_inches[-1]
    boundary = points_inches[:-1]
    
    # Create a binary mask image of the cleaning area
    mask_image, scale, min_x, min_y, padding = create_boundary_mask(boundary)
    
    # Apply safety margin by eroding the mask
    eroded_mask = apply_safety_margin(mask_image, edge_buffer, scale)
    
    # Find valid cleaning area boundaries after erosion
    area_bounds = find_valid_area_bounds(eroded_mask)
    
    if area_bounds is None:
        # No valid area to clean after applying safety margins
        return []
    
    # Generate rectilinear path coordinates
    path_coords = generate_rectilinear_passes(
        eroded_mask, 
        area_bounds,
        surface_cleaner_diameter, 
        path_overlap,
        scale, 
        min_x, 
        min_y, 
        padding
    )
    
    return path_coords

def create_boundary_mask(boundary: np.ndarray) -> Tuple[np.ndarray, int, float, float, int]:
    """Create a binary mask image representing the boundary area.
    
    Args:
        boundary: Array of boundary points in inches
        
    Returns:
        Tuple containing:
        - mask: Binary image where the cleaning area is white (255)
        - scale: Factor used to convert inches to pixels
        - min_x: Minimum x-coordinate in inches
        - min_y: Minimum y-coordinate in inches
        - padding: Padding around the boundary in pixels
    """
    scale = 10  # Scale factor to convert inches to pixels
    padding = 100  # Padding around the boundary in pixels
    
    # Find boundary extents
    min_x = np.min(boundary[:,0])
    max_x = np.max(boundary[:,0])
    min_y = np.min(boundary[:,1])
    max_y = np.max(boundary[:,1])
    
    # Create mask image
    width = int((max_x - min_x) * scale) + 2 * padding
    height = int((max_y - min_y) * scale) + 2 * padding
    mask = np.zeros((height, width), dtype=np.uint8)
    
    # Convert boundary points to image coordinates
    image_points = []
    for point in boundary:
        x = int((point[0] - min_x) * scale) + padding
        y = int((point[1] - min_y) * scale) + padding
        image_points.append([x, y])
    
    # Draw filled polygon representing the cleaning area
    cv2.fillPoly(mask, [np.array(image_points)], 255)
    
    return mask, scale, min_x, min_y, padding

def apply_safety_margin(mask: np.ndarray, edge_buffer: float, scale: int) -> np.ndarray:
    """Apply a safety margin by eroding the mask.
    
    Args:
        mask: Binary mask image
        edge_buffer: Safety margin from edges in inches
        scale: Factor used to convert inches to pixels
        
    Returns:
        Eroded mask with safety margins applied
    """
    # Calculate erosion kernel size based on edge_buffer
    kernel_size = int(edge_buffer * scale / 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    
    # Erode the mask to create safety margin from edges
    eroded_mask = cv2.erode(mask, kernel)
    
    return eroded_mask

def find_valid_area_bounds(eroded_mask: np.ndarray) -> Optional[Dict[str, int]]:
    """Find the bounding box of the valid cleaning area after applying safety margins.
    
    Args:
        eroded_mask: Binary mask with safety margins applied
        
    Returns:
        Dictionary with min_x, max_x, min_y, max_y pixel coordinates, or None if no valid area
    """
    # Find non-zero pixels (valid cleaning area)
    y_indices, x_indices = np.where(eroded_mask > 0)
    
    if len(y_indices) == 0 or len(x_indices) == 0:
        # No valid area to clean
        return None
        
    return {
        'min_x': np.min(x_indices),
        'max_x': np.max(x_indices),
        'min_y': np.min(y_indices),
        'max_y': np.max(y_indices)
    }

def generate_rectilinear_passes(
    eroded_mask: np.ndarray,
    area_bounds: Dict[str, int],
    surface_cleaner_diameter: float,
    path_overlap: float,
    scale: int,
    min_x: float,
    min_y: float,
    padding: int
) -> List[Dict[str, float]]:
    """Generate a rectilinear (back-and-forth) path through the valid cleaning area.
    
    Args:
        eroded_mask: Binary mask with safety margins applied
        area_bounds: Dictionary with area boundaries in pixels
        surface_cleaner_diameter: Diameter of surface cleaner in inches
        path_overlap: Overlap between passes in inches
        scale: Factor used to convert inches to pixels
        min_x: Minimum x-coordinate in inches
        min_y: Minimum y-coordinate in inches
        padding: Padding around the boundary in pixels
        
    Returns:
        List of coordinates (as dicts with 'x' and 'y' keys) in inches
    """
    # Calculate effective cleaning width and spacing between passes
    effective_width = surface_cleaner_diameter - path_overlap
    spacing_pixels = int(effective_width * scale)
    
    # Extract area bounds
    min_pixel_x = area_bounds['min_x']
    max_pixel_x = area_bounds['max_x']
    min_pixel_y = area_bounds['min_y']
    max_pixel_y = area_bounds['max_y']
    
    # Start from the bottom-left corner and move in a rectilinear pattern
    current_y = min_pixel_y
    moving_right = True
    path_coords = []
    
    while current_y <= max_pixel_y:
        # Process current row
        row_coords = process_row(
            eroded_mask, 
            current_y, 
            moving_right, 
            path_coords,
            scale, 
            min_x, 
            min_y, 
            padding
        )
        
        if row_coords:
            path_coords.extend(row_coords)
        
        # Move to the next row
        current_y += spacing_pixels
        
        # Connect to next row if needed
        if current_y <= max_pixel_y:
            vertical_segment = connect_to_next_row(
                eroded_mask, 
                current_y, 
                moving_right, 
                path_coords,
                scale, 
                min_x, 
                min_y, 
                padding
            )
            
            if vertical_segment:
                path_coords.append(vertical_segment)
                # Switch direction for the next row
                moving_right = not moving_right
    
    return path_coords

def process_row(
    eroded_mask: np.ndarray,
    row_y: int,
    moving_right: bool,
    current_path: List[Dict[str, float]],
    scale: int,
    min_x: float,
    min_y: float,
    padding: int
) -> List[Dict[str, float]]:
    """Process a single horizontal row in the cleaning path.
    
    Args:
        eroded_mask: Binary mask with safety margins applied
        row_y: Y-coordinate of the current row in pixels
        moving_right: Whether we're currently moving right (vs left)
        current_path: Current path coordinates
        scale: Factor used to convert inches to pixels
        min_x: Minimum x-coordinate in inches
        min_y: Minimum y-coordinate in inches
        padding: Padding around the boundary in pixels
        
    Returns:
        List of new coordinates to add to the path
    """
    # Find the valid x-coordinates for this row
    row = eroded_mask[row_y, :]
    valid_x = np.where(row > 0)[0]
    
    if len(valid_x) == 0:
        return []
        
    row_min_x = valid_x[0]
    row_max_x = valid_x[-1]
    
    # Convert to inches for the path
    min_inch_x = (row_min_x - padding) / scale + min_x
    min_inch_y = (row_y - padding) / scale + min_y
    max_inch_x = (row_max_x - padding) / scale + min_x
    max_inch_y = (row_y - padding) / scale + min_y
    
    new_coords = []
    
    # Add path points for this row
    if moving_right:
        # Move to the leftmost point if this is the first row or we need to move up
        if len(current_path) == 0 or current_path[-1]['y'] != min_inch_y:
            new_coords.append({'x': min_inch_x, 'y': min_inch_y})
        # Move right
        new_coords.append({'x': max_inch_x, 'y': max_inch_y})
    else:
        # Move to the rightmost point if we need to move up
        if len(current_path) == 0 or current_path[-1]['y'] != min_inch_y:
            new_coords.append({'x': max_inch_x, 'y': max_inch_y})
        # Move left
        new_coords.append({'x': min_inch_x, 'y': min_inch_y})
    
    return new_coords

def connect_to_next_row(
    eroded_mask: np.ndarray,
    next_row_y: int,
    moving_right: bool,
    current_path: List[Dict[str, float]],
    scale: int,
    min_x: float,
    min_y: float,
    padding: int
) -> Optional[Dict[str, float]]:
    """Create a vertical segment to connect to the next row.
    
    Args:
        eroded_mask: Binary mask with safety margins applied
        next_row_y: Y-coordinate of the next row in pixels
        moving_right: Whether we're currently moving right (vs left)
        current_path: Current path coordinates
        scale: Factor used to convert inches to pixels
        min_x: Minimum x-coordinate in inches
        min_y: Minimum y-coordinate in inches
        padding: Padding around the boundary in pixels
        
    Returns:
        Dictionary with coordinates for the vertical segment, or None if not needed
    """
    if not current_path:
        return None
        
    # Find the valid x-coordinates for the next row
    new_row = eroded_mask[next_row_y, :]
    new_valid_x = np.where(new_row > 0)[0]
    
    if len(new_valid_x) == 0:
        return None
        
    new_row_min_x = new_valid_x[0]
    new_row_max_x = new_valid_x[-1]
    
    # Convert to inches
    new_min_inch_x = (new_row_min_x - padding) / scale + min_x
    new_max_inch_x = (new_row_max_x - padding) / scale + min_x
    new_inch_y = (next_row_y - padding) / scale + min_y
    
    # Create vertical segment to connect to the next row
    if moving_right:
        # We're at the rightmost point, connect vertically to next row
        return {'x': current_path[-1]['x'], 'y': new_inch_y}
    else:
        # We're at the leftmost point, connect vertically to next row
        return {'x': current_path[-1]['x'], 'y': new_inch_y} 