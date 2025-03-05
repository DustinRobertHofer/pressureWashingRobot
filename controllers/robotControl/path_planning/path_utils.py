"""
Path Planning Utility Functions

This module contains utility functions for path planning,
coordinate conversions, and path manipulation.
"""

import numpy as np
import cv2


# Unit conversion constants
METERS_TO_FEET = 3.28084
FEET_TO_INCHES = 12
INCHES_TO_METERS = 0.0254


def convert_boundary_to_np_array(boundary_points, to_feet=True):
    """
    Convert boundary points from dictionary format to numpy arrays.
    
    Args:
        boundary_points: List of dictionaries with 'x' and 'y' coordinates
        to_feet: If True, convert from meters to feet
        
    Returns:
        tuple: (np_points, origin_x, origin_y)
    """
    np_points = []
    origin_x = boundary_points[0]['x']
    origin_y = boundary_points[0]['y']
    
    for point in boundary_points:
        if to_feet:
            # Convert to feet (1 meter = ~3.28084 feet)
            x = (point['x'] - origin_x) * METERS_TO_FEET
            y = (point['y'] - origin_y) * METERS_TO_FEET
        else:
            x = point['x'] - origin_x
            y = point['y'] - origin_y
        np_points.append(np.array([x, y]))
        
    return np_points, origin_x, origin_y


def generate_mask_from_boundary(boundary_points, edge_buffer, scale=10, padding=100):
    """
    Generate a mask image from boundary points.
    
    Args:
        boundary_points: Array of points representing the boundary
        edge_buffer: Buffer from the edge in inches
        scale: Scale factor to convert inches to pixels
        padding: Padding around the boundary in pixels
        
    Returns:
        tuple: (eroded_mask, min_x, min_y, scale, padding)
    """
    # Get the boundary extents
    min_x = np.min(boundary_points[:,0])
    max_x = np.max(boundary_points[:,0])
    min_y = np.min(boundary_points[:,1])
    max_y = np.max(boundary_points[:,1])
    
    width = int((max_x - min_x) * scale) + 2 * padding
    height = int((max_y - min_y) * scale) + 2 * padding
    
    # Create mask image
    mask = np.zeros((height, width), dtype=np.uint8)
    
    # Convert boundary points to image coordinates
    image_points = []
    for point in boundary_points:
        x = int((point[0] - min_x) * scale) + padding
        y = int((point[1] - min_y) * scale) + padding
        image_points.append([x, y])
    
    # Draw filled polygon
    cv2.fillPoly(mask, [np.array(image_points)], 255)
    
    # Erode the mask by edge_buffer
    kernel_size = int(edge_buffer * scale / 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded_mask = cv2.erode(mask, kernel)
    
    return eroded_mask, min_x, min_y, scale, padding


def convert_path_to_meters(path_coordinates, origin_x, origin_y):
    """
    Convert path coordinates from inches back to meters.
    
    Args:
        path_coordinates: List of path coordinates in inches
        origin_x: X origin in meters
        origin_y: Y origin in meters
        
    Returns:
        list: List of dictionaries with 'x' and 'y' coordinates in meters
    """
    waypoints = []
    
    for coord in path_coordinates:
        # Convert from inches to meters
        x_meters = coord['x'] * INCHES_TO_METERS + origin_x
        y_meters = coord['y'] * INCHES_TO_METERS + origin_y
        waypoints.append({'x': x_meters, 'y': y_meters})
    
    return waypoints 