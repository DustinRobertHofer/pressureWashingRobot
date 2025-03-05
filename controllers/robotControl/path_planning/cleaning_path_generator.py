"""
Cleaning Path Generator

This module provides functionality to generate efficient cleaning paths
for the pressure washing robot based on defined area boundaries.
"""

import numpy as np
import cv2
from .path_utils import (
    convert_boundary_to_np_array,
    generate_mask_from_boundary,
    convert_path_to_meters,
    FEET_TO_INCHES
)


def generate_cleaning_path(boundary_points, surface_cleaner_diameter=12, path_overlap=3, edge_buffer=8):
    """
    Generate a cleaning path from boundary points.
    
    Args:
        boundary_points: List of dictionaries with 'x' and 'y' coordinates marking the boundary (in meters)
        surface_cleaner_diameter: Diameter of cleaning head in inches
        path_overlap: Overlap between passes in inches
        edge_buffer: Buffer from edges in inches
        
    Returns:
        List of waypoints for the robot to follow
    """
    # Convert points to numpy arrays and from meters to feet
    np_points, origin_x, origin_y = convert_boundary_to_np_array(boundary_points, to_feet=True)

    # Add a virtual water supply point (slightly behind start point)
    water_supply = np.array([np_points[0][0], np_points[0][1] - 2])
    points = np_points + [water_supply]
    
    # Convert points to inches
    points_inches = np.array(points) * FEET_TO_INCHES

    # Calculate path using the path generation algorithm
    path_coordinates = _calculate_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer)
    
    # Convert path coordinates back to meters and to waypoint format
    waypoints = []
    for i in range(0, len(path_coordinates), 2):
        waypoint_pair = path_coordinates[i:i+2]
        waypoints.extend(convert_path_to_meters(waypoint_pair, origin_x, origin_y))
    
    return waypoints


def _calculate_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer):
    """
    Internal function to calculate the cleaning path.
    
    Args:
        points_inches: List of points in inches
        surface_cleaner_diameter: Diameter of cleaning head in inches
        path_overlap: Overlap between passes in inches
        edge_buffer: Buffer from edges in inches
        
    Returns:
        List of coordinates in inches relative to the origin point
    """
    # Extract points
    water_supply = points_inches[-1]
    boundary = points_inches[:-1]
    
    # Create a mask image for the boundary
    scale = 10  # Scale factor to convert inches to pixels
    padding = 100  # Padding around the boundary in pixels
    
    # Generate the mask from the boundary
    eroded_mask, min_x, min_y, scale, padding = generate_mask_from_boundary(boundary, edge_buffer, scale, padding)
    
    # Calculate spacing between passes in pixels
    spacing_pixels = int((surface_cleaner_diameter - path_overlap) * scale)
    
    # Generate path coordinates using the raster scan approach
    return _generate_raster_scan_path(eroded_mask, min_x, min_y, scale, padding, spacing_pixels)


def _generate_raster_scan_path(eroded_mask, min_x, min_y, scale, padding, spacing_pixels):
    """
    Generate a path using raster scan approach.
    
    Args:
        eroded_mask: Eroded mask image
        min_x: Minimum x value of the boundary
        min_y: Minimum y value of the boundary
        scale: Scale factor
        padding: Padding around the boundary
        spacing_pixels: Spacing between passes in pixels
        
    Returns:
        List of path coordinates
    """
    height, width = eroded_mask.shape
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


class CleaningPathGenerator:
    """
    Class for generating cleaning paths for the robot.
    
    This class provides functionality to generate and manipulate 
    cleaning paths based on different algorithms and requirements.
    """
    
    # Default parameters
    DEFAULT_SURFACE_CLEANER_DIAMETER = 12  # inches
    DEFAULT_PATH_OVERLAP = 3               # inches
    DEFAULT_EDGE_BUFFER = 8                # inches
    
    def __init__(self, surface_cleaner_diameter=None, path_overlap=None, edge_buffer=None):
        """
        Initialize the cleaning path generator.
        
        Args:
            surface_cleaner_diameter: Diameter of the cleaning head in inches
            path_overlap: Overlap between passes in inches
            edge_buffer: Buffer from edges in inches
        """
        self.surface_cleaner_diameter = surface_cleaner_diameter or self.DEFAULT_SURFACE_CLEANER_DIAMETER
        self.path_overlap = path_overlap or self.DEFAULT_PATH_OVERLAP
        self.edge_buffer = edge_buffer or self.DEFAULT_EDGE_BUFFER
        
    def generate_path(self, boundary_points):
        """
        Generate a cleaning path given boundary points.
        
        Args:
            boundary_points: List of dictionaries with 'x' and 'y' coordinates
            
        Returns:
            List of waypoints for the robot to follow
        """
        return generate_cleaning_path(
            boundary_points, 
            self.surface_cleaner_diameter,
            self.path_overlap,
            self.edge_buffer
        )
        
    def set_cleaner_diameter(self, diameter_inches):
        """Set the surface cleaner diameter in inches."""
        self.surface_cleaner_diameter = diameter_inches
        
    def set_path_overlap(self, overlap_inches):
        """Set the path overlap in inches."""
        self.path_overlap = overlap_inches
        
    def set_edge_buffer(self, buffer_inches):
        """Set the edge buffer in inches."""
        self.edge_buffer = buffer_inches 