import numpy as np
import cv2
from typing import List, Dict, Tuple, Optional, NamedTuple
from config.robot_config import ROBOT_PARAMS

# Constants for unit conversions
FEET_PER_METER = 3.28084
INCHES_PER_FOOT = 12
METERS_PER_INCH = 0.0254

class Point:
    """Represents a 2D point with x and y coordinates."""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        
    @classmethod
    def from_dict(cls, point_dict: Dict[str, float]) -> 'Point':
        """Create a Point from a dictionary with 'x' and 'y' keys."""
        return cls(point_dict['x'], point_dict['y'])
    
    def to_dict(self) -> Dict[str, float]:
        """Convert Point to a dictionary with 'x' and 'y' keys."""
        return {'x': self.x, 'y': self.y}
    
    def to_array(self) -> np.ndarray:
        """Convert Point to a numpy array."""
        return np.array([self.x, self.y])

class BoundingBox:
    """Represents a rectangular bounding box."""
    def __init__(self, min_x: float, min_y: float, max_x: float, max_y: float):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
    
    @classmethod
    def from_dict(cls, bounds_dict: Dict[str, float]) -> 'BoundingBox':
        """Create a BoundingBox from a dictionary."""
        return cls(
            bounds_dict['min_x'],
            bounds_dict['min_y'],
            bounds_dict['max_x'],
            bounds_dict['max_y']
        )
    
    @classmethod
    def from_points(cls, points: np.ndarray) -> 'BoundingBox':
        """Create a BoundingBox from an array of points."""
        min_x = np.min(points[:, 0])
        min_y = np.min(points[:, 1])
        max_x = np.max(points[:, 0])
        max_y = np.max(points[:, 1])
        return cls(min_x, min_y, max_x, max_y)

class CoordinateTransformer:
    """Handles coordinate transformations between different units and coordinate systems."""
    def __init__(self, origin_x: float, origin_y: float, scale: int = 10, padding: int = 100):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.scale = scale  # Scale factor to convert inches to pixels
        self.padding = padding  # Padding around the boundary in pixels
    
    def meters_to_feet(self, point: Point) -> np.ndarray:
        """Convert point from meters to feet relative to origin."""
        x_ft = (point.x - self.origin_x) * FEET_PER_METER
        y_ft = (point.y - self.origin_y) * FEET_PER_METER
        return np.array([x_ft, y_ft])
    
    def inches_to_meters(self, point: Dict[str, float]) -> Dict[str, float]:
        """Convert point from inches to meters and add origin offset."""
        x_meters = point['x'] * METERS_PER_INCH + self.origin_x
        y_meters = point['y'] * METERS_PER_INCH + self.origin_y
        return {'x': x_meters, 'y': y_meters}
    
    def inches_to_pixels(self, x_inch: float, y_inch: float, min_x: float, min_y: float) -> Tuple[int, int]:
        """Convert inches to pixel coordinates."""
        x_pixel = int((x_inch - min_x) * self.scale) + self.padding
        y_pixel = int((y_inch - min_y) * self.scale) + self.padding
        return x_pixel, y_pixel
    
    def pixels_to_inches(self, x_pixel: int, y_pixel: int, min_x: float, min_y: float) -> Tuple[float, float]:
        """Convert pixel coordinates to inches."""
        x_inch = (x_pixel - self.padding) / self.scale + min_x
        y_inch = (y_pixel - self.padding) / self.scale + min_y
        return x_inch, y_inch

class CleaningAreaMask:
    """Handles creation and manipulation of the binary mask representing the cleaning area."""
    def __init__(self, boundary_points: np.ndarray, transformer: CoordinateTransformer):
        self.boundary = boundary_points
        self.transformer = transformer
        
        # Calculate boundary extents
        self.bounds = BoundingBox.from_points(boundary_points)
        
        # Create the mask image
        self.mask, self.width, self.height = self._create_mask()
    
    def _create_mask(self) -> Tuple[np.ndarray, int, int]:
        """Create a binary mask image from boundary points."""
        width = int((self.bounds.max_x - self.bounds.min_x) * self.transformer.scale) + 2 * self.transformer.padding
        height = int((self.bounds.max_y - self.bounds.min_y) * self.transformer.scale) + 2 * self.transformer.padding
        mask = np.zeros((height, width), dtype=np.uint8)
        
        # Convert boundary points to image coordinates
        image_points = []
        for point in self.boundary:
            x, y = self.transformer.inches_to_pixels(
                point[0], point[1], self.bounds.min_x, self.bounds.min_y
            )
            image_points.append([x, y])
        
        # Draw filled polygon representing the cleaning area
        cv2.fillPoly(mask, [np.array(image_points)], 255)
        
        return mask, width, height
    
    def apply_safety_margin(self, edge_buffer: float) -> np.ndarray:
        """Apply a safety margin by eroding the mask."""
        # Calculate erosion kernel size based on edge_buffer
        kernel_size = int(edge_buffer * self.transformer.scale / 2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # Erode the mask to create safety margin from edges
        return cv2.erode(self.mask, kernel)
    
    def find_valid_area_bounds(self, eroded_mask: np.ndarray) -> Optional[Dict[str, int]]:
        """Find the bounding box of the valid cleaning area after applying safety margins."""
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

class PathPlanner:
    """Plans and generates cleaning paths for the robot."""
    def __init__(self, transformer: CoordinateTransformer):
        self.transformer = transformer
    
    def generate_cleaning_path(self, boundary_points: List[Dict[str, float]]) -> List[Dict[str, float]]:
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
        
        # Convert boundary points to numpy arrays and from meters to feet
        boundary_points_ft = [self.transformer.meters_to_feet(Point.from_dict(point)) for point in boundary_points]
        
        # Add a virtual water supply point (slightly behind start point)
        water_supply = np.array([boundary_points_ft[0][0], boundary_points_ft[0][1] - 2])
        points_ft = boundary_points_ft + [water_supply]
        
        # Convert points to inches for internal calculations
        points_inches = np.array(points_ft) * INCHES_PER_FOOT
        
        # Calculate path using rectilinear path generator
        path_coordinates = self.calculate_rectilinear_path(points_inches, surface_cleaner_diameter, path_overlap, edge_buffer)
        
        # Convert path coordinates back to meters and to waypoint format
        waypoints = [self.transformer.inches_to_meters(point) for point in path_coordinates]
        
        return waypoints
    
    def calculate_rectilinear_path(self, points_inches: np.ndarray, 
                                  surface_cleaner_diameter: float, 
                                  path_overlap: float, 
                                  edge_buffer: float) -> List[Dict[str, float]]:
        """Calculate a rectilinear (back-and-forth) path that ensures complete coverage."""
        # Extract water supply point and boundary points
        water_supply = points_inches[-1]
        boundary = points_inches[:-1]
        
        # Create a binary mask image of the cleaning area
        area_mask = CleaningAreaMask(boundary, self.transformer)
        
        # Apply safety margin by eroding the mask
        eroded_mask = area_mask.apply_safety_margin(edge_buffer)
        
        # Find valid cleaning area boundaries after erosion
        area_bounds = area_mask.find_valid_area_bounds(eroded_mask)
        
        if area_bounds is None:
            # No valid area to clean after applying safety margins
            return []
        
        # Generate rectilinear path coordinates
        return self.generate_rectilinear_passes(
            eroded_mask, 
            area_bounds,
            surface_cleaner_diameter, 
            path_overlap,
            area_mask.bounds.min_x, 
            area_mask.bounds.min_y
        )
    
    def generate_rectilinear_passes(
        self,
        eroded_mask: np.ndarray,
        area_bounds: Dict[str, int],
        surface_cleaner_diameter: float,
        path_overlap: float,
        min_x: float,
        min_y: float
    ) -> List[Dict[str, float]]:
        """Generate a rectilinear (back-and-forth) path through the valid cleaning area."""
        # Calculate effective cleaning width and spacing between passes
        effective_width = surface_cleaner_diameter - path_overlap
        spacing_pixels = int(effective_width * self.transformer.scale)
        
        # Start from the bottom-left corner and move in a rectilinear pattern
        current_y = area_bounds['min_y']
        max_y = area_bounds['max_y']
        moving_right = True
        path_coords = []
        
        while current_y <= max_y:
            # Process current row
            row_coords = self._process_row(eroded_mask, current_y, moving_right, path_coords, min_x, min_y)
            
            if row_coords:
                path_coords.extend(row_coords)
            
            # Move to the next row
            current_y += spacing_pixels
            
            # Connect to next row if needed
            if current_y <= max_y:
                vertical_segment = self._connect_to_next_row(eroded_mask, current_y, moving_right, path_coords, min_x, min_y)
                
                if vertical_segment:
                    path_coords.append(vertical_segment)
                    # Switch direction for the next row
                    moving_right = not moving_right
        
        return path_coords
    
    def _process_row(
        self,
        eroded_mask: np.ndarray,
        row_y: int,
        moving_right: bool,
        current_path: List[Dict[str, float]],
        min_x: float,
        min_y: float
    ) -> List[Dict[str, float]]:
        """Process a single horizontal row in the cleaning path."""
        # Find the valid x-coordinates for this row
        row = eroded_mask[row_y, :]
        valid_x = np.where(row > 0)[0]
        
        if len(valid_x) == 0:
            return []
            
        row_min_x = valid_x[0]
        row_max_x = valid_x[-1]
        
        # Convert to inches for the path
        min_inch_x, min_inch_y = self.transformer.pixels_to_inches(row_min_x, row_y, min_x, min_y)
        max_inch_x, max_inch_y = self.transformer.pixels_to_inches(row_max_x, row_y, min_x, min_y)
        
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
    
    def _connect_to_next_row(
        self,
        eroded_mask: np.ndarray,
        next_row_y: int,
        moving_right: bool,
        current_path: List[Dict[str, float]],
        min_x: float,
        min_y: float
    ) -> Optional[Dict[str, float]]:
        """Create a vertical segment to connect to the next row."""
        if not current_path:
            return None
            
        # Find the valid x-coordinates for the next row
        new_row = eroded_mask[next_row_y, :]
        valid_x = np.where(new_row > 0)[0]
        
        if len(valid_x) == 0:
            return None
        
        # Convert to inches
        _, new_inch_y = self.transformer.pixels_to_inches(0, next_row_y, min_x, min_y)
        
        # Create vertical segment to connect current position to the next row
        return {'x': current_path[-1]['x'], 'y': new_inch_y}

def generate_cleaning_path(boundary_points: List[Dict[str, float]]) -> List[Dict[str, float]]:
    """Generate an optimal cleaning path from boundary points.
    
    This is the main entry point function for the path planner module.
    
    Args:
        boundary_points: List of dictionaries with 'x' and 'y' coordinates marking the boundary perimeter (in meters)
        
    Returns:
        List of waypoints (as dicts with 'x' and 'y' keys) for the robot to follow (in meters)
    """
    # Set the origin to the first boundary point for relative coordinates
    origin_x = boundary_points[0]['x']
    origin_y = boundary_points[0]['y']
    
    # Create coordinate transformer and path planner
    transformer = CoordinateTransformer(origin_x, origin_y)
    planner = PathPlanner(transformer)
    
    # Generate and return the cleaning path
    return planner.generate_cleaning_path(boundary_points) 