"""
SLAM Manager for the pressure washing robot.
This module integrates the SimpleSLAM implementation with the robot's sensor and navigation systems.
"""
import os
import sys
import numpy as np
import math
import time

# Add parent directory to path to resolve imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from .simple_slam import SimpleLaserScanner, SimpleSLAM
from config.robot_config import ROBOT_PARAMS, NAVIGATION_PARAMS

class SLAMManager:
    """Manages SLAM integration with the robot control system"""
    
    def __init__(self, sensor_manager, state):
        """Initialize SLAM manager with required subsystems"""
        self.sensor_manager = sensor_manager
        self.state = state
        self.map_size_meters = 10  # Default map size in meters
        
        # Create laser parameters object
        self.laser_params = SimpleLaserScanner(
            scan_size=360,  # 360 points for full LiDAR scan
            scan_rate_hz=10,  # Scan rate in Hz
            detection_angle_degrees=360,  # Full 360 degree LiDAR scan
            distance_no_detection_mm=8000  # 8 meters max range for LiDAR
        )
        
        # Create SLAM object
        self.slam = SimpleSLAM(
            self.laser_params,
            map_size_pixels=500,  # 500x500 pixel map
            map_size_meters=self.map_size_meters  # 10x10 meter map
        )
        
        # Tracking variables
        self.is_mapping = False
        self.last_position = {'x': 0, 'y': 0, 'theta': 0}
        self.map_folder = "maps"
        
        # Create maps directory if it doesn't exist
        if not os.path.exists(self.map_folder):
            os.makedirs(self.map_folder)
    
    def start_mapping(self):
        """Begin the SLAM mapping process"""
        self.is_mapping = True
        # Initialize with current position
        current_pos = self.state.get_position()
        self.last_position = {
            'x': current_pos['x'],
            'y': current_pos['y'],
            'theta': current_pos['theta'],
            'previous_x': current_pos['x'],
            'previous_y': current_pos['y']
        }
    
    def stop_mapping(self):
        """Stop the SLAM mapping process"""
        self.is_mapping = False
    
    def update(self):
        """Update SLAM with latest sensor readings"""
        if not self.is_mapping:
            return
            
        # Get latest sensor data
        # First, ensure sensors are updated
        self.sensor_manager.update()
        sensor_data = self.sensor_manager.get_sensor_data()
        
        # Get LiDAR readings if available
        scan_distances = None
        if 'laser' in sensor_data:
            raw_ranges = sensor_data['laser'].get('ranges', None)
            if raw_ranges:
                # Preprocess the LiDAR data to handle invalid values
                scan_distances = []
                for value in raw_ranges:
                    # Replace infinity values with a large but finite number
                    # and filter out any negative values
                    if math.isinf(value):
                        scan_distances.append(50.0)  # 50 meters is effectively "no detection"
                    elif value < 0:
                        scan_distances.append(0.0)   # Replace negative values with 0
                    else:
                        scan_distances.append(value) # Keep valid measurements
                
                print(f"Using LiDAR data with {len(scan_distances)} points for SLAM update")
            else:
                print("LiDAR data available but ranges are empty")
        else:
            print("No LiDAR data available for SLAM update")
        
        # Get current position from wheel encoders and compass
        current_pos = self.state.get_position()
        
        # Prepare odometry data
        odometry = {
            'x': current_pos['x'],
            'y': current_pos['y'],
            'theta': current_pos['theta'],
            'previous_x': self.last_position['x'],
            'previous_y': self.last_position['y']
        }
        
        # Update SLAM with new data
        self.slam.update(odometry, scan_distances)
        
        # Save current position for next update
        self.last_position = odometry.copy()
    
    def get_map(self):
        """Get the current SLAM map"""
        return self.slam.get_map()
    
    def get_position(self):
        """Get the current position estimate from SLAM"""
        return self.slam.get_position()
    
    def display_map(self):
        """Display the current map"""
        self.slam.display_map()
    
    def save_map(self, filename="map"):
        """Save the current SLAM map to disk"""
        # Create the maps directory if it doesn't exist
        if not os.path.exists(self.map_folder):
            os.makedirs(self.map_folder)
            
        # Add a timestamp to avoid overwriting files
        timestamp = int(time.time())
        
        # Save the map grid
        map_grid = self.slam.get_map().grid
        grid_filename = os.path.join(self.map_folder, f"{filename}_{timestamp}.npy")
        np.save(grid_filename, map_grid)
        
        # Also save a human-readable image
        map_image = os.path.join(self.map_folder, f"{filename}_{timestamp}.png")
        # Save image of the map
        try:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(10, 10))
            plt.imshow(map_grid, cmap='gray_r', origin='lower')
            plt.title('SLAM Map')
            plt.colorbar(label='Occupancy (100=occupied, -1=free, 0=unknown)')
            # Save without displaying
            plt.savefig(map_image)
            plt.close()
        except Exception as e:
            print(f"Error saving map image: {e}")
        
        print(f"Map saved to {grid_filename} and {map_image}")
        return True
        
    def load_map(self, filename="map"):
        """Load a map from a file"""
        filepath = os.path.join(self.map_folder, f"{filename}.npy")
        if os.path.exists(filepath):
            map_obj = self.slam.get_map()
            map_obj.grid = np.load(filepath)
            return True
        return False
    
    def get_cleaning_boundary(self):
        """Get a cleaning boundary from the SLAM map"""
        return self.slam.get_cleaning_boundary()
    
    def set_cleaning_boundary(self, boundary_points):
        """Set the cleaning boundary on the SLAM map
        
        Args:
            boundary_points: List of dictionaries with 'x' and 'y' coordinates defining the boundary
        """
        # Convert boundary points to pixels on the map
        size_pixels = self.slam.map.size_pixels
        center = self.slam.map.center
        pixels_per_meter = self.slam.map.pixels_per_meter
        
        # Mark the boundary in the map
        for i in range(len(boundary_points)):
            # Convert meters to pixels, negate y to match flipped coordinate system
            x1 = int(center + boundary_points[i]['x'] * pixels_per_meter)
            y1 = int(center - boundary_points[i]['y'] * pixels_per_meter) # Negate y coordinate
            
            # Get the next point (wrapping around to the first point)
            next_idx = (i + 1) % len(boundary_points)
            x2 = int(center + boundary_points[next_idx]['x'] * pixels_per_meter)
            y2 = int(center - boundary_points[next_idx]['y'] * pixels_per_meter) # Negate y coordinate
            
            # Draw a line between these points as a boundary
            self._draw_boundary_line(x1, y1, x2, y2)
        
        print(f"Added cleaning boundary with {len(boundary_points)} points to SLAM map")
        return True
        
    def _draw_boundary_line(self, x1, y1, x2, y2):
        """Draw a boundary line between two points using Bresenham's algorithm
        
        The boundary is marked on the map as occupied (value 100)
        """
        # Use the internal _bresenham_line method to get the points along the line
        line_points = self.slam.map._bresenham_line(x1, y1, x2, y2)
        
        # Mark these points as boundaries (occupied) in the map
        for x, y in line_points:
            if 0 <= x < self.slam.map.size_pixels and 0 <= y < self.slam.map.size_pixels:
                self.slam.map.grid[y, x] = 100  # 100 indicates occupied space
    
    def is_position_free(self, x, y):
        """Check if a position is free according to the map"""
        occupancy = self.slam.get_occupancy(x, y)
        return occupancy == -1  # -1 indicates free space
    
    def display_map_periodic(self, current_time, time_interval=5.0):
        """Display the map periodically based on the time interval
        
        Args:
            current_time: Current simulation time in seconds
            time_interval: Time interval between map displays in seconds (default: 5.0)
        
        Returns:
            bool: True if map was displayed, False otherwise
        """
        # Initialize last display time if not already set
        if not hasattr(self, 'last_map_display_time'):
            self.last_map_display_time = 0
            
        # Check if enough time has passed since the last display
        if current_time - self.last_map_display_time >= time_interval:
            # Display the map
            self.display_map()
            # Update last display time
            self.last_map_display_time = current_time
            return True
            
        return False
    
    def generate_cleaning_path_from_map(self):
        """Generate a cleaning path from the SLAM map"""
        # Get the boundary of the mapped area
        boundary = self.get_cleaning_boundary()
        
        if not boundary:
            return []
            
        # This can be integrated with your existing path_planner.py logic
        # For now, we'll return the boundary points as a simple path
        return boundary 