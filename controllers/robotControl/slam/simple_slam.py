"""
Simple SLAM implementation inspired by BreezySLAM
This simplified version works on Windows and provides basic SLAM functionality
for the pressure washing robot.
"""
import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import os
import time

class SimpleLaserScanner:
    """Simple class to manage laser scanner parameters"""
    def __init__(self, scan_size=360, scan_rate_hz=10, 
                detection_angle_degrees=360, distance_no_detection_mm=5000):
        self.scan_size = scan_size
        self.scan_rate_hz = scan_rate_hz
        self.detection_angle_degrees = detection_angle_degrees
        self.distance_no_detection_mm = distance_no_detection_mm
        self.detection_angle_radians = math.radians(detection_angle_degrees)
        self.angle_increment = self.detection_angle_radians / (scan_size - 1) if scan_size > 1 else 0

class Map:
    """2D occupancy grid map for SLAM"""
    
    def __init__(self, size_pixels, size_meters):
        """Initialize a square map with given size and resolution"""
        self.size_pixels = size_pixels
        self.size_meters = size_meters
        self.pixels_per_meter = size_pixels / size_meters
        
        # Initialize the map grid (0 = unknown, 100 = occupied, -1 = free)
        self.grid = np.zeros((size_pixels, size_pixels), dtype=np.int8)
        
        # Map center
        self.center = size_pixels // 2
        
    def update(self, x, y, theta, scan_distances):
        """Update map using LiDAR scan data from current robot position"""
        # Convert robot position from meters to pixels
        robot_x = int(self.center + x * self.pixels_per_meter)
        robot_y = int(self.center + y * self.pixels_per_meter)
        
        # Mark robot's position as free space
        if 0 <= robot_x < self.size_pixels and 0 <= robot_y < self.size_pixels:
            self.grid[robot_y, robot_x] = -1
        
        if scan_distances is None:
            return
        
        # Process each LiDAR scan point
        num_points = len(scan_distances)
        if num_points == 0:
            return
            
        angle_increment = 2 * math.pi / num_points
        
        for i, distance in enumerate(scan_distances):
            # Skip invalid readings (negative, infinity, or very large values)
            if distance <= 0 or math.isinf(distance) or distance > 100:
                continue
                
            # Convert distance from meters to pixels (with safety check)
            distance_pixels = min(distance * self.pixels_per_meter, self.size_pixels)
            
            # Calculate angle of this laser ray
            angle = theta + i * angle_increment
            
            # Calculate endpoint of the ray (safely convert to int)
            try:
                end_x = robot_x + int(distance_pixels * math.cos(angle))
                end_y = robot_y + int(distance_pixels * math.sin(angle))
            except (ValueError, OverflowError):
                # Skip this ray if calculation fails
                print(f"Warning: Invalid ray calculation for distance={distance}")
                continue
            
            # Check if endpoint is within map bounds
            if 0 <= end_x < self.size_pixels and 0 <= end_y < self.size_pixels:
                # Mark endpoint as occupied
                self.grid[end_y, end_x] = 100
                
                # Use Bresenham's line algorithm to trace the ray and mark free space
                line_points = self._bresenham_line(robot_x, robot_y, end_x, end_y)
                for px, py in line_points[:-1]:  # All but the last point are free
                    if 0 <= px < self.size_pixels and 0 <= py < self.size_pixels:
                        # Only mark as free if not already marked as occupied
                        if self.grid[py, px] != 100:
                            self.grid[py, px] = -1
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Use Bresenham's line algorithm to trace a line between two points"""
        # Ensure inputs are integers
        x0, y0 = int(x0), int(y0)
        x1, y1 = int(x1), int(y1)
        
        # Calculate deltas and steps
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        # Safety check for maximum line length
        max_points = 2 * self.size_pixels
        point_count = 0
        
        line_points = []
        while (x0 != x1 or y0 != y1) and point_count < max_points:
            line_points.append((x0, y0))
            point_count += 1
            
            # Update free space in occupancy grid
            if 0 <= x0 < self.size_pixels and 0 <= y0 < self.size_pixels:
                # Only mark as free if not already marked as occupied
                if self.grid[y0, x0] != 100:
                    self.grid[y0, x0] = -1
            
            # Calculate next point
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
            
            # Safety check for bounds
            if not (0 <= x0 < self.size_pixels * 2 and 0 <= y0 < self.size_pixels * 2):
                break
                
        # Add the endpoint if we didn't reach it
        if point_count < max_points and (x0 != x1 or y0 != y1):
            line_points.append((x1, y1))
            
        return line_points
    
    def get_pixel(self, x, y):
        """Get the occupancy value at a specific pixel"""
        if 0 <= x < self.size_pixels and 0 <= y < self.size_pixels:
            return self.grid[y, x]
        return 0
        
    def get_meter(self, x, y):
        """Get the occupancy value at coordinates in meters"""
        # Convert meters to pixels
        pix_x = int(self.center + x * self.pixels_per_meter)
        pix_y = int(self.center + y * self.pixels_per_meter)
        return self.get_pixel(pix_x, pix_y)
    
    def display(self, robot_x=None, robot_y=None, robot_theta=None):
        """Display the map using matplotlib and optionally save it"""
        plt.figure(figsize=(10, 10))
        
        # Display the grid with proper scaling
        plt.imshow(self.grid, cmap='gray_r', origin='lower')
        
        # Mark robot position if provided
        if robot_x is not None and robot_y is not None:
            # Convert robot position from meters to pixels
            robot_pix_x = self.center + robot_x * self.pixels_per_meter
            robot_pix_y = self.center + robot_y * self.pixels_per_meter
            
            # Draw robot as a red dot
            plt.plot(robot_pix_x, robot_pix_y, 'ro', markersize=10)
            
            # Draw direction indicator if orientation is provided
            if robot_theta is not None:
                direction_length = 20  # Length of direction indicator in pixels
                dx = direction_length * math.cos(robot_theta)
                dy = direction_length * math.sin(robot_theta)
                plt.arrow(robot_pix_x, robot_pix_y, dx, dy, 
                          head_width=8, head_length=10, fc='red', ec='red')
        
        plt.title('SLAM Map')
        plt.colorbar(label='Occupancy (100=occupied, -1=free, 0=unknown)')
        
        # Save the map to an image file
        timestamp = int(time.time())
        maps_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'maps')
        if not os.path.exists(maps_dir):
            os.makedirs(maps_dir)
        
        # Save the file
        map_file = os.path.join(maps_dir, f'slam_map_{timestamp}.png')
        plt.savefig(map_file)
        print(f"Map saved to: {map_file}")
        
        # Show the map
        plt.show()

class SimpleSLAM:
    """Simple SLAM implementation for robot mapping"""
    
    def __init__(self, laser_scanner, map_size_pixels=500, map_size_meters=10):
        """Initialize the SLAM system with laser parameters and map size"""
        self.laser_scanner = laser_scanner
        
        # Create the map
        self.map = Map(map_size_pixels, map_size_meters)
        
        # Initialize robot position (in meters and radians)
        self.x = 0
        self.y = 0
        self.theta = 0
        
    def update(self, odometry, scan_distances):
        """Update SLAM with new odometry and scan data"""
        # Update position based on odometry
        self.x = odometry['x']
        self.y = odometry['y']
        self.theta = odometry['theta']
        
        # Debug position
        print(f"SLAM position update: x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.1f}°")
        
        # Skip map update if no scan data
        if scan_distances is None or len(scan_distances) == 0:
            print("No LiDAR scan data available for SLAM update")
            return
            
        # Update the map with the current position and scan data
        self.map.update(self.x, self.y, self.theta, scan_distances)
        
        # TODO: Implement particle filter for better position estimation
        # For now, we're just using odometry which will drift over time
        
    def get_map(self):
        """Return the current SLAM map"""
        return self.map
        
    def get_position(self):
        """Return the current position estimate"""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'uncertainty': 0.0  # Assuming no uncertainty for now
        }
        
    def display_map(self):
        """Display the current map with robot position"""
        self.map.display(self.x, self.y, self.theta)
        
    def get_occupancy(self, x, y):
        """Check if a specific location (in meters) is occupied"""
        return self.map.get_meter(x, y)
        
    def get_cleaning_boundary(self):
        """Extract a cleaning boundary from the map"""
        # Simple implementation: Find contiguous free spaces
        # This is a placeholder - a real implementation would do more sophisticated boundary detection
        
        free_space = np.where(self.map.grid == -1)
        if len(free_space[0]) == 0:
            return []
            
        # Get min/max coordinates of free space
        min_y, max_y = np.min(free_space[0]), np.max(free_space[0])
        min_x, max_x = np.min(free_space[1]), np.max(free_space[1])
        
        # Convert to meters
        center = self.map.center
        pixels_per_meter = self.map.pixels_per_meter
        
        min_x_m = (min_x - center) / pixels_per_meter
        max_x_m = (max_x - center) / pixels_per_meter
        min_y_m = (min_y - center) / pixels_per_meter
        max_y_m = (max_y - center) / pixels_per_meter
        
        # Return corners of free space boundary
        return [
            {'x': min_x_m, 'y': min_y_m},
            {'x': max_x_m, 'y': min_y_m},
            {'x': max_x_m, 'y': max_y_m},
            {'x': min_x_m, 'y': max_y_m}
        ] 