from controller import Display
import math

class PathVisualizer:
    def __init__(self, robot, width=400, height=400, window_name="Robot Path Visualization"):
        """Initialize the path visualizer with a Webots Display device
        
        Args:
            robot: The Webots Robot instance
            width: Width of the display in pixels
            height: Height of the display in pixels
            window_name: Name of the window
        """
        # Create a display window
        self.display = robot.getDevice(window_name) if robot.getDevice(window_name) else None
        
        # If the display doesn't exist in the world, we can't create one from the controller
        if not self.display:
            print(f"WARNING: Could not find Display device named '{window_name}'. "
                  f"Please add a Display node to your world file with the name '{window_name}'.")
            return
            
        self.width = width
        self.height = height
        
        # Set display properties
        self.display_size = min(width, height)
        
        # Initialize colors
        self.background_color = 0xA9A9A9  # Dark Gray
        self.grid_color = 0xCCCCCC        # Light Gray
        self.path_color = 0x0000CC        # DarkBlue
        self.path_horizontal_color = 0x00AAFF  # Blue for horizontal segments
        self.path_vertical_color = 0x00AAFF    # Light blue for vertical segments
        self.robot_color = 0xFF0000       # Red
        self.waypoint_color = 0x006600    # Dark Green
        self.waypoint_start_color = 0x008800  # Darker green for start
        self.waypoint_end_color = 0xFF8800    # Orange for end
        
        # Path tracking
        self.path_points = []
        
        # Display scale and origin (for converting world coordinates to pixel coordinates)
        self.scale_factor = 1.0   # Will be calculated based on area size
        self.origin_x = 0.0
        self.origin_y = 0.0
        
        # Initialize display
        self.clear()
        
    def set_scale(self, min_x, max_x, min_y, max_y):
        """Set the scale based on the area bounds
        
        Args:
            min_x, max_x, min_y, max_y: Bounds of the area in world coordinates
        """
        # Add margins (10% on each side)
        margin_x = (max_x - min_x) * 0.1
        margin_y = (max_y - min_y) * 0.1
        
        min_x -= margin_x
        max_x += margin_x
        min_y -= margin_y
        max_y += margin_y
        
        # Calculate scale factor to fit the area in the display
        width_scale = self.display_size / (max_x - min_x)
        height_scale = self.display_size / (max_y - min_y)
        self.scale_factor = min(width_scale, height_scale)
        
        # Calculate origin for centered drawing
        self.origin_x = min_x
        self.origin_y = min_y
        
    def world_to_pixel(self, world_x, world_y):
        """Convert world coordinates to pixel coordinates
        
        Args:
            world_x, world_y: Coordinates in the world frame
            
        Returns:
            Tuple of (pixel_x, pixel_y)
        """
        pixel_x = int((world_x - self.origin_x) * self.scale_factor)
        # Invert y-axis (in display, (0,0) is top-left)
        pixel_y = int(self.height - (world_y - self.origin_y) * self.scale_factor)
        return (pixel_x, pixel_y)
        
    def clear(self):
        """Clear the display"""
        if not self.display:
            return
            
        # Fill background
        self.display.setColor(self.background_color)
        self.display.fillRectangle(0, 0, self.width, self.height)
        
    def draw_grid(self, grid_size=1.0):
        """Draw a grid
        
        Args:
            grid_size: Size of grid cells in world units (default: 1 meter)
        """
        if not self.display:
            return
            
        self.display.setColor(self.grid_color)
        
        # Calculate grid bounds
        x_start = math.floor(self.origin_x / grid_size) * grid_size
        x_end = x_start + (self.width / self.scale_factor)
        y_start = math.floor(self.origin_y / grid_size) * grid_size
        y_end = y_start + (self.height / self.scale_factor)
        
        # Draw vertical grid lines
        x = x_start
        while x <= x_end:
            start = self.world_to_pixel(x, y_start)
            end = self.world_to_pixel(x, y_end)
            self.display.drawLine(start[0], start[1], end[0], end[1])
            x += grid_size
            
        # Draw horizontal grid lines
        y = y_start
        while y <= y_end:
            start = self.world_to_pixel(x_start, y)
            end = self.world_to_pixel(x_end, y)
            self.display.drawLine(start[0], start[1], end[0], end[1])
            y += grid_size
            
    def draw_boundary(self, boundary_points):
        """Draw the boundary of the cleaning area
        
        Args:
            boundary_points: List of points defining the boundary
        """
        if not self.display or not boundary_points:
            return
            
        self.display.setColor(0x000000)  # Black
        
        # Draw each edge of the boundary
        for i in range(len(boundary_points)):
            p1 = boundary_points[i]
            p2 = boundary_points[(i + 1) % len(boundary_points)]
            
            pixel1 = self.world_to_pixel(p1['x'], p1['y'])
            pixel2 = self.world_to_pixel(p2['x'], p2['y'])
            
            self.display.drawLine(pixel1[0], pixel1[1], pixel2[0], pixel2[1])
    
    def draw_waypoints(self, waypoints):
        """Draw all the waypoints
        
        Args:
            waypoints: List of waypoint coordinates
        """
        if not self.display or not waypoints:
            return
            
        # Draw path segments connecting waypoints
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            
            pixel1 = self.world_to_pixel(p1['x'], p1['y'])
            pixel2 = self.world_to_pixel(p2['x'], p2['y'])
            
            # Determine if this is a horizontal or vertical segment
            if abs(p1['y'] - p2['y']) < 0.01:  # Horizontal segment
                self.display.setColor(self.path_horizontal_color)
            else:  # Vertical segment
                self.display.setColor(self.path_vertical_color)
            
            self.display.drawLine(pixel1[0], pixel1[1], pixel2[0], pixel2[1])
        
        # Draw waypoints as circles
        radius = 2  # pixels
        
        # Draw normal waypoints
        self.display.setColor(self.waypoint_color)
        for i in range(1, len(waypoints) - 1):
            point = waypoints[i]
            pixel = self.world_to_pixel(point['x'], point['y'])
            self.display.fillOval(pixel[0] - radius, pixel[1] - radius, 
                                 radius * 2, radius * 2)
        
        # Draw start waypoint with special color (larger)
        if waypoints:
            start_point = waypoints[0]
            start_pixel = self.world_to_pixel(start_point['x'], start_point['y'])
            self.display.setColor(self.waypoint_start_color)
            self.display.fillOval(start_pixel[0] - radius*2, start_pixel[1] - radius*2, 
                                radius * 4, radius * 4)
        
        # Draw end waypoint with special color (larger)
        if len(waypoints) > 1:
            end_point = waypoints[-1]
            end_pixel = self.world_to_pixel(end_point['x'], end_point['y'])
            self.display.setColor(self.waypoint_end_color)
            self.display.fillOval(end_pixel[0] - radius*2, end_pixel[1] - radius*2, 
                                radius * 4, radius * 4)
            
    def add_path_point(self, x, y):
        """Add a point to the robot's path
        
        Args:
            x, y: World coordinates of the point
        """
        self.path_points.append((x, y))
        
    def draw_path(self):
        """Draw the robot's path so far"""
        if not self.display or len(self.path_points) < 2:
            return
            
        self.display.setColor(self.path_color)
        
        # Draw lines connecting path points
        for i in range(len(self.path_points) - 1):
            p1 = self.path_points[i]
            p2 = self.path_points[i + 1]
            
            pixel1 = self.world_to_pixel(p1[0], p1[1])
            pixel2 = self.world_to_pixel(p2[0], p2[1])
            
            self.display.drawLine(pixel1[0], pixel1[1], pixel2[0], pixel2[1])
            
    def draw_robot(self, x, y, theta, size=10):
        """Draw the robot at the specified position
        
        Args:
            x, y: World coordinates of the robot center
            theta: Heading angle in radians
            size: Size of the robot marker in pixels
        """
        if not self.display:
            return
            
        pixel = self.world_to_pixel(x, y)
        
        # Draw robot body (circle)
        self.display.setColor(self.robot_color)
        self.display.fillOval(pixel[0] - size//2, pixel[1] - size//2, size, size)
        
        # Draw heading line
        end_x = pixel[0] + int(size * math.cos(theta))
        end_y = pixel[1] - int(size * math.sin(theta))  # Note: display y-axis is inverted
        self.display.drawLine(pixel[0], pixel[1], end_x, end_y)
        
    def update_visualization(self, robot_x, robot_y, robot_theta, boundary_points=None, waypoints=None):
        """Update the complete visualization
        
        Args:
            robot_x, robot_y: Current robot position
            robot_theta: Current robot heading
            boundary_points: List of boundary points (optional)
            waypoints: List of waypoints (optional)
        """
        if not self.display:
            return
            
        # Add current position to path
        self.add_path_point(robot_x, robot_y)
        
        # Redraw everything
        self.clear()
        self.draw_grid()
        
        if boundary_points:
            self.draw_boundary(boundary_points)
            
        if waypoints:
            self.draw_waypoints(waypoints)
            
        self.draw_path()
        self.draw_robot(robot_x, robot_y, robot_theta) 