from math import atan2, pi, sqrt, cos, sin
from config.robot_config import NAVIGATION_PARAMS, MOTION_PARAMS


class Navigator:
    """
    Handles robot navigation including path following and motion commands.
    """
    def __init__(self, robot, timestep):
        """
        Initialize the navigation system.
        
        Args:
            robot: The robot instance to control
            timestep: The simulation timestep
        """
        self.robot = robot
        self.current_path = []
        self.current_goal = None
        self.path_index = 0
        
        # Load parameters from configuration
        self.waypoint_threshold = NAVIGATION_PARAMS['waypoint_threshold']
        self.heading_threshold = NAVIGATION_PARAMS['heading_threshold']
        self.TURN_THRESHOLD = NAVIGATION_PARAMS['turn_threshold']
        self.MAX_LINEAR_SPEED = MOTION_PARAMS['max_linear_speed']
        self.MAX_ANGULAR_SPEED = MOTION_PARAMS['max_angular_speed']
        
    def normalize_angle(self, angle):
        """
        Normalize angle to the range [-pi, pi].
        
        Args:
            angle: The angle to normalize in radians
            
        Returns:
            The normalized angle
        """
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle
        
    def get_turn_direction(self, current, target):
        """
        Determine which direction to turn (1 for left, -1 for right).
        
        Args:
            current: Current heading in radians
            target: Target heading in radians
            
        Returns:
            Direction coefficient (1 for left, -1 for right)
        """
        diff = self.normalize_angle(target - current)
        return 1 if diff > 0 else -1
        
    def set_path(self, waypoints):
        """
        Set a path for the robot to follow.
        
        Args:
            waypoints: List of waypoint dictionaries with 'x' and 'y' keys
        """
        self.current_path = waypoints
        self.path_index = 0
        self.current_goal = self.current_path[self.path_index] if waypoints else None
        
    def generate_avoidance_waypoint(self, x, y, theta, target):
        """
        Generate a temporary waypoint to help avoid an obstacle.
        
        Args:
            x: Current x position
            y: Current y position
            theta: Current heading in radians
            target: Target waypoint dictionary
            
        Returns:
            Waypoint dictionary with 'x' and 'y' keys
        """
        # Calculate perpendicular point at a safe distance
        perp_angle = theta + (self.AVOIDANCE_TURN_ANGLE * self.avoidance_direction)
        safe_x = x + self.SAFE_DISTANCE * cos(perp_angle)
        safe_y = y + self.SAFE_DISTANCE * sin(perp_angle)
        
        # Create waypoint slightly ahead in the avoidance direction
        return {'x': safe_x, 'y': safe_y}
        
    def get_next_command(self, current_position):
        """
        Get the next movement command to reach the goal.
        
        Args:
            current_position: Dictionary with 'x', 'y', and 'theta' keys
            
        Returns:
            Command dictionary with movement instructions
        """
        # If no path or path completed, stop
        if not self.current_path or self.path_index >= len(self.current_path):
            self.current_goal = None
            return {'type': 'stop'}
            
        # Extract current position components
        x = current_position['x']
        y = current_position['y']
        theta = current_position['theta']
        
        # Get current target point
        target = self.current_path[self.path_index]
        
        # Calculate distance and direction to target
        dx = target['x'] - x
        dy = target['y'] - y
        distance = sqrt(dx*dx + dy*dy)
        
        # If close enough to waypoint, move to next one
        if distance < self.waypoint_threshold:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                return {'type': 'stop'}
            target = self.current_path[self.path_index]
            dx = target['x'] - x
            dy = target['y'] - y
            distance = sqrt(dx*dx + dy*dy)
        
        # Calculate target heading
        target_heading = atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - theta)
        
        # If heading error is large, turn in place
        if abs(heading_error) > self.TURN_THRESHOLD:
            return {
                'type': 'move',
                'linear_velocity': 0,
                'angular_velocity': self.MAX_ANGULAR_SPEED * self.get_turn_direction(theta, target_heading)
            }
        
        # Otherwise, move towards target with proportional angular correction
        return {
            'type': 'move',
            'linear_velocity': self.MAX_LINEAR_SPEED,
            'angular_velocity': self.MAX_ANGULAR_SPEED * heading_error / pi
        }

    def set_thresholds(self, waypoint_threshold, heading_threshold):
        """
        Set the waypoint and heading thresholds.
        
        Args:
            waypoint_threshold: Distance in meters to consider waypoint reached
            heading_threshold: Angle in radians to consider heading aligned
        """
        self.waypoint_threshold = waypoint_threshold
        self.heading_threshold = heading_threshold 