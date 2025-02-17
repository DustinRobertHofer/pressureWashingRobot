from math import atan2, pi, sqrt, cos, sin

class Navigator:
    def __init__(self, robot, timestep):
        """Initialize navigation system"""
        self.robot = robot
        self.current_path = []
        self.current_goal = None
        self.path_index = 0
        
        # Default thresholds (will be updated by set_thresholds)
        self.waypoint_threshold = 0.1
        self.heading_threshold = 0.1
        self.TURN_THRESHOLD = 0.05  # About 11.5 degrees - angle at which to turn in place
        self.MAX_LINEAR_SPEED = 0.8
        self.MAX_ANGULAR_SPEED = 0.8
        
        # Obstacle avoidance parameters
        self.avoiding_obstacle = False
        self.avoidance_direction = 1  # 1 for left, -1 for right
        self.AVOIDANCE_TURN_ANGLE = pi/2  # 90 degrees for avoidance maneuver
        self.MIN_OBSTACLE_DISTANCE = 0.5  # Minimum distance to keep from obstacles
        self.SAFE_DISTANCE = 0.8  # Distance to maintain while avoiding
        self.avoidance_waypoint = None
        self.original_target = None
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle
        
    def get_turn_direction(self, current, target):
        """Determine which direction to turn (1 for left, -1 for right)"""
        diff = self.normalize_angle(target - current)
        return 1 if diff > 0 else -1
        
    def set_path(self, waypoints):
        """Set a path for the robot to follow"""
        self.current_path = waypoints
        self.path_index = 0
        self.current_goal = self.current_path[self.path_index] if waypoints else None
        
    def generate_avoidance_waypoint(self, x, y, theta, target):
        """Generate a temporary waypoint to help avoid the obstacle"""
        # Calculate perpendicular point at a safe distance
        perp_angle = theta + (self.AVOIDANCE_TURN_ANGLE * self.avoidance_direction)
        safe_x = x + self.SAFE_DISTANCE * cos(perp_angle)
        safe_y = y + self.SAFE_DISTANCE * sin(perp_angle)
        
        # Create waypoint slightly ahead in the avoidance direction
        return {'x': safe_x, 'y': safe_y}
        
    def get_next_command(self, current_position):
        """Get the next movement command to reach the goal"""
        # If no path or path completed, stop
        if not self.current_path or self.path_index >= len(self.current_path):
            self.current_goal = None
            return {'type': 'stop'}
            
        # Get current position and obstacle info
        x = current_position['x']
        y = current_position['y']
        theta = current_position['theta']
        obstacle_detected = current_position.get('obstacle_detected', False)
        obstacle_distance = current_position.get('obstacle_distance', float('inf'))
        
        # Get current target point
        target = self.avoidance_waypoint if self.avoiding_obstacle else self.current_path[self.path_index]
        
        # Handle obstacle avoidance
        if obstacle_detected:
            if not self.avoiding_obstacle:
                # Start avoidance maneuver
                self.avoiding_obstacle = True
                self.original_target = self.current_path[self.path_index]
                
                # Choose avoidance direction based on target position
                target_angle = atan2(target['y'] - y, target['x'] - x)
                angle_diff = self.normalize_angle(target_angle - theta)
                self.avoidance_direction = 1 if angle_diff > 0 else -1
                
                # Generate temporary avoidance waypoint
                self.avoidance_waypoint = self.generate_avoidance_waypoint(x, y, theta, target)
                target = self.avoidance_waypoint
            
            # Calculate avoidance angle
            avoidance_angle = theta + (self.AVOIDANCE_TURN_ANGLE * self.avoidance_direction)
            
            # Generate avoidance command with reduced speed near obstacles
            speed_factor = min(1.0, obstacle_distance / self.MIN_OBSTACLE_DISTANCE)
            return {
                'type': 'move',
                'linear_velocity': max(0.1, self.MAX_LINEAR_SPEED * speed_factor * 0.5),
                'angular_velocity': self.MAX_ANGULAR_SPEED * self.avoidance_direction
            }
        else:
            # If we were avoiding but now clear, check if we should return to original path
            if self.avoiding_obstacle:
                # Calculate distance to avoidance waypoint
                dx_avoid = self.avoidance_waypoint['x'] - x
                dy_avoid = self.avoidance_waypoint['y'] - y
                dist_to_avoid = sqrt(dx_avoid*dx_avoid + dy_avoid*dy_avoid)
                
                if dist_to_avoid < self.waypoint_threshold:
                    # Reached avoidance waypoint, return to original path
                    self.avoiding_obstacle = False
                    self.avoidance_waypoint = None
                    target = self.original_target
            
            # Normal navigation logic
            dx = target['x'] - x
            dy = target['y'] - y
            distance = sqrt(dx*dx + dy*dy)
            
            # If close enough to waypoint, move to next one
            if not self.avoiding_obstacle and distance < self.waypoint_threshold:
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
            
            # Otherwise, move towards target
            return {
                'type': 'move',
                'linear_velocity': self.MAX_LINEAR_SPEED * (0.5 if self.avoiding_obstacle else 1.0),
                'angular_velocity': self.MAX_ANGULAR_SPEED * heading_error / pi
            }

    def set_thresholds(self, waypoint_threshold, heading_threshold):
        """Set the waypoint and heading thresholds"""
        self.waypoint_threshold = waypoint_threshold
        self.heading_threshold = heading_threshold 