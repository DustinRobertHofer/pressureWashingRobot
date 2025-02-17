from math import atan2, pi, sqrt

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
        
    def get_next_command(self, current_position):
        """Get the next movement command to reach the goal"""
        # If no path or path completed, stop
        if not self.current_path or self.path_index >= len(self.current_path):
            self.current_goal = None
            return {'type': 'stop'}
            
        # Get current position
        x = current_position['x']
        y = current_position['y']
        theta = current_position['theta']
        
        # Get current target point
        target = self.current_path[self.path_index]
        
        # Calculate distance and angle to target
        dx = target['x'] - x
        dy = target['y'] - y
        distance = sqrt(dx*dx + dy*dy)
        
        # Check if we've reached the current target
        if distance < self.waypoint_threshold:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                self.current_goal = None
                return {'type': 'stop'}
            self.current_goal = self.current_path[self.path_index]
            target = self.current_goal
            dx = target['x'] - x
            dy = target['y'] - y
            distance = sqrt(dx*dx + dy*dy)
        
        # Calculate desired heading
        desired_theta = self.normalize_angle(atan2(dy, dx))
        angle_diff = abs(self.normalize_angle(desired_theta - theta))
        
        # Determine turn direction
        turn_direction = -1 if abs(angle_diff - pi) < 0.1 else self.get_turn_direction(theta, desired_theta)
        
        # If we're significantly off angle, turn in place
        if angle_diff > self.TURN_THRESHOLD:
            return {
                'type': 'move',
                'linear_velocity': 0.0,
                'angular_velocity': self.MAX_ANGULAR_SPEED * turn_direction
            }
        
        # If we're close enough to the desired angle, move straight
        if angle_diff < self.heading_threshold:
            return {
                'type': 'move',
                'linear_velocity': self.MAX_LINEAR_SPEED * min(1.0, distance),
                'angular_velocity': 0.0
            }
        
        # Otherwise, move forward while making small angle corrections
        return {
            'type': 'move',
            'linear_velocity': self.MAX_LINEAR_SPEED * min(1.0, distance) * 0.5,
            'angular_velocity': self.MAX_ANGULAR_SPEED * turn_direction * 0.3
        }

    def set_thresholds(self, waypoint_threshold, heading_threshold):
        """Set the waypoint and heading thresholds"""
        self.waypoint_threshold = waypoint_threshold
        self.heading_threshold = heading_threshold 