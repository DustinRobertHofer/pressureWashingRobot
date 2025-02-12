from math import atan2, pi, sqrt

class Navigator:
    def __init__(self, robot, timestep):
        """Initialize navigation system"""
        self.robot = robot  # Store the robot's drive system
        self.current_path = []
        self.current_goal = None
        self.path_index = 0
        self.final_orientation = None  # Target orientation for final position
        self.test_type = None  # Track which test is running
        
        # Navigation parameters
        self.GOAL_THRESHOLD = 0.05  # Distance in meters to consider goal reached (5cm)
        self.MAX_LINEAR_SPEED = 0.8  # Maximum linear speed in m/s (increased from 0.4)
        self.MAX_ANGULAR_SPEED = 0.8  # Maximum angular speed in rad/s
        self.ANGLE_THRESHOLD = 0.01  # About 3 degrees
        self.TURN_THRESHOLD = 0.05    # About 11.5 degrees - angle at which to turn in place
        
        # Initial heading is pi/2 (facing Y direction)
        self.INITIAL_HEADING = pi/2
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        
        return angle
        
    def get_turn_direction(self, current, target):
        """Determine which direction to turn (1 for left, -1 for right)"""
        # Normalize both angles to [-pi, pi]
        current = self.normalize_angle(current)
        target = self.normalize_angle(target)
            
        # Calculate the difference
        diff = target - current
        
        # Normalize the difference to [-pi, pi]
        if diff > pi:
            diff -= 2 * pi
        elif diff < -pi:
            diff += 2 * pi
            
        # Return direction based on shortest path
        # Positive diff means turn left (CCW), negative means turn right (CW)
        return 1 if diff > 0 else -1
        
    def set_square_test(self):
        """Set up a 1m x 1m square test path"""
        # Define the square corners (relative to start position)
        self.current_path = [
            {'x': 1.0, 'y': 0.0},  # First corner (forward 1m)
            {'x': 1.0, 'y': 1.0},  # Second corner (left 1m)
            {'x': 0.0, 'y': 1.0},  # Third corner (back 1m)
            {'x': 0.0, 'y': 0.0}   # Back to start
        ]
        self.path_index = 0
        self.current_goal = self.current_path[self.path_index]
        self.final_orientation = 0  # Return to original orientation (facing Y)
        self.test_type = "small"  # Set test type
        print(f"Starting square test. Moving to point {self.path_index + 1}: ({self.current_goal['x']:.2f}, {self.current_goal['y']:.2f})")
        
    def set_big_square_test(self):
        """Set up a 5m x 5m square test path"""
        # Define the square corners (relative to start position)
        self.current_path = [
            {'x': 4.8, 'y': 0.0},  # First corner (forward 5m)
            {'x': 4.8, 'y': 6.3},  # Second corner (left 5m)
            {'x': 0.0, 'y': 6.3},  # Third corner (back 5m)
            {'x': 0.0, 'y': 0.0}   # Back to start
        ]
        self.path_index = 0
        self.current_goal = self.current_path[self.path_index]
        self.final_orientation = 0  # Return to original orientation (facing Y)
        self.test_type = "big"  # Set test type
        print(f"Starting big square test. Moving to point {self.path_index + 1}: ({self.current_goal['x']:.2f}, {self.current_goal['y']:.2f})")
        
    def set_path(self, waypoints, final_orientation=None):
        """Set a custom path for the robot to follow
        
        Args:
            waypoints: List of dictionaries with 'x' and 'y' coordinates
            final_orientation: Final orientation in radians (optional)
        """
        self.current_path = waypoints
        self.path_index = 0
        self.current_goal = self.current_path[self.path_index] if waypoints else None
        self.final_orientation = final_orientation
        self.test_type = None  # This is a custom path, not a test
        
        if self.current_goal:
            print(f"Starting custom path with {len(waypoints)} waypoints")
            print(f"Moving to point {self.path_index + 1}: ({self.current_goal['x']:.2f}, {self.current_goal['y']:.2f})")
        
    def get_next_command(self, current_position):
        """Get the next movement command to reach the goal"""
        if not self.current_goal:
            return {'type': 'stop'}
            
        # Get current position
        x = current_position['x']
        y = current_position['y']
        theta = current_position['theta']  # Already normalized by sensor manager
        
        # Check if we've completed the path and need to handle final orientation
        if self.path_index >= len(self.current_path):
            if self.final_orientation is not None:
                # Calculate angle to final orientation
                angle_to_final = self.normalize_angle(self.final_orientation - theta)
                print(f"Final turn - Current: {theta:.2f} rad, Target: {self.final_orientation:.2f} rad, Diff: {angle_to_final:.2f} rad")
                
                if abs(angle_to_final) > self.ANGLE_THRESHOLD:
                    # Need to turn to final orientation
                    turn_direction = self.get_turn_direction(theta, self.final_orientation)
                    return {
                        'type': 'move',
                        'linear_velocity': 0.0,
                        'angular_velocity': self.MAX_ANGULAR_SPEED * turn_direction * 0.5  # Half speed for precision
                    }
            
            # If no final orientation or already at correct orientation, stop
            completion_message = "Path completed!"
            if self.test_type:  # If this was a specific test, add that info
                completion_message = f"{self.test_type.capitalize()} square test completed!"
            print(completion_message)
            self.current_goal = None
            self.test_type = None  # Reset test type
            return {'type': 'stop'}
        
        # Get current target point
        target = self.current_path[self.path_index]
        
        # Calculate distance and angle to target
        dx = target['x'] - x
        dy = target['y'] - y
        distance = sqrt(dx*dx + dy*dy)
        
        # Check if we've reached the current target
        if distance < self.GOAL_THRESHOLD:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                return self.get_next_command(current_position)  # Will handle final orientation
            self.current_goal = self.current_path[self.path_index]
            print(f"Reached point {self.path_index}. Moving to point {self.path_index + 1}: ({self.current_goal['x']:.2f}, {self.current_goal['y']:.2f})")
            target = self.current_path[self.path_index]
            dx = target['x'] - x
            dy = target['y'] - y
            distance = sqrt(dx*dx + dy*dy)
            
        # Calculate desired heading
        desired_theta = self.normalize_angle(atan2(dy, dx))
        
        # Calculate angle difference
        angle_diff = abs(self.normalize_angle(desired_theta - theta))
        
        # Determine turn direction
        if abs(angle_diff - pi) < 0.1:  # If we're close to 180 degrees difference
            # Always turn right in this case
            turn_direction = -1
        else:
            # Calculate turn direction normally
            diff = self.normalize_angle(desired_theta - theta)
            turn_direction = 1 if diff > 0 else -1
        
        # Debug angle information
        print(f"Current: {theta:.2f} ({theta*180/pi:.1f}°), Desired: {desired_theta:.2f} ({desired_theta*180/pi:.1f}°), Diff: {angle_diff:.2f} ({angle_diff*180/pi:.1f}°), Turn Dir: {turn_direction}")
        
        # If we're significantly off angle, turn in place
        if angle_diff > self.TURN_THRESHOLD:
            return {
                'type': 'move',
                'linear_velocity': 0.0,
                'angular_velocity': self.MAX_ANGULAR_SPEED * turn_direction
            }
        
        # If we're close enough to the desired angle, stop turning
        if angle_diff < self.ANGLE_THRESHOLD:
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

    def follow_path(self, current_position):
        """Execute one step of path following
        
        Args:
            current_position: Dictionary with current 'x', 'y', and 'theta' values
            
        Returns:
            bool: True if path is still being followed, False if completed
        """
        if not self.current_goal:
            return False
            
        # Get the next movement command
        command = self.get_next_command(current_position)
        
        # Execute the command based on its type
        if command['type'] == 'stop':
            self.robot.set_velocities(0, 0)  # Stop the robot
            return False
        elif command['type'] == 'move':
            # Execute the movement command
            linear_velocity = command['linear_velocity']
            angular_velocity = command['angular_velocity']
            
            # Send the velocities to the robot's drive system
            self.robot.set_velocities(linear_velocity, angular_velocity)
            
        return True 