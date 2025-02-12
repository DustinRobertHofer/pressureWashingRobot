from math import cos, sin, pi

class State:
    def __init__(self, robot, timestep):
        """Initialize state tracking system"""
        self.x = 0.0  # X position in meters
        self.y = 0.0  # Y position in meters
        self.theta = 0.0  # Heading in radians
        
        # Parameters for turn detection
        self.prev_theta = 0.0
        self.TURN_THRESHOLD = 0.01  # About 0.57 degrees in radians
        
    def update(self, sensor_data):
        """Update state estimation using sensor data"""
        # Get sensor readings
        wheel_data = sensor_data['wheel_data']
        compass_heading = sensor_data['compass_heading']
        
        # Update heading
        self.prev_theta = self.theta
        self.theta = self.normalize_angle(compass_heading)
        
        # Check if turning
        is_turning = abs(self.theta - self.prev_theta) > self.TURN_THRESHOLD
        
        # Only update position if not turning
        if not is_turning and wheel_data['forward_motion'] != 0:
            forward_motion = wheel_data['forward_motion']
            self.x += forward_motion * cos(self.theta)
            self.y += forward_motion * sin(self.theta)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        return angle
            
    def get_position(self):
        """Return current estimated position and heading"""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta
        }
        
    def reset(self):
        """Reset position estimation to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_theta = 0.0 