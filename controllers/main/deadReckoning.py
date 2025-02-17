from math import sin, cos, pi

# comment

class DeadReckoning:
    def __init__(self, robot, timestep):
        """Initialize dead reckoning system using wheel encoders"""
        # Get and enable wheel encoders
        self.left_encoder = robot.getDevice('left_pos')
        self.right_encoder = robot.getDevice('right_pos')
        self.left_encoder.enable(timestep)
        self.right_encoder.enable(timestep)
        
        # Robot physical parameters (in meters)
        self.WHEEL_RADIUS = 0.08  # 8cm = 0.08m
        self.WHEEL_BASE = 0.18122 * 2 +.04 # 36.195cm = 0.360975m (distance between wheels)
        
        # Store previous encoder values
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        # Initialize position state
        self.x = 0.0  # X position in meters
        self.y = 0.0  # Y position in meters
        self.theta = 0.0  # Heading in radians
        
        # Wait for first valid encoder readings
        robot.step(timestep)
        self.prev_left_ticks = self.left_encoder.getValue()
        self.prev_right_ticks = self.right_encoder.getValue()

    def update(self):
        """Update position estimate based on wheel encoder readings"""
        # Get current encoder values
        left_ticks = self.left_encoder.getValue()
        right_ticks = self.right_encoder.getValue()
        
        # Calculate change in encoder ticks
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        
        # Convert ticks to distance traveled by each wheel
        left_dist = delta_left * self.WHEEL_RADIUS
        right_dist = delta_right * self.WHEEL_RADIUS
        
        # Calculate distance traveled by robot center
        dist = (left_dist + right_dist) / 2.0
        
        # Calculate change in heading (reversed the sign to match conventional rotation)
        delta_theta = (left_dist - right_dist) / self.WHEEL_BASE  # Changed from (right - left) to (left - right)
        
        # Average heading during movement
        avg_theta = self.theta + delta_theta / 2.0
        
        # Update position
        self.x += dist * cos(avg_theta)
        self.y += dist * sin(avg_theta)
        self.theta = (self.theta + delta_theta) % (2 * pi)
        
        # Store current encoder values for next update
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        
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
        
        # Reset encoder references
        self.prev_left_ticks = self.left_encoder.getValue()
        self.prev_right_ticks = self.right_encoder.getValue() 