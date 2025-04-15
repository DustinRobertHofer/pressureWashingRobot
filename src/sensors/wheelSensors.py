class WheelSensors:
    def __init__(self, robot, timestep):
        """Initialize wheel encoder sensors"""
        # Get and enable wheel encoders
        self.left_encoder = robot.getDevice('left_pos')
        self.right_encoder = robot.getDevice('right_pos')
        
        if not (self.left_encoder and self.right_encoder):
            print("WARNING: Could not find wheel position sensors.")
            return
            
        print("Successfully initialized wheel encoders: left_pos, right_pos")
        
        self.left_encoder.enable(timestep)
        self.right_encoder.enable(timestep)
        
        # Robot physical parameters (in meters)
        self.WHEEL_RADIUS = 0.08  # 8cm = 0.08m
        
        # Store previous positions in meters
        self.prev_left_distance = 0
        self.prev_right_distance = 0
        
        # Wait for first valid encoder readings and convert to distances
        robot.step(timestep)
        self.prev_left_distance = self._ticks_to_distance(self.left_encoder.getValue())
        self.prev_right_distance = self._ticks_to_distance(self.right_encoder.getValue())
    
    def _ticks_to_distance(self, ticks):
        """Convert encoder ticks to distance in meters"""
        return ticks * self.WHEEL_RADIUS
    
    def get_wheel_distances(self, is_turning=False):
        """Get the current wheel distances and changes since last reading in meters"""
        if not (self.left_encoder and self.right_encoder):
            return {
                'left_distance': 0,
                'right_distance': 0,
                'delta_left': 0,
                'delta_right': 0,
                'forward_motion': 0
            }
            
        # Get current encoder values and convert to distances
        left_distance = self._ticks_to_distance(self.left_encoder.getValue())
        right_distance = self._ticks_to_distance(self.right_encoder.getValue())
        
        # Calculate change in distances
        delta_left = left_distance - self.prev_left_distance
        delta_right = right_distance - self.prev_right_distance
        
        # Store current distances for next update
        self.prev_left_distance = left_distance
        self.prev_right_distance = right_distance
        
        # Calculate forward motion (average of wheels) when not turning
        forward_motion = 0 if is_turning else (delta_left + delta_right) / 2
        
        return {
            'left_distance': left_distance,
            'right_distance': right_distance,
            'delta_left': delta_left,
            'delta_right': delta_right,
            'forward_motion': forward_motion
        }
    
    def reset(self):
        """Reset distance references"""
        if not (self.left_encoder and self.right_encoder):
            return
            
        self.prev_left_distance = self._ticks_to_distance(self.left_encoder.getValue())
        self.prev_right_distance = self._ticks_to_distance(self.right_encoder.getValue()) 