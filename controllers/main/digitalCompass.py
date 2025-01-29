
import math

class DigitalCompass:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # Initialize compass
        self.compass = robot.getDevice('compass')
        self.compass.enable(timestep)
        
        # Wait for initial stable reading
        for _ in range(10):
            robot.step(timestep)
        
        # Store initial bearing for reference
        self.initial_bearing = self.get_bearing()
        
        # For bearing smoothing
        self.bearing_history = []
        self.history_size = 5
    

    def get_bearing(self):
        """Get unfiltered bearing from compass"""
        north = self.compass.getValues()
        #print(f"Raw compass values: {north}")  # Debug print
        
        # Check if we're getting valid values
        if all(v == 0 for v in north):
            print("Warning: Compass returning all zeros!")
            return 0
            
        # Convert compass values to bearing (degrees from North)
        rad = math.atan2(north[1], north[0])
        bearing = (rad * 180.0 / math.pi) + 180
        return bearing
    
    def get_vectors(self):
        """Get vectors from compass"""
        north = self.compass.getValues()
        return north

 
    
    def get_bearing_change(self):
        """Get change in bearing from initial position"""
        current = self.get_bearing()
        diff = current - self.initial_bearing
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff
    
