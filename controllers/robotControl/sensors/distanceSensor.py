class DistanceSensor:
    def __init__(self, robot, timestep):
        """Initialize distance sensor"""
        # Get and enable the distance sensor
        self.sensor = robot.getDevice('distance sensor')
        
        if not self.sensor:
            print("WARNING: Could not find distance sensor.")
            return
            
        print("Successfully initialized distance sensor")
        self.sensor.enable(timestep)
        
        # Wait for first valid reading
        robot.step(timestep)
        
    def get_distance(self):
        """Get current distance reading in meters"""
        if not self.sensor:
            return None
            
        # Get raw sensor value and convert to meters
        return self.sensor.getValue() / 1000.0  # Convert mm to meters 