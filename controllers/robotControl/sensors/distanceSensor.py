class DistanceSensor:
    def __init__(self, robot, timestep, sensor_name):
        """Initialize distance sensor"""
        # Get and enable the distance sensor
        self.sensor = robot.getDevice(sensor_name)
        
        # If named sensor not found, try to get the first distance sensor
        if not self.sensor:
            # Try to get any distance sensor (assumes only one distance sensor)
            try:
                # In some Webots versions, we can use this approach to get a sensor by index/type
                self.sensor = robot.getDistanceSensor(0)
            except:
                # If that fails, print warning
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