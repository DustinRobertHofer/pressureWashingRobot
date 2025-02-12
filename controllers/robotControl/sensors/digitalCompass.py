from math import atan2, pi

class DigitalCompass:
    def __init__(self, robot, timestep):
        """Initialize compass sensor"""
        # Get and enable compass
        self.compass = robot.getDevice('compass')
        self.compass.enable(timestep)
        
        # Wait for first valid reading
        robot.step(timestep)
        
    def get_bearing(self):
        """Get current heading in radians"""
        # Get compass values (points north)
        values = self.compass.getValues()
        
        # Calculate heading (atan2 returns angle in range [-pi, pi])
        # We use -values[0] because the compass x-axis is inverted
        heading = atan2(-values[0], values[1])
        
        # Convert to [0, 2pi] range
        if heading < 0:
            heading += 2 * pi
            
        return heading 