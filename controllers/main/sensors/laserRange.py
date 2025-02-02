

class LaserRange:
    def __init__(self, robot, timestep):
        self.distance_sensor = robot.getDevice("distance sensor")
        self.distance_sensor.enable(timestep)
        self.max_distance = 2.0  # Maximum reliable detection distance in meters
        self.min_safe_distance = 1.0  # Minimum safe distance in meters

    def get_distance(self):
        """Get the current distance reading in meters"""
        return self.distance_sensor.getValue()/1000

    def is_obstacle_detected(self):
        """Check if an obstacle is within the minimum safe distance"""
        distance = self.get_distance()
        return distance < self.min_safe_distance and distance >= 0

  