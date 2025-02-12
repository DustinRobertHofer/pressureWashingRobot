from .wheelSensors import WheelSensors
from .digitalCompass import DigitalCompass
from .distanceSensor import DistanceSensor

class SensorManager:
    def __init__(self, robot, timestep):
        """Initialize all robot sensors"""
        # Initialize individual sensors
        self.wheel_sensors = WheelSensors(robot, timestep)
        self.compass = DigitalCompass(robot, timestep)
        self.distance_sensor = DistanceSensor(robot, timestep)
        
        # Store latest sensor data
        self.sensor_data = {}
        
    def update(self):
        """Update all sensor readings"""
        # Get latest readings from all sensors
        wheel_data = self.wheel_sensors.get_wheel_distances()
        compass_data = self.compass.get_bearing()
        distance = self.distance_sensor.get_distance()
        
        # Combine all sensor data
        self.sensor_data = {
            'wheel_data': wheel_data,
            'compass_heading': compass_data
        }
        
        # Only include distance data if sensor is available
        if distance is not None:
            self.sensor_data['distance'] = distance
        
    def get_sensor_data(self):
        """Return the latest sensor readings"""
        return self.sensor_data 