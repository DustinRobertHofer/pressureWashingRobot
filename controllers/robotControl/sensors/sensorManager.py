from .wheelSensors import WheelSensors
from .digitalCompass import DigitalCompass
from .distanceSensor import DistanceSensor
from .laserRange import LaserRangeSensor

class SensorManager:
    def __init__(self, robot, timestep):
        """Initialize all robot sensors"""
        # Initialize individual sensors
        self.wheel_sensors = WheelSensors(robot, timestep)
        self.compass = DigitalCompass(robot, timestep)
        self.distance_sensor = DistanceSensor(robot, timestep)
        self.laser_sensor = LaserRangeSensor(robot, timestep)
        
        # Store latest sensor data
        self.sensor_data = {}
        
    def update(self):
        """Update all sensor readings"""
        # Get latest readings from all sensors
        wheel_data = self.wheel_sensors.get_wheel_distances()
        compass_data = self.compass.get_bearing()
        distance = self.distance_sensor.get_distance()
        laser_data = self.laser_sensor.get_distances()
        
        # Combine all sensor data
        self.sensor_data = {
            'wheel_data': wheel_data,
            'compass_heading': compass_data
        }
        
        # Only include distance data if sensor is available
        if distance is not None:
            self.sensor_data['distance'] = distance
            
        # Include laser data if available - check both has_sensor flag and ranges data
        if laser_data and laser_data.get('has_sensor', False):
            # Check if we actually got range data
            ranges = laser_data.get('ranges')
            if ranges is not None:
                print(f"Found laser sensor with {len(ranges) if ranges else 0} data points")
                self.sensor_data['laser'] = laser_data
            else:
                print("Laser sensor found but no range data available")
        else:
            print("No laser sensor data available in update")
            # Don't add 'laser' to sensor_data if not available
        
    def get_sensor_data(self):
        """Return the latest sensor readings"""
        return self.sensor_data 