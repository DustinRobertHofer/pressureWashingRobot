from sensors.wheelSensors import WheelSensors
from sensors.digitalCompass import DigitalCompass
from sensors.distanceSensor import DistanceSensor
from config.robot_config import ROBOT_PARAMS
from math import tan, cos

class SensorManager:
    def __init__(self, robot, timestep):
        """Initialize all robot sensors"""
        # Initialize individual sensors
        self.wheel_sensors = WheelSensors(robot, timestep)
        self.compass = DigitalCompass(robot, timestep)
        self.forward_laser = DistanceSensor(robot, timestep, 'forward_laser')
        self.left_laser = DistanceSensor(robot, timestep, 'left_laser')
        self.right_laser = DistanceSensor(robot, timestep, 'right_laser')
        
        # Store latest sensor data
        self.sensor_data = {}
       
    def update(self):
        """Update all sensor readings"""
        # Get latest readings from all sensors
        wheel_data = self.wheel_sensors.get_wheel_distances()
        compass_data = self.compass.get_bearing()
        forward_distance = self.forward_laser.get_distance()
        left_fwd_distance = self.calculate_fwd_distance(self.left_laser.get_distance())
        right_fwd_distance = self.calculate_fwd_distance(self.right_laser.get_distance())
        left_side_distance = self.calculate_side_distance(self.left_laser.get_distance())
        right_side_distance = self.calculate_side_distance(self.right_laser.get_distance())

        
        # Combine all sensor data
        self.sensor_data = {
            'wheel_data': wheel_data,
            'compass_heading': compass_data,
            'forward_distance': forward_distance,
            'left_fwd_distance': left_fwd_distance,
            'right_fwd_distance': right_fwd_distance,
            'left_side_distance': left_side_distance,
            'right_side_distance': right_side_distance
        }

        #print('forward_distance: ', forward_distance, 'left_fwd_distance: ', left_fwd_distance, 'right_fwd_distance: ', right_fwd_distance, 'left_side_distance: ', left_side_distance, 'right_side_distance: ', right_side_distance)
   
    def calculate_height(self, distance_raw):
        """Calculate height from raw distance"""
        return (distance_raw**2 / (1+tan(ROBOT_PARAMS['sensor_pitch'])**2+tan(ROBOT_PARAMS['sensor_yaw'])**2*tan(ROBOT_PARAMS['sensor_pitch'])))**0.5
    

    def calculate_fwd_distance(self, distance_raw):
        """Calculate side distance from raw distance"""
        height = self.calculate_height(distance_raw)
        return (height**2 * tan(ROBOT_PARAMS['sensor_pitch'])**2)**0.5

    def calculate_side_distance(self, distance_raw):
        """Calculate forward distance from raw distance"""
        fwd_distance = self.calculate_fwd_distance(distance_raw)
        return (fwd_distance**2 * tan(ROBOT_PARAMS['sensor_yaw'])**2)**0.5
    
 

    def get_sensor_data(self):
        """Return the latest sensor readings"""
        return self.sensor_data 