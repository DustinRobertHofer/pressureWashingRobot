import time
from sensors.laserRange import PiLaserRangeFinder
from sensors.digitalCompass import PiCompass

class PiSensorManager:
    """Sensor manager for Raspberry Pi hardware implementation."""
    
    def __init__(self):
        """Initialize all sensors for Raspberry Pi."""
        # Initialize sensors
        self.laser_range = PiLaserRangeFinder()
        self.compass = PiCompass()
        
        # Store latest sensor data
        self.sensor_data = {}
        self.last_update_time = 0
        self.update_interval = 0.05  # 50ms update interval
        
        # Track wheel data (this would be obtained from the motors)
        self.wheel_data = {
            'left_distance': 0,
            'right_distance': 0,
            'delta_left': 0,
            'delta_right': 0,
            'forward_motion': 0
        }
        
        # Perform initial sensor update
        self.update()
        
    def update(self):
        """Update all sensor readings."""
        current_time = time.time()
        
        # Only update at specified interval
        if current_time - self.last_update_time < self.update_interval:
            return
            
        self.last_update_time = current_time
        
        # Get laser range finder reading
        distance = self.laser_range.get_distance()
        print(distance)
        
        # Get compass heading
        compass_heading = self.compass.get_bearing()
        
        # Combine all sensor data
        self.sensor_data = {
            'wheel_data': self.wheel_data,
            'compass_heading': compass_heading,
            'distance': distance
        }
        
        # Only include distance data if sensor is available
        if distance is not None:
            self.sensor_data['distance'] = distance
    
    def set_wheel_data(self, left_motor, right_motor):
        """Update wheel data from motor encoders."""
        # Get delta distances from motor encoders
        delta_left = left_motor.get_delta_distance()
        delta_right = right_motor.get_delta_distance()
        
        # Calculate forward motion (average of wheels)
        forward_motion = (delta_left + delta_right) / 2
        
        # Update wheel data
        self.wheel_data = {
            'left_distance': left_motor.get_distance(),
            'right_distance': right_motor.get_distance(),
            'delta_left': delta_left,
            'delta_right': delta_right,
            'forward_motion': forward_motion
        }
    
    def get_sensor_data(self):
        """Return the latest sensor readings."""
        return self.sensor_data
    
    def calibrate_compass(self):
        """Calibrate the digital compass."""
        self.compass.calibrate()
    
    def cleanup(self):
        """Clean up all sensor resources."""
        self.laser_range.cleanup()
        self.compass.cleanup() 