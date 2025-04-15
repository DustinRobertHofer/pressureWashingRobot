from gpiozero import DigitalInputDevice
import time
from math import pi
from config.robotConfig import ROBOT_PARAMS, MOTOR_PINS

class WheelEncoder:
    """Wheel encoder implementation for Raspberry Pi."""
    
    def __init__(self, wheel_name):
        """Initialize wheel encoder with GPIO pins."""
        self.wheel_name = wheel_name
        self.pin_config = MOTOR_PINS[wheel_name]
        
        # Set up GPIO pins using gpiozero
        self.encoder_a_pin = self.pin_config['encoder_a']
        self.encoder_b_pin = self.pin_config['encoder_b']
        
        # Configure GPIO using gpiozero
        self.encoder_a = DigitalInputDevice(self.encoder_a_pin, pull_up=True)
        self.encoder_b = DigitalInputDevice(self.encoder_b_pin, pull_up=True)
        
        # Set up encoder event detection
        self.encoder_a.when_activated = self._encoder_callback
        self.encoder_a.when_deactivated = self._encoder_callback
        
        # Initialize encoder state
        self.encoder_count = 0
        self.distance_traveled = 0
        self.prev_encoder_count = 0
        self.prev_time = time.time()
        self.current_velocity = 0
        
        # Physical parameters
        self.wheel_radius = ROBOT_PARAMS['wheel_radius']
        self.ticks_per_rev = ROBOT_PARAMS['encoder_ticks_per_rev']
        
        print(f"Wheel encoder for {wheel_name} initialized")
    
    def _encoder_callback(self):
        """Called when encoder signal changes state."""
        # Simple encoder counting
        if self.encoder_b.value == self.encoder_a.value:
            self.encoder_count += 1  # Forward rotation
        else:
            self.encoder_count -= 1  # Backward rotation
            
        # Calculate distance based on encoder count
        self.distance_traveled = self._ticks_to_distance(self.encoder_count)
        
        # Update velocity
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt > 0.01:  # Only update velocity if 10ms has passed
            delta_distance = self._ticks_to_distance(self.encoder_count - self.prev_encoder_count)
            self.current_velocity = delta_distance / dt
            self.prev_encoder_count = self.encoder_count
            self.prev_time = current_time
    
    def _ticks_to_distance(self, ticks):
        """Convert encoder ticks to distance in meters."""
        revolutions = ticks / self.ticks_per_rev
        return revolutions * 2 * pi * self.wheel_radius
    
    def get_encoder_count(self):
        """Get current encoder count."""
        return self.encoder_count
    
    def get_distance(self):
        """Get distance traveled in meters."""
        return self.distance_traveled
    
    def get_delta_distance(self):
        """Get change in distance since last call (for odometry)."""
        current_count = self.encoder_count
        delta_count = current_count - self.prev_encoder_count
        self.prev_encoder_count = current_count
        return self._ticks_to_distance(delta_count)
    
    def get_velocity(self):
        """Get the current velocity in meters per second."""
        # Update velocity calculation
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt > 0.05:  # Update velocity calculation if 50ms has passed
            delta_distance = self._ticks_to_distance(self.encoder_count - self.prev_encoder_count)
            self.current_velocity = delta_distance / dt
            self.prev_encoder_count = self.encoder_count
            self.prev_time = current_time
        return self.current_velocity
    
    def reset(self):
        """Reset encoder count and distance."""
        self.encoder_count = 0
        self.prev_encoder_count = 0
        self.distance_traveled = 0
        self.current_velocity = 0
        self.prev_time = time.time()
    
    def cleanup(self):
        """Clean up GPIO resources."""
        # No need to manually clean up gpiozero resources
        # They'll be closed when Device.close_all() is called or on program exit
        pass 