import RPi.GPIO as GPIO
import time
from config.robotConfig import MOTOR_PINS, MOTION_PARAMS, ROBOT_PARAMS

class StepperMotor:
    """Stepper motor controller for Raspberry Pi using RPi.GPIO."""
    
    def __init__(self, motor_name):
        """Initialize stepper motor with RPi.GPIO."""
        self.motor_name = motor_name
        self.pin_config = MOTOR_PINS[motor_name]
        
        # Set up GPIO pins
        self.dir_pin = self.pin_config['dir1_pin']
        self.step_pin = self.pin_config['step1_pin']
        
        # Configure GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        
        # Physical parameters
        self.wheel_radius = ROBOT_PARAMS['wheel_radius']
        self.steps_per_rev = 400  # 400 steps = 1 revolution for 0.9 degree stepper
        
        # Motor state
        self.current_velocity = 0
        self.current_duty_cycle = 0
        self.step_count = 0
        self.distance_traveled = 0
        self.prev_step_count = 0
        
        # Default step delay (can be adjusted for speed control)
        self.step_delay = 0.002  # seconds
        
    def set_velocity(self, velocity):
        """Set the velocity of the motor in rad/s."""
        self.current_velocity = velocity
        
        # Determine direction based on velocity sign
        if velocity >= 0:
            self._set_direction(True)  # Forward
        else:
            self._set_direction(False)  # Backward
        
        # Convert velocity to step frequency
        abs_velocity = abs(velocity)
        max_velocity = MOTION_PARAMS['max_linear_speed'] / self.wheel_radius
        
        if abs_velocity == 0:
            self.step_delay = float('inf')  # No stepping
        else:
            # Convert to step delay (inverse of frequency)
            percentage = min(1.0, abs_velocity / max_velocity)
            min_delay = 0.001  # Fastest speed (1ms delay)
            max_delay = 0.01   # Slowest speed (10ms delay)
            self.step_delay = max_delay - percentage * (max_delay - min_delay)
        
        # Store duty cycle for compatibility with motor.py
        if self.step_delay == float('inf'):
            self.current_duty_cycle = 0
        else:
            # Convert to percentage (0-100)
            min_duty = MOTION_PARAMS['min_duty_cycle']
            max_duty = MOTION_PARAMS['max_duty_cycle']
            self.current_duty_cycle = min_duty + (1.0 - self.step_delay / max_delay) * (max_duty - min_duty)
    
    def _set_direction(self, forward):
        """Set motor direction: True for forward, False for backward."""
        GPIO.output(self.dir_pin, not forward)  # Inverted logic based on Motor_Test_2.py
    
    def step(self, steps=1):
        """Step the motor a specified number of steps."""
        for _ in range(steps):
            GPIO.output(self.step_pin, True)
            time.sleep(self.step_delay)
            GPIO.output(self.step_pin, False)
            time.sleep(self.step_delay)
            
            # Update step count and distance
            if self.current_velocity >= 0:
                self.step_count += 1
            else:
                self.step_count -= 1
            
            self.distance_traveled = self._steps_to_distance(self.step_count)
    
    def _steps_to_distance(self, steps):
        """Convert steps to distance in meters."""
        revolutions = steps / self.steps_per_rev
        return revolutions * 2 * 3.14159 * self.wheel_radius
    
    def stop(self):
        """Stop the motor."""
        self.current_velocity = 0
        self.current_duty_cycle = 0
        self.step_delay = float('inf')
    
    def get_encoder_count(self):
        """Get current step count (compatibility with motor.py)."""
        return self.step_count
    
    def get_distance(self):
        """Get distance traveled in meters."""
        return self.distance_traveled
    
    def get_delta_distance(self):
        """Get change in distance since last call (for odometry)."""
        current_count = self.step_count
        delta_count = current_count - self.prev_step_count
        self.prev_step_count = current_count
        return self._steps_to_distance(delta_count)
    
    def get_velocity(self):
        """Get the current velocity of the motor."""
        return self.current_velocity
    
    def cleanup(self):
        """Clean up GPIO resources."""
        self.stop()
        GPIO.cleanup()
