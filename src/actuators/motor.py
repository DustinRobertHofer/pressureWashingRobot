from time import sleep
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice
from config.robotConfig import MOTOR_PINS, MOTION_PARAMS, ROBOT_PARAMS

class PiMotor:
    """Motor controller for Raspberry Pi using gpiozero."""
    
    def __init__(self, motor_name):
        """Initialize motor with gpiozero devices."""
        self.motor_name = motor_name
        self.pin_config = MOTOR_PINS[motor_name]
        
        # Set up GPIO pins using gpiozero
        # self.pwm_pin = self.pin_config['pwm_pin']
        self.dir1_pin = self.pin_config['dir1_pin']
        # self.dir2_pin = self.pin_config[dir_pin]
        self.step1_pin = self.pin_config['step1_pin']
        # self.step2_pin = self.pin_config[step_pin]
        # self.encoder_a = self.pin_config['encoder_a']
        # self.encoder_b = self.pin_config['encoder_b']
        
        # Configure GPIO using gpiozero
        self.pwm = PWMOutputDevice(
            pin=self.pwm_pin,
            frequency=MOTION_PARAMS['pwm_frequency'],
            initial_value=0
        )
        self.dir1 = DigitalOutputDevice(self.dir1_pin, initial_value=False)
        #self.dir2 = DigitalOutputDevice(self.dir2_pin, initial_value=False)
        
        # Encoder pins
        # self.encoder_a_device = DigitalInputDevice(self.encoder_a, pull_up=True)
        # self.encoder_b_device = DigitalInputDevice(self.encoder_b, pull_up=True)
        
        # Set up encoder event detection
        # self.encoder_a_device.when_activated = self._encoder_callback
        # self.encoder_a_device.when_deactivated = self._encoder_callback
        
        # Initialize encoder state
        # self.encoder_count = 0
        # self.distance_traveled = 0
        # self.prev_encoder_count = 0
        
        # Physical parameters
        self.wheel_radius = ROBOT_PARAMS['wheel_radius']
        self.ticks_per_rev = ROBOT_PARAMS['encoder_ticks_per_rev']
        
        # Motor state
        self.current_velocity = 0
        self.current_duty_cycle = 0
        
    def _encoder_callback(self):
        """Called when encoder signal changes state."""
        # Simple encoder counting (could be improved for better accuracy)
        if self.encoder_b_device.value == self.encoder_a_device.value:
            self.encoder_count += 1  # Forward rotation
        else:
            self.encoder_count -= 1  # Backward rotation
            
        # Calculate distance based on encoder count
        self.distance_traveled = self._ticks_to_distance(self.encoder_count)
    
    def _ticks_to_distance(self, ticks):
        """Convert encoder ticks to distance in meters."""
        revolutions = ticks / self.ticks_per_rev
        return revolutions * 2 * 3.14159 * self.wheel_radius
    
    def set_velocity(self, velocity):
        """Set the velocity of the motor in rad/s."""
        self.current_velocity = velocity
        
        # Determine direction based on velocity sign
        if velocity >= 0:
            self._set_direction(True)  # Forward
        else:
            self._set_direction(False)  # Backward
        
        # Convert velocity to PWM duty cycle (simple linear mapping)
        abs_velocity = abs(velocity)
        max_velocity = MOTION_PARAMS['max_linear_speed'] / self.wheel_radius
        
        if abs_velocity == 0:
            duty_cycle = 0
        else:
            # Convert to percentage, apply minimum duty cycle if motor is turning
            percentage = min(1.0, abs_velocity / max_velocity)
            min_duty = MOTION_PARAMS['min_duty_cycle']
            max_duty = MOTION_PARAMS['max_duty_cycle']
            # gpiozero uses 0-1 value range instead of 0-100 for duty cycle
            duty_cycle = (min_duty + percentage * (max_duty - min_duty)) / 100.0
        
        self.current_duty_cycle = duty_cycle * 100.0  # Store as percentage for compatibility
        self.pwm.value = duty_cycle
    
    def _set_direction(self, forward):
        """Set motor direction: True for forward, False for backward."""
        if forward:
            self.dir1.on()
            self.dir2.off()
        else:
            self.dir1.off()
            self.dir2.on()
    
    def stop(self):
        """Stop the motor."""
        self.pwm.value = 0
        self.current_velocity = 0
        self.current_duty_cycle = 0
    
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
        """Get the current velocity of the motor."""
        return self.current_velocity
    
    def cleanup(self):
        """Clean up GPIO resources."""
        self.stop()
        # No need to manually clean up gpiozero resources
        # They'll be closed when Device.close_all() is called or on program exit 