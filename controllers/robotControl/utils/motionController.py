from config.robot_config import ROBOT_PARAMS, MOTION_PARAMS
from actuators.motor import Motor


class MotionController:
    """Controls robot movement by translating motion commands to motor velocities."""
    
    def __init__(self, robot, timestep):
        """Initialize motion control system with motors and configuration parameters.
        
        Args:
            robot: The robot instance to control
            timestep: Simulation timestep
        """
        # Initialize motors
        self.left_motor = Motor(robot, ROBOT_PARAMS['left_motor_name'])
        self.right_motor = Motor(robot, ROBOT_PARAMS['right_motor_name'])
        
        # Load physical parameters from config
        self.wheel_radius = ROBOT_PARAMS['wheel_radius']
        self.wheel_base = ROBOT_PARAMS['wheel_distance']
        
        # Speed limits
        self.max_linear_speed = MOTION_PARAMS['max_linear_speed']
        self.max_angular_speed = MOTION_PARAMS['max_angular_speed']
        
    def execute_command(self, command):
        """Execute a movement command based on its type.
        
        Args:
            command: Dictionary containing command type and parameters
        """
        if not self._motors_available():
            return
            
        if command is None:
            self.stop()
            print("No command to execute, stopping robot")
            return
            
        if command['type'] == 'move':
            self.set_velocity(command['linear_velocity'], command['angular_velocity'])
        elif command['type'] == 'stop':
            self.stop()
            
    def set_velocity(self, linear_velocity, angular_velocity):
        """Set robot linear and angular velocity.
        
        Args:
            linear_velocity: Forward/backward velocity in m/s
            angular_velocity: Rotational velocity in rad/s
        """
        if not self._motors_available():
            return
        
        # Apply velocity limits
        linear_velocity = self._limit_value(linear_velocity, -self.max_linear_speed, self.max_linear_speed)
        angular_velocity = self._limit_value(angular_velocity, -self.max_angular_speed, self.max_angular_speed)
            
        # Convert to wheel velocities using differential drive kinematics
        left_speed = (linear_velocity + angular_velocity * self.wheel_base / 2) / self.wheel_radius
        right_speed = (linear_velocity - angular_velocity * self.wheel_base / 2) / self.wheel_radius
        
        # Set motor velocities
        self.left_motor.set_velocity(left_speed)
        self.right_motor.set_velocity(right_speed)
        
    def stop(self):
        """Stop all robot motion by setting wheel velocities to zero."""
        if not self._motors_available():
            return
        self.left_motor.set_velocity(0.0)
        self.right_motor.set_velocity(0.0)
    
    def _motors_available(self):
        """Check if motors are available for control.
        
        Returns:
            bool: True if motors are available, False otherwise
        """
        if not (self.left_motor and self.right_motor):
            print("WARNING: Could not find motors. Robot will not be able to move.")
            return False
        return True
        
    def _limit_value(self, value, min_value, max_value):
        """Limit a value to specified range.
        
        Args:
            value: Value to limit
            min_value: Minimum allowed value
            max_value: Maximum allowed value
            
        Returns:
            float: Value constrained to the specified range
        """
        return max(min_value, min(value, max_value))
