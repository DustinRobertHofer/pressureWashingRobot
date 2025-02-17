from config.robot_config import ROBOT_PARAMS

class MotionController:
    def __init__(self, robot, timestep):
        """Initialize motion control system"""
        self.left_motor = robot.getDevice('left_wheel')
        self.right_motor = robot.getDevice('right_wheel')
        
        if not (self.left_motor and self.right_motor):
            print("WARNING: Could not find motors. Robot will not be able to move.")
            return
            
        # Enable position sensors and initialize velocities
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Robot physical parameters from config
        self.WHEEL_RADIUS = ROBOT_PARAMS['wheel_radius']
        self.WHEEL_BASE = ROBOT_PARAMS['wheel_distance']
        
        # Initialize speed limits (will be set by set_max_speeds)
        self.max_linear_speed = 0.0
        self.max_angular_speed = 0.0
        
    def execute_command(self, command):
        """Execute a movement command"""
        if not (self.left_motor and self.right_motor):
            return
            
        if command is None:
            self.stop()
            return
            
        if command['type'] == 'move':
            self.set_velocity(command['linear_velocity'], command['angular_velocity'])
        elif command['type'] == 'stop':
            self.stop()
            
    def set_velocity(self, linear_velocity, angular_velocity):
        """Set robot linear and angular velocity"""
        if not (self.left_motor and self.right_motor):
            return
            
        # Limit velocities to maximum values
        linear_velocity = min(max(-self.max_linear_speed, linear_velocity), self.max_linear_speed)
        angular_velocity = min(max(-self.max_angular_speed, angular_velocity), self.max_angular_speed)
            
        # Convert to wheel velocities
        left_speed = (linear_velocity + angular_velocity * self.WHEEL_BASE / 2) / self.WHEEL_RADIUS
        right_speed = (linear_velocity - angular_velocity * self.WHEEL_BASE / 2) / self.WHEEL_RADIUS
        
        # Set motor velocities
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        
    def stop(self):
        """Stop all motion"""
        if not (self.left_motor and self.right_motor):
            return
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_max_speeds(self, max_linear_speed, max_angular_speed):
        """Set the maximum linear and angular speeds"""
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed 