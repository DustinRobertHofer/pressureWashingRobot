class MotionController:
    def __init__(self, robot, timestep):
        """Initialize motion control system"""
        # Get motor devices with correct names
        self.left_motor = robot.getDevice('left_wheel')
        self.right_motor = robot.getDevice('right_wheel')
        
        if not (self.left_motor and self.right_motor):
            print("WARNING: Could not find motors. Robot will not be able to move.")
            return
            
        print("Successfully initialized motors: left_wheel, right_wheel")
            
        # Enable position sensors
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Initialize velocities
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Robot physical parameters
        self.MAX_SPEED = 10.0  # Maximum motor speed
        self.WHEEL_RADIUS = 0.08  # Wheel radius in meters
        self.WHEEL_BASE = 0.18122 * 2 + 0.04  # Distance between wheels in meters
        
    def execute_command(self, command):
        """Execute a movement command"""
        if not (self.left_motor and self.right_motor):
            print("WARNING: Motors not available. Cannot execute movement command.")
            return
            
        if command is None:
            self.stop()
            return
            
        command_type = command.get('type', '')
        
        if command_type == 'move':
            self.set_velocity(command['linear_velocity'], command['angular_velocity'])
        elif command_type == 'stop':
            self.stop()
            
    def set_velocity(self, linear_velocity, angular_velocity):
        """Set robot linear and angular velocity"""
        if not (self.left_motor and self.right_motor):
            return
            
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