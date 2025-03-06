from config.robot_config import ROBOT_PARAMS, MOTION_PARAMS
from actuators.motor import Motor

class MotionController:
    def __init__(self, robot, timestep):
        """Initialize motion control system"""
        self.left_motor = Motor(robot, ROBOT_PARAMS['left_motor_name'])
        self.right_motor = Motor(robot, ROBOT_PARAMS['right_motor_name'])
        
        # Robot physical parameters from config
        self.WHEEL_RADIUS = ROBOT_PARAMS['wheel_radius']
        self.WHEEL_BASE = ROBOT_PARAMS['wheel_distance']
        
        # Initialize speed limits from motion parameters
        self.max_linear_speed = MOTION_PARAMS['max_linear_speed']
        self.max_angular_speed = MOTION_PARAMS['max_angular_speed']
        
    def execute_command(self, command):
        """Execute a movement command"""
        if not (self.left_motor and self.right_motor):
            print("WARNING: Could not find motors. Robot will not be able to move.")
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
        """Set robot linear and angular velocity"""
        if not (self.left_motor and self.right_motor):
            print("WARNING: Could not find motors. Robot will not be able to move.")
            return
        
        # Limit velocities to maximum values            
        if angular_velocity > self.max_angular_speed:
            angular_velocity = self.max_angular_speed
        elif angular_velocity < -self.max_angular_speed:
            angular_velocity = -self.max_angular_speed
        
        if linear_velocity > self.max_linear_speed:
            linear_velocity = self.max_linear_speed
        elif linear_velocity < -self.max_linear_speed:
            linear_velocity = -self.max_linear_speed
            
        # Convert to wheel velocities
        left_speed = (linear_velocity + angular_velocity * self.WHEEL_BASE / 2) / self.WHEEL_RADIUS
        right_speed = (linear_velocity - angular_velocity * self.WHEEL_BASE / 2) / self.WHEEL_RADIUS
        
        # Set motor velocities
        self.left_motor.set_velocity(left_speed)
        self.right_motor.set_velocity(right_speed)
        
    def stop(self):
        """Stop all motion"""
        if not (self.left_motor and self.right_motor):
            return
        self.left_motor.set_velocity(0.0)
        self.right_motor.set_velocity(0.0)
