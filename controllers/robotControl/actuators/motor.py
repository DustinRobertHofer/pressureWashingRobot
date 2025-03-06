from config.robot_config import ROBOT_PARAMS

class Motor:
    def __init__(self, robot, device_name):
        """Initialize motor in Webots"""
        self.motor = robot.getDevice(device_name)
        
        if not self.motor:
            print(f"WARNING: Could not find motor: {device_name}")
            return
            
        # Initialize velocities
        self.motor.setPosition(float('inf'))
        self.motor.setVelocity(0.0)

    def set_velocity(self, velocity):
        """Set the velocity of the motor"""
        self.motor.setVelocity(velocity)

    def get_velocity(self):
        """Get the velocity of the motor"""
        return self.motor.getVelocity()
    
