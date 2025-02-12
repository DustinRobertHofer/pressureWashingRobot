from controller import Robot
from navigation.navigator import Navigator
from motion.motionController import MotionController
from sensors.sensorManager import SensorManager
from utils.state import State

class RobotController:
    def __init__(self, robot, timestep):
        """Initialize the robot controller with all its subsystems"""
        self.robot = robot
        self.timestep = timestep
        
        # Print available devices to help with debugging
        print("\nAvailable devices:")
        for i in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(i)
            print(f"Device {i}: {device.getName()} (type: {device.getNodeType()})")
        print()
        
        # Initialize subsystems
        self.sensor_manager = SensorManager(robot, timestep)
        self.state = State(robot, timestep)
        self.motion_controller = MotionController(robot, timestep)
        self.navigator = Navigator(robot, timestep)
        
    def setup(self):
        """Perform any necessary setup operations"""
        # Start the square test
        self.navigator.set_big_square_test()
        
    def step(self):
        """Main control loop - called every timestep"""
        # Update sensor readings and state
        self.sensor_manager.update()
        self.state.update(self.sensor_manager.get_sensor_data())
        
        # Get navigation commands
        nav_command = self.navigator.get_next_command(self.state.get_position())
        
        # Execute motion
        self.motion_controller.execute_command(nav_command)
        
        # Print current position for debugging
        pos = self.state.get_position()
        print(f"Position: ({pos['x']:.2f}, {pos['y']:.2f}), Heading: {pos['theta']:.2f} rad")
        
    def cleanup(self):
        """Perform any necessary cleanup operations"""
        self.motion_controller.stop()
        
def main():
    """Main function to be called by Webots"""
    # Initialize the Robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Create and setup the controller
    controller = RobotController(robot, timestep)
    controller.setup()
    
    # Main control loop
    while robot.step(timestep) != -1:
        controller.step()
        
    # Cleanup when done
    controller.cleanup()

# This is the entry point of the controller
if __name__ == "__main__":
    main()
