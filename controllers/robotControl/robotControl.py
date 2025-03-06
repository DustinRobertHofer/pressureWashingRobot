from controller import Robot
from navigation.navigator import Navigator
from utils.motionController import MotionController
from utils.sensorManager import SensorManager
from utils.state import State
from utils.path_planner import generate_cleaning_path
from config.robot_config import (
    CLEANING_AREAS,
    SIMULATION_PARAMS,
    NAVIGATION_PARAMS
)

class RobotController:
    def __init__(self, robot, timestep):
        """Initialize the robot controller with all its subsystems"""
        self.robot = robot
        self.timestep = timestep
        
        # Initialize subsystems
        self.sensor_manager = SensorManager(robot, timestep)
        self.state = State(robot, timestep)
        self.motion_controller = MotionController(robot, timestep)
        self.navigator = Navigator(robot, timestep)


    def setup(self):
        """Perform any necessary setup operations"""
        # Get boundary points from config and set the cleaning path
        boundary_points = CLEANING_AREAS['rectangle']  # Can easily switch to 'L_shape' or other patterns
        cleaning_path = generate_cleaning_path(boundary_points)
        self.navigator.set_path(cleaning_path)
        
    def step(self):
        """Main control loop - called every timestep"""
        # Update sensor readings and state
        self.sensor_manager.update()
        self.state.update(self.sensor_manager.get_sensor_data())
        
        # Get current state
        current_state = self.state.get_position()
        
        # Check for obstacles using distance sensor
        sensor_data = self.sensor_manager.get_sensor_data()
        if 'distance' in sensor_data:
            # Log distance reading for debugging
            print(f"Distance sensor reading: {sensor_data['distance']:.2f}m")
            
            # Stop if obstacle is too close
            if sensor_data['distance'] < NAVIGATION_PARAMS['safe_distance']:
                print(f"OBSTACLE DETECTED at {sensor_data['distance']:.2f}m! Stopping robot.")
                self.motion_controller.stop()
                return
        else:
            print("No distance sensor data available")
        
        # Get and execute navigation commands
        nav_command = self.navigator.get_next_command(current_state)
            
        self.motion_controller.execute_command(nav_command)
        
        # If we've stopped, exit the simulation
        if nav_command['type'] == 'stop':
            print("Path completed. Cleaning operation finished.")
            self.robot.step(self.timestep)  # One final step to ensure everything is updated
            return -1  # Signal to main loop to exit
        
    def cleanup(self):
        """Perform any necessary cleanup operations"""
        self.motion_controller.stop()
        
def main():
    """Main function to be called by Webots"""
    # Initialize the Robot with configured timestep
    robot = Robot()
    timestep = SIMULATION_PARAMS['basic_time_step']
    
    # Create and setup the controller
    controller = RobotController(robot, timestep)
    controller.setup()
    
    # Main control loop
    while robot.step(timestep) != -1:
        if controller.step() == -1:
            break
        
    controller.cleanup()

if __name__ == "__main__":
    main()
