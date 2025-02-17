from controller import Robot
from navigation.navigator import Navigator
from motion.motionController import MotionController
from sensors.sensorManager import SensorManager
from utils.state import State
from path_planner import generate_cleaning_path
from config.robot_config import (
    ROBOT_PARAMS, 
    MOTION_PARAMS, 
    NAVIGATION_PARAMS, 
    CLEANING_AREAS,
    SIMULATION_PARAMS
)

class RobotController:
    def __init__(self, robot, timestep):
        """Initialize the robot controller with all its subsystems"""
        self.robot = robot
        self.timestep = timestep
        
        # Initialize subsystems with configuration parameters
        self.sensor_manager = SensorManager(robot, timestep)
        self.state = State(robot, timestep)
        self.motion_controller = MotionController(robot, timestep)
        self.navigator = Navigator(robot, timestep)
        
        # Configure motion and navigation parameters after initialization
        self.motion_controller.set_max_speeds(
            MOTION_PARAMS['max_linear_speed'],
            MOTION_PARAMS['max_angular_speed']
        )
        self.navigator.set_thresholds(
            NAVIGATION_PARAMS['waypoint_threshold'],
            NAVIGATION_PARAMS['heading_threshold']
        )
        
    def generate_cleaning_path(self, points):
        """Generate a cleaning path from a set of boundary points"""
        return generate_cleaning_path(points, 
                                   ROBOT_PARAMS['surface_cleaner_diameter'],
                                   ROBOT_PARAMS['path_overlap'],
                                   ROBOT_PARAMS['edge_buffer'])

    def setup(self):
        """Perform any necessary setup operations"""
        # Get boundary points from config and set the cleaning path
        boundary_points = CLEANING_AREAS['rectangle']  # Can easily switch to 'L_shape' or other patterns
        cleaning_path = self.generate_cleaning_path(boundary_points)
        self.navigator.set_path(cleaning_path)
        
    def step(self):
        """Main control loop - called every timestep"""
        # Update sensor readings and state
        self.sensor_manager.update()
        self.state.update(self.sensor_manager.get_sensor_data())
        
        # Get and execute navigation commands
        nav_command = self.navigator.get_next_command(self.state.get_position())
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
