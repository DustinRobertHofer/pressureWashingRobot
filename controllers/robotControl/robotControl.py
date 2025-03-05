from controller import Robot
from navigation.navigator import Navigator
from motion.motionController import MotionController
from sensors.sensorManager import SensorManager
from utils.state import State
from path_planning import generate_cleaning_path
from config.robot_config import (
    ROBOT_PARAMS, 
    MOTION_PARAMS, 
    NAVIGATION_PARAMS, 
    CLEANING_AREAS,
    SIMULATION_PARAMS
)
from slam.slam_manager import SLAMManager


class RobotController:
    """
    Main robot controller class that coordinates all subsystems.
    
    This class is responsible for integrating various components like sensing,
    navigation, motion control, and SLAM for autonomous cleaning operations.
    """
    
    def __init__(self, robot, timestep):
        """
        Initialize the robot controller with all its subsystems.
        
        Args:
            robot: The Webots Robot instance
            timestep: Simulation timestep in milliseconds
        """
        self.robot = robot
        self.timestep = timestep
        
        # Initialize subsystems
        self._init_subsystems()
        
        # Configure subsystems with parameters from config
        self._configure_subsystems()
        
    def _init_subsystems(self):
        """Initialize all subsystem components."""
        self.sensor_manager = SensorManager(self.robot, self.timestep)
        self.state = State(self.robot, self.timestep)
        self.motion_controller = MotionController(self.robot, self.timestep)
        self.navigator = Navigator(self.robot, self.timestep)
        self.slam_manager = SLAMManager(self.sensor_manager, self.state)
        
    def _configure_subsystems(self):
        """Configure subsystems with parameters from configuration."""
        # Set motion parameters
        self.motion_controller.set_max_speeds(
            MOTION_PARAMS['max_linear_speed'],
            MOTION_PARAMS['max_angular_speed']
        )
        
        # Set navigation parameters
        self.navigator.set_thresholds(
            NAVIGATION_PARAMS['waypoint_threshold'],
            NAVIGATION_PARAMS['heading_threshold']
        )
        
    def generate_cleaning_path(self, boundary_points):
        """
        Generate a cleaning path from a set of boundary points.
        
        Args:
            boundary_points: List of coordinate points defining the cleaning area boundary
            
        Returns:
            List of waypoints representing the cleaning path
        """
        return generate_cleaning_path(
            boundary_points, 
            ROBOT_PARAMS['surface_cleaner_diameter'],
            ROBOT_PARAMS['path_overlap'],
            ROBOT_PARAMS['edge_buffer']
        )

    def setup(self):
        """Perform all necessary setup operations before starting the main control loop."""
        # Get boundary points from configuration
        boundary_points = CLEANING_AREAS['L_shape']  # Can be 'rectangle', 'L_shape', etc.
        
        # Initialize SLAM mapping
        self._initialize_slam(boundary_points)
        
        # Initialize navigation with cleaning path
        cleaning_path = self.generate_cleaning_path(boundary_points)
        self.navigator.set_path(cleaning_path)
        
        print(f"Robot initialized with a cleaning path of {len(cleaning_path)} waypoints")
    
    def _initialize_slam(self, boundary_points):
        """
        Initialize the SLAM system with boundary information.
        
        Args:
            boundary_points: List of coordinate points defining the cleaning area boundary
        """
        self.slam_manager.start_mapping()
        self.slam_manager.set_cleaning_boundary(boundary_points)
        
    def step(self):
        """
        Execute one control cycle.
        
        This is the main control loop that's called every timestep. It updates
        sensors, state, SLAM, and handles navigation and motion control.
        
        Returns:
            -1 if control should exit, None otherwise
        """
        # Update sensors and system state
        self._update_system_state()
        
        # Handle navigation and motion
        return self._process_navigation()
    
    def _update_system_state(self):
        """Update all sensor readings and system state information."""
        # Update sensors and state
        self.sensor_manager.update()
        self.state.update(self.sensor_manager.get_sensor_data())
        
        # Update SLAM system
        self.slam_manager.update()
        
        # Visualize SLAM data periodically
        current_time = self.robot.getTime()
        self.slam_manager.display_map_periodic(current_time, 5.0)
    
    def _process_navigation(self):
        """
        Process navigation commands and handle obstacle avoidance.
        
        Returns:
            -1 if navigation is complete, None otherwise
        """
        # Get current position from SLAM
        current_state = self.slam_manager.get_position()
        
        # Get next navigation command
        nav_command = self.navigator.get_next_command(current_state)
        
        # Handle obstacle detection
        self._handle_obstacles()
        
        # Execute motion command
        self.motion_controller.execute_command(nav_command)
        
        # Check if we've reached the end of the path
        if nav_command['type'] == 'stop':
            print("Path completed. Cleaning operation finished.")
            self.robot.step(self.timestep)  # One final step to update simulation
            return -1  # Signal to main loop to exit
        
        return None  # Continue execution
    
    def _handle_obstacles(self):
        """Handle obstacle detection and avoidance."""
        if self.state.get_position().get('obstacle_detected', False):
            obstacle_distance = self.state.get_position()['obstacle_distance']
            print(f"Obstacle detected at distance: {obstacle_distance:.2f}m")
            # Additional obstacle avoidance logic can be added here
        
    def cleanup(self):
        """Perform any necessary cleanup operations before shutting down."""
        self.motion_controller.stop()
        print("Robot control system shutdown complete")


def main():
    """
    Main entry point for the robot controller program.
    
    This function initializes the robot, creates the controller,
    and runs the main control loop.
    """
    # Initialize the Robot with configured timestep
    robot = Robot()
    timestep = SIMULATION_PARAMS['basic_time_step']
    
    # Create and setup the controller
    controller = RobotController(robot, timestep)
    controller.setup()
    
    # Main control loop
    while robot.step(timestep) != -1:
        result = controller.step()
        if result == -1:
            break
        
    controller.cleanup()


if __name__ == "__main__":
    main()