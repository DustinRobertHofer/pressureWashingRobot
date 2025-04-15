from controller import Robot
from navigation.navigator import Navigator
from utils.motionController import MotionController
from utils.sensorManager import SensorManager
from utils.state import State
from utils.path_planner import generate_cleaning_path
from utils.path_visualizer import PathVisualizer

from config.robot_config import (
    SIMULATION_PARAMS,
    NAVIGATION_PARAMS,
    VISUALIZATION_PARAMS,
    CLEANING_AREAS  # Import the cleaning areas from config
)


class RobotController:
    def __init__(self, robot, timestep):
        """Initialize the robot controller with all its subsystems"""
        self.robot = robot
        self.timestep = timestep
        
        # Initialize core components
        self._init_subsystems()
        self._init_visualization()
        
        # Initialize state variables
        self.cleaning_path = None
        self.boundary_points = None
        self.time_counter = 0
        
        # Set the cleaning area directly from robot_config
        self.selected_area = 'rectangle'  # Default to rectangle area
        self.boundary_points = CLEANING_AREAS[self.selected_area]

    def _init_subsystems(self):
        """Initialize robot subsystems"""
        self.sensor_manager = SensorManager(self.robot, self.timestep)
        self.state = State(self.robot, self.timestep)
        self.motion_controller = MotionController(self.robot, self.timestep)
        self.navigator = Navigator(self.robot, self.timestep)

    def _init_visualization(self):
        """Initialize visualization if enabled"""
        self.visualizer = None
        if VISUALIZATION_PARAMS['enable']:
            self.visualizer = PathVisualizer(
                self.robot,
                width=VISUALIZATION_PARAMS['width'],
                height=VISUALIZATION_PARAMS['height'],
                window_name=VISUALIZATION_PARAMS['window_name']
            )

    def _setup_visualization(self):
        """Setup visualization parameters"""
        if not self.visualizer:
            return
            
        # Calculate bounds of the cleaning area
        min_x = min(point['x'] for point in self.boundary_points)
        max_x = max(point['x'] for point in self.boundary_points)
        min_y = min(point['y'] for point in self.boundary_points)
        max_y = max(point['y'] for point in self.boundary_points)
        
        # Set visualization scale
        self.visualizer.set_scale(min_x, max_x, min_y, max_y)
        
        # Initial visualization
        current_state = self.state.get_position()
        self._update_visualization(current_state)

    def _update_visualization(self, current_state):
        """Update visualization with current state"""
        if not self.visualizer:
            return
            
        self.visualizer.update_visualization(
            current_state['x'],
            current_state['y'],
            current_state['theta'],
            self.boundary_points,
            self.cleaning_path
        )

    def setup(self):
        """Perform necessary setup operations before starting cleaning"""
        # Use the boundary points directly from config and setup path
        print(f"Using cleaning area: {self.selected_area}")
        print(f"Boundary points: {self.boundary_points}")
        
        self.cleaning_path = generate_cleaning_path(self.boundary_points)
        self.navigator.set_path(self.cleaning_path)
        
        # Setup visualization
        self._setup_visualization()
        
        # No need to wait for start command, immediately begin
        print("Starting cleaning operation automatically...")

    def _check_obstacles(self, sensor_data):
        """Check for obstacles using sensor data and handle if needed"""
        if 'distance' not in sensor_data:
            print("No distance sensor data available")
            return False
            
        # Stop if obstacle is too close
        if sensor_data['distance'] < NAVIGATION_PARAMS['safe_distance']:
            print(f"OBSTACLE DETECTED at {sensor_data['distance']:.2f}m! Stopping robot.")
            self.motion_controller.stop()
            return True
        
        return False

    def _handle_visualization_update(self, current_state):
        """Handle periodic visualization updates"""
        if not (self.visualizer and VISUALIZATION_PARAMS['enable']):
            return
            
        # Only update visualization periodically to save computational resources
        self.time_counter += 1
        if self.time_counter >= VISUALIZATION_PARAMS['update_interval']:
            self.time_counter = 0
            self._update_visualization(current_state)

    def step(self):
        """Main control loop - called every timestep"""
        # Update sensor readings and state
        self.sensor_manager.update()
        sensor_data = self.sensor_manager.get_sensor_data()
        self.state.update(sensor_data)
        
        # Get current state
        current_state = self.state.get_position()
        
        # Check for obstacles
        if self._check_obstacles(sensor_data):
            return
        
        # Get and execute navigation commands
        nav_command = self.navigator.get_next_command(current_state)
        self.motion_controller.execute_command(nav_command)
        
        # Update visualization if enabled
        self._handle_visualization_update(current_state)
        
        # Check if cleaning is complete
        if nav_command['type'] == 'stop':
            return self._handle_completion(current_state)

    def _handle_completion(self, current_state):
        """Handle completion of the cleaning operation"""
        print("Path completed. Cleaning operation finished!")
        return True

    def cleanup(self):
        """Perform any necessary cleanup operations"""
        self.motion_controller.stop()
        
        # No UI process to terminate anymore


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
        result = controller.step()
        if result:
            break
        
    controller.cleanup()


if __name__ == "__main__":
    main()
