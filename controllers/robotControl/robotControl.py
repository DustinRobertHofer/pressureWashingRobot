from controller import Robot
from navigation.navigator import Navigator
from utils.motionController import MotionController
from utils.sensorManager import SensorManager
from utils.state import State
from utils.path_planner import generate_cleaning_path
from utils.path_visualizer import PathVisualizer
from utils.RobotClient import RobotClient
from utils.RobotInterface import RobotInterface
import threading
import subprocess
import os
from config.robot_config import (
    SIMULATION_PARAMS,
    NAVIGATION_PARAMS,
    VISUALIZATION_PARAMS
)


class RobotController:
    def __init__(self, robot, timestep):
        """Initialize the robot controller with all its subsystems"""
        self.robot = robot
        self.timestep = timestep
        
        # Initialize core components
        self._init_subsystems()
        self._init_ui()
        self._init_communication()
        self._init_visualization()
        
        # Initialize state variables
        self.cleaning_path = None
        self.boundary_points = None
        self.time_counter = 0

    def _init_subsystems(self):
        """Initialize robot subsystems"""
        self.sensor_manager = SensorManager(self.robot, self.timestep)
        self.state = State(self.robot, self.timestep)
        self.motion_controller = MotionController(self.robot, self.timestep)
        self.navigator = Navigator(self.robot, self.timestep)

    def _init_ui(self):
        """Initialize and launch the UI"""
        print("Starting UserInterfaceMaster.py...")
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(current_dir, '../..'))
        ui_master_path = os.path.join(project_root, 'UI', 'UserInterfaceMaster.py')
        print(f"UserInterfaceMaster.py path: {ui_master_path}")
        self.ui_process = subprocess.Popen(['python', ui_master_path])

    def _init_communication(self):
        """Initialize communication interfaces"""
        # Initialize the interface
        self.robot_interface = RobotInterface.get_instance()
        
        # Initialize and start the client
        self.client = RobotClient()
        self.client_thread = threading.Thread(target=self.client.start)
        self.client_thread.daemon = True
        self.client_thread.start()
        print("RobotClient started automatically")

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

    def get_boundary_points(self):
        """Get the current boundary points from the interface"""
        return self.robot_interface.get_boundary_points()

    def _wait_for_boundary_points(self):
        """Wait until boundary points are received from the server"""
        print("Waiting for boundary points from server...")
        while True:
            boundary_points = self.get_boundary_points()
            if boundary_points is not None:
                print("Boundary points received from server!")
                self.boundary_points = boundary_points
                break
            self.robot.step(self.timestep)

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

    def _wait_for_start_command(self):
        """Wait for start cleaning command from the client"""
        print("Waiting for start cleaning command...")
        while True:
            if self.client.get_cleaning_status():
                print("Start cleaning command received! Beginning cleaning operation.")
                break
            self.robot.step(self.timestep)

    def setup(self):
        """Perform necessary setup operations before starting cleaning"""
        # Get boundary points and setup path
        self._wait_for_boundary_points()
        self.cleaning_path = generate_cleaning_path(self.boundary_points)
        self.navigator.set_path(self.cleaning_path)
        
        # Setup visualization
        self._setup_visualization()
        
        # Wait for start command
        self._wait_for_start_command()

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
        print("Path completed. Cleaning operation finished.")
        
        # Final visualization update
        if self.visualizer:
            self._update_visualization(current_state)
            
        self.robot.step(self.timestep)  # One final step to ensure everything is updated
        return -1  # Signal to main loop to exit

    def cleanup(self):
        """Perform any necessary cleanup operations"""
        self.motion_controller.stop()
        
        # Terminate the UI process if it's still running
        if hasattr(self, 'ui_process') and self.ui_process is not None:
            print("Terminating UserInterfaceMaster.py...")
            self.ui_process.terminate()


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
        if result == -1:
            break
        
    controller.cleanup()


if __name__ == "__main__":
    main()
