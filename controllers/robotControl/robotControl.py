from controller import Robot
from navigation.navigator import Navigator
from utils.motionController import MotionController
from utils.sensorManager import SensorManager
from utils.state import State
from utils.path_planner import generate_cleaning_path
from utils.path_visualizer import PathVisualizer
import socket
import time
from config.robot_config import (
    CLEANING_AREAS,
    SIMULATION_PARAMS,
    NAVIGATION_PARAMS,
    VISUALIZATION_PARAMS
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
        
        # Initialize visualization if enabled
        self.visualizer = None
        self.cleaning_path = None
        self.boundary_points = None
        self.time_counter = 0
        
        if VISUALIZATION_PARAMS['enable']:
            self.visualizer = PathVisualizer(
                robot,
                width=VISUALIZATION_PARAMS['width'],
                height=VISUALIZATION_PARAMS['height'],
                window_name=VISUALIZATION_PARAMS['window_name']
            )

    def setup(self):
        """Perform any necessary setup operations"""

        # Get boundary points from config and set the cleaning path
        self.boundary_points = CLEANING_AREAS['L_shape']  # Can easily switch to 'L_shape' or other patterns
        self.cleaning_path = generate_cleaning_path(self.boundary_points)
        self.navigator.set_path(self.cleaning_path)
        
        # Initialize visualizer scale if available
        if self.visualizer:
            # Calculate bounds of the cleaning area
            min_x = min(point['x'] for point in self.boundary_points)
            max_x = max(point['x'] for point in self.boundary_points)
            min_y = min(point['y'] for point in self.boundary_points)
            max_y = max(point['y'] for point in self.boundary_points)
            
            # Set visualization scale
            self.visualizer.set_scale(min_x, max_x, min_y, max_y)
            
            # Initial visualization
            current_state = self.state.get_position()
            self.visualizer.update_visualization(
                current_state['x'],
                current_state['y'],
                current_state['theta'],
                self.boundary_points,
                self.cleaning_path
            )
        
    def step(self):
        """Main control loop - called every timestep"""
        # Update sensor readings and state
        self.sensor_manager.update()
        self.state.update(self.sensor_manager.get_sensor_data())
        global client_socket
        
        # Get current state
        current_state = self.state.get_position()
        
        # Check for obstacles using distance sensor
        sensor_data = self.sensor_manager.get_sensor_data()
        if 'distance' in sensor_data:
            # Log distance reading for debugging
            #print(f"Distance sensor reading: {sensor_data['distance']:.2f}m")
            data = (f"Distance sensor reading: {sensor_data['distance']:.2f}m")
            self.send_to_server(client_socket, data)
            
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
        
        # Update visualization if enabled
        if self.visualizer and VISUALIZATION_PARAMS['enable']:
            # Only update visualization periodically to save computational resources
            self.time_counter += 1
            if self.time_counter >= VISUALIZATION_PARAMS['update_interval']:
                self.time_counter = 0
                self.visualizer.update_visualization(
                    current_state['x'],
                    current_state['y'],
                    current_state['theta'],
                    self.boundary_points,
                    self.cleaning_path
                )
        
        # If we've stopped, exit the simulation
        if nav_command['type'] == 'stop':
            print("Path completed. Cleaning operation finished.")
            
            # Final visualization update
            if self.visualizer:
                self.visualizer.update_visualization(
                    current_state['x'],
                    current_state['y'],
                    current_state['theta'],
                    self.boundary_points,
                    self.cleaning_path
                )
                
            self.robot.step(self.timestep)  # One final step to ensure everything is updated
            return -1  # Signal to main loop to exit
    
    def send_to_server(self, connection, data):
        connection.send(data.encode())  
        
    def cleanup(self):
        """Perform any necessary cleanup operations"""
        self.motion_controller.stop()  

def main():
    """Main function to be called by Webots"""
    # Initialize the Robot with configured timestep
    robot = Robot()
    timestep = SIMULATION_PARAMS['basic_time_step']
    
    # Create the controller
    controller = RobotController(robot, timestep)

    host = socket.gethostname()
    port = 5000
    connectionStatus = 0

    global client_socket
    client_socket = socket.socket()
    try:
        client_socket.connect((host, port))
        message = ("Hello Server")
        client_socket.send(message.encode())
        connectionStatus = 1
    except socket.error as e:
         print(f"Socket error: {e}")
         connectionStatus = 0
    except KeyboardInterrupt:
        print("Client interrupted by user.")
        connectionStatus = 0

    # Setup the controller
    controller.setup()
    
    # Main control loop
    while robot.step(timestep) != -1:
        if connectionStatus == 1:
            data = client_socket.recv(1024).decode()
            print(f"Received from server: {data}")
            time.sleep(1)
        if controller.step() == -1:
            client_socket.close()
            connectionStatus = 0
            break
        
    controller.cleanup()

if __name__ == "__main__":
    main()
