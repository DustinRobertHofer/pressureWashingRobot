from controller import Robot
from navigation.navigator import Navigator
from motion.motionController import MotionController
from sensors.sensorManager import SensorManager
from utils.state import State
from path_planner import generate_cleaning_path

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
        
    def generate_cleaning_path(self, points):
        """Generate a cleaning path from a set of boundary points
        Args:
            points: List of dictionaries with 'x' and 'y' coordinates marking the boundary
        Returns:
            List of waypoints for the robot to follow
        """
        # Convert points to numpy arrays for PathGenerator
        np_points = []
        for i, point in enumerate(points):
            if i == 0:
                # First point becomes origin (0,0)
                origin_x = point['x']
                origin_y = point['y']
                np_points.append(np.array([0, 0]))
            else:
                # Convert subsequent points relative to origin
                np_points.append(np.array([
                    point['x'] - origin_x,
                    point['y'] - origin_y
                ]))
        
        # Configure path generation parameters
        surface_cleaner_diameter = 12  # inches 
        path_overlap = 3  # inches
        edge_buffer = 8  # inches
        
        # Generate path coordinates using PathGenerator
        # Note: This will need to be modified to not show visualization
        path_coordinates = generate_path(np_points, surface_cleaner_diameter, path_overlap, edge_buffer)
        
        # Convert path coordinates back to waypoints
        waypoints = []
        for i in range(0, len(path_coordinates), 2):  # Coordinates come in pairs of x,y
            waypoints.append({
                'x': path_coordinates[i] / 12 + origin_x,  # Convert inches back to meters
                'y': path_coordinates[i+1] / 12 + origin_y
            })
        
        return waypoints

    def setup(self):
        """Perform any necessary setup operations"""
        # L-shaped boundary points (in meters)
        boundary_points = [
            {'x': 0.0, 'y': 0.0},     # Starting point
            {'x': 4.8, 'y': 0.0},     # Right edge of top
            {'x': 4.8, 'y': 3.0},     # Top-right inner corner
            {'x': 2.4, 'y': 3.0},     # Bottom-right inner corner
            {'x': 2.4, 'y': 6.3},     # Top-right outer corner
            {'x': 0.0, 'y': 6.3}      # Top-left corner
        ]
        
        # Generate and set the cleaning path
        cleaning_path = generate_cleaning_path(boundary_points)
        self.navigator.set_path(cleaning_path)
        
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
