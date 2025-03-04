from controller import Robot
from navigation.navigator import Navigator
from motion.motionController import MotionController
from sensors.sensorManager import SensorManager
from utils.state import State
from utils.safety import SafetyController
from path_planner import generate_cleaning_path, CleaningPathGenerator
from slam.slam_manager import SLAMManager
from config.robot_config import (
    ROBOT_PARAMS, 
    MOTION_PARAMS, 
    NAVIGATION_PARAMS, 
    CLEANING_AREAS,
    SIMULATION_PARAMS
)
import math

class RobotController:
    def __init__(self, robot, timestep):
        """Initialize the robot controller with all its subsystems"""
        self.robot = robot
        self.timestep = timestep
        
        # Check if the laser sensor exists in the robot
        self.check_laser_sensor_exists()
        
        # Initialize subsystems with configuration parameters
        self.sensor_manager = SensorManager(robot, timestep)
        self.state = State(robot, timestep)
        self.motion_controller = MotionController(robot, timestep)
        self.navigator = Navigator(robot, timestep)
        
        # Initialize SLAM manager
        self.slam_manager = SLAMManager(self.sensor_manager, self.state)
        
        # Configure motion and navigation parameters after initialization
        self.motion_controller.set_max_speeds(
            MOTION_PARAMS['max_linear_speed'],
            MOTION_PARAMS['max_angular_speed']
        )
        self.navigator.set_thresholds(
            NAVIGATION_PARAMS['waypoint_threshold'],
            NAVIGATION_PARAMS['heading_threshold']
        )
        
        # Added flags for SLAM operation
        self.use_slam = False
        self.mapping_complete = False
        
        # For auto-display of map
        self.map_display_timer = 0
        self.map_display_interval = 5000  # Display map every 5000 ms (5 seconds)
        
        # For exploration mode
        self.exploration_active = False
        self.random_turn_timer = 0
        self.random_turn_interval = 5000  # Change direction every 5 seconds
        self.obstacle_detected = False
        self.turn_direction = 1  # 1 for right, -1 for left
        
        # Initialize the UI for map controls
        self.setup_ui()
        
    def check_laser_sensor_exists(self):
        """Check if the laser/LiDAR sensor exists in the robot model"""
        try:
            laser = self.robot.getDevice('laser')
            if laser:
                print("✓ Laser sensor 'laser' exists in robot model")
                return True
            else:
                print("❌ ERROR: Laser sensor 'laser' not found in robot model")
                print("SLAM functionality will be disabled")
                return False
        except Exception as e:
            print(f"❌ ERROR: Unable to check for laser sensor: {e}")
            print("SLAM functionality will be disabled")
            return False

    def generate_cleaning_path(self, points):
        """Generate a cleaning path from a list of boundary points"""
        # Use the cleaning path generator
        return self.cleaning_path_generator.generate_path(points)

    def setup(self):
        """Initialize robot configuration and systems"""
        # Initialize state variables
        self.velocity = 0.0
        self.rotation = 0.0
        self.exit_requested = False
        
        # Initialize SLAM and cleaning path generator
        self.slam_manager = SLAMManager(self.sensor_manager, self.state)
        self.mapping_complete = False
        self.use_slam = False  # Start with SLAM disabled by default
        
        # Initialize cleaning path generator
        self.cleaning_path_generator = CleaningPathGenerator()
        
        # Update sensors to get initial readings
        self.sensor_manager.update()
        
        # Check if LiDAR is present and enable SLAM if it is
        if self.check_laser_sensor_exists():
            print("LiDAR sensor found - enabling SLAM")
            self.use_slam = True
            
            # Check if we already have a map
            if self.mapping_complete:
                # Load previously created map if available
                if self.slam_manager.load_map():
                    print("Loaded existing SLAM map")
                    # Generate cleaning path from the map
                    boundary_points = self.slam_manager.get_cleaning_boundary()
                    if boundary_points:
                        cleaning_path = self.generate_cleaning_path(boundary_points)
                        self.navigator.set_path(cleaning_path)
                    else:
                        print("No valid cleaning boundary found in map")
                        # Fallback to predefined area
                        boundary_points = CLEANING_AREAS['rectangle']
                        cleaning_path = self.generate_cleaning_path(boundary_points)
                        self.navigator.set_path(cleaning_path)
                else:
                    print("No saved map found, starting new mapping with LiDAR...")
                    self.slam_manager.start_mapping()
            else:
                # Start SLAM mapping process
                print("Starting new SLAM mapping with LiDAR...")
                self.slam_manager.start_mapping()
        else:
            print("LiDAR sensor not found - SLAM will remain disabled")
            # Use predefined cleaning area from config
            boundary_points = CLEANING_AREAS['rectangle']  # Can easily switch to 'L_shape' or other patterns
            cleaning_path = self.generate_cleaning_path(boundary_points)
            self.navigator.set_path(cleaning_path)
        
    def setup_ui(self):
        """Set up a simple UI in Webots for map controls"""
        try:
            # Try to create a display UI
            self.window_width = 120
            self.window_height = 40
            self.display = self.robot.getDisplay("display")
            
            # If display doesn't exist, create it
            if not self.display:
                self.display = self.robot.createDisplay("display", self.window_width, self.window_height)
            
            # Configure the display
            if self.display:
                # Clear to white
                self.display.setColor(0xFFFFFF)
                self.display.fillRectangle(0, 0, self.window_width, self.window_height)
                
                # Draw "Display Map" button
                self.display.setColor(0x0000FF)  # Blue
                self.display.fillRectangle(10, 10, 100, 30)
                
                # Add text
                self.display.setColor(0xFFFFFF)  # White text
                self.display.drawText("Display Map", 25, 18)
                
                print("UI setup complete - Use the blue button to display the map")
            else:
                print("Warning: Could not create Webots display for UI")
        except Exception as e:
            print(f"Warning: Could not set up UI: {e}")
            self.display = None

    def toggle_exploration(self, active=True):
        """Toggle exploration mode on/off"""
        self.exploration_active = active
        
        if active:
            print("Exploration mode activated - robot will drive autonomously")
            # Reset exploration parameters
            self.random_turn_timer = 0
            self.obstacle_detected = False
            self.turn_direction = 0  # Will be set on first obstacle
        else:
            print("Exploration mode deactivated")
            self.motion_controller.stop()
            
    def step(self):
        """Perform one control step"""
        # Get current sensor readings and update state
        self.sensor_manager.update()
        sensor_data = self.sensor_manager.get_sensor_data()
        self.state.update(sensor_data)
        
        # Debug LiDAR data if available
        if 'laser' in sensor_data:
            ranges = sensor_data['laser'].get('ranges', [])
            if ranges:
                min_valid = float('inf')
                max_valid = 0
                inf_count = 0
                neg_count = 0
                
                for r in ranges:
                    if math.isinf(r):
                        inf_count += 1
                    elif r < 0:
                        neg_count += 1
                    else:
                        min_valid = min(min_valid, r)
                        max_valid = max(max_valid, r)
                
                if min_valid == float('inf'):
                    min_valid = "N/A"
                
                print(f"LiDAR stats: min={min_valid}, max={max_valid}, inf={inf_count}, neg={neg_count}")
        
        # Update SLAM if active and if we have laser sensors
        # Only attempt to use SLAM if we have laser sensors available
        if self.use_slam and not self.mapping_complete:
            # Check if we have laser data before trying to update SLAM
            if 'laser' in sensor_data:
                self.slam_manager.update()
                print(f"Updated SLAM with LiDAR data - {len(sensor_data['laser'].get('ranges', []))} points")
                
                # Check if it's time to auto-display the map
                self.map_display_timer += self.timestep
                if self.map_display_timer >= self.map_display_interval:
                    print("Auto-displaying SLAM map...")
                    self.display_slam_map()
                    self.map_display_timer = 0
            else:
                # Laser sensor not available, disable SLAM functionality
                if self.use_slam:
                    print("Laser sensor not found. SLAM functionality disabled.")
                    self.use_slam = False
        
        # If exploration mode is active, handle autonomous navigation
        if self.exploration_active:
            return self.explore_step(sensor_data)
            
        # Otherwise, follow normal navigation commands
        # Get next navigation command
        current_position = self.state.get_position()
        nav_command = self.navigator.get_next_command(current_position)
        
        # Execute motion command with error handling
        try:
            if nav_command['type'] == 'move':
                # Match the keys used by the navigator: linear_velocity and angular_velocity
                linear_velocity = nav_command.get('linear_velocity', 0.0)
                angular_velocity = nav_command.get('angular_velocity', 0.0)
                self.motion_controller.set_velocity(linear_velocity, angular_velocity)
            elif nav_command['type'] == 'stop':
                self.motion_controller.stop()
        except KeyError as e:
            print(f"Warning: Missing key in navigation command: {e}")
            print(f"Navigation command was: {nav_command}")
            # Default to stopping if command format is incorrect
            self.motion_controller.stop()
        
        # Return command for monitoring/debugging
        return nav_command
    
    def cleanup(self):
        """Perform any cleanup operations"""
        self.motion_controller.stop()
        # Save SLAM map if we were using SLAM
        if self.use_slam and self.slam_manager.is_mapping:
            self.slam_manager.stop_mapping()
            self.slam_manager.save_map()
            print("SLAM map saved during cleanup")
    
    def toggle_slam(self, use_slam=True):
        """Toggle SLAM mode on or off"""
        # First check if we have a LiDAR sensor before enabling SLAM
        if use_slam:
            # Update sensor data to check if LiDAR is available
            self.sensor_manager.update()
            sensor_data = self.sensor_manager.get_sensor_data()
            
            if 'laser' not in sensor_data:
                print("❌ WARNING: Cannot enable SLAM - No LiDAR sensor data found")
                print("Make sure your robot has a sensor named 'laser' in the Webots world file")
                print("SLAM will be disabled")
                self.use_slam = False
                return
            else:
                # Check if we have actual LiDAR data points
                laser_data = sensor_data['laser']
                if not laser_data.get('ranges'):
                    print("❌ WARNING: LiDAR sensor found but not returning data")
                    print("SLAM will be disabled")
                    self.use_slam = False
                    return
                print(f"✓ LiDAR sensor detected with {len(laser_data.get('ranges'))} data points")
                
        # If we have a LiDAR sensor or we're disabling SLAM, proceed
        self.use_slam = use_slam
        if not use_slam and self.slam_manager.is_mapping:
            print("Stopping SLAM mapping...")
            self.slam_manager.stop_mapping()
        elif use_slam and not self.slam_manager.is_mapping and not self.mapping_complete:
            print("Starting SLAM mapping with LiDAR...")
            self.slam_manager.start_mapping()
    
    def display_slam_map(self):
        """Display the current SLAM map"""
        if self.use_slam:
            self.slam_manager.display_map()
            
    def load_slam_map(self, filename="map"):
        """Load a previously saved SLAM map"""
        if self.slam_manager.load_map(filename):
            self.mapping_complete = True
            return True
        return False

    def explore_step(self, sensor_data):
        """Perform one step in exploration mode without safety features"""
        # Default motion parameters - increase speed since we're removing safety limits
        linear_velocity = 0.5  # Faster speed without safety restrictions
        angular_velocity = 0.0
        
        # Check if we have LiDAR data, but only use it for SLAM, not for obstacle avoidance
        if 'laser' in sensor_data:
            # Instead of obstacle avoidance, just implement random movements for exploration
            self.random_turn_timer += self.timestep
            if self.random_turn_timer >= self.random_turn_interval:
                # Apply a random rotation for exploration without caring about obstacles
                import random
                angular_velocity = random.uniform(-0.5, 0.5)  # Wider range for more aggressive turning
                self.random_turn_timer = 0
        
        # Apply the calculated velocity
        self.motion_controller.set_velocity(linear_velocity, angular_velocity)
        
        # Return command for monitoring
        return {
            'type': 'explore',
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity,
            'obstacle_detected': False  # Always false since we're not checking for obstacles
        }

def main():
    """Main function to run the robot controller"""
    # Create the Robot instance
    robot = Robot()
    
    # Get the time step of the current world
    timestep = int(robot.getBasicTimeStep())
    
    # Enable keyboard for map display and other commands
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    
    # Create controller
    controller = RobotController(robot, timestep)
    
    # Initialize the controller
    controller.setup()
    
    # For keyboard tracking
    display_map_requested = False
    save_map_requested = False
    exploration_toggle_requested = False
    
    # Main control loop
    while robot.step(timestep) != -1:
        # Check for keyboard input
        key = keyboard.getKey()
        
        # M key (ASCII 77) to display map
        if key == ord('M') or key == ord('m'):
            display_map_requested = True
            print("Map display requested - Will show after this step")
        
        # S key (ASCII 83) to save map
        elif key == ord('S') or key == ord('s'):
            save_map_requested = True
            print("Map save requested - Will save after this step")
            
        # E key (ASCII 69) to toggle exploration mode
        elif key == ord('E') or key == ord('e'):
            exploration_toggle_requested = True
            print("Exploration mode toggle requested")
        
        # Run one control step
        controller.step()
        
        # Handle map display after step (avoids blocking simulation)
        if display_map_requested:
            print("Displaying SLAM map...")
            controller.display_slam_map()
            display_map_requested = False
            
        # Handle map save after step
        if save_map_requested:
            print("Saving SLAM map...")
            controller.slam_manager.save_map()
            save_map_requested = False
            
        # Handle exploration mode toggle
        if exploration_toggle_requested:
            controller.toggle_exploration(not controller.exploration_active)
            exploration_toggle_requested = False
        
        # Check if exit was requested
        if controller.exit_requested:
            break
    
    # Cleanup
    controller.cleanup()
    
    print("Controller finished")

# Run the main function when the script is executed
if __name__ == "__main__":
    main()
