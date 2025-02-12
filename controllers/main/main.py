from controller import Robot
from driveSystem import DriveSystem
from sensors.digitalCompass import DigitalCompass
from sensors.laserRange import LaserRange
from state import State
from navigation import Navigator
from obstacleAvoidance import ObstacleAvoider

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

def delay(seconds):
    start = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime() - start >= seconds:
            break

def print_position(state):
    state.update()
    position = state.get_position()
    print(f"Position: ({position['x']:.2f}, {position['y']:.2f}), " +
          f"Heading: {position['theta_degrees']:.1f}°")

# Create sensor systems
sensor = DigitalCompass(robot, timestep)
laser_range = LaserRange(robot, timestep)
state = State(robot, timestep)



# Create obstacle avoider
obstacle_avoider = ObstacleAvoider(state, laser_range)

# Create drive system with state
drive = DriveSystem(robot, obstacle_avoider, sensor, timestep, state)

# Create navigation system
navigator = Navigator(drive, timestep)

# Main control loop
while robot.step(timestep) != -1:
    # Update the state
    state.update()
    current_position = state.get_position()
    
    # Set up the path if not already set
    if not navigator.current_goal:
        navigator.set_big_square_test()  # Use the 5m x 5m square test
    
    # Follow the path
    if not navigator.follow_path(current_position):
        print("Path following completed")
        break
    
    # Print current position for debugging
    print_position(state)

    



    
    
    