from controller import Robot
from driveSystem import DriveSystem
from sensors.digitalCompass import DigitalCompass
from state import State

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
state = State(robot, timestep)

# Then create drive system with state
drive = DriveSystem(robot, sensor, timestep, state)

# Main control loop
while robot.step(timestep) != -1:
  
    print_position(state)
    delay(1)
    # Example movement sequence - one step at a time
    drive.drive_distance(8.0, 0.5)  # Drive 0.5 meters
    drive.stop()
    print_position(state)
    delay(1)

    drive.turn(8.0, 90)
    drive.stop()
    print_position(state)
    delay(1)

    

    
    
    