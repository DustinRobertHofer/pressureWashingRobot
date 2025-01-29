from controller import Robot
from driveSystem import DriveSystem
from sensors.digitalCompass import DigitalCompass

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Create sensor system first
sensor = DigitalCompass(robot, timestep)

# Then create drive system
drive = DriveSystem(robot, sensor, timestep)

# Main control loop
while robot.step(timestep) != -1:
    # Example movement sequence
     drive.forward(8.0, duration=2)  # Move forward at speed 2 for 2 seconds
     drive.stop()   
     drive.turn(8.0, 90)  # Turn left 90 degrees at speed 2
     drive.stop()
     drive.forward(8.0, duration=2)
     drive.stop()
     drive.turn(8.0, -90)
     drive.stop()
     drive.forward(8.0, duration=2)
     drive.stop()
     drive.turn(8.0, 90)
     drive.stop()
     drive.forward(8.0, duration=2)
     drive.stop()
    
    