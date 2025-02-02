from controller import Robot
import math


class DriveSystem:
    def __init__(self, robot, sensor, timestep, state=None):
        self.robot = robot
        self.sensor = sensor
        self.timestep = timestep
        self.state = state
        self.MAX_SPEED = 5.0
        
        # Initialize motors
        self.left_front = robot.getDevice('left_wheel')
        self.right_front = robot.getDevice('right_wheel')
        
        # Set up motors for velocity control
        for motor in [self.left_front, self.right_front]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
    
        self.sensors = sensor
    
    def set_left_speed(self, speed):
        self.left_front.setVelocity(speed)
        
    def set_right_speed(self, speed):
        self.right_front.setVelocity(speed)

    def _ramp_speed(self, target_speeds, steps=10):
        """Gradually change speeds to avoid slipping"""
        motors = [self.left_front, self.right_front]
        current_speeds = [motor.getVelocity() for motor in motors]
        
        for step in range(steps):
            for motor, current, target in zip(motors, current_speeds, target_speeds):
                speed = current + (target - current) * (step + 1) / steps
                motor.setVelocity(speed)
            
            # Small delay between speed changes
            start_time = self.robot.getTime()
            while self.robot.step(self.timestep) != -1:
                if self.robot.getTime() - start_time >= 0.05:
                    break
    
    def stop(self):
        """Smoothly bring the robot to a stop"""
        self._ramp_speed([0.0, 0.0])
        
        # Wait until fully stopped
        while self.robot.step(self.timestep) != -1:
            velocities = [abs(motor.getVelocity()) 
                        for motor in [self.left_front, self.right_front]]
            if all(v < 0.01 for v in velocities):
                break
    
    def forward(self, speed, steps=10, duration=None):
        """Move forward with smooth acceleration"""
        speed = min(speed, self.MAX_SPEED)
        self._ramp_speed([speed] * 2, steps)
        
        if duration:
            start_time = self.robot.getTime()
            while self.robot.step(self.timestep) != -1:
                if self.state:
                    self.state.update()
                if self.robot.getTime() - start_time >= duration:
                    self.stop()
                    break
    
    def backward(self, speed, steps=10, duration=None):
        """Move backward with smooth acceleration"""
        speed = min(speed, self.MAX_SPEED)
        self._ramp_speed([-speed] * 2, steps)
        
        if duration:
            start_time = self.robot.getTime()
            while self.robot.step(self.timestep) != -1:
                if self.state:
                    self.state.update()
                if self.robot.getTime() - start_time >= duration:
                    self.stop()
                    break
    
    def turn(self, speed, degrees):
        """Turn in place by specified degrees using state tracking"""
        if self.state is None:
            print("Error: State tracking required for turning")
            return
            
        # Convert current heading to degrees for easier math
        start_heading = self.state.theta * 180 / math.pi
        target_heading = (start_heading + degrees) % 360
        rampupTicks = 10
        #print(f"Starting turn from {start_heading:.1f}° to {target_heading:.1f}°")
        
        while self.robot.step(self.timestep) != -1:
            self.state.update()
            
            # Get current heading in degrees
            current_heading = (self.state.theta * 180 / math.pi) % 360
            
            # Calculate shortest angle to target
            error = target_heading - current_heading

            #error = ((target_heading - current_heading + 180) % 360) - 180
            
            # Calculate proportional speed based on error
            if rampupTicks > 0:
                speed_factor = rampupTicks / 10
                rampupTicks -= 1
            else:
                speed_factor = min(abs(error) / 45.0, 1)  # Gradually reduce speed as error decreases

            turn_speed = speed * speed_factor
            

            # Set motor speeds based on turn direction
            if degrees > 0:  # Turn right
                self.set_right_speed(-turn_speed)
                self.set_left_speed(turn_speed)
            else:  # Turn left
                self.set_right_speed(turn_speed)
                self.set_left_speed(-turn_speed)
            
            #print(f"Current: {current_heading:.1f}°, Target: {target_heading:.1f}°, Error: {error:.1f}°, Speed Factor: {speed_factor:.1f}")
            
            # Stop when close enough
            if abs(error) < 0.5:  # 0.5-degree tolerance
                break
        
        self.stop()

    def is_moving(self):
        """Check if either motor is moving"""
        velocities = [abs(motor.getVelocity()) 
                     for motor in [self.left_front, self.right_front]]
        return any(v > 0.01 for v in velocities)

    def drive_distance(self, speed, distance):
        """Move forward a specific distance in meters"""
        if self.state is None:
            print("Error: State tracking required for distance-based movement")
            return
        
        speed = min(speed, self.MAX_SPEED)
        start_x = self.state.x
        start_y = self.state.y
        
        # Start moving
        self._ramp_speed([speed] * 2)
        
        while self.robot.step(self.timestep) != -1:
            self.state.update()
            
            # Calculate distance traveled
            dx = self.state.x - start_x
            dy = self.state.y - start_y
            traveled = math.sqrt(dx*dx + dy*dy)
            
            if traveled >= distance:
                break
        
        self.stop()
