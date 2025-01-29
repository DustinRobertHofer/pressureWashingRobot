from controller import Robot
import math


class DriveSystem:
    def __init__(self, robot, sensor, timestep):
        self.robot = robot
        self.sensor = sensor
        self.timestep = timestep
        self.MAX_SPEED = 5.0
        
        # Initialize motors
        self.left_front = robot.getDevice('left_wheel')
        self.right_front = robot.getDevice('right_wheel')
        
        # Set up motors for velocity control
        for motor in [self.left_front, self.right_front]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        # Robot physical parameters
        self.WHEEL_RADIUS = 2.0  # cm
        self.TURN_RADIUS = 7.125  # cm (distance from center to wheels)
        
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
                if self.robot.getTime() - start_time >= duration:
                    self.stop()
                    break
    
    def turn(self, speed, degrees):
        """Turn in place by specified degrees using compass feedback"""
        start_bearing = self.sensors.get_bearing()
        target_bearing = (start_bearing + degrees) % 360
        
        print(f"Starting turn from {start_bearing:.1f}° to {target_bearing:.1f}°")
        
        while self.robot.step(self.timestep) != -1:
            current_bearing = self.sensors.get_bearing()
            error = ((target_bearing - current_bearing + 180) % 360) - 180
            
            # Calculate proportional speed based on error
            speed_factor = min(abs(error) / 45.0, 1.0)  # Gradually reduce speed as error decreases
            turn_speed = speed * speed_factor
            
            # Set motor speeds based on turn direction
            if degrees < 0:  # Turn left
                self.set_right_speed(turn_speed)
                self.set_left_speed(-turn_speed)
            else:  # Turn right
                self.set_right_speed(-turn_speed)
                self.set_left_speed(turn_speed)
            
            print(f"Current: {current_bearing:.1f}°, Target: {target_bearing:.1f}°, Error: {error:.1f}°")
            
            # Stop when close enough
            if abs(error) < 0.05:  # 1-degree tolerance
                break
        
        self.stop()
