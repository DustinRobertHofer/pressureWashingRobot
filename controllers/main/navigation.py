import numpy as np
import math
from pathGeneration.PathGenerator import PathCoordinates

class Navigator:
    def __init__(self, state, drive_system):
        self.state = state
        self.drive = drive_system
        self.path_points = None
        self.current_point_index = 0
        self.position_tolerance = 0.1  # meters
        self.angle_tolerance = 2.0  # degrees
        self.default_speed = 3.0

    def load_path(self):
        """Load and process path points from PathGenerator"""
        # PathCoordinates comes as a flat array, reshape into pairs
        points = np.array(PathCoordinates).reshape(-1, 2)
      
        # Convert from inches to meters
        self.path_points = (points - 8) * 0.0254  # 1 inch = 0.0254 meters
        print(self.path_points)
        self.current_point_index = 0
        return self.path_points


    def get_angle_to_target(self, target_x, target_y):
        """Calculate angle to target point relative to current position"""
        current_pos = self.state.get_position()
        dx = target_x - current_pos['x']
        dy = target_y - current_pos['y']
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Normalize target angle to 0-360
        target_angle = target_angle % 360
        
        # Calculate the smallest turning angle
        current_angle = current_pos['theta_degrees'] % 360
        angle_diff = target_angle - current_angle
        
        # Normalize to -180 to 180
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
            
        return angle_diff

    def get_distance_to_target(self, target_x, target_y):
        """Calculate distance to target point"""
        current_pos = self.state.get_position()
        dx = target_x - current_pos['x']
        dy = target_y - current_pos['y']
        return math.sqrt(dx*dx + dy*dy)

    def move_to_next_point(self,speed):
        """Move to the next point in the path"""
        if self.path_points is None or self.current_point_index >= len(self.path_points):
            return False

        target = self.path_points[self.current_point_index]
        target_x, target_y = target[0], target[1]

        # Calculate angle to target
        angle_to_target = self.get_angle_to_target(target_x, target_y)

        # Turn towards target if needed
        if abs(angle_to_target) > self.angle_tolerance:
            self.drive.turn(speed, angle_to_target)


        # Move to target
        distance = self.get_distance_to_target(target_x, target_y)
        if distance > self.position_tolerance:
            self.drive.drive_distance(speed, distance)


        self.current_point_index += 1
        return True


    def navigate_path(self,speed):
        """Navigate through all points in the path"""
        if self.path_points is None:

            self.load_path()
            
        while self.move_to_next_point(speed):
            current_pos = self.state.get_position()
            print(f"Reached point {self.current_point_index} at position: "
                f"({current_pos['x']:.2f}, {current_pos['y']:.2f})")


            

        print("Path navigation completed")
