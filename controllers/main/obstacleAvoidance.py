import math

class ObstacleAvoider:
    def __init__(self, state, drive_system, laser_range):
        self.state = state
        self.drive = drive_system
        self.laser_range = laser_range
        self.avoidance_speed = 2.0
        self.turn_speed = 2.0
        self.min_turn_angle = 45  # Minimum angle to turn when avoiding


    def is_obstacle_detected(self):
        if self.laser_range.get_distance_to_obstacle() < 1.0:
            return True
        else:
            return False
    def get_distance_to_obstacle(self):
        return self.laser_range.get_distance_to_obstacle()

    


