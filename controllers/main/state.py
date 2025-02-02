from math import pi
from deadReckoning import DeadReckoning

class State:
    def __init__(self, robot, timestep):
        """Initialize state tracking system"""
        self.robot = robot
        self.timestep = timestep
        
        # Initialize position tracking systems
        self.dead_reckoning = DeadReckoning(robot, timestep)
        
        # Current position state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self):
        """Update state estimation using available sensor data"""
        # Get position estimate from dead reckoning
        dr_state = self.dead_reckoning.update()
        
        # For now, just use dead reckoning data directly
        self.x = dr_state['x']
        self.y = dr_state['y']
        self.theta = dr_state['theta']

    def get_position(self):
        """Return current estimated position and heading"""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'theta_degrees': self.theta * 180 / pi
        }
    
    def reset(self):
        """Reset position estimation to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.dead_reckoning.reset() 