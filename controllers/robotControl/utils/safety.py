"""
Safety controller for the pressure washing robot.
Handles obstacle detection and emergency stops.
"""

class SafetyController:
    """Manages safety features like obstacle detection and emergency stops"""
    
    def __init__(self, sensor_manager, state, motion_controller):
        """Initialize safety controller with required subsystems"""
        self.sensor_manager = sensor_manager
        self.state = state
        self.motion_controller = motion_controller
        
        # Safety parameters
        self.min_obstacle_distance = 0.5  # meters
        self.emergency_stop_active = False
        
    def check_safety(self):
        """Check for safety hazards and take appropriate action"""
        # Get sensor data
        sensor_data = self.sensor_manager.get_sensor_data()
        
        # Check front distance sensor if available
        if 'distance' in sensor_data:
            distance = sensor_data['distance']
            if distance < self.min_obstacle_distance:
                self.emergency_stop()
                return False
                
        # Check LiDAR data for obstacles if available
        if 'laser' in sensor_data:
            laser_data = sensor_data['laser']
            ranges = laser_data.get('ranges', [])
            
            # Simple check - stop if any point is too close
            if ranges and min(ranges) < self.min_obstacle_distance:
                self.emergency_stop()
                return False
                
        # No safety issues detected
        self.emergency_stop_active = False
        return True
        
    def emergency_stop(self):
        """Perform emergency stop"""
        if not self.emergency_stop_active:
            print("⚠️ SAFETY: Emergency stop activated - obstacle detected!")
            self.motion_controller.stop()
            self.emergency_stop_active = True
            
    def reset_safety(self):
        """Reset safety system after emergency stop"""
        self.emergency_stop_active = False
        print("Safety system reset") 