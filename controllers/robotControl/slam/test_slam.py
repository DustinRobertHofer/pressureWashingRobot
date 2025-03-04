"""
Test file for the SimpleSLAM implementation.
This file can be run directly to test the SLAM implementation with simulated data.
"""
import os
import sys
import numpy as np
import math
import matplotlib.pyplot as plt

# Add parent directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import our SLAM implementation
from simple_slam import SimpleLaserScanner, SimpleSLAM

def generate_test_data():
    """Generate simulated laser scan data for testing SLAM"""
    # Simulate a robot moving in a square room (5m x 5m)
    # The robot will follow a circular path
    
    # Laser scanner parameters
    laser_params = SimpleLaserScanner(
        scan_size=360,
        scan_rate_hz=10,
        detection_angle_degrees=360,
        distance_no_detection_mm=5000
    )
    
    # Create a SLAM object
    slam = SimpleSLAM(laser_params, map_size_pixels=500, map_size_meters=10)
    
    # Room size (5m x 5m)
    room_width = 5
    room_height = 5
    
    # Simulate robot moving in a circle
    circle_radius = 1.5  # meters
    center_x = room_width / 2
    center_y = room_height / 2
    
    # Number of steps in the simulation
    num_steps = 100
    
    for step in range(num_steps):
        # Calculate robot position along the circle
        angle = step * (2 * math.pi / num_steps)
        robot_x = center_x + circle_radius * math.cos(angle)
        robot_y = center_y + circle_radius * math.sin(angle)
        robot_theta = angle + math.pi/2  # Point tangent to the circle
        
        # Generate simulated laser scan
        scan_distances = []
        for i in range(360):
            scan_angle = math.radians(i)
            # Calculate absolute angle in world frame
            abs_angle = robot_theta + scan_angle
            
            # Cast ray to each wall and find closest intersection
            distances = []
            
            # Check intersection with right wall (x = room_width)
            if math.cos(abs_angle) > 0.001:  # Avoid division by zero
                t = (room_width - robot_x) / math.cos(abs_angle)
                if t > 0:
                    wall_y = robot_y + t * math.sin(abs_angle)
                    if 0 <= wall_y <= room_height:
                        distances.append(t)
            
            # Check intersection with top wall (y = room_height)
            if math.sin(abs_angle) > 0.001:
                t = (room_height - robot_y) / math.sin(abs_angle)
                if t > 0:
                    wall_x = robot_x + t * math.cos(abs_angle)
                    if 0 <= wall_x <= room_width:
                        distances.append(t)
            
            # Check intersection with left wall (x = 0)
            if math.cos(abs_angle) < -0.001:
                t = -robot_x / math.cos(abs_angle)
                if t > 0:
                    wall_y = robot_y + t * math.sin(abs_angle)
                    if 0 <= wall_y <= room_height:
                        distances.append(t)
            
            # Check intersection with bottom wall (y = 0)
            if math.sin(abs_angle) < -0.001:
                t = -robot_y / math.sin(abs_angle)
                if t > 0:
                    wall_x = robot_x + t * math.cos(abs_angle)
                    if 0 <= wall_x <= room_width:
                        distances.append(t)
            
            # Add some noise (up to 5cm)
            if distances:
                distance = min(distances) * 1000  # Convert to mm
                distance += np.random.normal(0, 50)  # Add noise (5cm std dev)
                scan_distances.append(distance)
            else:
                scan_distances.append(laser_params.distance_no_detection_mm)
        
        # Update SLAM with simulated data
        odometry = {
            'x': robot_x,
            'y': robot_y,
            'theta': robot_theta,
            'previous_x': center_x + circle_radius * math.cos(angle - (2 * math.pi / num_steps)) if step > 0 else robot_x,
            'previous_y': center_y + circle_radius * math.sin(angle - (2 * math.pi / num_steps)) if step > 0 else robot_y
        }
        
        slam.update(odometry, scan_distances)
        
        # Display every 10 steps
        if step % 10 == 0:
            print(f"Step {step}: Robot at ({robot_x:.2f}, {robot_y:.2f}), heading: {math.degrees(robot_theta):.1f} degrees")
    
    # Display the final map
    print("Displaying final map...")
    slam.display_map()
    
    # Return SLAM object for further analysis
    return slam

if __name__ == "__main__":
    print("Testing SimpleSLAM with simulated data...")
    slam = generate_test_data()
    
    # Get room boundaries
    boundaries = slam.get_cleaning_boundary()
    print("Extracted room boundaries:")
    for point in boundaries:
        print(f"  ({point['x']:.2f}, {point['y']:.2f})")
        
    print("Test complete!") 