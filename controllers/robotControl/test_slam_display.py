"""
Utility script to directly display the SLAM map.
Run this script to display the current state of the SLAM map.
"""
import os
import sys
import time

# Add parent directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from slam.slam_manager import SLAMManager
from slam.simple_slam import SimpleLaserScanner, SimpleSLAM

def display_latest_slam_map():
    """Display the most recent SLAM map if available"""
    # Check if maps directory exists
    maps_dir = os.path.join(current_dir, 'maps')
    if not os.path.exists(maps_dir):
        print("No maps directory found. Has SLAM mapping been run yet?")
        return False
        
    # Try to find and load the map data
    try:
        from slam.simple_slam import SimpleLaserScanner, SimpleSLAM
        
        # Create temporary objects to display map
        laser_params = SimpleLaserScanner(
            scan_size=360,
            scan_rate_hz=10,
            detection_angle_degrees=360,
            distance_no_detection_mm=8000
        )
        
        slam = SimpleSLAM(
            laser_params,
            map_size_pixels=500,
            map_size_meters=10
        )
        
        # Try to load the most recent map
        map_loaded = False
        try:
            # Look for saved map files
            map_files = [f for f in os.listdir(maps_dir) if f.startswith('map_') and f.endswith('.npy')]
            if map_files:
                # Sort by modification time to get the most recent
                map_files.sort(key=lambda f: os.path.getmtime(os.path.join(maps_dir, f)), reverse=True)
                latest_map = map_files[0]
                
                # Load the map
                map_path = os.path.join(maps_dir, latest_map)
                print(f"Loading map from {map_path}")
                # Directly create and display the map
                import numpy as np
                from matplotlib import pyplot as plt
                
                # Load the grid
                grid = np.load(map_path)
                
                # Display the grid
                plt.figure(figsize=(10, 10))
                plt.imshow(grid, cmap='gray_r', origin='lower')
                plt.title(f'SLAM Map (from {latest_map})')
                plt.colorbar(label='Occupancy (100=occupied, -1=free, 0=unknown)')
                plt.show()
                
                map_loaded = True
            else:
                print("No map files found in the maps directory.")
        except Exception as e:
            print(f"Error loading map: {e}")
        
        # If we couldn't load a saved map, display an empty one
        if not map_loaded:
            print("Creating and displaying a new empty map")
            slam.display_map()
            
        return True
    
    except Exception as e:
        print(f"Error displaying map: {e}")
        return False

if __name__ == "__main__":
    print("Attempting to display the latest SLAM map...")
    if display_latest_slam_map():
        print("Map display complete.")
    else:
        print("Failed to display map.") 