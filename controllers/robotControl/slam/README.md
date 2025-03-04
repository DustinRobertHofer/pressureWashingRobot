# SLAM Module for Pressure Washing Robot

This module provides a simplified SLAM (Simultaneous Localization and Mapping) implementation that works on Windows for the pressure washing robot.

## Overview

The SLAM module enables the robot to:
1. Build a map of its environment using laser range sensors
2. Track its position within that map
3. Extract boundaries for cleaning operations
4. Save and load maps

## Usage

### Testing the SLAM Implementation

You can test the SLAM implementation with simulated data:

```bash
cd controllers/robotControl/slam
python test_slam.py
```

This will simulate a robot moving in a square room and building a map using the SLAM algorithm.

### Integration with the Robot Controller

The SLAM module is integrated with the robot controller. To enable SLAM:

1. Uncomment the following line in the `main()` function in `robotControl.py`:
   ```python
   # controller.toggle_slam(True)
   ```

2. Ensure your robot has a laser range sensor available.

3. Run your robot controller as usual within the Webots simulator.

### SLAM Manager API

The `SLAMManager` class provides the following methods:

- `start_mapping()` - Begin the SLAM mapping process
- `stop_mapping()` - Stop the SLAM mapping process
- `update()` - Update SLAM with latest sensor readings
- `get_map()` - Get the current SLAM map
- `display_map()` - Display the current map
- `save_map(filename)` - Save the current map to a file
- `load_map(filename)` - Load a map from a file
- `get_cleaning_boundary()` - Get a cleaning boundary from the SLAM map
- `is_position_free(x, y)` - Check if a position is free
- `generate_cleaning_path_from_map()` - Generate a cleaning path from the SLAM map

## Dependencies

The SLAM implementation uses:
- numpy - For efficient array operations
- matplotlib - For map visualization
- math - For mathematical operations
- os - For file operations

## Troubleshooting

If you encounter import errors:
1. Ensure all `__init__.py` files are in place
2. Check the import paths in the SLAM manager
3. Make sure the path resolution code is working correctly

## Author

This SLAM implementation is a simplified version inspired by BreezySLAM, adapted for the pressure washing robot project. 