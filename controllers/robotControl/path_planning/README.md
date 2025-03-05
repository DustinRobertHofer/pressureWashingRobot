# Path Planning Module

This module handles the generation of cleaning paths and routes for the pressure washing robot based on defined area boundaries.

## Components

### CleaningPathGenerator

A class that handles the generation of efficient cleaning paths for different area shapes. The generator creates paths that:

- Cover the entire area with minimal overlap
- Respect boundary constraints and edge buffers
- Generate efficient back-and-forth patterns

### Utility Functions

The module also includes utility functions for:
- Coordinate conversions (meters to feet/inches and back)
- Path optimization
- Boundary processing

## Usage

### Basic Usage

```python
from path_planning import generate_cleaning_path

# Define boundary points (in meters)
boundary_points = [
    {'x': 0.0, 'y': 0.0},
    {'x': 4.8, 'y': 0.0},
    {'x': 4.8, 'y': 1.5},
    {'x': 1.5, 'y': 1.5},
    {'x': 1.5, 'y': 4.3},
    {'x': 0.0, 'y': 4.3}
]

# Generate a cleaning path
cleaning_path = generate_cleaning_path(
    boundary_points, 
    surface_cleaner_diameter=12,  # inches
    path_overlap=3,               # inches
    edge_buffer=8                 # inches
)

# Use the cleaning path for navigation
print(f"Generated a cleaning path with {len(cleaning_path)} waypoints")
```

### Using the CleaningPathGenerator Class

For more control and reusability:

```python
from path_planning import CleaningPathGenerator

# Create a generator with custom parameters
generator = CleaningPathGenerator(
    surface_cleaner_diameter=14,  # inches
    path_overlap=4,               # inches
    edge_buffer=10                # inches
)

# Generate paths for different areas
area1_path = generator.generate_path(area1_boundary)
area2_path = generator.generate_path(area2_boundary)

# Adjust parameters for a different cleaning head
generator.set_cleaner_diameter(18)
generator.set_path_overlap(5)
area3_path = generator.generate_path(area3_boundary)
```

## Algorithm Details

The path generation algorithm works by:

1. Converting boundary points to a consistent coordinate system
2. Creating a mask image representation of the area
3. Applying edge buffers to ensure safety
4. Generating a raster scan pattern with optimal overlap
5. Converting the resulting path back to the original coordinate system

This approach ensures efficient coverage while respecting the physical constraints of the robot and cleaning equipment. 