# Occupancy Grid Mapper

This directory contains implementations of an occupancy grid mapping system for processing LiDAR sensor data from a mobile robot.

## Directory Structure

```
Python/
├── improved/          # Improved implementation with dynamic grid expansion
├── original/          # Original fixed-size grid implementation
├── shared/            # Shared utilities and classes
├── data/              # LiDAR data files
└── test_imports.py    # Import verification test script
```

## Implementations

### Improved Implementation (`improved/`)

The improved version features:
- **Dynamic grid expansion**: Automatically grows as the robot explores new areas
- **Negative coordinate support**: Handles coordinates in all quadrants
- **Probabilistic Bayesian updates**: Uses log-odds for efficient probability calculations
- **Better memory efficiency**: Only allocates grid space as needed

**Key files:**
- `main_improved.py` - Main processing function and examples
- `grid_map_improved.py` - `OccupancyGridMapper` class with dynamic grid
- `ray_tracing_improved.py` - Ray tracing algorithm for dynamic bounds
- `determine_cell_index_improved.py` - Cell indexing with offset-based approach
- `example_simple.py` - Minimal usage example
- `example_visualize.py` - Visualization examples
- `comparison.py` - Comparison between original and improved approaches
- `visualize_grid_map_improved.py` - Visualization utilities
- `debug_*.py` - Debug and testing scripts
- `*.md` - Documentation files

**Usage:**
```bash
cd improved
python3 example_simple.py
```

### Original Implementation (`original/`)

The original implementation uses a fixed-size grid with predefined dimensions.

**Key files:**
- `main.py` - Main processing function
- `grid_map.py` - Grid map visualization and animation
- `ray_tracing.py` - Ray tracing algorithm for fixed grid
- `determine_cell_index.py` - Cell indexing for fixed grid

**Usage:**
```bash
cd original
python3 main.py
```

## Shared Components (`shared/`)

Common utilities used by both implementations:

- `read_json.py` - Reads LiDAR data from JSON files
- `calc_coordinates.py` - Coordinate transformation utilities
- `classes/` - Data classes:
  - `cell.py` - Cell representation
  - `robot_position.py` - Robot pose (x, y, theta)
  - `lidar_sensor_data.py` - LiDAR scan data
  - `coordinates.py` - 2D coordinate representation

## Data (`data/`)

Contains LiDAR sensor data files:
- `real_data.json` - Real robot LiDAR scans with pose information

Each scan includes:
- 360 distance measurements (one per degree)
- Robot pose (x, y, theta)
- Timestamp

## Getting Started

1. **Test the setup:**
   ```bash
   python3 test_imports.py
   ```

2. **Run the improved implementation:**
   ```bash
   cd improved
   python3 example_simple.py
   ```

3. **Explore the documentation:**
   ```bash
   cd improved
   cat START_HERE.md
   ```

## Requirements

- Python 3.x
- NumPy
- Matplotlib (for visualization)

## Algorithm Overview

Both implementations use:
1. **Ray tracing**: Traces LiDAR beams through the grid to mark free cells
2. **Cell indexing**: Converts world coordinates to grid indices
3. **Occupancy updates**: Updates cell probabilities based on sensor observations

The improved version enhances this with dynamic grid management and probabilistic Bayesian updates using log-odds representation.

## Notes

- Files within `improved/` and `original/` directories should be run from within their respective directories
- The `shared/` directory is automatically accessible via the Python path setup in each module
- See individual README files in subdirectories for detailed documentation
