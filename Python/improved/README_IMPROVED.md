# Improved Occupancy Grid Mapper

## Overview

This is an improved version of the occupancy grid mapping and ray tracing algorithms that handles:
- **Dynamic coordinate systems** with the first robot position as the origin
- **Negative coordinates** in all directions
- **Dynamic grid expansion** as the robot explores new areas
- **Probabilistic occupancy mapping** using Bayesian updates

## Key Improvements

### 1. Dynamic Origin
- The **first robot position** becomes the origin (0, 0) of the map
- All subsequent positions and measurements are relative to this origin
- No need to pre-calculate coordinate translations

### 2. Negative Coordinate Support
- The grid automatically handles negative coordinates
- The robot can move in any direction from the origin
- Grid indices remain positive (0, 1, 2, ...) while representing potentially negative world coordinates

### 3. Dynamic Grid Expansion
- The grid automatically expands when the robot explores new areas
- No need to pre-define grid size based on the entire trajectory
- Efficient memory usage - starts small and grows as needed

### 4. Clean Probability Output
- Get occupancy probabilities as a simple 2D numpy array
- Range: 0.0 (definitely free) to 1.0 (definitely occupied)
- Easy integration with path planning and navigation algorithms

## File Structure

### Improved Files
- **`determine_cell_index_improved.py`**: Cell index determination with dynamic bounds
- **`ray_tracing_improved.py`**: Ray tracing algorithm with negative coordinate support
- **`grid_map_improved.py`**: Dynamic occupancy grid mapper class
- **`main_improved.py`**: Demonstration and usage examples

### Original Files (Unchanged)
- `determine_cell_index.py`
- `ray_tracing.py`
- `grid_map.py`
- `main.py`
- All files in `classes/` directory
- `read_json.py`
- `calc_coordinates.py`

## Usage

### Basic Usage

```python
from read_json import read_json
from main_improved import process_lidar_data_improved

# Read lidar data
lidar_data_list = read_json()

# Process data and build occupancy grid
mapper = process_lidar_data_improved(
    lidar_data_list=lidar_data_list,
    cell_width=0.1,              # 10cm cells
    initial_grid_size=200,       # Start with 200x200 cells
    prob_occ_occ=0.9,           # P(occupied | measured occupied)
    prob_occ_free=0.3,          # P(occupied | measured free)
    max_lidar_range=4.2,        # Max lidar range in meters
    step=1,                      # Process every position (use step=10 for faster)
    verbose=True                 # Print progress
)

# Get probability grid (2D numpy array)
prob_grid = mapper.get_probability_grid()

# Get grid with metadata
grid_data = mapper.get_probability_grid_with_bounds()
```

### Getting Occupancy Probabilities

```python
# Method 1: Get just the probability matrix
prob_matrix = mapper.get_probability_grid()
# Returns: numpy array of shape (n_rows, n_cols) with values in [0.0, 1.0]

# Method 2: Get probability matrix with bounds information
grid_data = mapper.get_probability_grid_with_bounds()
# Returns dictionary with:
#   - 'probabilities': 2D numpy array
#   - 'min_x', 'max_x', 'min_y', 'max_y': Grid bounds in world coordinates
#   - 'n_rows', 'n_cols': Grid dimensions
#   - 'cell_width': Size of each cell
#   - 'origin_x', 'origin_y': Origin position (first robot position)
```

### Coordinate Conversions

```python
# Convert world coordinates to grid indices
row, col = mapper.world_to_grid(x=1.5, y=-0.5)

# Convert grid indices to world coordinates (cell center)
x, y = mapper.grid_to_world(row=10, col=20)

# Get grid information
info = mapper.get_grid_info()
print(f"Grid covers X: [{info['min_x']}, {info['max_x']}]")
print(f"Grid covers Y: [{info['min_y']}, {info['max_y']}]")
```

### Saving and Loading Grids

```python
from main_improved import save_probability_grid_to_file, load_probability_grid_from_file

# Save grid to file
save_probability_grid_to_file(mapper, "my_map.npz")

# Load grid from file
grid_data = load_probability_grid_from_file("my_map.npz")
prob_matrix = grid_data['probabilities']
```

## Algorithm Details

### Occupancy Grid Mapping

The system uses **log-odds representation** for efficient Bayesian updates:

```
log_odds = log(p / (1 - p))
```

**Update Rules:**
- Free cell observation: `log_odds += log(P(occ|free) / (1 - P(occ|free)))`
- Occupied cell observation: `log_odds += log(P(occ|occ) / (1 - P(occ|occ)))`

**Probability Interpretation:**
- `p < 0.3`: Cell is likely **free**
- `0.3 ≤ p ≤ 0.7`: Cell is **unknown**
- `p > 0.7`: Cell is likely **occupied**

### Ray Tracing Algorithm

The improved ray tracing algorithm:
1. Works in world coordinates (can be negative)
2. Converts to grid indices dynamically using current grid bounds
3. Uses X-step or Y-step algorithm based on beam angle
4. Returns free cells along the ray and the occupied cell at the end

**Angle Selection:**
- `45° < α < 135°` or `225° < α < 315°`: **Y-step algorithm**
- Other angles: **X-step algorithm**

### Grid Expansion

The grid automatically expands when coordinates outside current bounds are detected:
1. Calculate required bounds with margin (10% or 1m minimum)
2. Create new larger grid initialized to unknown (log-odds = 0)
3. Copy old grid data to appropriate position in new grid
4. Update bounds and dimensions

## Running the Demo

```bash
cd /workspace/src/mgm-project/Python
python main_improved.py
```

The demo will:
1. Load lidar data from `data/real_data.json`
2. Set the first robot position as origin
3. Process all scans and build the occupancy grid
4. Print statistics and grid information
5. Save the grid to `occupancy_grid_improved.npz`
6. Demonstrate coordinate queries

## Parameters

### OccupancyGridMapper Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cell_width` | 0.1 | Cell size in meters |
| `initial_size` | 100 | Initial grid size (cells) |
| `prob_occupied_given_occupied` | 0.9 | P(occupied \| measured occupied) |
| `prob_occupied_given_free` | 0.3 | P(occupied \| measured free) |
| `origin_x` | 0.0 | X coordinate of origin |
| `origin_y` | 0.0 | Y coordinate of origin |

### Sensor Model Tuning

**High Confidence (default):**
- `prob_occ_occ = 0.9, prob_occ_free = 0.3`
- Fast convergence, good for accurate sensors

**Low Confidence:**
- `prob_occ_occ = 0.7, prob_occ_free = 0.4`
- Slower convergence, more robust to sensor noise

**Very High Confidence:**
- `prob_occ_occ = 0.95, prob_occ_free = 0.2`
- Very fast convergence, use only with very accurate sensors

## Integration with ROS/ROS2

The improved mapper can be easily integrated with ROS:

```python
# Subscribe to robot pose and lidar scans
# On each message:

# 1. Update robot position
robot_pos = RobotPosition(X=msg.pose.x, Y=msg.pose.y, Theta=msg.pose.theta)

# 2. Process lidar scans
for i, distance in enumerate(msg.ranges):
    lidar_data = LidarData(alpha=i, length=distance if distance > 0 else -1.0)
    
    # Ray trace
    free_cells, occupied_cell = ray_tracing_improved(
        robot_pos, lidar_data, 
        w=mapper.cell_width,
        min_x=mapper.min_x,
        min_y=mapper.min_y
    )
    
    # Update grid
    mapper.update_with_scan(free_cells, [occupied_cell] if occupied_cell else [])

# 3. Publish occupancy grid
prob_grid = mapper.get_probability_grid()
# Convert to OccupancyGrid message...
```

## Performance Considerations

### Memory Usage
- Initial: ~100-200 KB (for 100x100 grid with float64)
- Scales with explored area
- Grid expands automatically but never shrinks

### Processing Speed
- Ray tracing: ~0.1-0.5 ms per beam (depending on length)
- Grid update: ~0.01 ms per cell
- Grid expansion: ~1-10 ms (only when needed)

### Optimization Tips
1. **Use larger cells** for faster processing (e.g., 0.2m instead of 0.1m)
2. **Process every Nth scan** for real-time performance (step=10)
3. **Limit max range** to reduce ray tracing computation
4. **Reduce initial_size** if memory is limited

## Differences from Original Implementation

| Aspect | Original | Improved |
|--------|----------|----------|
| Origin | Fixed at (0, 0) bottom-left | First robot position |
| Coordinates | Only positive | Positive and negative |
| Grid size | Fixed | Dynamic expansion |
| Visualization | Matplotlib animation | Probability matrix output |
| Cell indexing | Assumes positive coords | Offset-based indexing |
| Output format | Visual only | Numpy array + metadata |

## Known Limitations

1. **Grid never shrinks**: Once expanded, the grid maintains its size
2. **Memory**: Very large explorations may require significant memory
3. **No loop closure**: The mapper doesn't detect or correct for loop closures
4. **No pose correction**: Robot pose is assumed to be accurate

## Future Improvements

Potential enhancements:
- [ ] Grid pruning to remove unused cells
- [ ] Multi-resolution grids (hierarchical)
- [ ] Integration with SLAM for loop closure
- [ ] GPU acceleration for large grids
- [ ] Compressed grid storage
- [ ] Real-time ROS node implementation

## License

Same as the original implementation.

## Contact

For questions or issues, please contact the original author: Balassa Hodi
