# Visualization Guide for Improved Occupancy Grid Map

## Quick Start

### Run the Visualization Demo
```bash
python visualize_grid_map_improved.py
```
This will process your lidar data and display the occupancy grid map!

### Try Different Examples
```bash
python example_visualize.py
```
Choose from 6 different examples showing various ways to visualize your maps.

## Color Scheme

The visualization uses a simple, intuitive color scheme based on occupancy probability:

| Probability Range | Color | Meaning |
|------------------|-------|---------|
| **p < 0.3** | **White** | Free space - safe for navigation |
| **0.3 ≤ p ≤ 0.7** | **Gray** | Unknown - not explored or uncertain |
| **p > 0.7** | **Black** | Occupied - obstacle detected |

**Special markers:**
- **Red star (★)**: Origin (first robot position)

## Basic Usage

### Method 1: Easiest Way (from OccupancyGridMapper)

```python
from visualize_grid_map_improved import visualize_from_mapper
from main_improved import process_lidar_data_improved
from read_json import read_json

# Process your data
lidar_data = read_json()
mapper = process_lidar_data_improved(lidar_data, cell_width=0.1)

# Visualize - one line!
visualize_from_mapper(mapper)
```

### Method 2: From Probability Array

```python
from visualize_grid_map_improved import visualize_occupancy_grid

# Get your probability map (2D numpy array)
prob_map = mapper.get_probability_grid()
grid_data = mapper.get_probability_grid_with_bounds()

# Visualize
visualize_occupancy_grid(prob_map, grid_bounds=grid_data)
```

### Method 3: Just the Array (No World Coordinates)

```python
# If you just have a probability array
prob_map = mapper.get_probability_grid()

# Visualize with grid indices instead of world coordinates
visualize_occupancy_grid(prob_map, grid_bounds=None)
```

## Function Reference

### `visualize_from_mapper(mapper, ...)`

**Easiest way** - visualize directly from OccupancyGridMapper object.

**Parameters:**
- `mapper`: OccupancyGridMapper object
- `title`: Plot title (default: "Occupancy Grid Map (Improved)")
- `show_grid_lines`: Show grid lines (default: True)
- `show_colorbar`: Show colorbar with legend (default: True)
- `figsize`: Figure size in inches, tuple (width, height) (default: (12, 10))
- `save_path`: Path to save image, or None to display (default: None)

**Example:**
```python
visualize_from_mapper(
    mapper,
    title="My Robot Map",
    show_grid_lines=True,
    figsize=(14, 12),
    save_path="my_map.png"  # Save to file instead of showing
)
```

### `visualize_occupancy_grid(probability_map, ...)`

**More flexible** - visualize any 2D probability array.

**Parameters:**
- `probability_map`: 2D numpy array with values 0.0 to 1.0
- `grid_bounds`: Dictionary with grid metadata (optional)
  - Keys: `'min_x'`, `'max_x'`, `'min_y'`, `'max_y'`, `'cell_width'`, `'origin_x'`, `'origin_y'`
  - If None, displays grid indices instead of world coordinates
- `title`: Plot title (default: "Occupancy Grid Map")
- `show_grid_lines`: Show grid lines (default: True)
- `show_colorbar`: Show colorbar (default: True)
- `figsize`: Figure size (default: (12, 10))
- `save_path`: Path to save, or None to display (default: None)

**Example:**
```python
prob_map = get_my_probability_map()  # Your 2D array

visualize_occupancy_grid(
    prob_map,
    grid_bounds=None,  # Use grid indices
    title="My Custom Map",
    show_colorbar=True
)
```

## Common Use Cases

### 1. Quick Visualization
```python
# Fastest way to see your map
from visualize_grid_map_improved import visualize_from_mapper

visualize_from_mapper(mapper)
```

### 2. Save Map to File
```python
# Save instead of displaying
visualize_from_mapper(
    mapper,
    save_path="occupancy_map.png"
)
```

### 3. Custom Styling
```python
# Customize appearance
visualize_from_mapper(
    mapper,
    title="High Resolution Map",
    show_grid_lines=False,  # Cleaner look
    figsize=(16, 12)  # Larger
)
```

### 4. Visualize During Processing
```python
# Visualize map at different stages
for i in range(0, len(lidar_data), 100):
    # Process first i scans
    mapper = process_lidar_data_improved(lidar_data[:i], ...)
    
    # Visualize current state
    visualize_from_mapper(
        mapper,
        title=f"Map after {i} scans",
        save_path=f"map_step_{i}.png"
    )
```

### 5. Compare Multiple Maps
```python
import matplotlib.pyplot as plt
from visualize_grid_map_improved import visualize_occupancy_grid

# Get different maps
map1 = mapper1.get_probability_grid()
map2 = mapper2.get_probability_grid()

# Create side-by-side comparison
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

im1 = ax1.imshow(map1, cmap='gray', vmin=0, vmax=1, origin='lower')
ax1.set_title('Map 1')
plt.colorbar(im1, ax=ax1)

im2 = ax2.imshow(map2, cmap='gray', vmin=0, vmax=1, origin='lower')
ax2.set_title('Map 2')
plt.colorbar(im2, ax=ax2)

plt.tight_layout()
plt.show()
```

## Understanding the Visualization

### Elements Shown

1. **Main Grid**: The occupancy probabilities displayed with the color scheme
2. **Grid Lines**: Optional lines showing cell boundaries (every ~1 meter)
3. **Colorbar**: Shows the probability scale and thresholds
4. **Origin Marker**: Red star showing the first robot position
5. **Statistics Box**: Top-left corner shows cell counts and percentages
6. **Title**: Shows grid dimensions and cell width
7. **Axes**: World coordinates (meters) or grid indices

### Reading the Map

- **White areas**: Safe to navigate (definitely free)
- **Black areas**: Obstacles detected (definitely occupied)
- **Gray areas**: Unexplored or uncertain regions
- **Red star**: Where the robot started (origin at 0, 0)

### Statistics Box

Located in the top-left corner, shows:
- **Free cells**: Count and percentage with p < 0.3
- **Unknown cells**: Count and percentage with 0.3 ≤ p ≤ 0.7
- **Occupied cells**: Count and percentage with p > 0.7

## Tips & Tricks

### 1. Performance

For large grids, visualization may be slow. Speed it up:
```python
# Process fewer positions
mapper = process_lidar_data_improved(data, step=10)  # Every 10th

# Or use smaller figure
visualize_from_mapper(mapper, figsize=(8, 6))
```

### 2. High-Quality Output

For publications or presentations:
```python
visualize_from_mapper(
    mapper,
    figsize=(16, 14),  # Larger
    show_grid_lines=True,
    save_path="high_res_map.png"
)

# Then manually adjust DPI in the code if needed
# (edit visualize_grid_map_improved.py, change dpi=150 to dpi=300)
```

### 3. Different Color Schemes

If you want to change colors, edit the `cmap` definition in `visualize_occupancy_grid()`:

```python
# Current (in visualize_grid_map_improved.py):
cmap = LinearSegmentedColormap.from_list('occupancy', [
    (0.0, 'white'),   # Free is white
    (0.3, 'white'),
    (0.3, 'gray'),    # Unknown is gray
    (0.7, 'gray'),
    (0.7, 'black'),   # Occupied is black
    (1.0, 'black')
], N=n_bins)

# Alternative - blue for free, red for occupied:
cmap = LinearSegmentedColormap.from_list('occupancy', [
    (0.0, 'blue'),
    (0.3, 'blue'),
    (0.3, 'yellow'),
    (0.7, 'yellow'),
    (0.7, 'red'),
    (1.0, 'red')
], N=n_bins)
```

### 4. Export for Other Software

```python
# Save the probability map as a numpy file
prob_map = mapper.get_probability_grid()
np.save('occupancy_map.npy', prob_map)

# Or as a CSV
np.savetxt('occupancy_map.csv', prob_map, delimiter=',')

# Or as an image (grayscale)
from PIL import Image
img = Image.fromarray((prob_map * 255).astype(np.uint8))
img.save('occupancy_map_grayscale.png')
```

## Integration with Your Application

### ROS Visualization

```python
# In your ROS node
from visualize_grid_map_improved import visualize_from_mapper

def publish_map_callback(timer_event):
    # Update and visualize periodically
    prob_map = mapper.get_probability_grid()
    
    # Save to file that ROS can read
    visualize_from_mapper(
        mapper,
        save_path='/tmp/current_map.png',
        show_grid_lines=False
    )
    
    # Publish the image via ROS
    # ... your ROS publishing code
```

### Live Updates

```python
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create figure once
fig, ax = plt.subplots(figsize=(12, 10))

def update_plot(frame):
    # Process new data
    process_new_scan(...)
    
    # Get updated map
    prob_map = mapper.get_probability_grid()
    
    # Update plot
    ax.clear()
    ax.imshow(prob_map, cmap='gray', vmin=0, vmax=1, origin='lower')
    ax.set_title(f'Live Map - Frame {frame}')

ani = FuncAnimation(fig, update_plot, interval=100)
plt.show()
```

## Troubleshooting

### Issue: Figure is too small/large
**Solution:** Adjust `figsize` parameter
```python
visualize_from_mapper(mapper, figsize=(16, 12))  # Larger
visualize_from_mapper(mapper, figsize=(8, 6))    # Smaller
```

### Issue: Grid lines are cluttered
**Solution:** Turn them off or edit the step size in the code
```python
visualize_from_mapper(mapper, show_grid_lines=False)
```

### Issue: Map appears rotated/flipped
**Solution:** The visualization uses `origin='lower'` to match the coordinate system. This should be correct for your data.

### Issue: Can't see the origin marker
**Solution:** The origin might be outside the visible area if the grid expanded. Check the red star location.

## Examples Summary

| Example | File | Purpose |
|---------|------|---------|
| Quick demo | `visualize_grid_map_improved.py` | Process data and visualize |
| Interactive | `example_visualize.py` | Choose from 6 examples |
| Integration | See above code snippets | Use in your application |

## Next Steps

1. **Run the demo**: `python visualize_grid_map_improved.py`
2. **Try examples**: `python example_visualize.py`
3. **Integrate**: Import functions into your code
4. **Customize**: Modify colors, sizes, etc. as needed

---

**Happy Visualizing! 🎨🗺️**
