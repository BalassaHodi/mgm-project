# Project Structure Overview

## Directory Structure After Improvements

```
Python/
├── 📘 START_HERE.md                      ← 🚀 BEGIN HERE!
├── 📘 README_IMPROVED.md                  ← Complete documentation
├── 📘 FILES_SUMMARY.md                    ← API reference
│
├── 🆕 IMPROVED FILES (Your Real-Life Application)
│   ├── determine_cell_index_improved.py   ← Cell indexing with negative coords
│   ├── ray_tracing_improved.py            ← Ray tracing with dynamic bounds
│   ├── grid_map_improved.py               ← OccupancyGridMapper class
│   ├── main_improved.py                   ← Full demonstration
│   ├── example_simple.py                  ← Quick start example
│   └── comparison.py                      ← Compare old vs new
│
├── 📦 ORIGINAL FILES (Unchanged)
│   ├── determine_cell_index.py            ← Original cell indexing
│   ├── ray_tracing.py                     ← Original ray tracing
│   ├── grid_map.py                        ← Original grid map with matplotlib
│   └── main.py                            ← Original main
│
├── 🔧 SHARED UTILITIES (Used by both versions)
│   ├── calc_coordinates.py                ← Coordinate calculations
│   ├── read_json.py                       ← JSON data reader
│   ├── classes/
│   │   ├── __init__.py
│   │   ├── cell.py                        ← Cell class
│   │   ├── coordinates.py                 ← Coordinates class
│   │   ├── robot_position.py              ← RobotPosition class
│   │   └── lidar_sensor_data.py           ← LidarData class
│   └── data/
│       └── real_data.json                 ← Your lidar data
```

## File Relationships Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    YOUR APPLICATION                          │
│                                                              │
│  from main_improved import process_lidar_data_improved      │
│  from grid_map_improved import OccupancyGridMapper          │
└──────────────────────┬───────────────────────────────────────┘
                       │
                       │ uses
                       ↓
┌─────────────────────────────────────────────────────────────┐
│              main_improved.py                                │
│  • process_lidar_data_improved()                            │
│  • save_probability_grid_to_file()                          │
│  • load_probability_grid_from_file()                        │
└──────┬────────────────────────┬─────────────────────────────┘
       │                        │
       │                        │
       ↓                        ↓
┌──────────────────┐    ┌──────────────────────────┐
│ ray_tracing      │    │  OccupancyGridMapper     │
│    _improved.py  │    │  (grid_map_improved.py)  │
│                  │    │                          │
│ • ray_tracing    │    │ • get_probability_grid() │
│   _improved()    │    │ • update_with_scan()     │
│ • x_step_algo    │    │ • world_to_grid()        │
│   _improved()    │    │ • grid_to_world()        │
│ • y_step_algo    │    │ • _expand_grid_if_needed()│
│   _improved()    │    │                          │
└────┬─────────────┘    └──────────────────────────┘
     │
     │ uses
     ↓
┌──────────────────────────┐
│ determine_cell_index     │
│      _improved.py        │
│                          │
│ • determine_cell_index   │
│   _improved()            │
└──────┬───────────────────┘
       │
       │ uses
       ↓
┌─────────────────────────────────────────┐
│         SHARED UTILITIES                │
│                                         │
│  • calc_coordinates.py                  │
│  • read_json.py                         │
│  • classes/cell.py                      │
│  • classes/coordinates.py               │
│  • classes/robot_position.py            │
│  • classes/lidar_sensor_data.py         │
└─────────────────────────────────────────┘
```

## Data Flow

```
1. DATA INPUT
   └─→ real_data.json
       └─→ read_json.py
           └─→ List of {robot_position, scans}

2. PROCESSING (for each scan)
   └─→ ray_tracing_improved()
       ├─→ determine_cell_index_improved()  (convert world → grid)
       ├─→ calc_coordinates()               (calculate beam endpoints)
       └─→ Returns: (free_cells, occupied_cell)

3. GRID UPDATE
   └─→ OccupancyGridMapper.update_with_scan()
       ├─→ Update log-odds for free cells
       ├─→ Update log-odds for occupied cells
       └─→ Expand grid if needed

4. OUTPUT
   └─→ OccupancyGridMapper.get_probability_grid()
       └─→ 2D numpy array (probabilities 0.0 to 1.0)
           └─→ USE IN YOUR APPLICATION!
```

## Usage Patterns

### Pattern 1: Batch Processing (Offline)
```
read_json() → process_lidar_data_improved() → get_probability_grid()
```

**Code:**
```python
data = read_json()
mapper = process_lidar_data_improved(data)
prob_map = mapper.get_probability_grid()
```

### Pattern 2: Real-Time Processing (Online)
```
Initialize → For each scan: ray_trace → update_grid → get_map
```

**Code:**
```python
# Once at start
mapper = OccupancyGridMapper(origin_x=first_x, origin_y=first_y)

# For each scan
for scan in scans:
    free, occ = ray_tracing_improved(...)
    mapper.update_with_scan(free, [occ] if occ else [])
    current_map = mapper.get_probability_grid()
```

## Quick Reference: Which File to Use?

| Task | File to Use |
|------|-------------|
| **Get started quickly** | `example_simple.py` |
| **Full demonstration** | `main_improved.py` |
| **Compare with original** | `comparison.py` |
| **Read documentation** | `README_IMPROVED.md` |
| **API reference** | `FILES_SUMMARY.md` |
| **Integrate in your app** | Import from `grid_map_improved.py` |
| **Process one scan** | `ray_tracing_improved.py` |
| **Convert coordinates** | `determine_cell_index_improved.py` |

## Class Overview

### OccupancyGridMapper

**Purpose:** Manages the dynamic occupancy grid map

**Key Attributes:**
- `cell_width` - Size of each cell (meters)
- `min_x, max_x, min_y, max_y` - Grid bounds
- `n_rows, n_cols` - Grid dimensions
- `log_odds_grid` - Internal probability storage
- `origin_x, origin_y` - First robot position

**Key Methods:**
- `get_probability_grid()` - Get probability matrix
- `get_probability_grid_with_bounds()` - Get matrix + metadata
- `update_with_scan()` - Update grid with new data
- `world_to_grid()` - Convert world coords → grid indices
- `grid_to_world()` - Convert grid indices → world coords
- `_expand_grid_if_needed()` - Auto-expand grid

## Function Overview

### ray_tracing_improved()

**Purpose:** Trace a lidar beam and find free/occupied cells

**Input:**
- Robot position (X, Y, Theta)
- Lidar data (alpha, length)
- Cell width
- Current grid bounds (min_x, min_y)

**Output:**
- `free_cells`: List of Cell objects (free space)
- `occupied_cell`: Cell object or None (detected obstacle)

### determine_cell_index_improved()

**Purpose:** Convert world coordinates to grid cell indices

**Input:**
- World coordinates (X, Y)
- Grid bounds (min_x, min_y)
- Cell width

**Output:**
- Cell object with (row, column) indices

## Coordinate Systems Explained

### World Coordinates (Real-Life)
```
• Units: meters
• Origin: First robot position
• Range: -∞ to +∞ (any real number)
• Example: (1.5, -2.3) meters from first position
```

### Grid Coordinates (Internal)
```
• Units: cell indices
• Origin: (0, 0) at grid bottom-left
• Range: 0 to n_rows-1, 0 to n_cols-1
• Example: row=45, col=120
```

### Conversion
```
World → Grid:   use determine_cell_index_improved()
                or mapper.world_to_grid()

Grid → World:   use mapper.grid_to_world()
```

## Memory Layout

### Grid Storage
```
log_odds_grid[row, col]  ← NumPy array (float64)

Row 0 = Bottom of map
Row n_rows-1 = Top of map
Col 0 = Left of map  
Col n_cols-1 = Right of map
```

### Probability Interpretation
```
log_odds = 0     → probability = 0.5 (unknown)
log_odds > 0     → probability > 0.5 (likely occupied)
log_odds < 0     → probability < 0.5 (likely free)

Conversion: p = 1 - 1/(1 + exp(log_odds))
```

## Example Workflow

```
Step 1: Start
  ├─ Load data: read_json()
  └─ First position becomes origin (0, 0)

Step 2: Initialize
  └─ Create OccupancyGridMapper(origin_x, origin_y)

Step 3: For each robot position:
  ├─ Expand grid if needed
  │  └─ _expand_grid_if_needed()
  │
  ├─ For each lidar beam:
  │  ├─ Ray trace: ray_tracing_improved()
  │  │  └─ Returns free cells & occupied cell
  │  └─ Collect all cells
  │
  └─ Update grid: update_with_scan()
     ├─ Update free cells (reduce occupancy)
     └─ Update occupied cells (increase occupancy)

Step 4: Get result
  └─ Get probability matrix: get_probability_grid()
     └─ Use in your application!
```

## Testing Workflow

```
1. Run simple example:
   $ python example_simple.py
   ✓ Should complete in ~5-10 seconds
   ✓ Shows basic usage

2. Run full demo:
   $ python main_improved.py
   ✓ Should complete in ~30-60 seconds
   ✓ Shows detailed statistics

3. Run comparison:
   $ python comparison.py
   ✓ Shows why improved version is better
   ✓ Explains coordinate handling

4. Integrate in your app:
   ✓ Import OccupancyGridMapper
   ✓ Follow pattern from example_simple.py
```

## Performance Characteristics

| Operation | Time Complexity | When |
|-----------|----------------|------|
| Ray tracing | O(cells_along_ray) | Each lidar beam |
| Cell update | O(1) | Each cell |
| Grid expansion | O(old_rows × old_cols) | When bounds exceeded |
| Get probability | O(n_rows × n_cols) | When requested |
| Coordinate conversion | O(1) | Any time |

**Typical Performance:**
- Ray trace: 0.1-0.5 ms per beam
- Process 360 beams: 36-180 ms
- Grid expansion: 1-10 ms (rare)
- Get probability matrix: <1 ms

## Tips for Best Performance

1. **Use appropriate cell size**: 0.1m is good balance
2. **Process every Nth scan**: Use `step=10` for 10x speedup
3. **Limit initial grid size**: Start with 100-200 cells
4. **Let grid grow organically**: Don't pre-allocate huge grid
5. **Reuse mapper object**: Don't recreate for each scan

---

**🎉 You're ready to start!**

Begin with: `python example_simple.py`
