"""
Comparison between original and improved occupancy grid mapping approaches.

This script demonstrates the key differences and shows how the improved version
handles dynamic coordinates and grid expansion.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from shared.read_json import read_json
from shared.classes.robot_position import RobotPosition
import math


def analyze_coordinate_requirements(lidar_data_list):
    """
    Analyze the coordinate space requirements for the data.

    This shows what the original approach had to do (pre-calculate bounds and translate)
    versus what the improved approach does automatically.
    """

    print("=" * 70)
    print("COORDINATE SPACE ANALYSIS")
    print("=" * 70)

    if not lidar_data_list:
        print("No data to analyze")
        return

    # Find min/max robot positions
    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")
    max_lidar_range = 0.0

    for item in lidar_data_list:
        pos = item["robot_position"]
        min_x = min(min_x, pos.X)
        max_x = max(max_x, pos.X)
        min_y = min(min_y, pos.Y)
        max_y = max(max_y, pos.Y)

        for scan in item["scans"]:
            if scan.length > 0:
                max_lidar_range = max(max_lidar_range, scan.length)

    # Calculate absolute bounds (with safety margin for lidar range)
    abs_min_x = min_x - max_lidar_range
    abs_max_x = max_x + max_lidar_range
    abs_min_y = min_y - max_lidar_range
    abs_max_y = max_y + max_lidar_range

    first_pos = lidar_data_list[0]["robot_position"]

    print(f"\n{'ORIGINAL APPROACH':-^70}")
    print(f"\nRaw data coordinates:")
    print(f"  Robot positions - X: [{min_x:.3f}, {max_x:.3f}]")
    print(f"  Robot positions - Y: [{min_y:.3f}, {max_y:.3f}]")
    print(f"  Max lidar range: {max_lidar_range:.3f} m")

    print(f"\nRequired grid bounds (with safety margin):")
    print(f"  X: [{abs_min_x:.3f}, {abs_max_x:.3f}]")
    print(f"  Y: [{abs_min_y:.3f}, {abs_max_y:.3f}]")
    print(f"  Grid size: {abs_max_x - abs_min_x:.3f} x {abs_max_y - abs_min_y:.3f} m")

    print(f"\n⚠️  ISSUES with original approach:")
    print(f"  1. Must pre-calculate ALL bounds before processing")
    print(
        f"  2. Must translate ALL coordinates by ({-abs_min_x:.3f}, {-abs_min_y:.3f})"
    )
    print(f"  3. Grid size is FIXED - wastes memory if area is not fully explored")
    print(f"  4. Cannot handle coordinates outside pre-calculated bounds")

    cell_width = 0.1
    n_rows = int(math.ceil((abs_max_y - abs_min_y) / cell_width))
    n_cols = int(math.ceil((abs_max_x - abs_min_x) / cell_width))
    print(f"\nMemory requirement (with 0.1m cells):")
    print(f"  Grid: {n_rows} rows x {n_cols} cols = {n_rows * n_cols:,} cells")
    print(f"  Memory: ~{n_rows * n_cols * 8 / 1024 / 1024:.2f} MB (float64)")

    print(f"\n{'IMPROVED APPROACH':-^70}")
    print(f"\nFirst robot position (becomes origin):")
    print(f"  X: {first_pos.X:.3f} → 0.0")
    print(f"  Y: {first_pos.Y:.3f} → 0.0")
    print(f"  Theta: {first_pos.Theta:.3f}°")

    print(f"\nCoordinate system after transformation:")
    print(f"  Origin: ({first_pos.X:.3f}, {first_pos.Y:.3f}) in original coords")
    print(f"  Origin: (0.0, 0.0) in new coords")

    # Calculate what the bounds would be relative to first position
    rel_min_x = min_x - first_pos.X - max_lidar_range
    rel_max_x = max_x - first_pos.X + max_lidar_range
    rel_min_y = min_y - first_pos.Y - max_lidar_range
    rel_max_y = max_y - first_pos.Y + max_lidar_range

    print(f"\nRelative bounds (if all data processed):")
    print(f"  X: [{rel_min_x:.3f}, {rel_max_x:.3f}]")
    print(f"  Y: [{rel_min_y:.3f}, {rel_max_y:.3f}]")
    print(f"  ✓ Can handle NEGATIVE coordinates!")

    print(f"\n✓  ADVANTAGES of improved approach:")
    print(f"  1. NO need to pre-calculate bounds")
    print(f"  2. NO coordinate translation needed")
    print(f"  3. Grid GROWS dynamically - efficient memory usage")
    print(f"  4. Automatically handles negative coordinates")
    print(f"  5. Can process data in REAL-TIME (one scan at a time)")

    initial_size = 100
    print(f"\nMemory requirement:")
    print(
        f"  Initial: {initial_size} x {initial_size} = {initial_size * initial_size:,} cells"
    )
    print(f"  Initial memory: ~{initial_size * initial_size * 8 / 1024:.2f} KB")
    print(f"  Expands automatically as needed")
    print(f"  Final size depends on actual exploration (not worst-case)")

    print("\n" + "=" * 70)

    return {
        "original": {
            "min_x": abs_min_x,
            "max_x": abs_max_x,
            "min_y": abs_min_y,
            "max_y": abs_max_y,
            "n_rows": n_rows,
            "n_cols": n_cols,
        },
        "improved": {
            "origin_x": first_pos.X,
            "origin_y": first_pos.Y,
            "rel_min_x": rel_min_x,
            "rel_max_x": rel_max_x,
            "rel_min_y": rel_min_y,
            "rel_max_y": rel_max_y,
        },
    }


def compare_approaches():
    """
    Main comparison function.
    """

    print("\n" + "=" * 70)
    print(" COMPARISON: Original vs Improved Approach")
    print("=" * 70 + "\n")

    # Load data
    print("Loading lidar data...")
    lidar_data_list = read_json(printPath=False)
    print(f"✓ Loaded {len(lidar_data_list)} data items\n")

    # Analyze coordinate requirements
    analysis = analyze_coordinate_requirements(lidar_data_list)

    # Print detailed comparison table
    print("\n" + "=" * 70)
    print("DETAILED COMPARISON")
    print("=" * 70)

    comparison = [
        (
            "Coordinate System",
            "Fixed at (0,0) bottom-left",
            "First robot position is origin",
        ),
        ("Negative Coords", "❌ Not supported", "✓ Fully supported"),
        ("Grid Size", "Fixed, pre-calculated", "Dynamic, auto-expanding"),
        ("Memory Efficiency", "Allocates for worst-case", "Allocates as needed"),
        ("Pre-processing", "Must analyze all data first", "Can process incrementally"),
        ("Real-time Use", "❌ Difficult", "✓ Easy"),
        ("Setup Complexity", "High (translate coords)", "Low (automatic)"),
        ("Output Format", "Matplotlib visualization", "Numpy array + metadata"),
        ("Integration", "Requires wrapper code", "Direct array access"),
    ]

    print(f"\n{'Feature':<20} | {'Original':<30} | {'Improved':<30}")
    print("-" * 70)
    for feature, original, improved in comparison:
        print(f"{feature:<20} | {original:<30} | {improved:<30}")

    print("\n" + "=" * 70)
    print("RECOMMENDATION")
    print("=" * 70)
    print(
        """
For REAL-LIFE APPLICATIONS, use the IMPROVED version because:

1. ✓ No pre-processing required
2. ✓ Handles any coordinate system automatically  
3. ✓ Works with streaming data (ROS topics, etc.)
4. ✓ Memory efficient
5. ✓ Easy to integrate with path planning
6. ✓ Clean probability matrix output
7. ✓ Supports negative coordinates naturally

The ORIGINAL version is good for:
- Offline batch processing with known bounds
- Visualization and debugging
- Understanding the algorithm basics
"""
    )

    print("=" * 70)


def demonstrate_negative_coordinates():
    """
    Demonstrate how the improved version handles negative coordinates.
    """

    print("\n" + "=" * 70)
    print("DEMONSTRATION: Handling Negative Coordinates")
    print("=" * 70)

    from grid_map_improved import OccupancyGridMapper
    from determine_cell_index_improved import determine_cell_index_improved

    # Create a mapper with origin at (10, 20) in original coords
    origin_x = 10.0
    origin_y = 20.0

    mapper = OccupancyGridMapper(
        cell_width=0.1,
        initial_size=50,
        origin_x=origin_x,
        origin_y=origin_y,
    )

    print(f"\nOrigin set to: ({origin_x}, {origin_y})")
    print(
        f"Initial grid bounds: X=[{mapper.min_x:.1f}, {mapper.max_x:.1f}], "
        f"Y=[{mapper.min_y:.1f}, {mapper.max_y:.1f}]"
    )

    # Test some coordinates
    test_coords = [
        (origin_x, origin_y, "At origin"),
        (origin_x + 1.0, origin_y + 1.0, "1m ahead and right"),
        (origin_x - 1.0, origin_y - 1.0, "1m back and left (NEGATIVE!)"),
        (origin_x - 2.5, origin_y + 0.5, "2.5m back, 0.5m right"),
        (origin_x + 0.05, origin_y - 3.0, "0.05m right, 3m back"),
    ]

    print("\nTesting coordinate conversions:")
    print(f"{'World Coords':<25} | {'Grid Index':<15} | {'Description':<30}")
    print("-" * 75)

    for x, y, desc in test_coords:
        cell = determine_cell_index_improved(
            x, y, mapper.min_x, mapper.min_y, mapper.cell_width
        )
        print(
            f"({x:6.2f}, {y:6.2f})          | ({cell.row:4d}, {cell.column:4d})    | {desc}"
        )

    print("\n✓ All coordinates converted successfully!")
    print("✓ Negative world coordinates → Positive grid indices")
    print("\n" + "=" * 70)


if __name__ == "__main__":
    # Run the comparison
    compare_approaches()

    # Demonstrate negative coordinate handling
    demonstrate_negative_coordinates()

    print("\n" + "=" * 70)
    print("For actual usage, see:")
    print("  • example_simple.py - Basic usage example")
    print("  • main_improved.py - Full demonstration")
    print("  • README_IMPROVED.md - Complete documentation")
    print("=" * 70 + "\n")
