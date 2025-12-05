"""
Simple example demonstrating the basic usage of the improved occupancy grid mapper.

This script shows the minimal code needed to:
1. Load lidar data
2. Process it with the improved algorithm
3. Get the probability matrix output
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from shared.read_json import read_json
from main_improved import process_lidar_data_improved
import numpy as np


def simple_example():
    """
    Minimal example of using the improved occupancy grid mapper.
    """

    print("=" * 60)
    print("SIMPLE EXAMPLE: Improved Occupancy Grid Mapper")
    print("=" * 60)

    # Step 1: Load your lidar data
    print("\n[1/4] Loading lidar data...")
    lidar_data_list = read_json()
    print(f"✓ Loaded {len(lidar_data_list)} scans")

    # Step 2: Process the data
    print("\n[2/4] Processing lidar data...")
    print("      (This may take a moment...)")

    mapper = process_lidar_data_improved(
        lidar_data_list=lidar_data_list,
        cell_width=0.1,  # 10cm cells
        step=10,  # Process every 10th position for speed
        verbose=False,  # Suppress detailed output
    )
    print("✓ Processing complete")

    # Step 3: Get the probability matrix
    print("\n[3/4] Extracting probability matrix...")

    # Option A: Just the probability matrix
    prob_matrix = mapper.get_probability_grid()
    print(f"✓ Probability matrix shape: {prob_matrix.shape}")
    print(f"  Range: [{prob_matrix.min():.4f}, {prob_matrix.max():.4f}]")

    # Option B: Probability matrix with metadata
    grid_data = mapper.get_probability_grid_with_bounds()
    print(f"\n  Grid bounds:")
    print(f"    X: [{grid_data['min_x']:.2f}, {grid_data['max_x']:.2f}] m")
    print(f"    Y: [{grid_data['min_y']:.2f}, {grid_data['max_y']:.2f}] m")
    print(f"  Origin: ({grid_data['origin_x']:.2f}, {grid_data['origin_y']:.2f})")

    # Step 4: Use the probability matrix
    print("\n[4/4] Using the probability matrix...")

    # Count different cell types
    free_cells = np.sum(prob_matrix < 0.3)
    occupied_cells = np.sum(prob_matrix > 0.7)
    unknown_cells = np.sum((prob_matrix >= 0.3) & (prob_matrix <= 0.7))
    total_cells = prob_matrix.size

    print(f"\n  Cell statistics:")
    print(f"    Total cells: {total_cells:,}")
    print(f"    Free:        {free_cells:,} ({100*free_cells/total_cells:.1f}%)")
    print(
        f"    Occupied:    {occupied_cells:,} ({100*occupied_cells/total_cells:.1f}%)"
    )
    print(f"    Unknown:     {unknown_cells:,} ({100*unknown_cells/total_cells:.1f}%)")

    # Example: Check occupancy at specific location
    print(f"\n  Example queries:")

    # Check at origin (first robot position)
    origin_row, origin_col = mapper.world_to_grid(
        grid_data["origin_x"], grid_data["origin_y"]
    )
    if origin_row is not None:
        prob = prob_matrix[origin_row, origin_col]
        print(f"    Occupancy at origin: {prob:.4f} (should be free)")

    # Check 1 meter ahead
    test_x = grid_data["origin_x"] + 1.0
    test_y = grid_data["origin_y"]
    result = mapper.world_to_grid(test_x, test_y)
    if result is not None:
        row, col = result
        prob = prob_matrix[row, col]
        status = "FREE" if prob < 0.3 else ("OCCUPIED" if prob > 0.7 else "UNKNOWN")
        print(f"    Occupancy at (1.0, 0.0): {prob:.4f} [{status}]")

    print("\n" + "=" * 60)
    print("✓ EXAMPLE COMPLETE")
    print("=" * 60)
    print("\nYou now have a probability matrix that you can use for:")
    print("  • Path planning")
    print("  • Navigation")
    print("  • Obstacle avoidance")
    print("  • Map visualization")
    print("  • Any other robotics application!")
    print("=" * 60)

    return mapper, prob_matrix


def get_occupancy_at_position(mapper, prob_matrix, x, y):
    """
    Helper function to check occupancy probability at a specific world coordinate.

    Parameters:
    -----------
    mapper : OccupancyGridMapper
        The occupancy grid mapper object
    prob_matrix : np.ndarray
        The probability matrix from mapper.get_probability_grid()
    x : float
        World X coordinate
    y : float
        World Y coordinate

    Returns:
    --------
    float or None
        Occupancy probability (0.0 to 1.0), or None if out of bounds
    """
    result = mapper.world_to_grid(x, y)
    if result is None:
        return None

    row, col = result
    return prob_matrix[row, col]


def find_free_cells(prob_matrix, threshold=0.3):
    """
    Get indices of all free cells.

    Parameters:
    -----------
    prob_matrix : np.ndarray
        The probability matrix
    threshold : float
        Maximum probability to consider a cell as free (default: 0.3)

    Returns:
    --------
    tuple
        (rows, cols) arrays of free cell indices
    """
    return np.where(prob_matrix < threshold)


def find_occupied_cells(prob_matrix, threshold=0.7):
    """
    Get indices of all occupied cells.

    Parameters:
    -----------
    prob_matrix : np.ndarray
        The probability matrix
    threshold : float
        Minimum probability to consider a cell as occupied (default: 0.7)

    Returns:
    --------
    tuple
        (rows, cols) arrays of occupied cell indices
    """
    return np.where(prob_matrix > threshold)


if __name__ == "__main__":
    # Run the simple example
    mapper, prob_matrix = simple_example()

    # Additional examples of using the output
    print("\n" + "=" * 60)
    print("ADDITIONAL HELPER FUNCTIONS")
    print("=" * 60)

    # Find free cells
    free_rows, free_cols = find_free_cells(prob_matrix)
    print(f"\nFound {len(free_rows)} free cells")

    # Find occupied cells
    occ_rows, occ_cols = find_occupied_cells(prob_matrix)
    print(f"Found {len(occ_rows)} occupied cells")

    # Check specific position
    grid_data = mapper.get_probability_grid_with_bounds()
    test_x = grid_data["origin_x"] + 0.5
    test_y = grid_data["origin_y"] - 0.3
    prob = get_occupancy_at_position(mapper, prob_matrix, test_x, test_y)
    if prob is not None:
        print(f"\nOccupancy at ({test_x:.2f}, {test_y:.2f}): {prob:.4f}")
    else:
        print(f"\nPosition ({test_x:.2f}, {test_y:.2f}) is out of bounds")

    print("\n" + "=" * 60)
