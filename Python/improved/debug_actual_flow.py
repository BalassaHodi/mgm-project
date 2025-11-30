"""
Debug script to simulate the actual processing flow and check for bugs
"""

import numpy as np
from grid_map_improved import OccupancyGridMapper


def test_actual_processing_flow():
    """Simulate the actual processing flow to find the bug"""

    print("=" * 60)
    print("SIMULATING ACTUAL PROCESSING FLOW")
    print("=" * 60)

    # Create mapper
    mapper = OccupancyGridMapper(
        origin_x=0.0, origin_y=0.0, cell_width=0.1, initial_size=100
    )

    print(f"\nInitial grid:")
    print(f"  min_x={mapper.min_x}, max_x={mapper.max_x}")
    print(f"  min_y={mapper.min_y}, max_y={mapper.max_y}")
    print(f"  size: {mapper.n_rows} x {mapper.n_cols}")

    # Mark a specific cell as occupied (in world coordinates x=2.0, y=1.5)
    test_x = 2.0
    test_y = 1.5

    # Calculate indices BEFORE expansion
    from determine_cell_index_improved import determine_cell_index_improved

    test_cell_before = determine_cell_index_improved(
        test_x, test_y, mapper.min_x, mapper.min_y, mapper.cell_width
    )

    print(f"\n--- BEFORE EXPANSION ---")
    print(f"Test point: ({test_x}, {test_y})")
    print(f"  Cell indices: row={test_cell_before.row}, col={test_cell_before.column}")

    # Update this cell
    mapper.log_odds_grid[test_cell_before.row, test_cell_before.column] = (
        10.0  # Strong occupied
    )

    # Verify the cell was set
    print(
        f"  Log-odds value: {mapper.log_odds_grid[test_cell_before.row, test_cell_before.column]}"
    )

    # Get probability
    prob_before = mapper.get_probability_grid()[
        test_cell_before.row, test_cell_before.column
    ]
    print(f"  Probability: {prob_before:.4f}")

    # Now simulate an expansion (e.g., robot moves to x=-6.0, y=-5.0)
    print(f"\n--- TRIGGERING EXPANSION ---")
    new_points_x = [-6.0, -5.0, 2.0]  # Include the test point
    new_points_y = [-5.0, -4.0, 1.5]

    mapper._expand_grid_if_needed(new_points_x, new_points_y)

    print(f"\nAfter expansion:")
    print(f"  min_x={mapper.min_x}, max_x={mapper.max_x}")
    print(f"  min_y={mapper.min_y}, max_y={mapper.max_y}")
    print(f"  size: {mapper.n_rows} x {mapper.n_cols}")

    # Calculate indices AFTER expansion for the same world point
    test_cell_after = determine_cell_index_improved(
        test_x, test_y, mapper.min_x, mapper.min_y, mapper.cell_width
    )

    print(f"\n--- AFTER EXPANSION ---")
    print(f"Test point: ({test_x}, {test_y}) [same point]")
    print(f"  Cell indices: row={test_cell_after.row}, col={test_cell_after.column}")

    # Check if the log-odds value was preserved
    log_odds_after = mapper.log_odds_grid[test_cell_after.row, test_cell_after.column]
    print(f"  Log-odds value: {log_odds_after}")

    if abs(log_odds_after - 10.0) < 0.001:
        print(f"  ✓ Cell data preserved correctly!")
    else:
        print(f"  ✗ BUG: Cell data NOT preserved!")
        print(f"     Expected: 10.0, Got: {log_odds_after}")

        # Check what happened to the old indices
        if (
            test_cell_before.row < mapper.n_rows
            and test_cell_before.column < mapper.n_cols
        ):
            value_at_old_indices = mapper.log_odds_grid[
                test_cell_before.row, test_cell_before.column
            ]
            print(
                f"     Value at old indices ({test_cell_before.row}, {test_cell_before.column}): {value_at_old_indices}"
            )

    # Get probability
    prob_after = mapper.get_probability_grid()[
        test_cell_after.row, test_cell_after.column
    ]
    print(f"  Probability: {prob_after:.4f}")

    # Additional check: scan the entire grid to find where the value 10.0 ended up
    print(f"\n--- SCANNING FOR VALUE 10.0 ---")
    occupied_cells = np.where(mapper.log_odds_grid > 9.0)
    if len(occupied_cells[0]) > 0:
        for i in range(len(occupied_cells[0])):
            row = occupied_cells[0][i]
            col = occupied_cells[1][i]
            value = mapper.log_odds_grid[row, col]
            # Convert back to world coordinates
            world_x = mapper.min_x + (col + 0.5) * mapper.cell_width
            world_y = mapper.min_y + (row + 0.5) * mapper.cell_width
            print(
                f"  Found at grid[{row}, {col}] = {value:.2f}, world coords: ({world_x:.3f}, {world_y:.3f})"
            )
    else:
        print(f"  ⚠️  Value 10.0 not found anywhere in the grid!")


if __name__ == "__main__":
    test_actual_processing_flow()
