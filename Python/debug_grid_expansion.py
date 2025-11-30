"""
Debug script to test grid expansion logic
"""

import numpy as np


def test_cell_mapping():
    """Test that cells maintain their world coordinates after grid expansion"""

    cell_width = 0.1

    # Old grid
    old_min_x = -5.0
    old_min_y = -5.0
    old_n_cols = 100
    old_n_rows = 100
    old_max_x = old_min_x + old_n_cols * cell_width  # -5.0 + 10.0 = 5.0
    old_max_y = old_min_y + old_n_rows * cell_width  # -5.0 + 10.0 = 5.0

    print("OLD GRID:")
    print(f"  Bounds: X=[{old_min_x}, {old_max_x}], Y=[{old_min_y}, {old_max_y}]")
    print(f"  Size: {old_n_rows} x {old_n_cols}")

    # New grid (expanded to the left and bottom)
    new_min_x = -7.0
    new_min_y = -6.0
    new_max_x = 5.0
    new_max_y = 5.0
    new_n_cols = int(np.ceil((new_max_x - new_min_x) / cell_width))  # 120
    new_n_rows = int(np.ceil((new_max_y - new_min_y) / cell_width))  # 110

    print("\nNEW GRID:")
    print(f"  Bounds: X=[{new_min_x}, {new_max_x}], Y=[{new_min_y}, {new_max_y}]")
    print(f"  Size: {new_n_rows} x {new_n_cols}")

    # Test point: x=3.0, y=2.0 (should be in both grids)
    test_x = 3.0
    test_y = 2.0

    # Calculate indices in old grid
    old_col = int((test_x - old_min_x) // cell_width)
    old_row = int((test_y - old_min_y) // cell_width)

    print(f"\nTEST POINT: ({test_x}, {test_y})")
    print(f"  Old grid indices: row={old_row}, col={old_col}")

    # Calculate indices in new grid
    new_col = int((test_x - new_min_x) // cell_width)
    new_row = int((test_y - new_min_y) // cell_width)

    print(f"  New grid indices: row={new_row}, col={new_col}")

    # Verify cell centers
    old_center_x = old_min_x + (old_col + 0.5) * cell_width
    old_center_y = old_min_y + (old_row + 0.5) * cell_width

    new_center_x = new_min_x + (new_col + 0.5) * cell_width
    new_center_y = new_min_y + (new_row + 0.5) * cell_width

    print(f"  Old cell center: ({old_center_x}, {old_center_y})")
    print(f"  New cell center: ({new_center_x}, {new_center_y})")
    print(
        f"  Centers match: {abs(old_center_x - new_center_x) < 0.001 and abs(old_center_y - new_center_y) < 0.001}"
    )

    # Now test the overlap calculation
    print("\n" + "=" * 60)
    print("TESTING OVERLAP CALCULATION")
    print("=" * 60)

    # Find overlapping region in world coordinates
    overlap_min_x = max(old_min_x, new_min_x)
    overlap_max_x = min(old_max_x, new_max_x)
    overlap_min_y = max(old_min_y, new_min_y)
    overlap_max_y = min(old_max_y, new_max_y)

    print(
        f"\nOverlap region: X=[{overlap_min_x}, {overlap_max_x}], Y=[{overlap_min_y}, {overlap_max_y}]"
    )

    # Calculate indices in old grid (using CURRENT CODE logic)
    import math

    old_col_start = int(math.floor((overlap_min_x - old_min_x) / cell_width))
    old_col_end = int(math.ceil((overlap_max_x - old_min_x) / cell_width))
    old_row_start = int(math.floor((overlap_min_y - old_min_y) / cell_width))
    old_row_end = int(math.ceil((overlap_max_y - old_min_y) / cell_width))

    print(f"\nOld grid overlap indices:")
    print(f"  Rows: [{old_row_start}, {old_row_end})")
    print(f"  Cols: [{old_col_start}, {old_col_end})")

    # Calculate indices in new grid (using CURRENT CODE logic)
    new_col_start = int(math.floor((overlap_min_x - new_min_x) / cell_width))
    new_col_end = int(math.ceil((overlap_max_x - new_min_x) / cell_width))
    new_row_start = int(math.floor((overlap_min_y - new_min_y) / cell_width))
    new_row_end = int(math.ceil((overlap_max_y - new_min_y) / cell_width))

    print(f"\nNew grid overlap indices:")
    print(f"  Rows: [{new_row_start}, {new_row_end})")
    print(f"  Cols: [{new_col_start}, {new_col_end})")

    # Check sizes match
    old_overlap_rows = old_row_end - old_row_start
    old_overlap_cols = old_col_end - old_col_start
    new_overlap_rows = new_row_end - new_row_start
    new_overlap_cols = new_col_end - new_col_start

    print(f"\nOld overlap size: {old_overlap_rows} x {old_overlap_cols}")
    print(f"New overlap size: {new_overlap_rows} x {new_overlap_cols}")
    print(
        f"Sizes match: {old_overlap_rows == new_overlap_rows and old_overlap_cols == new_overlap_cols}"
    )

    # Now verify that the cell at (old_row, old_col) would be copied to (new_row, new_col)
    print("\n" + "=" * 60)
    print("TESTING CELL COPY MAPPING")
    print("=" * 60)

    if (
        old_row_start <= old_row < old_row_end
        and old_col_start <= old_col < old_col_end
    ):
        # This cell is in the overlap region
        offset_row = old_row - old_row_start
        offset_col = old_col - old_col_start

        target_row = new_row_start + offset_row
        target_col = new_col_start + offset_col

        print(f"\nCell at old indices ({old_row}, {old_col}) would be copied to:")
        print(f"  New indices: ({target_row}, {target_col})")
        print(f"  Expected indices: ({new_row}, {new_col})")
        print(f"  Mapping correct: {target_row == new_row and target_col == new_col}")

        if target_row != new_row or target_col != new_col:
            print(f"\n⚠️  BUG FOUND! Cell mapping is incorrect!")
            print(
                f"  Difference: row_diff={target_row - new_row}, col_diff={target_col - new_col}"
            )
    else:
        print(f"\nTest cell is not in overlap region")


if __name__ == "__main__":
    test_cell_mapping()
