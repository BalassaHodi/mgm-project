"""
Created by: Balassa Hodi
Improved version that works with dynamic grid bounds and negative coordinates.
Description: For a given coordinate of a point (X, Y) we determine which cell does it belong to
in a dynamically sized grid map that can have negative coordinates.
Note that the algorithm is a self-developed work, so it might not be the most efficient one.
"""

from classes.cell import Cell


def determine_cell_index_improved(
    X: float,
    Y: float,
    min_x: float,
    min_y: float,
    w: float,
    n: int = None,
    m: int = None,
    check_bounds: bool = False,
):
    """
    Determines which cell a point (X, Y) belongs to in a grid map with dynamic bounds.

    The inputs are the global coordinates of the point (X, Y), the minimum X and Y coordinates
    of the grid (min_x, min_y), and the width of one cell (w).

    The output is the number of row and column in a Cell object with properties row and column.

    Parameters:
    -----------
    X : float
        Global X coordinate of the point
    Y : float
        Global Y coordinate of the point
    min_x : float
        Minimum X coordinate of the grid (bottom-left corner X)
    min_y : float
        Minimum Y coordinate of the grid (bottom-left corner Y)
    w : float
        Width (and height) of each cell
    n : int, optional
        Number of rows in the grid (for bounds checking)
    m : int, optional
        Number of columns in the grid (for bounds checking)
    check_bounds : bool, optional
        Whether to check if the point is within grid bounds (default: False)

    Returns:
    --------
    Cell
        Cell object with row and column indices (0-indexed)

    Notes:
    ------
    - The grid can handle negative global coordinates
    - Cell indices are always non-negative (0, 1, 2, ...)
    - The origin of the grid is at (min_x, min_y)
    - If a point is on the edge of a cell, it belongs to the cell with the smaller index
    """

    # Translate coordinates to grid-local coordinates (relative to min_x, min_y)
    local_x = X - min_x
    local_y = Y - min_y

    # Calculate cell indices using floor division
    # This works correctly for both positive and negative local coordinates
    column = int(local_x // w)
    row = int(local_y // w)

    # Check if indices are valid (non-negative)
    if row < 0 or column < 0:
        if check_bounds:
            print(
                f"\033[33mWarning: The coordinates ({X}, {Y}) result in negative indices: row={row}, col={column}\033[0m"
            )
            print(f"Grid minimum is ({min_x}, {min_y}), cell width is {w}")
        return Cell(max(0, row), max(0, column))

    # Optional: Check upper bounds
    if check_bounds and n is not None and m is not None:
        if row >= n or column >= m:
            print(
                f"\033[31mThe coordinates ({X}, {Y}) are out of the upper border.\033[0m"
            )
            print(f"Grid dimensions: {n} rows x {m} columns")
            print(f"Calculated indices: row={row}, col={column}")
            return Cell(min(row, n - 1), min(column, m - 1))

    return Cell(row, column)
