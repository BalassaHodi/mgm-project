"""
Created by: Balassa Hodi
Description: For a given coordinate of a point (X, Y) we determine which cell does it belong to in an (n x m) grid map.
Note that the algorithm is a self-developed work, so it might not be the most efficient one.
"""

from classes.cell import Cell


def determine_cell_index(X: float, Y: float, n: int, m: int, w: float):
    """
    The inputs are the global coordinates of the point (X, Y), the dimensions of the grid map (n, m), and the width of one cell (w). \n
    The output is the number of row and column in an object with properties row and column. \n
    ---
    Note:
    <ul>
        <li> The origin of the global coordinate system is at the bottom left of the grid map, and the indexes increase. </li>
        <li> The indexes of the first cell is (1,1) NOT (0,0). </li>
        <li> If a point is on the edge of a cell, it belongs to the cell with the smaller index. </li>
    </ul>
    """

    row, column = 0, 0

    # Calculate row index
    for i in range(n + 1):
        Ydiff = w * i - Y
        if 0 <= Ydiff and Ydiff < w:
            row = i
            break

    # Calculate column index
    for i in range(m + 1):
        Xdiff = w * i - X
        if 0 <= Xdiff and Xdiff < w:
            column = i
            break

    if row == 0 or column == 0:
        print(f"\033[31mThe coordinates ({X}, {Y}) are out of the border.\033[0m")
        return Cell(0, 0)

    return Cell(row, column)
