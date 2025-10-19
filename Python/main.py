"""
This is the main function for debugging and testing.
The functions are determined in the other files in the folder.
"""

import timeit
from bresenham import bresenham
from determine_cell_index import determine_cell_index


def main(args=None):

    # #############
    # # bresenham debug
    # x0, y0, x1, y1 = 0, 0, 10, 10
    # points = list(bresenham(x0, y0, x1, y1))
    # print(points)
    # #############

    #############
    # determine_cell_index debug
    X = 120.15
    Y = 160.38
    w = 15.0
    n = 19
    m = 24

    startTime = timeit.default_timer() * 1000.0
    cell = determine_cell_index(X, Y, n, m, w)
    stopTime = timeit.default_timer() * 1000.0

    print(
        f"The coordinates ({X}, {Y}) belong to the cell ({cell.row}, {cell.column}). \nThe runtime of the code: {stopTime - startTime} ms."
    )
    #############


if __name__ == "__main__":
    main()
