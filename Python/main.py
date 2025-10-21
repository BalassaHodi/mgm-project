"""
This is the main function for debugging and testing.
The functions are determined in the other files in the folder.
"""

import timeit
from determine_cell_index import determine_cell_index
from ray_tracing import ray_tracing
from classes.robot_position import RobotPosition
from classes.cell import Cell
from classes.lidar_sensor_data import LidarData


def main(args=None):

    #############
    # determine_cell_index debug
    # X = 120.15
    # Y = 160.38
    # w = 15.0
    # n = 19
    # m = 24

    # startTime = timeit.default_timer() * 1000.0
    # cell = determine_cell_index(X, Y, n, m, w)
    # stopTime = timeit.default_timer() * 1000.0

    # print("determine_cell_index debug:")

    # print(
    #     f"The coordinates ({X}, {Y}) belong to the cell ({cell.row}, {cell.column}). \nThe runtime of the code: {stopTime - startTime} ms."
    # )

    # print()
    #############

    #############
    # ray_tracing debug
    startTime = timeit.default_timer() * 1000.0
    freeCells, endCell = ray_tracing(
        RobotPosition(X=7.78, Y=9.32, Theta=155.0),
        LidarData(alpha=200.0, length=100.0),
        n=15,
        m=110,
        w=1.0,
    )
    stopTime = timeit.default_timer() * 1000.0

    print("ray_tracing debug:")
    print("The free cells are:")
    for cell in freeCells:
        print(f"({cell.row}, {cell.column})", end=" ")

    print()

    print(f"The last occupied cell: ({endCell.row}, {endCell.column})")

    print()

    print("The total number of free cells:", len(freeCells))
    print(f"The runtime of the ray tracing code: {stopTime - startTime} ms.")

    print()
    #############


if __name__ == "__main__":
    main()
