"""
This is the main function for debugging and testing.
The functions are determined in the other files in the folder.
"""

import timeit

from determine_cell_index import determine_cell_index
from ray_tracing import ray_tracing
from read_json import read_json

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
    # # ray_tracing debug
    # startTime = timeit.default_timer() * 1000.0
    # freeCells, endCell = ray_tracing(
    #     RobotPosition(X=7.78, Y=9.32, Theta=155.0),
    #     LidarData(alpha=200.0, length=100.0),
    #     n=15,
    #     m=110,
    #     w=1.0,
    # )
    # stopTime = timeit.default_timer() * 1000.0

    # print("ray_tracing debug:")
    # print("The free cells are:")
    # for cell in freeCells:
    #     print(f"({cell.row}, {cell.column})", end=" ")

    # print()

    # print(f"The last occupied cell: ({endCell.row}, {endCell.column})")

    # print()

    # print("The total number of free cells:", len(freeCells))
    # print(f"The runtime of the ray tracing code: {stopTime - startTime} ms.")

    # print()
    #############

    #############
    # # read_json debug
    # startTime = timeit.default_timer() * 1000.0
    # lidarDataList = read_json()
    # stopTime = timeit.default_timer() * 1000.0

    # print("read_json debug:")
    # print(
    #     f"The total number of data items read from the json file: {len(lidarDataList)}"
    # )
    # print(f"The runtime of the read_json code: {stopTime - startTime} ms.")

    # print()
    #############

    #############
    # debug ray_tracing with real data from json
    print("ray_tracing debug with real data:")

    lidarDataList = read_json()
    dataNumber = 0
    robotPosition = lidarDataList[dataNumber]["robot_position"]

    # the json file contains X-Y coordinates that are negative or too small (so probably the coordinate system is not set to the left-bottom corner)
    # that's why the coordinates are modified here for testing purpose (in order not the get an error)
    robotPosition.X = 5.0
    robotPosition.Y = 5.0

    print(
        f"Robot position: X={robotPosition.X}, Y={robotPosition.Y}, Theta={robotPosition.Theta}"
    )

    startTime = timeit.default_timer() * 1000.0

    for i in range(len(lidarDataList[dataNumber]["scans"])):
        lidarData = lidarDataList[dataNumber]["scans"][i]

        # here the n, m, w values are set for testing purpose, they should be determined according to the grid map
        freeCells, endCell = ray_tracing(robotPosition, lidarData, n=500, m=500, w=0.1)

        # # this part is commented out to measure only the runtime of the ray tracing function
        # print("The free cells are:")
        # for cell in freeCells:
        #     print(f"({cell.row}, {cell.column})", end=" ")

        # print()

        # print(f"The last occupied cell: ({endCell.row}, {endCell.column})")

        # print()

        # print("The total number of free cells:", len(freeCells))

        # print()

    stopTime = timeit.default_timer() * 1000.0

    print(
        f"The runtime of the ray tracing code for {len(lidarDataList[dataNumber]['scans'])} data: {stopTime - startTime} ms."
    )
    #############


if __name__ == "__main__":
    main()
