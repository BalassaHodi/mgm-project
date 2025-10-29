"""
This is the main function for debugging and testing.
The functions are determined in the other files in the folder.
"""

import math
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
    # check the maximum and minimum positions of the robot and the maximum lidar data from the json file in order to translate the coordinate system
    lidarDataList = read_json(printPath=True)
    minX, maxX = float("inf"), float("-inf")
    minY, maxY = float("inf"), float("-inf")
    maxLidarLength = float("-inf")
    minLidarLength = float("inf")

    for item in lidarDataList:
        robotPosition = item["robot_position"]
        minX = robotPosition.X if minX > robotPosition.X else minX
        maxX = robotPosition.X if maxX < robotPosition.X else maxX
        minY = robotPosition.Y if minY > robotPosition.Y else minY
        maxY = robotPosition.Y if maxY < robotPosition.Y else maxY

        for lidarData in item["scans"]:
            maxLidarLength = (
                lidarData.length
                if maxLidarLength < lidarData.length
                else maxLidarLength
            )
            minLidarLength = (
                lidarData.length
                if minLidarLength > lidarData.length
                else minLidarLength
            )
    print(
        f"The minimum position: ({minX}, {minY}); The maximum position: ({maxX}, {maxY}); The maximum lidar length: {maxLidarLength}; The minimum lidar length: {minLidarLength}"
    )

    # from these values, we have to calculate the absolute minimum and maximum coordinates that the grid map has to cover
    # this calculation is a safe calculation, because the maximum lidar length may not been measured at the minimum or maximum robot position
    # there may be more efficient alorithms to calculate the true absolute minimum and maximum coordinates based on the related robot positions and lidar lengths
    absoluteMinX = minX - maxLidarLength
    absoluteMaxX = maxX + maxLidarLength
    absoluteMinY = minY - maxLidarLength
    absoluteMaxY = maxY + maxLidarLength

    print(
        f"The absolute minimum coordinates: ({absoluteMinX}, {absoluteMinY}); The absolute maximum coordinates: ({absoluteMaxX}, {absoluteMaxY})"
    )

    # so we have to translate the origin of the coordinate system by (-absoluteMinX, -absoluteMinY), that means that we have to add the absolute values of them to the robotPosition
    #  and then calculate the width and height of the grid map based on the absolute maximum coordinates
    gridMapWidth = math.ceil(absoluteMaxX + abs(absoluteMinX))
    gridMapHeight = math.ceil(absoluteMaxY + abs(absoluteMinY))

    # calculate n (rows) and m (columns) based on the w value
    cellWidth = 0.1
    gridMapRows = int(gridMapHeight / cellWidth)
    gridMapColumns = int(gridMapWidth / cellWidth)

    print(
        f"The width of the grid map: {gridMapWidth}; The height of the grid map: {gridMapHeight}"
    )
    print(f"The number of rows: {gridMapRows}; The number of columns: {gridMapColumns}")

    print()
    #############

    #############
    # debug ray_tracing with real data from json
    print("ray_tracing debug with real data after coordinate translation:")
    print(f"The number of data items to process: {len(lidarDataList)} \n")

    # lidarDataList = read_json()
    runtimes = []
    for dataNumber in range(len(lidarDataList)):
        print(f"Processing data item number: {dataNumber + 1}")
        robotPosition = lidarDataList[dataNumber]["robot_position"]

        # the json file contains X-Y coordinates that are negative or too small (so probably the coordinate system is not set to the left-bottom corner)
        # that's why the coordinates are modified here for testing purpose (in order not the get an error)
        # now we translated the origin of the coordinate system by (-absoluteMinX, -absoluteMinY) in the previous section, so we add (abs(absoluteMinX), abs(absoluteMinY)) to the robot position
        robotPosition.X += abs(absoluteMinX)
        robotPosition.Y += abs(absoluteMinY)

        print(
            f"Robot position: X={robotPosition.X}, Y={robotPosition.Y}, Theta={robotPosition.Theta}"
        )

        startTime = timeit.default_timer() * 1000.0

        for i in range(len(lidarDataList[dataNumber]["scans"])):
            lidarData = lidarDataList[dataNumber]["scans"][i]

            # here the n, m, w values are set for testing purpose, they should be determined according to the grid map
            freeCells, endCell = ray_tracing(
                robotPosition, lidarData, n=gridMapRows, m=gridMapColumns, w=cellWidth
            )

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

        runtimes.append(stopTime - startTime)

        print()

    averageRuntime = sum(runtimes) / len(runtimes)
    maxRuntime = max(runtimes)
    minRuntime = min(runtimes)

    print(
        f"Average runtime: {averageRuntime} ms; Max runtime: {maxRuntime} ms; Min runtime: {minRuntime} ms."
    )

    print()
    #############


if __name__ == "__main__":
    main()
