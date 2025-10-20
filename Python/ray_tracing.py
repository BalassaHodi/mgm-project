"""
Created by: Balassa Hodi
This ray-tracing algorithm was developed by me, so it might not be the most efficient algorithm.

How does it work?
* Based on the global angle of the lidar beam there are 2 different algorihms:
    1. 315 <= Alpha or Alpha <= 45  or 135 <= Alpha <= 225    -->    X-step algorithm
    2. 45 < Alpha < 135 or 225 < Alpha < 315                  -->    Y-step algorithm
* The algorithms:
    - The inputs:
        1. robot global position and orientation (X, Y, Theta)
        2. robot position cell indexes (n, m)
        3. lidar beam local orientation and length (alpha, length)
        4. lidar beam end cell indexes (p, q)
        5. cell width
    - Based on the inputs and the global angle of lidar beam the algorithm do the following:
        1. X-step algo: for every x-step, we find the y value of the line, and the two horizontally neighbouring cells are free cells. If the point is at a corner, the 4 neighbouring cells are free cells.
        2. Y-step algo: for every y-step, we find the x value of the line, and the two vertically neighbouring cells are free cells.

NOTE:
* alpha -> the angle of the lidar beam, that goes from 0...359
* Degree increases from the positiv x axis counter clockwise
* there
"""

from classes.robot_position import RobotPosition
from classes.cell import Cell
from classes.lidar_sensor_data import LidarData
from determine_cell_index import determine_cell_index
from calc_coordinates import calc_coordinates
from classes.coordinates import Coordinates
import math


def get_xy_trigonometric(x1: float, y1: float, l: float, A: float, type: str):
    if type == "y":
        return y1 + l * math.sin(math.radians(A))
    elif type == "x":
        return x1 + l * math.cos(math.radians(A))


def get_xy_line_eq(
    x1: float, y1: float, x2: float, y2: float, xk: float, yk: float, type: str
):
    if type == "y":
        return ((y2 - y1) * xk + y1 * x2 - y2 * x1) / (x2 - x1)
    elif type == "x":
        return ((x2 - x1) * yk - y1 * x2 + y2 * x1) / (y2 - y1)


def x_step_algo(
    xSign: int,
    startCell: Cell,
    endCell: Cell,
    w: float,
    robot_pos: RobotPosition,
    endCoordinates: Coordinates,
    freeCells: list,
    n: int,
    m: int,
):
    """
    How does it work briefly?
    1. For every x-step, we find the y value of the line.
    2. The two horizontally neighbouring cells are free cells.
    3. If the point is at a corner, the 4 neighbouring cells are free cells.

    The steps of the algorithm:
    1. Find the start cell and the end cell indexes.
    2. Determine the length of the for loop based on the index difference of the start and end cell in the x direction.
    3. in the for loop in every iteration do:
        - calculate the x coordinate of the current cell edge
        - calculate the y coordinate of the line at this x position
        - if the y coordinate indicates the same cell as the previous iteration, only the new cell in the x direction shall be added to the free cells
        - else determine the two horizontally neighbouring cells based on the calculated y coordinate
        - if the point is at a corner, determine the 4 neighbouring cells
    """

    # find the start cell and the end cell indexes
    startCellColumn = startCell.column
    startCellRow = startCell.row
    endCellColumn = endCell.column
    endCellRow = endCell.row

    # the loop
    for i in range(startCellColumn, endCellColumn, xSign):
        # previous cell row initialization
        previousCellRow = startCellRow

        # calculate the x coordinate of the current cell edge
        x = i * w if xSign == 1 else (i - 1) * w

        # calculate the y coordinate of the line at this x position
        y = get_xy_line_eq(
            robot_pos.X,
            robot_pos.Y,
            endCoordinates.x,
            endCoordinates.y,
            x,
            0,
            "y",
        )

        # determine the two horizontally neighbouring cells based on the calculated y coordinate
        cellLeft = determine_cell_index(x, y, n, m, w)
        cellRight = Cell(cellLeft.row, cellLeft.column + 1)

        # add the free cells to the list
        if xSign == 1:
            if i != endCellColumn - 1:
                freeCells.append(cellRight)
            elif i == endCellColumn - 1 and cellRight.row != endCellRow:
                freeCells.append(cellRight)

            if cellRight.row != previousCellRow:
                freeCells.append(cellLeft)
        else:
            if i != endCellColumn + 1:
                freeCells.append(cellLeft)
            elif i == endCellColumn + 1 and cellLeft.row != endCellRow:
                freeCells.append(cellLeft)

            if cellLeft.row != previousCellRow:
                freeCells.append(cellRight)

        # previous cell row update
        previousCellRow = cellRight.row if xSign == 1 else cellLeft.row

    freeCells.append(endCell)


def y_step_algo(
    ySign: int,
    startCell: Cell,
    endCell: Cell,
    w: float,
    robot_pos: RobotPosition,
    endCoordinates: Coordinates,
    freeCells: list,
    n: int,
    m: int,
):
    """
    Here comes some description.
    """
    # find the start cell and the end cell indexes
    startCellColumn = startCell.column
    startCellRow = startCell.row
    endCellColumn = endCell.column
    endCellRow = endCell.row

    # the loop
    for i in range(startCellRow, endCellRow, ySign):
        # previous cell column initialization
        previousCellColumn = startCellColumn

        # calculate the y coordinate of the current cell edge
        y = i * w if ySign == 1 else (i - 1) * w

        # calculate the y coordinate of the line at this x position
        x = get_xy_line_eq(
            robot_pos.X, robot_pos.Y, endCoordinates.x, endCoordinates.y, 0, y, "x"
        )

        # determine the two vertically neighbouring cells based on the calculated x coordinate
        cellDown = determine_cell_index(x, y, n, m, w)
        cellUp = Cell(cellDown.row + 1, cellDown.column)

        # add the free cells to the list
        if ySign == 1:
            if i != endCellRow - 1:
                freeCells.append(cellUp)
            elif i == endCellRow - 1 and cellUp.column != endCellColumn:
                freeCells.append(cellUp)

            if cellUp.column != previousCellColumn:
                freeCells.append(cellDown)
        else:
            if i != endCellRow + 1:
                freeCells.append(cellDown)
            elif i == endCellRow + 1 and cellDown.column != endCellColumn:
                freeCells.append(cellDown)

            if cellDown.column != previousCellColumn:
                freeCells.append(cellUp)

        # previous cell column update
        previousCellColumn = cellUp.column if ySign == 1 else cellDown.column

    freeCells.append(endCell)


def ray_tracing(
    robot_pos: RobotPosition,
    robot_cell: Cell,
    lidar_data: LidarData,
    lidar_cell: Cell,
    w: float,
):
    """
    Here comes some description.
    """

    # initial data
    X = robot_pos.X
    Y = robot_pos.Y
    Theta = robot_pos.Theta  # in degree
    n = 100  # grid dimensions --> hardcoded just for debugging
    m = 100  # grid dimensions --> hardcoded just for debugging
    alpha = lidar_data.alpha  # in degree
    l = lidar_data.length
    p = lidar_cell.row
    q = lidar_cell.column

    # output data
    freeCells = [robot_cell]

    # absolute lidar sensor angle:
    Alpha = Theta + alpha if Theta + alpha < 360 else Theta + alpha - 360

    # calculate end cell
    endCoordinates = calc_coordinates(X, Y, Alpha, l)
    endCell = determine_cell_index(endCoordinates.x, endCoordinates.y, n, m, w)

    # check which algorithm shall we do
    if 45 < Alpha < 135:
        ySign = 1
        y_step_algo(
            ySign, robot_cell, endCell, w, robot_pos, endCoordinates, freeCells, n, m
        )
    elif 45 <= Alpha <= 225:
        xSign = -1
        x_step_algo(
            xSign, robot_cell, endCell, w, robot_pos, endCoordinates, freeCells, n, m
        )
    elif 225 < Alpha < 315:
        ySign = -1
        y_step_algo(
            ySign, robot_cell, endCell, w, robot_pos, endCoordinates, freeCells, n, m
        )
    else:
        xSign = 1
        x_step_algo(
            xSign, robot_cell, endCell, w, robot_pos, endCoordinates, freeCells, n, m
        )

    return freeCells
