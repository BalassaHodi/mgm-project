"""
Created by: Balassa Hodi
Improved version that works with dynamic grid bounds and negative coordinates.

This ray-tracing algorithm was developed by me, so it might not be the most efficient algorithm.

How does it work?
* Based on the global angle of the lidar beam there are 2 different algorithms:
    1. 315 <= Alpha or Alpha <= 45  or 135 <= Alpha <= 225    -->    X-step algorithm
    2. 45 < Alpha < 135 or 225 < Alpha < 315                  -->    Y-step algorithm
* The algorithms:
    - The inputs:
        1. robot global position and orientation (X, Y, Theta)
        2. lidar beam local orientation and length (alpha, length)
        3. grid map bounds (min_x, min_y, max_x, max_y)
        4. cell width
    - Based on the inputs and the global angle of lidar beam the algorithm do the following:
        1. X-step algo: for every x-step, we find the y value of the line, and the two horizontally neighbouring cells are free cells.
        2. Y-step algo: for every y-step, we find the x value of the line, and the two vertically neighbouring cells are free cells.

NOTE:
* alpha -> the angle of the lidar beam, that goes from 0...359
* Degree always increases from the positive x axis counter-clockwise
* This improved version handles negative coordinates and dynamic grid bounds
"""

from classes.robot_position import RobotPosition
from classes.cell import Cell
from classes.lidar_sensor_data import LidarData
from determine_cell_index_improved import determine_cell_index_improved
from calc_coordinates import calc_coordinates
from classes.coordinates import Coordinates
import math


def get_xy_trigonometric(x1: float, y1: float, l: float, A: float, type: str):
    """
    This function calculates the x or y coordinate of a point based on trigonometric functions. \n
    ---
    Inputs:
    * x1: float -> the x coordinate of the starting point
    * y1: float -> the y coordinate of the starting point
    * l: float -> the length from the starting point to the target point
    * A: float -> the angle in degrees (0...359, where 0 is along the positive x-axis and increases counter-clockwise)
    * type: str -> "x" to calculate the x coordinate, "y" to calculate the y coordinate
    ---
    Output:
    * float -> the calculated x or y coordinate of the target point (depending on the 'type' input)
    """

    if type == "y":
        return y1 + l * math.sin(math.radians(A))
    elif type == "x":
        return x1 + l * math.cos(math.radians(A))


def get_xy_line_eq(
    x1: float, y1: float, x2: float, y2: float, xk: float, yk: float, type: str
):
    """
    This function calculates the x or y coordinate of a point on a line defined by its two endpoints. \n
    ---
    Inputs:
    * x1: float -> the x coordinate of the first endpoint of the line
    * y1: float -> the y coordinate of the first endpoint of the line
    * x2: float -> the x coordinate of the second endpoint of the line
    * y2: float -> the y coordinate of the second endpoint of the line
    * xk: float -> the x coordinate of the point on the line of which we want to calculate the y (used when calculating y)
    * yk: float -> the y coordinate of the point on the line of which we want to calculate the x (used when calculating x)
    * type: str -> "x" to calculate the x coordinate, "y" to calculate the y coordinate
    ---
    Output:
    * float -> the calculated x or y coordinate of the point on the line (depending on the 'type' input)
    """
    if type == "y":
        return ((y2 - y1) * xk + y1 * x2 - y2 * x1) / (x2 - x1)
    elif type == "x":
        return ((x2 - x1) * yk - y1 * x2 + y2 * x1) / (y2 - y1)


def x_step_algo_improved(
    xSign: int,
    startCell: Cell,
    endCell: Cell,
    w: float,
    robot_pos: RobotPosition,
    endCoordinates: Coordinates,
    freeCells: list,
    min_x: float,
    min_y: float,
):
    """
    Improved version of x_step_algo that works with dynamic grid bounds.

    How does it work briefly?
    1. For every x-step, we find the y value of the line.
    2. The two horizontally neighbouring cells are free cells.
    3. If the point is at a corner, the 4 neighbouring cells are free cells. NOTE: this is not implemented in the algorithm.

    The steps of the algorithm:
    1. Find the start cell and the end cell indexes.
    2. Determine the length of the for loop based on the index difference of the start and end cell in the x direction.
    3. in the for loop in every iteration do:
        - calculate the x coordinate of the current cell edge
        - calculate the y coordinate of the line at this x position
        - if the y coordinate indicates the same cell as the previous iteration, only the new cell in the x direction shall be added to the free cells
        - else determine the two horizontally neighbouring cells based on the calculated y coordinate
    """

    # define the start cell and the end cell indexes
    startCellColumn = startCell.column
    startCellRow = startCell.row
    endCellColumn = endCell.column
    endCellRow = endCell.row

    # previous cell row initialization
    previousCellRow = startCellRow

    # the loop
    for i in range(startCellColumn, endCellColumn, xSign):
        # calculate the x coordinate of the current cell edge in global coordinates
        x = min_x + i * w if xSign == 1 else min_x + (i + 1) * w

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
        cellLeft = determine_cell_index_improved(x, y, min_x, min_y, w)
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


def y_step_algo_improved(
    ySign: int,
    startCell: Cell,
    endCell: Cell,
    w: float,
    robot_pos: RobotPosition,
    endCoordinates: Coordinates,
    freeCells: list,
    min_x: float,
    min_y: float,
):
    """
    Improved version of y_step_algo that works with dynamic grid bounds.
    Same as x_step_algo_improved, but in the y direction.
    """
    # define the start cell and the end cell indexes
    startCellColumn = startCell.column
    startCellRow = startCell.row
    endCellColumn = endCell.column
    endCellRow = endCell.row

    # previous cell column initialization
    previousCellColumn = startCellColumn

    # the loop
    for i in range(startCellRow, endCellRow, ySign):

        # calculate the y coordinate of the current cell edge in global coordinates
        y = min_y + i * w if ySign == 1 else min_y + (i + 1) * w

        # calculate the x coordinate of the line at this y position
        x = get_xy_line_eq(
            robot_pos.X, robot_pos.Y, endCoordinates.x, endCoordinates.y, 0, y, "x"
        )

        # determine the two vertically neighbouring cells based on the calculated x coordinate
        cellDown = determine_cell_index_improved(x, y, min_x, min_y, w)
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


def ray_tracing_improved(
    robot_pos: RobotPosition,
    lidar_data: LidarData,
    w: float,
    min_x: float,
    min_y: float,
    max_range: float = 4.2,
):
    """
    Improved version of ray tracing that works with dynamic grid bounds and negative coordinates.

    This function performs ray tracing to determine those cells that are free along the path of a lidar beam.
    The first cell is where the robot is located, and the last cell is where the lidar beam ends.

    Parameters:
    -----------
    robot_pos : RobotPosition
        The global position and orientation of the robot (X, Y, Theta: [0...359])
    lidar_data : LidarData
        The local orientation and length of the lidar beam (alpha, length)
    w : float
        The width of a single cell in the grid
    min_x : float
        Minimum X coordinate of the current grid
    min_y : float
        Minimum Y coordinate of the current grid
    max_range : float, optional
        Maximum range of the lidar sensor (default: 4.2 meters)

    Returns:
    --------
    freeCells : list
        A list of Cell objects representing the free cells along the lidar beam path
    endCell : Cell or None
        The cell where the lidar beam ends (occupied cell), or None if no obstacle detected
    """

    # initial data
    X = robot_pos.X
    Y = robot_pos.Y
    Theta = robot_pos.Theta  # in degree
    alpha = lidar_data.alpha  # in degree
    l = lidar_data.length

    # determine robot cell (initial cell)
    robotCell = determine_cell_index_improved(X, Y, min_x, min_y, w)

    # output data
    freeCells = (
        []
    )  # Start with empty list - we'll add robotCell only if there's a valid detection

    # absolute lidar sensor angle:
    Alpha = Theta + alpha if Theta + alpha < 360 else Theta + alpha - 360

    # calculate end cell
    if l == -1.0:
        # No detection - don't trace anything
        # We have no information about this direction
        return [robotCell], None  # Only mark robot's cell, no occupied cell
        endCell = determine_cell_index_improved(
            endCoordinates.x, endCoordinates.y, min_x, min_y, w
        )
        # Mark that we should not add any occupied cell
        no_detection = True
    else:
        endCoordinates = calc_coordinates(X, Y, Alpha, l)
        endCell = determine_cell_index_improved(
            endCoordinates.x, endCoordinates.y, min_x, min_y, w
        )
        no_detection = False

    # Add robot cell to free cells for valid detections
    freeCells = [robotCell]

    # check which algorithm shall we do
    if 45 < Alpha < 135:
        ySign = 1
        y_step_algo_improved(
            ySign,
            robotCell,
            endCell,
            w,
            robot_pos,
            endCoordinates,
            freeCells,
            min_x,
            min_y,
        )
    elif 135 <= Alpha <= 225:
        xSign = -1
        x_step_algo_improved(
            xSign,
            robotCell,
            endCell,
            w,
            robot_pos,
            endCoordinates,
            freeCells,
            min_x,
            min_y,
        )
    elif 225 < Alpha < 315:
        ySign = -1
        y_step_algo_improved(
            ySign,
            robotCell,
            endCell,
            w,
            robot_pos,
            endCoordinates,
            freeCells,
            min_x,
            min_y,
        )
    else:
        xSign = 1
        x_step_algo_improved(
            xSign,
            robotCell,
            endCell,
            w,
            robot_pos,
            endCoordinates,
            freeCells,
            min_x,
            min_y,
        )

    return freeCells, endCell
