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

from Python.classes.robot_position import RobotPosition
from Python.classes.cell import Cell
from Python.classes.lidar_sensor_data import LidarData
from determine_cell_index import determine_cell_index
from calc_coordinates import calc_coordinates
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


def x_step_algo(xSign: int):
    pass


def y_step_algo(ySign: int):
    pass


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
    n = robot_cell.row
    m = robot_cell.column
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
        y_step_algo(ySign)
    elif 45 <= Alpha <= 225:
        xSign = -1
        x_step_algo(xSign)
    elif 225 < Alpha < 315:
        ySign = -1
        y_step_algo(ySign)
    else:
        xSign = 1
        x_step_algo(xSign)
