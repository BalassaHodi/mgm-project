"""
This module contains the OccupancyGridMapper class.
It is responsible for creating and updating an occupancy grid map.
---
The key features:
*IMPORTANT*
- the size and the resolution of the grid map is a static parameter defined at initialization.
- this means that the grid map is not dynamically resized, which could lead to imperfect mapping,
  so the choice of the size should be made carefully based on the expected environment size.
*GRID MAP*
- the local origin of the grid map (so cell [0,0]) is at the bottom-left corner of the map.
- the global origin (so the world coordinates [0,0]) is at the center of the grid map.
- the grid map is represented as a 2D numpy array, where each cell contains a probablility value
  indicating the likelihood of that cell being occupied. Range: [0,100]
- the occupancy pobabilities are calculated using a log-odds method.
*RAY TRACING*
- the ray tracing algorithm is responsible for determining which cells did the laser rays pass through
- the alogrithm is implemented in a separate function and is imported into this module.
- the further explanation is in the ray tracing function definition.
*SCAN PROCESSING*
- the process_scan method is the main method that has to be called to process a new laser scan and update
  the map from it.
---
Usage:
1. Initialize the OccupancyGridMap
    map = OccupancyGridMap()
2. Process a laser scan
    map.process_scan(robot_pose, laser_scan)
3. Access the created occupancy grid map
    map.gridMap
"""

import numpy as np


class OccupancyGridMap:
    """
    Class for creating and updating an occupancy grid map.
    Note that the size and resolution of the grid map is static and defined at initialization, so it does not resize dynamically.

    ---
    Inputs:
    - mapWidth: width of the grid map in meters
    - mapHeight: height of the grid map in meters
    - resolution: size of each grid cell in meters (default: 0.1m)
    ---
    Methods (public):
    - process_scan(robot_pose, lidar_data): Process a laser scan and update the occupancy grid map
    ---
    Attributes (public):
    - gridMap: 2D numpy array representing the occupancy grid map
    """

    def __init__(self, mapWidth: float, mapHeight: float, resolution: float = 0.1):
        self._mapWidth = mapWidth  # x dimension
        self._mapHeight = mapHeight  # y dimension
        self._resolution = resolution

        # Calculate grid dimensions
        self._gridCols = int(self._mapWidth / self._resolution)
        self._gridRows = int(self._mapHeight / self._resolution)

        # Initialize grid map with probablity values of 50 (unknown)
        self.gridMap = np.full((self._gridRows, self._gridCols), 0.5)

        # Initialize grid map with log-odds values of 0
        self._logOddsMap = np.zeros((self._gridRows, self._gridCols), dtype=float)

    def _world_to_grid(self, x: float, y: float):
        """Convert world coordinates to grid indices."""

        # Convert world coordinates to grid indices
        # if the coordinate is at the edge, the cell on the right/top is chosen
        gridCol = int((x + self._mapWidth / 2) / self._resolution)
        gridRow = int((y + self._mapHeight / 2) / self._resolution)

        # return None if out pf bounds
        if 0 <= gridCol < self._gridCols and 0 <= gridRow < self._gridRows:
            return gridRow, gridCol
        else:
            return None, None

    def _polar_to_world(
        self, angle: float, distance: float, x0: float = 0, y0: float = 0
    ):
        """Convert polar coordinates to world coordinates."""

        x = x0 + distance * np.cos(angle)
        y = y0 + distance * np.sin(angle)

        return x, y

    def _trace_ray(self, robot_pose, lidar_data):
        """Ray tracing algorithm to determine the free and occupied ce1 lls along a laser ray."""

        # local functions
        def get_xy(
            x1: float,
            y1: float,
            x2: float,
            y2: float,
            xk: float = None,
            yk: float = None,
        ):
            """Calculate the x or y coordinate of a point on a line using line equation."""

            if yk is not None:
                # calculate x coordinate
                if y2 - y1 == 0:
                    return None
                else:
                    return ((x2 - x1) * yk - y1 * x2 + y2 * x1) / (y2 - y1)
            elif xk is not None:
                # calculate y coordinate
                if x2 - x1 == 0:
                    return None
                else:
                    return ((y2 - y1) * xk + y1 * x2 - y2 * x1) / (x2 - x1)

        def x_step_algorithm(
            xSign: int,
            startRow: int,
            startCol: int,
            endRow: int,
            endCol: int,
            robotX: float,
            robotY: float,
            endX: float,
            endY: float,
        ):
            """Implements X-step ray tracing algorithm."""

            # update cell as free: starting cell
            self._update_cell(startRow, startCol, is_occupied=False)

            # store the previous cell row
            prevRow = startRow

            # the main loop that steps through the grid cells in x direction
            for col in range(startCol, endCol, xSign):
                # calculate the x coordinate of the current cell edge
                x = (
                    (col + 1) * self._resolution - self._mapWidth / 2
                    if xSign == 1
                    else col * self._resolution - self._mapWidth / 2
                )

                # calculate the y coordinate of the line at this x coordinate
                y = get_xy(robotX, robotY, endX, endY, xk=x)
                if y is None:
                    # vertical line case, but this cannot happen in x_step_algorithm
                    return

                # determine the two horizontally adjacent cells
                cellRightRow, cellRightCol = self._world_to_grid(x, y)
                cellLeftRow, cellLeftCol = cellRightRow, cellRightCol - 1

                # update the cells
                if xSign == 1:
                    if col != endCol - 1:
                        # update cell as free: cellRight
                        self._update_cell(cellRightRow, cellRightCol, is_occupied=False)
                    elif col == endCol - 1 and cellRightRow != endRow:
                        # update cell as free: cellRight
                        self._update_cell(cellRightRow, cellRightCol, is_occupied=False)

                    if cellRightRow != prevRow:
                        # update cell as free: cellLeft
                        self._update_cell(cellLeftRow, cellLeftCol, is_occupied=False)
                else:
                    if col != endCol + 1:
                        # update cell as free: cellLeft
                        self._update_cell(cellLeftRow, cellLeftCol, is_occupied=False)
                    elif col == endCol + 1 and cellLeftRow != endRow:
                        # update cell as free: cellLeft
                        self._update_cell(cellLeftRow, cellLeftCol, is_occupied=False)

                    if cellLeftRow != prevRow:
                        # update cell as free: cellRight
                        self._update_cell(cellRightRow, cellRightCol, is_occupied=False)

                # update previous row
                prevRow = cellRightRow if xSign == 1 else cellLeftRow

        def y_step_algorithm(
            ySign: int,
            startRow: int,
            startCol: int,
            endRow: int,
            endCol: int,
            robotX: float,
            robotY: float,
            endX: float,
            endY: float,
        ):
            """Implements Y-step ray tracing algorithm."""

            # update cell as free: starting cell
            self._update_cell(startRow, startCol, is_occupied=False)

            # store the previous cell column
            prevCol = startCol

            # the main loop that steps through the grid cells in y direction
            for row in range(startRow, endRow, ySign):
                # calculate the y coordinate of the current cell edge
                y = (
                    (row + 1) * self._resolution - self._mapHeight / 2
                    if ySign == 1
                    else row * self._resolution - self._mapHeight / 2
                )

                # calculate the x coordinate of the line at this y coordinate
                x = get_xy(robotX, robotY, endX, endY, yk=y)
                if x is None:
                    # horizontal line case, but this cannot happen in y_step_algorithm
                    return

                # determine the two vertically adjacent cells
                cellTopRow, cellTopCol = self._world_to_grid(x, y)
                cellBottomRow, cellBottomCol = cellTopRow - 1, cellTopCol

                # update the cells
                if ySign == 1:
                    if row != endRow - 1:
                        # update cell as free: cellTop
                        self._update_cell(cellTopRow, cellTopCol, is_occupied=False)
                    elif row == endRow - 1 and cellTopCol != endCol:
                        # update cell as free: cellTop
                        self._update_cell(cellTopRow, cellTopCol, is_occupied=False)
                    if cellTopCol != prevCol:
                        # update cell as free: cellBottom
                        self._update_cell(
                            cellBottomRow, cellBottomCol, is_occupied=False
                        )
                else:
                    if row != endRow + 1:
                        # update cell as free: cellBottom
                        self._update_cell(
                            cellBottomRow, cellBottomCol, is_occupied=False
                        )
                    elif row == endRow + 1 and cellBottomCol != endCol:
                        # update cell as free: cellBottom
                        self._update_cell(
                            cellBottomRow, cellBottomCol, is_occupied=False
                        )

                    if cellBottomCol != prevCol:
                        # update cell as free: cellTop
                        self._update_cell(cellTopRow, cellTopCol, is_occupied=False)

                # update previous column
                prevCol = cellTopCol if ySign == 1 else cellBottomCol

        # Get robot pose
        robotX = robot_pose[0]
        robotY = robot_pose[1]
        robotTheta = robot_pose[2]  # in degrees

        # Get lidar data
        lidarAlpha = lidar_data[0]  # in degrees
        lidarDistance = lidar_data[1]

        # determine cell indices for robot position
        robotRow, robotCol = self._world_to_grid(robotX, robotY)

        # global angle of the laser ray
        globalAngle = (
            robotTheta + lidarAlpha
            if robotTheta + lidarAlpha < 360
            else robotTheta + lidarAlpha - 360
        )

        # calculate end coordinates and end cell indices
        endX, endY = self._polar_to_world(
            np.deg2rad(globalAngle), lidarDistance, robotX, robotY
        )
        endRow, endCol = self._world_to_grid(endX, endY)
        if endRow is None or endCol is None:
            return

        # call the algorithm based on the global angle of the ray
        if 45 < globalAngle < 135:
            # do y_step_algo here, with positive y steps
            ySign = 1
            y_step_algorithm(
                ySign,
                robotRow,
                robotCol,
                endRow,
                endCol,
                robotX,
                robotY,
                endX,
                endY,
            )
        elif 135 <= globalAngle <= 225:
            # do x_step_algo here with negative x steps
            xSign = -1
            x_step_algorithm(
                xSign,
                robotRow,
                robotCol,
                endRow,
                endCol,
                robotX,
                robotY,
                endX,
                endY,
            )
        elif 225 < globalAngle < 315:
            # do y_step_algo here with negative y steps
            ySign = -1
            y_step_algorithm(
                ySign,
                robotRow,
                robotCol,
                endRow,
                endCol,
                robotX,
                robotY,
                endX,
                endY,
            )
        else:
            # do x_step_algo here with positive x steps
            xSign = 1
            x_step_algorithm(
                xSign,
                robotRow,
                robotCol,
                endRow,
                endCol,
                robotX,
                robotY,
                endX,
                endY,
            )

        # update cell as occupied: end cell
        self._update_cell(endRow, endCol, is_occupied=True)

    def _update_cell(self, row: int, col: int, is_occupied: bool = False):
        """Update a cell in the occupancy grid map using log-odds method."""

        # local function
        def log_odds_to_prob(logOdds: float):
            """Convert log-odds value to probability"""
            return 1.0 - 1.0 / (1.0 + np.exp(logOdds))

        # Define values for log-odds updates
        probOccGivenOcc = 0.9
        probOccGivenFree = 0.3
        logOddsOcc = np.log(probOccGivenOcc / (1 - probOccGivenOcc))
        logOddsFree = np.log(probOccGivenFree / (1 - probOccGivenFree))

        # Update log-odds value of the cell
        if is_occupied:
            # update as occupied
            self._logOddsMap[row, col] += logOddsOcc
            self.gridMap[row, col] = log_odds_to_prob(self._logOddsMap[row, col])
        else:
            # update as free
            self._logOddsMap[row, col] += logOddsFree
            self.gridMap[row, col] = log_odds_to_prob(self._logOddsMap[row, col])

    def process_scan(self, robot_pose, lidar_data):
        """
        Process a laser scan by tracing the ray and updating the map.
        
        ---
        Inputs:
        - robot_pose: array of robot pose [x, y, theta] in [m, m, degrees]
        - lidar_data: array of lidar data [alpha, distance] in [degrees, m]
        """

        # trace the ray and update the map
        self._trace_ray(robot_pose, lidar_data)

    def _visualize_map(self):
        """Visualize the occupancy grid map using matplotlib (for debugging purposes)."""
        pass
