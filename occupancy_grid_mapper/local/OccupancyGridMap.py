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
    Methods:
    - trace_ray: traces a ray from the robot's position to a laser hit point
    - process_scan: processes a laser scan by tracing the ray and updating the map
    - update_map: updates the occupancy grid map based on the traced ray
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

        # Initialize grid map with the given size and resolution
        self.gridMap = np.full((self._gridRows, self._gridCols), 50.0)

    def world_to_grid(self):
        pass

    def calc_coordinates(self):
        pass

    def trace_ray(self):
        pass

    def process_scan(self):
        pass

    def update_map(self):
        pass
