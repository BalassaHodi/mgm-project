"""
Improved version of grid_map that works with dynamic grid bounds and negative coordinates.

This module contains the OccupancyGridMapper class for managing a dynamic occupancy grid map
with probabilistic updates. The grid automatically expands as the robot explores new areas.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import math
from shared.classes.cell import Cell
from shared.classes.robot_position import RobotPosition


class OccupancyGridMapper:
    """
    A dynamic occupancy grid mapper that uses Bayesian probability updates.

    The grid automatically expands to accommodate new measurements and can handle
    negative coordinates. The origin is set to the first robot position.

    Attributes:
    -----------
    cell_width : float
        Width and height of each cell in meters
    min_x : float
        Minimum X coordinate of the grid (bottom-left corner)
    min_y : float
        Minimum Y coordinate of the grid (bottom-left corner)
    max_x : float
        Maximum X coordinate of the grid (top-right corner)
    max_y : float
        Maximum Y coordinate of the grid (top-right corner)
    n_rows : int
        Current number of rows in the grid
    n_cols : int
        Current number of columns in the grid
    log_odds_grid : np.ndarray
        2D array storing log-odds values for each cell
    origin_x : float
        X coordinate of the origin (first robot position)
    origin_y : float
        Y coordinate of the origin (first robot position)
    """

    def __init__(
        self,
        cell_width: float = 0.1,
        initial_size: int = 100,
        prob_occupied_given_occupied: float = 0.9,
        prob_occupied_given_free: float = 0.3,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ):
        """
        Initialize the occupancy grid mapper.

        Parameters:
        -----------
        cell_width : float, optional
            Width and height of each cell in meters (default: 0.1)
        initial_size : int, optional
            Initial size of the grid in cells (creates initial_size x initial_size grid) (default: 100)
        prob_occupied_given_occupied : float, optional
            P(occupied | measured as occupied) - sensor model parameter (default: 0.9)
        prob_occupied_given_free : float, optional
            P(occupied | measured as free) - sensor model parameter (default: 0.3)
        origin_x : float, optional
            X coordinate of the origin (first robot position) (default: 0.0)
        origin_y : float, optional
            Y coordinate of the origin (first robot position) (default: 0.0)
        """
        self.cell_width = cell_width
        self.origin_x = origin_x
        self.origin_y = origin_y

        # Track cumulative data bounds
        self.data_min_x = None
        self.data_max_x = None
        self.data_min_y = None
        self.data_max_y = None
        self.first_expansion_done = (
            False  # Flag to force fitting to data on first expansion
        )

        # Initialize grid centered around origin
        half_size = initial_size // 2
        self.min_x = origin_x - half_size * cell_width
        self.max_x = origin_x + half_size * cell_width
        self.min_y = origin_y - half_size * cell_width
        self.max_y = origin_y + half_size * cell_width

        self.n_rows = initial_size
        self.n_cols = initial_size

        # Initialize log-odds grid (0 = unknown, probability 0.5)
        self.log_odds_grid = np.zeros((self.n_rows, self.n_cols), dtype=float)

        # Calculate log-odds updates
        self.log_odds_occupied = math.log(
            prob_occupied_given_occupied / (1 - prob_occupied_given_occupied)
        )
        self.log_odds_free = math.log(
            prob_occupied_given_free / (1 - prob_occupied_given_free)
        )

        # Store sensor model parameters
        self.prob_occ_occ = prob_occupied_given_occupied
        self.prob_occ_free = prob_occupied_given_free

    def _expand_grid_if_needed(self, x_coords: list, y_coords: list):
        """
        Expand the grid if any coordinates fall outside current bounds.

        Parameters:
        -----------
        x_coords : list
            List of X coordinates to check
        y_coords : list
            List of Y coordinates to check
        """
        if not x_coords or not y_coords:
            return

        min_x_needed = min(x_coords)
        max_x_needed = max(x_coords)
        min_y_needed = min(y_coords)
        max_y_needed = max(y_coords)

        # Update cumulative data bounds
        if self.data_min_x is None:
            self.data_min_x = min_x_needed
            self.data_max_x = max_x_needed
            self.data_min_y = min_y_needed
            self.data_max_y = max_y_needed
        else:
            self.data_min_x = min(self.data_min_x, min_x_needed)
            self.data_max_x = max(self.data_max_x, max_x_needed)
            self.data_min_y = min(self.data_min_y, min_y_needed)
            self.data_max_y = max(self.data_max_y, max_y_needed)

        # Add some margin (10% or at least 1 meter)
        margin = max(
            1.0,
            0.1
            * max(self.data_max_x - self.data_min_x, self.data_max_y - self.data_min_y),
        )
        min_x_with_margin = self.data_min_x - margin
        max_x_with_margin = self.data_max_x + margin
        min_y_with_margin = self.data_min_y - margin
        max_y_with_margin = self.data_max_y + margin

        # Check if grid adjustment is needed
        # Force adjustment on first expansion to fit data regardless of initial size
        needs_adjustment = (
            not self.first_expansion_done
            or min_x_with_margin < self.min_x
            or max_x_with_margin > self.max_x
            or min_y_with_margin < self.min_y
            or max_y_with_margin > self.max_y
        )

        if not needs_adjustment:
            return

        # Calculate new bounds
        # ALWAYS include existing grid bounds to preserve data
        new_min_x = min(self.min_x, min_x_with_margin)
        new_max_x = max(self.max_x, max_x_with_margin)
        new_min_y = min(self.min_y, min_y_with_margin)
        new_max_y = max(self.max_y, max_y_with_margin)

        # Mark that first expansion has been done
        if not self.first_expansion_done:
            self.first_expansion_done = True

        # Calculate new dimensions
        new_n_cols = int(math.ceil((new_max_x - new_min_x) / self.cell_width))
        new_n_rows = int(math.ceil((new_max_y - new_min_y) / self.cell_width))

        # Create new grid
        new_log_odds_grid = np.zeros((new_n_rows, new_n_cols), dtype=float)

        # Calculate overlapping region between old and new grids
        # Old grid covers [self.min_x, self.max_x] × [self.min_y, self.max_y]
        # New grid covers [new_min_x, new_max_x] × [new_min_y, new_max_y]

        # Find overlapping region in world coordinates
        overlap_min_x = max(self.min_x, new_min_x)
        overlap_max_x = min(self.max_x, new_max_x)
        overlap_min_y = max(self.min_y, new_min_y)
        overlap_max_y = min(self.max_y, new_max_y)

        if overlap_min_x < overlap_max_x and overlap_min_y < overlap_max_y:
            # There is an overlap, copy the data

            # Calculate indices in old grid
            old_col_start = int(
                math.floor((overlap_min_x - self.min_x) / self.cell_width)
            )
            old_col_end = int(math.ceil((overlap_max_x - self.min_x) / self.cell_width))
            old_row_start = int(
                math.floor((overlap_min_y - self.min_y) / self.cell_width)
            )
            old_row_end = int(math.ceil((overlap_max_y - self.min_y) / self.cell_width))

            # Calculate indices in new grid
            new_col_start = int(
                math.floor((overlap_min_x - new_min_x) / self.cell_width)
            )
            new_col_end = int(math.ceil((overlap_max_x - new_min_x) / self.cell_width))
            new_row_start = int(
                math.floor((overlap_min_y - new_min_y) / self.cell_width)
            )
            new_row_end = int(math.ceil((overlap_max_y - new_min_y) / self.cell_width))

            # Ensure indices are within bounds
            old_col_start = max(0, min(old_col_start, self.n_cols))
            old_col_end = max(0, min(old_col_end, self.n_cols))
            old_row_start = max(0, min(old_row_start, self.n_rows))
            old_row_end = max(0, min(old_row_end, self.n_rows))

            new_col_start = max(0, min(new_col_start, new_n_cols))
            new_col_end = max(0, min(new_col_end, new_n_cols))
            new_row_start = max(0, min(new_row_start, new_n_rows))
            new_row_end = max(0, min(new_row_end, new_n_rows))

            # Copy overlapping data
            old_data = self.log_odds_grid[
                old_row_start:old_row_end, old_col_start:old_col_end
            ]

            # Ensure the target slice matches the source shape
            actual_rows = old_data.shape[0]
            actual_cols = old_data.shape[1]

            new_log_odds_grid[
                new_row_start : new_row_start + actual_rows,
                new_col_start : new_col_start + actual_cols,
            ] = old_data

        # Update grid parameters
        self.log_odds_grid = new_log_odds_grid
        self.min_x = new_min_x
        self.max_x = new_max_x
        self.min_y = new_min_y
        self.max_y = new_max_y
        self.n_rows = new_n_rows
        self.n_cols = new_n_cols

    def update_with_scan(self, free_cells: list, occupied_cells: list):
        """
        Update the occupancy grid with a new scan.

        Parameters:
        -----------
        free_cells : list
            List of Cell objects representing free cells
        occupied_cells : list
            List of Cell objects representing occupied cells
        """
        # Update free cells
        for cell in free_cells:
            if 0 <= cell.row < self.n_rows and 0 <= cell.column < self.n_cols:
                self.log_odds_grid[cell.row, cell.column] += self.log_odds_free

        # Update occupied cells
        for cell in occupied_cells:
            if 0 <= cell.row < self.n_rows and 0 <= cell.column < self.n_cols:
                self.log_odds_grid[cell.row, cell.column] += self.log_odds_occupied

    def get_probability_grid(self):
        """
        Convert log-odds grid to probability grid.

        Returns:
        --------
        np.ndarray
            2D array of occupancy probabilities (0.0 to 1.0)
        """
        # Convert log-odds to probability: p = 1 - 1/(1 + exp(log_odds))
        # Using numerically stable computation
        prob_grid = np.zeros_like(self.log_odds_grid)

        # For positive log-odds
        pos_mask = self.log_odds_grid > 0
        prob_grid[pos_mask] = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds_grid[pos_mask]))

        # For negative log-odds (more numerically stable)
        neg_mask = self.log_odds_grid <= 0
        exp_val = np.exp(self.log_odds_grid[neg_mask])
        prob_grid[neg_mask] = exp_val / (1.0 + exp_val)

        return prob_grid

    def get_probability_grid_with_bounds(self):
        """
        Get the probability grid along with its bounds in world coordinates.

        Returns:
        --------
        dict
            Dictionary containing:
            - 'probabilities': 2D numpy array of occupancy probabilities
            - 'min_x': Minimum X coordinate of the grid
            - 'max_x': Maximum X coordinate of the grid
            - 'min_y': Minimum Y coordinate of the grid
            - 'max_y': Maximum Y coordinate of the grid
            - 'n_rows': Number of rows
            - 'n_cols': Number of columns
            - 'cell_width': Width of each cell
            - 'origin_x': X coordinate of the origin
            - 'origin_y': Y coordinate of the origin
        """
        return {
            "probabilities": self.get_probability_grid(),
            "min_x": self.min_x,
            "max_x": self.max_x,
            "min_y": self.min_y,
            "max_y": self.max_y,
            "n_rows": self.n_rows,
            "n_cols": self.n_cols,
            "cell_width": self.cell_width,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
        }

    def world_to_grid(self, x: float, y: float):
        """
        Convert world coordinates to grid indices.

        Parameters:
        -----------
        x : float
            World X coordinate
        y : float
            World Y coordinate

        Returns:
        --------
        tuple
            (row, col) grid indices, or None if out of bounds
        """
        col = int((x - self.min_x) // self.cell_width)
        row = int((y - self.min_y) // self.cell_width)

        if 0 <= row < self.n_rows and 0 <= col < self.n_cols:
            return (row, col)
        return None

    def grid_to_world(self, row: int, col: int):
        """
        Convert grid indices to world coordinates (cell center).

        Parameters:
        -----------
        row : int
            Grid row index
        col : int
            Grid column index

        Returns:
        --------
        tuple
            (x, y) world coordinates of the cell center
        """
        x = self.min_x + (col + 0.5) * self.cell_width
        y = self.min_y + (row + 0.5) * self.cell_width
        return (x, y)

    def get_grid_info(self):
        """
        Get information about the current grid state.

        Returns:
        --------
        dict
            Dictionary with grid information
        """
        return {
            "n_rows": self.n_rows,
            "n_cols": self.n_cols,
            "cell_width": self.cell_width,
            "min_x": self.min_x,
            "max_x": self.max_x,
            "min_y": self.min_y,
            "max_y": self.max_y,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "grid_width_m": self.max_x - self.min_x,
            "grid_height_m": self.max_y - self.min_y,
        }
