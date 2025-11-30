"""
This module contains the grid_map function for visualizing the occupancy grid map.
The grid map displays:
- Unknown/unexplored cells in gray
- Free cells in white
- Occupied cells in black
- Robot position as a red circle
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from shared.classes.cell import Cell
from shared.classes.robot_position import RobotPosition
import math
import numpy as np
import os


def grid_map(
    n: int,
    m: int,
    w: float,
    freeCells: list,
    occupiedCells: list,
    robotPosition: RobotPosition,
):
    """
    Creates and displays a grid map visualization.

    Parameters:
    -----------
    n : int
        Number of rows in the grid map
    m : int
        Number of columns in the grid map
    w : float
        Width (and height) of each cell in the grid
    freeCells : list
        List of Cell objects representing free (unoccupied) cells
    occupiedCells : list
        List of Cell objects representing occupied cells
    robotPosition : RobotPosition
        RobotPosition object containing the robot's X, Y coordinates and Theta orientation

    Returns:
    --------
    fig, ax : tuple
        The matplotlib figure and axes objects for further customization if needed
    """

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 10))

    # Set the limits of the plot based on grid dimensions
    ax.set_xlim(0, m * w)
    ax.set_ylim(0, n * w)

    # Set aspect ratio to equal so cells appear square
    ax.set_aspect("equal")

    # Add labels and title
    ax.set_xlabel("X (meters)", fontsize=12)
    ax.set_ylabel("Y (meters)", fontsize=12)
    ax.set_title(
        f"Occupancy Grid Map ({n} rows × {m} columns, cell width: {w}m)", fontsize=14
    )

    # First, fill all cells with gray (unknown/unexplored)
    for i in range(n):
        for j in range(m):
            rect = patches.Rectangle(
                (j * w, i * w),  # (x, y) position of bottom-left corner
                w,  # width
                w,  # height
                linewidth=0,
                edgecolor=None,
                facecolor="gray",
                alpha=0.3,
            )
            ax.add_patch(rect)

    # Draw grid lines
    for i in range(n + 1):
        ax.axhline(y=i * w, color="gray", linewidth=0.3, alpha=0.5)
    for j in range(m + 1):
        ax.axvline(x=j * w, color="gray", linewidth=0.3, alpha=0.5)

    # Draw free cells (white)
    for cell in freeCells:
        rect = patches.Rectangle(
            (cell.column * w, cell.row * w),  # (x, y) position of bottom-left corner
            w,  # width
            w,  # height
            linewidth=0,
            edgecolor=None,
            facecolor="white",
        )
        ax.add_patch(rect)

    # Draw occupied cells (black)
    for cell in occupiedCells:
        rect = patches.Rectangle(
            (cell.column * w, cell.row * w),  # (x, y) position of bottom-left corner
            w,  # width
            w,  # height
            linewidth=0,
            edgecolor=None,
            facecolor="black",
        )
        ax.add_patch(rect)

    # Draw robot position as a red circle
    robot_radius = w * 0.5  # Robot circle radius is half the cell width
    robot_circle = patches.Circle(
        (robotPosition.X, robotPosition.Y),  # (x, y) center position
        robot_radius,  # radius
        linewidth=2,
        edgecolor="darkred",
        facecolor="red",
        alpha=0.8,
        zorder=10,  # Draw on top of everything else
    )
    ax.add_patch(robot_circle)

    # Optional: Draw robot orientation as a line from the center
    # This shows which direction the robot is facing
    import math

    orientation_length = w * 0.8
    end_x = robotPosition.X + orientation_length * math.cos(
        math.radians(robotPosition.Theta)
    )
    end_y = robotPosition.Y + orientation_length * math.sin(
        math.radians(robotPosition.Theta)
    )
    ax.plot(
        [robotPosition.X, end_x],
        [robotPosition.Y, end_y],
        color="yellow",
        linewidth=2,
        zorder=11,
    )

    # Add legend
    from matplotlib.lines import Line2D

    legend_elements = [
        patches.Patch(facecolor="gray", edgecolor="black", alpha=0.3, label="Unknown"),
        patches.Patch(facecolor="white", edgecolor="black", label="Free Cells"),
        patches.Patch(facecolor="black", edgecolor="black", label="Occupied Cells"),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="red",
            markersize=10,
            label="Robot Position",
        ),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=10)

    # Invert y-axis so that row 0 is at the top (optional, comment out if not needed)
    # ax.invert_yaxis()

    # Display grid
    plt.tight_layout()
    plt.show()

    return fig, ax


def grid_map_animated(
    n: int,
    m: int,
    w: float,
    all_free_cells_list: list,
    all_occupied_cells_list: list,
    robot_positions_list: list,
    interval: int = 50,
    prob_occupied_given_occupied: float = 0.9,
    prob_occupied_given_free: float = 0.3,
    save_frames: bool = True,
    output_dir: str = None,
    numSim: int = 0,
):
    """
    Creates an animated grid map visualization with probability-based occupancy updates.
    Uses log-odds representation for efficient Bayesian updates.

    Parameters:
    -----------
    n : int
        Number of rows in the grid map
    m : int
        Number of columns in the grid map
    w : float
        Width (and height) of each cell in the grid
    all_free_cells_list : list
        List of lists of free cells for each frame
    all_occupied_cells_list : list
        List of lists of occupied cells for each frame
    robot_positions_list : list
        List of RobotPosition objects for each frame
    interval : int, optional
        Time interval between frames in milliseconds (default: 50ms)
    prob_occupied_given_occupied : float, optional
        Probability that a cell is occupied when measured as occupied (default: 0.9)
    prob_occupied_given_free : float, optional
        Probability that a cell is occupied when measured as free (default: 0.3)
    save_frames : bool, optional
        Whether to save each frame as an image (default: True)
    output_dir : str, optional
        Directory to save frames. If None, saves in current directory

    Returns:
    --------
    fig, ax, anim : tuple
        The matplotlib figure, axes, and animation objects
    """

    # Create output directory for frames if saving is enabled
    if save_frames:
        if output_dir is None:
            output_dir = (
                r"D:\_Dokumentumok\BME\2025-26 I\Mobil Gépek Mechatronikája\Képek"
            )
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        print(f"Frames will be saved to: {output_dir}")

    # Initialize occupancy grid with log-odds values
    # 0 = unknown (probability 0.5), positive = more likely occupied, negative = more likely free
    log_odds_grid = np.zeros((n, m), dtype=float)

    # Calculate log-odds updates for occupied and free observations
    # Log-odds = log(p / (1-p))
    log_odds_occupied = math.log(
        prob_occupied_given_occupied / (1 - prob_occupied_given_occupied)
    )
    log_odds_free = math.log(prob_occupied_given_free / (1 - prob_occupied_given_free))

    # Thresholds for visualization (in probability space)
    occupied_threshold = 0.7  # Above this probability, show as occupied (black)
    free_threshold = 0.3  # Below this probability, show as free (white)

    num_frames = len(all_free_cells_list)

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 10))

    # Set the limits of the plot based on grid dimensions
    ax.set_xlim(0, m * w)
    ax.set_ylim(0, n * w)

    # Set aspect ratio to equal so cells appear square
    ax.set_aspect("equal")

    # Add labels and title
    ax.set_xlabel("X (meters)", fontsize=12)
    ax.set_ylabel("Y (meters)", fontsize=12)
    title = ax.set_title(
        f"Occupancy Grid Map (Probabilistic) - Frame 0/{num_frames}",
        fontsize=14,
    )

    # Draw grid lines
    for i in range(n + 1):
        ax.axhline(y=i * w, color="gray", linewidth=0.3, alpha=0.5)
    for j in range(m + 1):
        ax.axvline(x=j * w, color="gray", linewidth=0.3, alpha=0.5)

    # Initialize cell patches dictionary (keyed by (row, col))
    cell_patches = {}

    # Initialize all cells as gray (unknown)
    for i in range(n):
        for j in range(m):
            rect = patches.Rectangle(
                (j * w, i * w),
                w,
                w,
                linewidth=0,
                edgecolor=None,
                facecolor="gray",
                alpha=0.3,
            )
            ax.add_patch(rect)
            cell_patches[(i, j)] = rect

    # Initialize robot marker
    robot_circle = patches.Circle(
        (0, 0),
        w * 0.5,
        linewidth=2,
        edgecolor="darkred",
        facecolor="red",
        alpha=0.8,
        zorder=10,
    )
    ax.add_patch(robot_circle)

    # Initialize orientation line
    (orientation_line,) = ax.plot([], [], color="yellow", linewidth=2, zorder=11)

    # Add legend
    from matplotlib.lines import Line2D

    legend_elements = [
        patches.Patch(facecolor="gray", edgecolor="black", alpha=0.3, label="Unknown"),
        patches.Patch(facecolor="white", edgecolor="black", label="Free (p < 0.3)"),
        patches.Patch(facecolor="black", edgecolor="black", label="Occupied (p > 0.7)"),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="red",
            markersize=10,
            label="Robot Position",
        ),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=10)

    def log_odds_to_probability(log_odds):
        """Convert log-odds to probability"""
        return 1.0 - (1.0 / (1.0 + math.exp(log_odds)))

    def update_cell_color(row, col):
        """Update the color of a cell based on its occupancy probability"""
        prob = log_odds_to_probability(log_odds_grid[row, col])

        if prob > occupied_threshold:
            # High probability of being occupied -> black
            cell_patches[(row, col)].set_facecolor("black")
            cell_patches[(row, col)].set_alpha(1.0)
        elif prob < free_threshold:
            # Low probability of being occupied (i.e., free) -> white
            cell_patches[(row, col)].set_facecolor("white")
            cell_patches[(row, col)].set_alpha(1.0)
        else:
            # Uncertain -> gray with varying transparency
            cell_patches[(row, col)].set_facecolor("gray")
            cell_patches[(row, col)].set_alpha(0.3)

    def init():
        """Initialize animation"""
        robot_circle.center = (0, 0)
        orientation_line.set_data([], [])
        return [robot_circle, orientation_line]

    def update(frame):
        """Update function for animation"""
        # Get pre-computed data for this frame
        robotPosition = robot_positions_list[frame]
        freeCells = all_free_cells_list[frame]
        occupiedCells = all_occupied_cells_list[frame]

        # Track which cells were updated for return value
        updated_cells = []

        # Update log-odds for free cells
        for cell in freeCells:
            log_odds_grid[cell.row, cell.column] += log_odds_free
            update_cell_color(cell.row, cell.column)
            updated_cells.append(cell_patches[(cell.row, cell.column)])

        # Update log-odds for occupied cells
        for cell in occupiedCells:
            log_odds_grid[cell.row, cell.column] += log_odds_occupied
            update_cell_color(cell.row, cell.column)
            updated_cells.append(cell_patches[(cell.row, cell.column)])

        # Update robot position
        robot_circle.center = (robotPosition.X, robotPosition.Y)

        # Update orientation line
        orientation_length = w * 0.8
        end_x = robotPosition.X + orientation_length * math.cos(
            math.radians(robotPosition.Theta)
        )
        end_y = robotPosition.Y + orientation_length * math.sin(
            math.radians(robotPosition.Theta)
        )
        orientation_line.set_data([robotPosition.X, end_x], [robotPosition.Y, end_y])

        # Update title
        title.set_text(
            f"Occupancy Grid Map (Probabilistic) - Frame {frame + 1}/{num_frames} | Position: ({robotPosition.X:.2f}, {robotPosition.Y:.2f})"
        )

        # Save frame as image
        if save_frames:
            filename = os.path.join(
                output_dir, f"real_data_sim_{numSim}_frame_{frame+1}.png"
            )
            fig.savefig(filename, dpi=150, bbox_inches="tight")
            if (frame + 1) % 10 == 0:
                print(f"Saved frame {frame + 1}/{num_frames}")

        # Only return changed objects instead of all cell patches
        return [robot_circle, orientation_line, title] + updated_cells

    # Create animation
    print(f"Creating probabilistic animation with {num_frames} frames...")
    print(f"Using Bayesian occupancy grid mapping:")
    print(f"  - P(occupied | measured occupied) = {prob_occupied_given_occupied}")
    print(f"  - P(occupied | measured free) = {prob_occupied_given_free}")
    print(f"  - Occupied threshold: {occupied_threshold}")
    print(f"  - Free threshold: {free_threshold}")

    anim = animation.FuncAnimation(
        fig,
        update,
        init_func=init,
        frames=num_frames,
        interval=interval,
        blit=False,  # Disable blit to keep all cells visible
        repeat=False,  # Don't repeat the animation
    )

    plt.tight_layout()
    plt.show()

    return fig, ax, anim
