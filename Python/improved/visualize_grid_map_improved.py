"""
Visualization tool for the improved occupancy grid map.

This module provides functions to visualize the probability-based occupancy grid map
using matplotlib with a color scheme based on occupancy probabilities:
- p < 0.3: Free space (white)
- 0.3 <= p <= 0.7: Unknown (gray)
- p > 0.7: Occupied (black)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap


def visualize_occupancy_grid(
    probability_map: np.ndarray,
    grid_bounds: dict = None,
    title: str = "Occupancy Grid Map",
    show_grid_lines: bool = True,
    show_colorbar: bool = True,
    figsize: tuple = (12, 10),
    save_path: str = None,
):
    """
    Visualize an occupancy grid map with probability-based coloring.

    Parameters:
    -----------
    probability_map : np.ndarray
        2D array of occupancy probabilities (0.0 to 1.0)
    grid_bounds : dict, optional
        Dictionary with keys: 'min_x', 'max_x', 'min_y', 'max_y', 'cell_width', 'origin_x', 'origin_y'
        If provided, displays real-world coordinates
    title : str, optional
        Title for the plot (default: "Occupancy Grid Map")
    show_grid_lines : bool, optional
        Whether to show grid lines (default: True)
    show_colorbar : bool, optional
        Whether to show the colorbar (default: True)
    figsize : tuple, optional
        Figure size (width, height) in inches (default: (12, 10))
    save_path : str, optional
        If provided, saves the figure to this path instead of showing it

    Returns:
    --------
    fig, ax : tuple
        Matplotlib figure and axes objects
    """

    # Create figure and axis
    fig, ax = plt.subplots(figsize=figsize)

    # Define color mapping
    # p < 0.3: white (free)
    # 0.3 <= p <= 0.7: gray (unknown)
    # p > 0.7: black (occupied)
    colors = ["white", "gray", "black"]
    n_bins = 100
    cmap = LinearSegmentedColormap.from_list(
        "occupancy",
        [
            (0.0, "white"),
            (0.3, "white"),
            (0.3, "gray"),
            (0.7, "gray"),
            (0.7, "black"),
            (1.0, "black"),
        ],
        N=n_bins,
    )

    # Display the grid
    # Note: imshow displays with origin at top-left by default
    # We use origin='lower' to match the coordinate system (bottom-left origin)
    im = ax.imshow(
        probability_map,
        cmap=cmap,
        vmin=0.0,
        vmax=1.0,
        origin="lower",
        aspect="equal",
        interpolation="nearest",
    )

    # Set extent based on whether we have grid bounds
    if grid_bounds is not None:
        # Use real-world coordinates
        # extent = [left, right, bottom, top]
        extent = [
            grid_bounds["min_x"],
            grid_bounds["max_x"],
            grid_bounds["min_y"],
            grid_bounds["max_y"],
        ]
        im.set_extent(extent)
        # Explicitly set axis limits to match the extent
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])

    # Set up axes based on whether we have grid bounds
    if grid_bounds is not None:
        ax.set_xlabel("X (meters)", fontsize=12)
        ax.set_ylabel("Y (meters)", fontsize=12)

        # Add origin marker if available
        if "origin_x" in grid_bounds and "origin_y" in grid_bounds:
            ax.plot(
                grid_bounds["origin_x"],
                grid_bounds["origin_y"],
                "r*",
                markersize=15,
                label="Origin (First Robot Position)",
                zorder=10,
            )
            ax.legend(loc="upper right")

        # Add grid lines at cell boundaries
        if show_grid_lines and "cell_width" in grid_bounds:
            cell_width = grid_bounds["cell_width"]
            n_rows, n_cols = probability_map.shape

            # Vertical lines (every 10 cells to avoid clutter)
            step = max(1, int(1.0 / cell_width))  # At least every 1 meter
            for i in range(0, n_cols + 1, step):
                x = grid_bounds["min_x"] + i * cell_width
                ax.axvline(x=x, color="gray", linewidth=0.3, alpha=0.3)

            # Horizontal lines
            for i in range(0, n_rows + 1, step):
                y = grid_bounds["min_y"] + i * cell_width
                ax.axhline(y=y, color="gray", linewidth=0.3, alpha=0.3)
    else:
        # Use grid indices
        ax.set_xlabel("Column Index", fontsize=12)
        ax.set_ylabel("Row Index", fontsize=12)

        if show_grid_lines:
            # Add grid lines at cell boundaries (every 10 cells)
            n_rows, n_cols = probability_map.shape
            step = max(1, n_cols // 20)  # About 20 lines

            for i in range(0, n_cols + 1, step):
                ax.axvline(x=i - 0.5, color="gray", linewidth=0.3, alpha=0.3)

            for i in range(0, n_rows + 1, step):
                ax.axhline(y=i - 0.5, color="gray", linewidth=0.3, alpha=0.3)

    # Add colorbar
    if show_colorbar:
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Occupancy Probability", rotation=270, labelpad=20, fontsize=12)

        # Add labels for interpretation
        cbar.ax.text(
            1.5,
            0.15,
            "Free\n(p < 0.3)",
            transform=cbar.ax.transAxes,
            fontsize=10,
            va="center",
        )
        cbar.ax.text(
            1.5,
            0.5,
            "Unknown\n(0.3 ≤ p ≤ 0.7)",
            transform=cbar.ax.transAxes,
            fontsize=10,
            va="center",
        )
        cbar.ax.text(
            1.5,
            0.85,
            "Occupied\n(p > 0.7)",
            transform=cbar.ax.transAxes,
            fontsize=10,
            va="center",
        )

    # Set title with grid info
    if grid_bounds is not None:
        n_rows, n_cols = probability_map.shape
        title_full = (
            f"{title}\n{n_rows} rows × {n_cols} columns, "
            f"cell width: {grid_bounds.get('cell_width', 'N/A')} m"
        )
    else:
        n_rows, n_cols = probability_map.shape
        title_full = f"{title}\n{n_rows} rows × {n_cols} columns"

    ax.set_title(title_full, fontsize=14, fontweight="bold")

    # Add statistics text box
    free_cells = np.sum(probability_map < 0.3)
    unknown_cells = np.sum((probability_map >= 0.3) & (probability_map <= 0.7))
    occupied_cells = np.sum(probability_map > 0.7)
    total_cells = probability_map.size

    stats_text = (
        f"Statistics:\n"
        f"Free: {free_cells:,} ({100*free_cells/total_cells:.1f}%)\n"
        f"Unknown: {unknown_cells:,} ({100*unknown_cells/total_cells:.1f}%)\n"
        f"Occupied: {occupied_cells:,} ({100*occupied_cells/total_cells:.1f}%)"
    )

    ax.text(
        0.02,
        0.98,
        stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    plt.tight_layout()

    # Save or show
    if save_path is not None:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Figure saved to: {save_path}")
    else:
        plt.show()

    return fig, ax


def visualize_from_mapper(
    mapper,
    title: str = "Occupancy Grid Map (Improved)",
    show_grid_lines: bool = True,
    show_colorbar: bool = True,
    figsize: tuple = (12, 10),
    save_path: str = None,
):
    """
    Visualize occupancy grid directly from an OccupancyGridMapper object.

    Parameters:
    -----------
    mapper : OccupancyGridMapper
        The occupancy grid mapper object
    title : str, optional
        Title for the plot
    show_grid_lines : bool, optional
        Whether to show grid lines (default: True)
    show_colorbar : bool, optional
        Whether to show the colorbar (default: True)
    figsize : tuple, optional
        Figure size (width, height) in inches
    save_path : str, optional
        If provided, saves the figure to this path

    Returns:
    --------
    fig, ax : tuple
        Matplotlib figure and axes objects
    """
    # Get probability grid and bounds from mapper
    grid_data = mapper.get_probability_grid_with_bounds()

    return visualize_occupancy_grid(
        probability_map=grid_data["probabilities"],
        grid_bounds=grid_data,
        title=title,
        show_grid_lines=show_grid_lines,
        show_colorbar=show_colorbar,
        figsize=figsize,
        save_path=save_path,
    )


def main():
    """
    Main function to demonstrate visualization with real data.
    """
    print("=" * 70)
    print("OCCUPANCY GRID MAP VISUALIZATION")
    print("=" * 70)

    # Import required modules
    from read_json import read_json
    from main_improved import process_lidar_data_improved

    print("\n[1/3] Loading lidar data...")
    lidar_data_list = read_json()
    print(f"✓ Loaded {len(lidar_data_list)} scans")

    print("\n[2/3] Processing lidar data...")
    print("      (Processing every 10th position for faster demo)")

    mapper = process_lidar_data_improved(
        lidar_data_list=lidar_data_list,
        cell_width=0.1,
        initial_grid_size=200,  # Initial grid size
        step=10,  # Process every 10th for speed
        verbose=False,
    )
    print("✓ Processing complete")

    print("\n[3/3] Visualizing occupancy grid...")

    # Get probability map
    prob_map = mapper.get_probability_grid()
    grid_data = mapper.get_probability_grid_with_bounds()

    print(f"  Grid size: {prob_map.shape[0]} rows × {prob_map.shape[1]} columns")
    print(f"  Coverage: X=[{grid_data['min_x']:.2f}, {grid_data['max_x']:.2f}] m")
    print(f"            Y=[{grid_data['min_y']:.2f}, {grid_data['max_y']:.2f}] m")
    print(f"  Origin: ({grid_data['origin_x']:.2f}, {grid_data['origin_y']:.2f})")

    # Create visualization
    print("\n✓ Creating visualization...")
    print("  Color scheme:")
    print("    • White  = Free space (p < 0.3)")
    print("    • Gray   = Unknown (0.3 ≤ p ≤ 0.7)")
    print("    • Black  = Occupied (p > 0.7)")
    print("    • Red *  = Origin (first robot position)")

    # Option 1: Visualize with bounds (world coordinates)
    visualize_from_mapper(
        mapper,
        title="Occupancy Grid Map - Real Data",
        show_grid_lines=True,
        show_colorbar=True,
        figsize=(14, 12),
    )

    # Option 2: You can also visualize just the array
    # visualize_occupancy_grid(
    #     prob_map,
    #     grid_bounds=grid_data,
    #     title="My Custom Map"
    # )

    print("\n" + "=" * 70)
    print("✓ VISUALIZATION COMPLETE")
    print("=" * 70)
    print("\nClose the window to exit.")


if __name__ == "__main__":
    main()
