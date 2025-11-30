"""
Improved main function that demonstrates the usage of the improved occupancy grid mapper.

This version:
- Uses the first robot position as the origin (0, 0)
- Handles negative coordinates
- Dynamically expands the grid as needed
- Provides a function to get the probability matrix output
"""

import math
import timeit
import numpy as np

from ray_tracing_improved import ray_tracing_improved
from read_json import read_json
from grid_map_improved import OccupancyGridMapper

from classes.robot_position import RobotPosition
from classes.cell import Cell
from classes.lidar_sensor_data import LidarData


def process_lidar_data_improved(
    lidar_data_list: list,
    cell_width: float = 0.1,
    initial_grid_size: int = 200,
    prob_occ_occ: float = 0.9,
    prob_occ_free: float = 0.3,
    max_lidar_range: float = 4.2,
    step: int = 1,
    verbose: bool = True,
):
    """
    Process lidar data and build an occupancy grid map with dynamic bounds.

    Parameters:
    -----------
    lidar_data_list : list
        List of dictionaries containing robot positions and lidar scans
    cell_width : float, optional
        Width of each grid cell in meters (default: 0.1)
    initial_grid_size : int, optional
        Initial grid size in cells (default: 200)
    prob_occ_occ : float, optional
        P(occupied | measured occupied) (default: 0.9)
    prob_occ_free : float, optional
        P(occupied | measured free) (default: 0.3)
    max_lidar_range : float, optional
        Maximum range of the lidar sensor (default: 4.2)
    step : int, optional
        Process every Nth position (default: 1, process all)
    verbose : bool, optional
        Print progress information (default: True)

    Returns:
    --------
    OccupancyGridMapper
        The occupancy grid mapper object with all scans processed
    """

    if not lidar_data_list:
        raise ValueError("lidar_data_list is empty")

    # Use the first robot position as the origin
    first_position = lidar_data_list[0]["robot_position"]
    origin_x = first_position.X
    origin_y = first_position.Y

    if verbose:
        print(f"\n{'='*60}")
        print(f"IMPROVED OCCUPANCY GRID MAPPER")
        print(f"{'='*60}")
        print(f"Origin set to first robot position: ({origin_x:.3f}, {origin_y:.3f})")
        print(f"Total data items: {len(lidar_data_list)}")
        print(f"Processing every {step} position(s)")
        print(f"Cell width: {cell_width} m")
        print(f"Initial grid size: {initial_grid_size} x {initial_grid_size} cells")
        print(f"Sensor model: P(occ|occ)={prob_occ_occ}, P(occ|free)={prob_occ_free}")
        print(f"{'='*60}\n")

    # Initialize the occupancy grid mapper
    mapper = OccupancyGridMapper(
        cell_width=cell_width,
        initial_size=initial_grid_size,
        prob_occupied_given_occupied=prob_occ_occ,
        prob_occupied_given_free=prob_occ_free,
        origin_x=origin_x,
        origin_y=origin_y,
    )

    start_time = timeit.default_timer() * 1000.0

    # Process each position
    for data_idx in range(0, len(lidar_data_list), step):
        if verbose and (data_idx % 100 == 0 or data_idx == 0):
            elapsed = timeit.default_timer() * 1000.0 - start_time
            print(
                f"Processing position {data_idx + 1}/{len(lidar_data_list)} "
                f"(Elapsed: {elapsed:.1f} ms)"
            )

        robot_position = lidar_data_list[data_idx]["robot_position"]
        scans = lidar_data_list[data_idx]["scans"]

        # Collect all coordinates from this scan for potential grid expansion
        x_coords = [robot_position.X]
        y_coords = [robot_position.Y]

        # Estimate potential reach of lidar beams
        for scan in scans:
            if scan.length > 0:
                # Calculate absolute angle
                alpha = robot_position.Theta + scan.alpha
                if alpha >= 360:
                    alpha -= 360
                # Estimate end point
                end_x = robot_position.X + scan.length * math.cos(math.radians(alpha))
                end_y = robot_position.Y + scan.length * math.sin(math.radians(alpha))
                x_coords.append(end_x)
                y_coords.append(end_y)
            else:
                # Use max range for no-detection
                alpha = robot_position.Theta + scan.alpha
                if alpha >= 360:
                    alpha -= 360
                end_x = robot_position.X + max_lidar_range * math.cos(
                    math.radians(alpha)
                )
                end_y = robot_position.Y + max_lidar_range * math.sin(
                    math.radians(alpha)
                )
                x_coords.append(end_x)
                y_coords.append(end_y)

        # Expand grid if needed
        mapper._expand_grid_if_needed(x_coords, y_coords)

        # Process each lidar scan
        all_free_cells = []
        all_occupied_cells = []

        for scan in scans:
            free_cells, end_cell = ray_tracing_improved(
                robot_position,
                scan,
                w=cell_width,
                min_x=mapper.min_x,
                min_y=mapper.min_y,
                max_range=max_lidar_range,
            )

            all_free_cells.extend(free_cells)
            if end_cell is not None:
                all_occupied_cells.append(end_cell)

        # Update the occupancy grid
        mapper.update_with_scan(all_free_cells, all_occupied_cells)

    stop_time = timeit.default_timer() * 1000.0

    if verbose:
        print(f"\n{'='*60}")
        print(f"PROCESSING COMPLETE")
        print(f"{'='*60}")
        print(f"Total processing time: {stop_time - start_time:.2f} ms")
        print(
            f"Average time per position: {(stop_time - start_time) / (len(lidar_data_list) / step):.2f} ms"
        )

        grid_info = mapper.get_grid_info()
        print(f"\nFinal Grid Information:")
        print(
            f"  Grid dimensions: {grid_info['n_rows']} rows x {grid_info['n_cols']} cols"
        )
        print(f"  Cell width: {grid_info['cell_width']} m")
        print(f"  Grid coverage:")
        print(
            f"    X: [{grid_info['min_x']:.3f}, {grid_info['max_x']:.3f}] ({grid_info['grid_width_m']:.3f} m)"
        )
        print(
            f"    Y: [{grid_info['min_y']:.3f}, {grid_info['max_y']:.3f}] ({grid_info['grid_height_m']:.3f} m)"
        )
        print(f"  Origin: ({grid_info['origin_x']:.3f}, {grid_info['origin_y']:.3f})")
        print(f"{'='*60}\n")

    return mapper


def save_probability_grid_to_file(
    mapper: OccupancyGridMapper, filename: str = "occupancy_grid.npz"
):
    """
    Save the occupancy grid probability map to a file.

    Parameters:
    -----------
    mapper : OccupancyGridMapper
        The occupancy grid mapper object
    filename : str, optional
        Output filename (default: "occupancy_grid.npz")
    """
    grid_data = mapper.get_probability_grid_with_bounds()

    # Save to npz file (compressed numpy format)
    np.savez_compressed(
        filename,
        probabilities=grid_data["probabilities"],
        min_x=grid_data["min_x"],
        max_x=grid_data["max_x"],
        min_y=grid_data["min_y"],
        max_y=grid_data["max_y"],
        n_rows=grid_data["n_rows"],
        n_cols=grid_data["n_cols"],
        cell_width=grid_data["cell_width"],
        origin_x=grid_data["origin_x"],
        origin_y=grid_data["origin_y"],
    )

    print(f"Occupancy grid saved to: {filename}")
    print(f"Grid shape: {grid_data['probabilities'].shape}")
    print(f"Min probability: {np.min(grid_data['probabilities']):.4f}")
    print(f"Max probability: {np.max(grid_data['probabilities']):.4f}")
    print(f"Mean probability: {np.mean(grid_data['probabilities']):.4f}")


def load_probability_grid_from_file(filename: str = "occupancy_grid.npz"):
    """
    Load the occupancy grid probability map from a file.

    Parameters:
    -----------
    filename : str, optional
        Input filename (default: "occupancy_grid.npz")

    Returns:
    --------
    dict
        Dictionary containing the grid data
    """
    data = np.load(filename)
    return {
        "probabilities": data["probabilities"],
        "min_x": float(data["min_x"]),
        "max_x": float(data["max_x"]),
        "min_y": float(data["min_y"]),
        "max_y": float(data["max_y"]),
        "n_rows": int(data["n_rows"]),
        "n_cols": int(data["n_cols"]),
        "cell_width": float(data["cell_width"]),
        "origin_x": float(data["origin_x"]),
        "origin_y": float(data["origin_y"]),
    }


def main(args=None):
    """
    Main function demonstrating the improved occupancy grid mapper.
    """

    print("=" * 70)
    print(" IMPROVED OCCUPANCY GRID MAPPER - DEMONSTRATION")
    print("=" * 70)

    # Read the lidar data
    print("\nReading lidar data from JSON file...")
    lidar_data_list = read_json(printPath=True)

    print(f"Successfully loaded {len(lidar_data_list)} data items")

    # Check the first robot position (which will become the origin)
    first_pos = lidar_data_list[0]["robot_position"]
    print(f"\nFirst robot position (will be used as origin):")
    print(f"  X: {first_pos.X:.3f} m")
    print(f"  Y: {first_pos.Y:.3f} m")
    print(f"  Theta: {first_pos.Theta:.3f} degrees")

    # Process the lidar data with improved algorithm
    # Use every 10th position for faster processing (you can change step=1 for all data)
    step = 10

    mapper = process_lidar_data_improved(
        lidar_data_list=lidar_data_list,
        cell_width=0.1,
        initial_grid_size=200,
        prob_occ_occ=0.9,
        prob_occ_free=0.3,
        max_lidar_range=4.2,
        step=step,
        verbose=True,
    )

    # Get the probability grid
    print("\nExtracting probability grid...")
    grid_data = mapper.get_probability_grid_with_bounds()

    prob_grid = grid_data["probabilities"]
    print(f"Probability grid shape: {prob_grid.shape}")
    print(f"Probability range: [{np.min(prob_grid):.4f}, {np.max(prob_grid):.4f}]")

    # Count cells by type
    unknown_cells = np.sum((prob_grid >= 0.3) & (prob_grid <= 0.7))
    free_cells = np.sum(prob_grid < 0.3)
    occupied_cells = np.sum(prob_grid > 0.7)
    total_cells = prob_grid.size

    print(f"\nCell Statistics:")
    print(f"  Total cells: {total_cells}")
    print(f"  Free cells (p < 0.3): {free_cells} ({100*free_cells/total_cells:.1f}%)")
    print(
        f"  Occupied cells (p > 0.7): {occupied_cells} ({100*occupied_cells/total_cells:.1f}%)"
    )
    print(
        f"  Unknown cells (0.3 <= p <= 0.7): {unknown_cells} ({100*unknown_cells/total_cells:.1f}%)"
    )

    # Save the probability grid to a file
    print("\nSaving probability grid to file...")
    save_probability_grid_to_file(mapper, "occupancy_grid_improved.npz")

    # Demonstrate loading the saved grid
    print("\nDemonstrating loading saved grid...")
    loaded_grid = load_probability_grid_from_file("occupancy_grid_improved.npz")
    print(f"Successfully loaded grid with shape: {loaded_grid['probabilities'].shape}")

    # Example: Query occupancy probability at specific world coordinates
    print("\n" + "=" * 70)
    print("EXAMPLE: Query occupancy at specific world coordinates")
    print("=" * 70)

    # Get some example positions
    example_positions = [
        (first_pos.X, first_pos.Y),  # Origin (first robot position)
        (first_pos.X + 1.0, first_pos.Y + 1.0),  # 1m diagonal from origin
        (first_pos.X - 0.5, first_pos.Y + 2.0),  # Negative X offset
    ]

    for x, y in example_positions:
        grid_idx = mapper.world_to_grid(x, y)
        if grid_idx is not None:
            row, col = grid_idx
            prob = prob_grid[row, col]
            print(f"Position ({x:.3f}, {y:.3f}):")
            print(f"  Grid index: row={row}, col={col}")
            print(f"  Occupancy probability: {prob:.4f}")
            if prob < 0.3:
                print(f"  Status: FREE")
            elif prob > 0.7:
                print(f"  Status: OCCUPIED")
            else:
                print(f"  Status: UNKNOWN")
        else:
            print(f"Position ({x:.3f}, {y:.3f}): OUT OF BOUNDS")
        print()

    print("=" * 70)
    print("DEMONSTRATION COMPLETE")
    print("=" * 70)
    print("\nThe probability grid is now ready for use in your application!")
    print("You can:")
    print("  1. Use mapper.get_probability_grid() to get the 2D probability array")
    print(
        "  2. Use mapper.get_probability_grid_with_bounds() to get grid with metadata"
    )
    print("  3. Use mapper.world_to_grid(x, y) to convert world coords to grid indices")
    print(
        "  4. Use mapper.grid_to_world(row, col) to convert grid indices to world coords"
    )
    print("=" * 70)


if __name__ == "__main__":
    main()
