"""
Simple example showing how to visualize your occupancy grid map.

This demonstrates the different ways you can use the visualization functions.
"""

import numpy as np
from visualize_grid_map_improved import visualize_occupancy_grid, visualize_from_mapper
from main_improved import process_lidar_data_improved
from read_json import read_json


def example_1_visualize_from_mapper():
    """
    Example 1: Easiest way - visualize directly from OccupancyGridMapper
    """
    print("\n" + "=" * 60)
    print("EXAMPLE 1: Visualize from OccupancyGridMapper")
    print("=" * 60)

    # Process your data
    print("Processing lidar data...")
    lidar_data = read_json()
    mapper = process_lidar_data_improved(
        lidar_data,
        cell_width=0.1,
        initial_grid_size=200,  # Initial grid size
        step=10,  # Every 10th position for speed
        verbose=False,
    )
    print("✓ Processing complete")

    # Visualize - this is the easiest way!
    print("Creating visualization...")
    visualize_from_mapper(mapper, title="My Occupancy Grid Map")


def example_2_visualize_array_with_bounds():
    """
    Example 2: Visualize a probability array with world coordinates
    """
    print("\n" + "=" * 60)
    print("EXAMPLE 2: Visualize array with bounds")
    print("=" * 60)

    # Get your data
    print("Processing lidar data...")
    lidar_data = read_json()
    mapper = process_lidar_data_improved(
        lidar_data, cell_width=0.1, initial_grid_size=200, step=10, verbose=False
    )

    # Get probability map and bounds
    prob_map = mapper.get_probability_grid()
    grid_data = mapper.get_probability_grid_with_bounds()

    print(f"✓ Got probability map with shape: {prob_map.shape}")

    # Visualize
    print("Creating visualization...")
    visualize_occupancy_grid(
        probability_map=prob_map,
        grid_bounds=grid_data,
        title="Occupancy Grid with Bounds",
    )


def example_3_visualize_array_only():
    """
    Example 3: Visualize just the probability array (no world coordinates)
    """
    print("\n" + "=" * 60)
    print("EXAMPLE 3: Visualize array only (grid indices)")
    print("=" * 60)

    # Get your data
    print("Processing lidar data...")
    lidar_data = read_json()
    mapper = process_lidar_data_improved(
        lidar_data, cell_width=0.1, initial_grid_size=200, step=10, verbose=False
    )

    # Get just the probability map
    prob_map = mapper.get_probability_grid()

    print(f"✓ Got probability map with shape: {prob_map.shape}")

    # Visualize without bounds (shows grid indices instead)
    print("Creating visualization...")
    visualize_occupancy_grid(
        probability_map=prob_map,
        grid_bounds=None,  # No bounds - will show grid indices
        title="Occupancy Grid (Grid Indices)",
    )


def example_4_save_to_file():
    """
    Example 4: Save visualization to a file instead of displaying
    """
    print("\n" + "=" * 60)
    print("EXAMPLE 4: Save visualization to file")
    print("=" * 60)

    # Get your data
    print("Processing lidar data...")
    lidar_data = read_json()
    mapper = process_lidar_data_improved(
        lidar_data, cell_width=0.1, initial_grid_size=200, step=10, verbose=False
    )

    print("✓ Processing complete")

    # Save to file instead of showing
    print("Saving visualization to file...")
    visualize_from_mapper(
        mapper,
        title="My Saved Map",
        save_path="occupancy_grid_map.png",  # Saves to file
    )

    print("✓ Map saved to: occupancy_grid_map.png")


def example_5_custom_appearance():
    """
    Example 5: Customize the appearance
    """
    print("\n" + "=" * 60)
    print("EXAMPLE 5: Customize appearance")
    print("=" * 60)

    # Get your data
    print("Processing lidar data...")
    lidar_data = read_json()
    mapper = process_lidar_data_improved(
        lidar_data, cell_width=0.1, initial_grid_size=200, step=10, verbose=False
    )

    print("✓ Processing complete")

    # Customize appearance
    print("Creating custom visualization...")
    visualize_from_mapper(
        mapper,
        title="Custom Styled Map",
        show_grid_lines=False,  # Hide grid lines for cleaner look
        show_colorbar=True,
        figsize=(16, 12),  # Larger figure
    )


def example_6_work_with_numpy_array():
    """
    Example 6: Show how to work with the probability array
    """
    print("\n" + "=" * 60)
    print("EXAMPLE 6: Work with probability array")
    print("=" * 60)

    # Get your data
    print("Processing lidar data...")
    lidar_data = read_json()
    mapper = process_lidar_data_improved(
        lidar_data, cell_width=0.1, initial_grid_size=200, step=10, verbose=False
    )

    # Get the probability map as a numpy array
    prob_map = mapper.get_probability_grid()

    print(f"\nProbability map properties:")
    print(f"  Type: {type(prob_map)}")
    print(f"  Shape: {prob_map.shape}")
    print(f"  Data type: {prob_map.dtype}")
    print(f"  Min value: {prob_map.min():.4f}")
    print(f"  Max value: {prob_map.max():.4f}")
    print(f"  Mean value: {prob_map.mean():.4f}")

    # You can do any numpy operations on it
    free_cells = np.sum(prob_map < 0.3)
    unknown_cells = np.sum((prob_map >= 0.3) & (prob_map <= 0.7))
    occupied_cells = np.sum(prob_map > 0.7)

    print(f"\nCell statistics:")
    print(f"  Free cells: {free_cells:,}")
    print(f"  Unknown cells: {unknown_cells:,}")
    print(f"  Occupied cells: {occupied_cells:,}")

    # You can modify it if needed
    # For example, threshold it to binary (free=0, occupied=1)
    binary_map = (prob_map > 0.5).astype(float)

    # Visualize both
    print("\nVisualizing original and binary maps...")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    # Original probability map
    grid_data = mapper.get_probability_grid_with_bounds()
    im1 = ax1.imshow(prob_map, cmap="gray", vmin=0, vmax=1, origin="lower")
    ax1.set_title("Original Probability Map")
    ax1.set_xlabel("Column")
    ax1.set_ylabel("Row")
    plt.colorbar(im1, ax=ax1, label="Probability")

    # Binary map
    im2 = ax2.imshow(binary_map, cmap="gray", vmin=0, vmax=1, origin="lower")
    ax2.set_title("Binary Map (threshold at 0.5)")
    ax2.set_xlabel("Column")
    ax2.set_ylabel("Row")
    plt.colorbar(im2, ax=ax2, label="Binary")

    plt.tight_layout()
    plt.show()


def main():
    """
    Run one of the examples.
    """
    print("\n" + "=" * 70)
    print("VISUALIZATION EXAMPLES")
    print("=" * 70)
    print("\nChoose an example to run:")
    print("  1 - Visualize from OccupancyGridMapper (easiest)")
    print("  2 - Visualize array with world coordinates")
    print("  3 - Visualize array with grid indices only")
    print("  4 - Save visualization to file")
    print("  5 - Customize appearance")
    print("  6 - Work with numpy array and visualize")

    choice = input("\nEnter choice (1-6, or press Enter for Example 1): ").strip()

    if choice == "" or choice == "1":
        example_1_visualize_from_mapper()
    elif choice == "2":
        example_2_visualize_array_with_bounds()
    elif choice == "3":
        example_3_visualize_array_only()
    elif choice == "4":
        example_4_save_to_file()
    elif choice == "5":
        example_5_custom_appearance()
    elif choice == "6":
        example_6_work_with_numpy_array()
    else:
        print("Invalid choice. Running Example 1...")
        example_1_visualize_from_mapper()


if __name__ == "__main__":
    main()
