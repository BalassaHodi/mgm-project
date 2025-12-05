#!/usr/bin/env python3
"""
Test script to verify the rearranged file structure works correctly.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def test_imports():
    """Test that all imports work correctly after rearrangement"""

    print("Testing imports after file rearrangement...")
    print("=" * 60)

    # Test shared utilities
    try:
        from shared.read_json import read_json
        from shared.calc_coordinates import calc_coordinates
        from shared.classes.cell import Cell
        from shared.classes.robot_position import RobotPosition
        from shared.classes.lidar_sensor_data import LidarData
        from shared.classes.coordinates import Coordinates

        print("✓ Shared utilities import successfully")
    except Exception as e:
        print(f"✗ Shared utilities import failed: {e}")
        import traceback

        traceback.print_exc()
        return False

    # Test improved implementation by running from its directory
    print("\nTesting improved implementation...")
    original_dir = os.getcwd()
    try:
        os.chdir("improved")
        sys.path.insert(0, os.getcwd())

        # Now imports should work since we're in the improved directory
        from main_improved import process_lidar_data_improved
        from grid_map_improved import OccupancyGridMapper

        print("✓ Improved implementation imports successfully")

        # Test a simple function call to ensure it actually works
        mapper = OccupancyGridMapper(
            origin_x=0.0, origin_y=0.0, cell_width=0.1, initial_size=10
        )
        print("✓ Improved OccupancyGridMapper instantiates successfully")

    except Exception as e:
        print(f"✗ Improved implementation import failed: {e}")
        import traceback

        traceback.print_exc()
        return False
    finally:
        os.chdir(original_dir)
        sys.path.pop(0)

    # Test original implementation by running from its directory
    print("\nTesting original implementation...")
    try:
        os.chdir("original")
        sys.path.insert(0, os.getcwd())

        from determine_cell_index import determine_cell_index

        print("✓ Original implementation imports successfully")

    except Exception as e:
        print(f"✗ Original implementation import failed: {e}")
        import traceback

        traceback.print_exc()
        return False
    finally:
        os.chdir(original_dir)
        sys.path.pop(0)

    print("\n" + "=" * 60)
    print("All imports successful! ✓")
    print()
    print("File structure:")
    print("  Python/")
    print("  ├── improved/     - Improved implementation + docs")
    print("  ├── original/     - Original implementation")
    print("  ├── shared/       - Shared utilities & classes")
    print("  └── data/         - Data files")
    print()
    print("Usage:")
    print("  - Run improved code: cd improved && python3 main_improved.py")
    print("  - Run original code: cd original && python3 main.py")

    return True


if __name__ == "__main__":
    success = test_imports()
    sys.exit(0 if success else 1)
