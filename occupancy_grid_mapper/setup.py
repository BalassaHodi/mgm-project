from setuptools import find_packages, setup

package_name = "occupancy_grid_mapper"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/occupancy_grid_mapper.launch.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="balassa",
    maintainer_email="hodi.balassa@edu.bme.hu",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "test_node = occupancy_grid_mapper.test_node:main",
            "get_extreme_values = occupancy_grid_mapper.get_extreme_values:main",
        ],
    },
)
