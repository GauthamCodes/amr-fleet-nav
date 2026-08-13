"""Build configuration for the amr_fleet_control package."""

from glob import glob

from setuptools import find_packages, setup

package_name = "amr_fleet_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # setuptools evaluates these globs at build time and colcon re-runs the build
        # whenever a file changes, so a new launch or config file is picked up.
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gautham",
    maintainer_email="gauthamanil888@gmail.com",
    description="Cooperative mapping and traffic coordination for the AMR fleet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_map_node = amr_fleet_control.fleet_map_node:main",
            "fleet_mission = amr_fleet_control.fleet_mission:main",
            "trajectory_predictor = amr_fleet_control.trajectory_predictor:main",
            "trajectory_probe = amr_fleet_control.trajectory_probe:main",
        ],
    },
)
