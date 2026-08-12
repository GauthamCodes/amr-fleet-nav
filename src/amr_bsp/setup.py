"""Build configuration for the amr_bsp package."""

from glob import glob

from setuptools import find_packages, setup

package_name = "amr_bsp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # setuptools evaluates this glob at build time, and colcon re-runs the build
        # whenever a file changes, so a new launch file is picked up. The ament_cmake
        # packages in this workspace list their scripts explicitly instead, because
        # CMake evaluates a glob only at configure time.
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    scripts=glob("scripts/*.py"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gautham",
    maintainer_email="gauthamanil888@gmail.com",
    description="BSP-style sensor validation for the AMR fleet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_bsp = amr_bsp.bsp_node:main",
        ],
    },
)
