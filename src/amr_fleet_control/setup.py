"""Build configuration for the amr_fleet_control package."""

from setuptools import find_packages, setup

package_name = "amr_fleet_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gautham",
    maintainer_email="gauthamanil888@gmail.com",
    description="Cooperative mapping and traffic coordination for the AMR fleet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        # FleetMapNode, TrajectoryPredictor and TrafficControlNode are registered
        # here in Phases 3, 6 and 7 respectively.
        "console_scripts": [],
    },
)
