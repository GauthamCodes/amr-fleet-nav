"""Build configuration for the amr_bsp package."""

from setuptools import find_packages, setup

package_name = "amr_bsp"

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
    description="BSP-style sensor validation for the AMR fleet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        # Validators and the PitchGate node are registered here in Phase 2.
        "console_scripts": [],
    },
)
