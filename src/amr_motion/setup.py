"""Build configuration for the amr_motion package."""

from glob import glob

from setuptools import find_packages, setup

package_name = "amr_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gautham",
    maintainer_email="gauthamanil888@gmail.com",
    description="Payload-adaptive jerk limiting for the AMR fleet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "payload_jerk_adapter = amr_motion.payload_jerk_adapter:main",
            "payload_trace = amr_motion.payload_trace:main",
        ],
    },
)
