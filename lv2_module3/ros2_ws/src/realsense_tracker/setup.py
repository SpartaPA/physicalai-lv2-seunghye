from glob import glob

from setuptools import find_packages, setup

package_name = "realsense_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Seunghye-J",
    maintainer_email="ivory0130@gmail.com",
    description="Module 3 HSV mask and detection analysis using EYE4 vision code",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = realsense_tracker.perception_node:main",
        ],
    },
)
