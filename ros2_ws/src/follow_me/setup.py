from setuptools import find_packages, setup
import os
from glob import glob

package_name = "follow_me"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROB4",
    maintainer_email="oliverfast18@gmail.com",
    description="Follow-me YOLO system for Jetson Orin Nano and Realsense D435i",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "yolo_person_center = follow_me.yolo_person_center:main",
            "person_angle = follow_me.person_angle:main",
            "person_distance = follow_me.person_distance:main",
            "distance_regulator = follow_me.distance_regulator:main",
            "angle_regulator = follow_me.angle_regulator:main",
            "command_sender = follow_me.command_sender:main",
        ],
    },
)