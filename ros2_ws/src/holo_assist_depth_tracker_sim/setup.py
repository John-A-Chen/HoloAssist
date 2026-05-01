from glob import glob
import os

from setuptools import find_packages, setup

package_name = "holo_assist_depth_tracker_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="john",
    maintainer_email="john@example.com",
    description="Geometry-driven simulation pipeline for HoloAssist ROS2 backend testing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_cube_truth_node = holo_assist_depth_tracker_sim.sim_cube_truth_node:main",
            "sim_cube_perception_node = holo_assist_depth_tracker_sim.sim_cube_perception_node:main",
            "sim_cube_moveit_bridge_node = holo_assist_depth_tracker_sim.sim_cube_moveit_bridge_node:main",
        ],
    },
)
