from setuptools import setup
from glob import glob
import os


package_name = "holo_assist_depth_tracker"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="john",
    maintainer_email="john@example.com",
    description="Depth-only blob tracker for ROS 2 Humble using RealSense depth images.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "holo_assist_depth_tracker_node = holo_assist_depth_tracker.depth_tracker_node:main",
            "depth_tracker_node = holo_assist_depth_tracker.depth_tracker_node:main",
            "holo_assist_depth_tracker_dashboard_relay = holo_assist_depth_tracker.dashboard_relay_node:main",
            "depth_tracker_dashboard_relay = holo_assist_depth_tracker.dashboard_relay_node:main",
            "holo_assist_webcam_image_publisher = holo_assist_depth_tracker.webcam_image_publisher_node:main",
            "webcam_image_publisher = holo_assist_depth_tracker.webcam_image_publisher_node:main",
        ],
    },
)
