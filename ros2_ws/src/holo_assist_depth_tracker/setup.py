from setuptools import find_packages, setup
from glob import glob
import os


package_name = "holo_assist_depth_tracker"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.urdf")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.png")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.dae")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.obj")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.STEP")),
        (os.path.join("share", package_name, "worlds", "urdf"), glob("worlds/urdf/*.xacro")),
        (
            os.path.join("share", package_name, "worlds", "meshes", "apriltags"),
            glob("worlds/meshes/apriltags/*.dae"),
        ),
        (
            os.path.join("share", package_name, "worlds", "textures", "apriltags"),
            glob("worlds/textures/apriltags/*.png"),
        ),
        (os.path.join("share", package_name, "worlds", "rviz"), glob("worlds/rviz/*.rviz")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.md")),
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
            "holo_assist_depth_tracker_foxglove_relay = holo_assist_depth_tracker.dashboard_relay_node:main",
            "foxglove_relay = holo_assist_depth_tracker.dashboard_relay_node:main",
            "holo_assist_webcam_image_publisher = holo_assist_depth_tracker.webcam_image_publisher_node:main",
            "webcam_image_publisher = holo_assist_depth_tracker.webcam_image_publisher_node:main",
            "workspace_perception_node = holo_assist_depth_tracker.workspace_perception_node:main",
            "holoassist_workspace_perception = holo_assist_depth_tracker.workspace_perception_node:main",
            "holoassist_detection_merge_node = holo_assist_depth_tracker.nodes.detection_merge_node:main",
            "holoassist_workspace_board_node = holo_assist_depth_tracker.nodes.workspace_board_node:main",
            "holoassist_cube_pose_node = holo_assist_depth_tracker.nodes.cube_pose_node:main",
            "holoassist_overlay_node = holo_assist_depth_tracker.nodes.overlay_node:main",
            "holoassist_sim_apriltag_publisher_node = holo_assist_depth_tracker.nodes.sim_apriltag_publisher_node:main",
        ],
    },
)
