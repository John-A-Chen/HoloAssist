from glob import glob

from setuptools import setup


package_name = "moveit_robot_control"
python_package_name = "moveit_robot_control_node"


setup(
    name=package_name,
    version="0.0.1",
    packages=[python_package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/coordinate_listener.launch.py",
                "launch/pick_place_system.launch.py",
                "launch/pick_place.launch.py",
                "launch/workspace_scene.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/old_files/launch",
            glob("old_files/launch/*.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.json"),
        ),
        (
            "share/" + package_name + "/meshes",
            glob("meshes/*.dae"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ollie",
    maintainer_email="ollie@example.com",
    description="MoveIt-based UR robot coordinate control with a topic-driven interface.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "moveit_robot_control = moveit_robot_control_node.moveit_robot_control:main",
            (
                "coordinate_listener = "
                "moveit_robot_control_node.moveit_robot_control:coordinate_listener_main"
            ),
            (
                "workspace_scene_manager = "
                "moveit_robot_control_node.workspace_scene_manager:main"
            ),
            (
                "pick_place_sequencer = "
                "moveit_robot_control_node.pick_place_sequencer:main"
            ),
        ],
    },
)
