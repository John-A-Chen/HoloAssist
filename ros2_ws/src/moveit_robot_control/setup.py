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
                "launch/ur_moveit.launch.py",
            ],
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
        ],
    },
)
