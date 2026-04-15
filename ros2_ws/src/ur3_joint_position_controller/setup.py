from setuptools import setup

package_name = "ur3_joint_position_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/ur3_joint_position_controller.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ollie",
    maintainer_email="ollie@example.com",
    description="Convert UR3 keyboard teleop topics into forward position controller commands.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ur3_joint_position_controller = ur3_joint_position_controller.ur3_joint_position_controller:main",
        ],
    },
)
