from setuptools import setup

package_name = "ur3_keyboard_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/keyboard_joint_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ollie",
    maintainer_email="ollie@example.com",
    description="Keyboard teleop for UR3 joint selection and direction commands.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_joint_teleop = ur3_keyboard_teleop.keyboard_joint_teleop:main",
        ],
    },
)
