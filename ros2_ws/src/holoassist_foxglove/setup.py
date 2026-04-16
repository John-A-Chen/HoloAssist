import os
from glob import glob

from setuptools import find_packages, setup


package_name = "holoassist_foxglove"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="john",
    maintainer_email="john@example.com",
    description="Foxglove runtime observability adapters and launch flows for HoloAssist.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "runtime_observability_node = holoassist_foxglove.runtime_observability_node:main",
            "obstacle_to_object_pose_adapter = holoassist_foxglove.obstacle_to_object_pose_adapter:main",
        ],
    },
)
